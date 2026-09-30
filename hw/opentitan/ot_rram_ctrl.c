/*
 * QEMU OpenTitan RRAM Controller device
 *
 * Copyright lowRISC contributors.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_fifo32.h"
#include "hw/opentitan/ot_flash.h"
#include "hw/opentitan/ot_lc_ctrl.h"
#include "hw/opentitan/ot_rram_ctrl.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "trace.h"

#define PARAM_NUM_REGIONS           10u
#define RRAM_CORE_SIZE              OT_RRAM_CORE_SIZE
#define RRAM_HOST_SIZE              0x200000u
#define RRAM_OTP_OFFSET             0x1ff600u
#define RRAM_OTP_SIZE               (RRAM_HOST_SIZE - RRAM_OTP_OFFSET)
#define RRAM_FIFO_DEPTH             16u
#define RRAM_INTERNAL_FIFO_CAPACITY 1024u
#define RRAM_WORD_U32_COUNT         4u
#define RRAM_PRIM_MMIO_SIZE         0x2000u
#define RRAM_PRIM_REG_BYTES         (OT_RRAM_PRIM_REGS_COUNT * sizeof(uint32_t))

#define RRAM_CREATOR_SEED_INFO_PAGE 5u
#define RRAM_OWNER_SEED_INFO_PAGE   6u

#define RRAM_FIFO_LVL_RST             0x0f0fu
#define RRAM_REGION_CFG_RST           0x99999u
#define RRAM_DEFAULT_REGION_RST       0x9999u
#define RRAM_INFO_CFG_RST             0x99999u
#define RRAM_HW_INFO_CFG_OVERRIDE_RST 0x99u

#define VFSH_MAGIC                  0x48534676u
#define VFSH_HDR_BYTES              32
#define VFSH_PAGES_PER_BANK_OFF     14
#define VFSH_DEFAULT_PAGES_PER_BANK 256u
#define VFSH_BANKS_COUNT            2u
#define VFSH_PAGE_BYTES             2048u
#define VFSH_DEFAULT_DATA_BYTES     0x100000u
#define VFSH_CREATOR_INFO_OFF       0x5000
#define RRAM_CREATOR_PARTITION_OFF  0x1fc000u

typedef enum {
    RRAM_OP_READ = 0u,
    RRAM_OP_PROG = 1u,
    RRAM_OP_HW_LOOPBACK = 2u,
    RRAM_OP_ERASE = 3u,
} OtRramOp;

REG32(INTR_STATE, 0x0u)
SHARED_FIELD(INTR_WR_EMPTY, 0u, 1u)
SHARED_FIELD(INTR_WR_LVL, 1u, 1u)
SHARED_FIELD(INTR_RD_FULL, 2u, 1u)
SHARED_FIELD(INTR_RD_LVL, 3u, 1u)
SHARED_FIELD(INTR_OP_DONE, 4u, 1u)
SHARED_FIELD(INTR_CORR_ERR, 5u, 1u)
REG32(INTR_ENABLE, 0x4u)
REG32(INTR_TEST, 0x8u)
REG32(ALERT_TEST, 0xcu)
REG32(DIS, 0x10u)
REG32(EXEC, 0x14u)
REG32(INIT, 0x18u)
REG32(CTRL_REGWEN, 0x1cu)
REG32(CONTROL, 0x20u)
FIELD(CONTROL, START, 0u, 1u)
FIELD(CONTROL, OP, 4u, 2u)
FIELD(CONTROL, PARTITION, 8u, 1u)
FIELD(CONTROL, NUM, 16u, 10u)
REG32(ADDR, 0x24u)
REG32(REGION_CFG_REGWEN_0, 0x28u)
REG32(REGION_CFG_REGWEN_9, 0x4cu)
REG32(REGION_CFG_0, 0x50u)
REG32(DEFAULT_REGION, 0xa0u)
REG32(INFO_REGWEN_0, 0xa4u)
REG32(INFO_REGWEN_7, 0xc0u)
REG32(INFO_CFG_0, 0xc4u)
SHARED_FIELD(INFO_CFG_EN, 0u, 4u)
SHARED_FIELD(INFO_CFG_RD_EN, 4u, 4u)
SHARED_FIELD(INFO_CFG_PROG_EN, 8u, 4u)
REG32(HW_INFO_CFG_OVERRIDE, 0xe4u)
REG32(OP_STATUS, 0xe8u)
FIELD(OP_STATUS, DONE, 0u, 1u)
FIELD(OP_STATUS, ERR, 1u, 1u)
REG32(STATUS, 0xecu)
FIELD(STATUS, RD_FULL, 0u, 1u)
FIELD(STATUS, RD_EMPTY, 1u, 1u)
FIELD(STATUS, WR_FULL, 2u, 1u)
FIELD(STATUS, WR_EMPTY, 3u, 1u)
FIELD(STATUS, INIT_DONE, 4u, 1u)
REG32(ERR_CODE, 0xf0u)
FIELD(ERR_CODE, OP_ERR, 0u, 1u)
FIELD(ERR_CODE, MP_ERR, 1u, 1u)
FIELD(ERR_CODE, RD_ERR, 2u, 1u)
FIELD(ERR_CODE, WR_ERR, 3u, 1u)
REG32(STD_FAULT_STATUS, 0xf4u)
REG32(FAULT_STATUS, 0xf8u)
REG32(ERR_ADDR, 0xfcu)
REG32(CORR_ERR_CNT, 0x100u)
REG32(CORR_ERR_LOC, 0x104u)
REG32(PHY_STATUS, 0x108u)
FIELD(PHY_STATUS, INIT_WIP, 0u, 1u)
FIELD(PHY_STATUS, PROG_NORMAL_AVAIL, 1u, 1u)
REG32(SCRATCH, 0x10cu)
REG32(FIFO_LVL, 0x110u)
FIELD(FIFO_LVL, WR, 0u, 5u)
FIELD(FIFO_LVL, RD, 8u, 5u)
REG32(FIFO_CLR, 0x114u)
REG32(CURR_FIFO_LVL, 0x118u)
FIELD(CURR_FIFO_LVL, WR, 0u, 5u)
FIELD(CURR_FIFO_LVL, RD, 8u, 5u)
REG32(WR_FIFO, 0x11cu)
REG32(RD_FIFO, 0x120u)

REG32(PRIM_CSR0_REGWEN, 0x0u)
REG32(PRIM_CSR1, 0x4u)
FIELD(PRIM_CSR1, VAL, 0u, 13u)
REG32(PRIM_CSR2, 0x8u)

#define R_PRIM_CSR2_RW_MASK  0x88u
#define R_PRIM_CSR2_W1C_MASK 0x77u

#define INTR_STATUS_MASK \
    (INTR_WR_EMPTY_MASK | INTR_WR_LVL_MASK | INTR_RD_FULL_MASK | \
     INTR_RD_LVL_MASK)
#define INTR_EVENT_MASK (INTR_OP_DONE_MASK | INTR_CORR_ERR_MASK)
#define INTR_ALL_MASK   (INTR_STATUS_MASK | INTR_EVENT_MASK)
#define R_FIFO_LVL_MASK (R_FIFO_LVL_WR_MASK | R_FIFO_LVL_RD_MASK)

/* Update interrupts */
static void ot_rram_ctrl_update_irqs(OtRramCtrlState *s)
{
    uint32_t state = s->regs[R_INTR_STATE];
    uint32_t enable = s->regs[R_INTR_ENABLE];
    uint32_t level = state & enable;

    for (unsigned i = 0u; i < OT_RRAM_IRQ_COUNT; i++) {
        ibex_irq_set(&s->irqs[i], (int)((level >> i) & 1u));
    }
}

/* Update status and current FIFO level */
static void ot_rram_ctrl_update_status(OtRramCtrlState *s)
{
    uint32_t wr_used = MIN(ot_fifo32_num_used(&s->wr_fifo), RRAM_FIFO_DEPTH);
    uint32_t rd_used = MIN(ot_fifo32_num_used(&s->rd_fifo), RRAM_FIFO_DEPTH);
    bool rd_full = (rd_used >= RRAM_FIFO_DEPTH);
    bool wr_full = (wr_used >= RRAM_FIFO_DEPTH);

    uint32_t status = 0;
    status = FIELD_DP32(status, STATUS, INIT_DONE, 1);
    status = FIELD_DP32(status, STATUS, RD_FULL, rd_full);
    status =
        FIELD_DP32(status, STATUS, RD_EMPTY, ot_fifo32_is_empty(&s->rd_fifo));
    status = FIELD_DP32(status, STATUS, WR_FULL, wr_full);
    status =
        FIELD_DP32(status, STATUS, WR_EMPTY, ot_fifo32_is_empty(&s->wr_fifo));
    s->regs[R_STATUS] = status;

    s->regs[R_CURR_FIFO_LVL] = FIELD_DP32(wr_used, CURR_FIFO_LVL, RD, rd_used);

    uint32_t wr_lvl = FIELD_EX32(s->regs[R_FIFO_LVL], FIFO_LVL, WR);
    uint32_t rd_lvl = FIELD_EX32(s->regs[R_FIFO_LVL], FIFO_LVL, RD);
    uint32_t status_irqs = 0;
    if (ot_fifo32_is_empty(&s->wr_fifo)) {
        status_irqs |= INTR_WR_EMPTY_MASK;
    }
    if (wr_used <= wr_lvl) {
        status_irqs |= INTR_WR_LVL_MASK;
    }
    if (rd_full) {
        status_irqs |= INTR_RD_FULL_MASK;
    }
    if (rd_used >= rd_lvl) {
        status_irqs |= INTR_RD_LVL_MASK;
    }
    s->regs[R_INTR_STATE] =
        (s->regs[R_INTR_STATE] & INTR_EVENT_MASK) | status_irqs |
        (s->regs[R_INTR_TEST] & INTR_STATUS_MASK);
    ot_rram_ctrl_update_irqs(s);
}

static bool ot_rram_ctrl_is_mp_err(OtRramCtrlState *s, uint32_t op,
                                   uint32_t part, uint32_t start_addr)
{
    if (op == RRAM_OP_HW_LOOPBACK) {
        return false;
    }
    if (!part && start_addr >= RRAM_OTP_OFFSET) {
        return true;
    }
    if (part && start_addr < OT_RRAM_INFO_STORAGE_BYTES) {
        uint32_t cfg =
            s->regs[R_INFO_CFG_0 + (start_addr / OT_RRAM_INFO_PAGE_BYTES)];
        bool en = SHARED_FIELD_EX32(cfg, INFO_CFG_EN) == OT_MULTIBITBOOL4_TRUE;
        bool perm = (op == RRAM_OP_READ) ?
                        (SHARED_FIELD_EX32(cfg, INFO_CFG_RD_EN) ==
                         OT_MULTIBITBOOL4_TRUE) :
                        (SHARED_FIELD_EX32(cfg, INFO_CFG_PROG_EN) ==
                         OT_MULTIBITBOOL4_TRUE);
        return !(en && perm);
    }
    return false;
}

/* Process WR FIFO */
static void ot_rram_ctrl_process_wr_fifo(OtRramCtrlState *s)
{
    if (!FIELD_EX32(s->regs[R_CONTROL], CONTROL, START)) {
        return;
    }

    uint32_t op = FIELD_EX32(s->regs[R_CONTROL], CONTROL, OP);
    uint32_t part = FIELD_EX32(s->regs[R_CONTROL], CONTROL, PARTITION);
    uint32_t num = FIELD_EX32(s->regs[R_CONTROL], CONTROL, NUM) + 1u;
    uint32_t start_addr = s->regs[R_ADDR] & (RRAM_HOST_SIZE - 1u) &
                          ~(uint32_t)(sizeof(uint32_t) - 1u);
    bool mp_err = ot_rram_ctrl_is_mp_err(s, op, part, start_addr);

    if (op == RRAM_OP_PROG || op == RRAM_OP_HW_LOOPBACK ||
        op == RRAM_OP_ERASE) {
        if (mp_err && ot_fifo32_num_used(&s->wr_fifo) >= 1u) {
            s->regs[R_ERR_CODE] |= R_ERR_CODE_MP_ERR_MASK;
            s->regs[R_OP_STATUS] =
                FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
        }
        if (ot_fifo32_num_used(&s->wr_fifo) >= num) {
            for (uint32_t i = 0; i < num; i++) {
                uint32_t val = ot_fifo32_pop(&s->wr_fifo);
                uint32_t addr = start_addr + i * sizeof(uint32_t);
                bool full_rram_word = ((i | (RRAM_WORD_U32_COUNT - 1u)) < num);
                if (part) {
                    if (!mp_err && full_rram_word &&
                        addr < sizeof(s->info_storage)) {
                        ((uint32_t *)s->info_storage)[addr / sizeof(uint32_t)] =
                            val;
                    }
                } else if (!mp_err && full_rram_word && addr < RRAM_HOST_SIZE) {
                    ((uint32_t *)s->storage)[addr / sizeof(uint32_t)] = val;
                }
            }
            if (!part && start_addr < RRAM_HOST_SIZE) {
                memory_region_set_dirty(&s->host_iomem, start_addr,
                                        num * sizeof(uint32_t));
            }

            if (!part && !mp_err && s->blk) {
                /* NOLINTBEGIN(clang-analyzer-optin.core.EnumCastOutOfRange) */
                blk_pwrite(
                    s->blk, start_addr,
                    (int64_t)num * (int64_t)sizeof(uint32_t),
                    s->storage + start_addr,
                    /* NOLINTEND(clang-analyzer-optin.core.EnumCastOutOfRange)
                     */
                    0); // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange)
            }

            if (mp_err) {
                s->regs[R_ERR_CODE] |= R_ERR_CODE_MP_ERR_MASK;
                s->regs[R_OP_STATUS] =
                    FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
                ibex_irq_set(&s->alerts[0], 1);
                ibex_irq_set(&s->alerts[0], 0);
            }

            if (num == 1u) {
                s->regs[R_ERR_ADDR] = s->last_wr_err_addr;
                s->last_wr_err_addr = start_addr;
            } else if (mp_err) {
                s->regs[R_ERR_ADDR] = start_addr;
                s->last_wr_err_addr = start_addr;
            }

            s->regs[R_OP_STATUS] =
                FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, DONE, 1);
            s->regs[R_CONTROL] =
                FIELD_DP32(s->regs[R_CONTROL], CONTROL, START, 0);
            s->regs[R_INTR_STATE] |= INTR_OP_DONE_MASK;
            s->regs[R_CTRL_REGWEN] = 1;
            ot_rram_ctrl_update_status(s);
        }
    }
}

/* Execute OP */
static void ot_rram_ctrl_execute_op(OtRramCtrlState *s)
{
    uint32_t op = FIELD_EX32(s->regs[R_CONTROL], CONTROL, OP);
    uint32_t part = FIELD_EX32(s->regs[R_CONTROL], CONTROL, PARTITION);
    uint32_t num = FIELD_EX32(s->regs[R_CONTROL], CONTROL, NUM) + 1u;
    uint32_t start_addr = s->regs[R_ADDR] & (RRAM_HOST_SIZE - 1u) &
                          ~(uint32_t)(sizeof(uint32_t) - 1u);
    bool mp_err = ot_rram_ctrl_is_mp_err(s, op, part, start_addr);

    if (op == RRAM_OP_READ) {
        for (uint32_t i = 0; i < num; i++) {
            if (!ot_fifo32_is_full(&s->rd_fifo)) {
                uint32_t addr = start_addr + i * sizeof(uint32_t);
                uint32_t val = UINT32_MAX;
                if (!mp_err) {
                    if (part) {
                        if (addr < sizeof(s->info_storage)) {
                            val =
                                ((uint32_t *)
                                     s->info_storage)[addr / sizeof(uint32_t)];
                        }
                    } else if (addr < RRAM_HOST_SIZE) {
                        val = ((uint32_t *)s->storage)[addr / sizeof(uint32_t)];
                    }
                }
                ot_fifo32_push(&s->rd_fifo, val);
            }
        }

        if (mp_err) {
            s->regs[R_ERR_CODE] |= R_ERR_CODE_MP_ERR_MASK;
            s->regs[R_OP_STATUS] =
                FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
            ibex_irq_set(&s->alerts[0], 1);
            ibex_irq_set(&s->alerts[0], 0);
        }

        if (num == 1u) {
            s->regs[R_ERR_ADDR] = s->last_rd_err_addr;
            s->last_rd_err_addr = start_addr;
        } else if (mp_err) {
            s->regs[R_ERR_ADDR] = start_addr;
            s->last_rd_err_addr = start_addr;
        }

        s->regs[R_OP_STATUS] =
            FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, DONE, 1);
        s->regs[R_CONTROL] = FIELD_DP32(s->regs[R_CONTROL], CONTROL, START, 0);
        s->regs[R_INTR_STATE] |= INTR_OP_DONE_MASK;
        s->regs[R_CTRL_REGWEN] = 1;
        ot_rram_ctrl_update_status(s);
    } else if (op == RRAM_OP_PROG) {
        s->regs[R_CTRL_REGWEN] = 0;
        ot_rram_ctrl_process_wr_fifo(s);
    } else if (op == RRAM_OP_HW_LOOPBACK) {
        s->regs[R_OP_STATUS] =
            FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, DONE, 1);
        s->regs[R_CONTROL] = FIELD_DP32(s->regs[R_CONTROL], CONTROL, START, 0);
        s->regs[R_INTR_STATE] |= INTR_OP_DONE_MASK;
        s->regs[R_CTRL_REGWEN] = 1;
        ot_rram_ctrl_update_status(s);
    }
}

static const uint8_t
    RRAM_DEFAULT_SEEDS[OT_RRAM_SEED_COUNT][OT_RRAM_SEED_BYTES] = {
        /* CreatorSeed (seeds[0]): RndCnstAllSeedsDefault[255:0] */
        {
            0x77u, 0x74u, 0x44u, 0x1bu, 0x72u, 0xfeu, 0x65u, 0x65u,
            0x55u, 0xcdu, 0x6du, 0x18u, 0x48u, 0x3bu, 0xb0u, 0x32u,
            0x34u, 0x98u, 0x90u, 0xf9u, 0xebu, 0xb0u, 0x4au, 0x98u,
            0x76u, 0x3au, 0x31u, 0x5au, 0xe2u, 0x77u, 0xdeu, 0x9cu,
        },
        /* OwnerSeed (seeds[1]): RndCnstAllSeedsDefault[511:256] */
        {
            0x91u, 0x40u, 0xd0u, 0xa6u, 0x29u, 0xe0u, 0xefu, 0xccu,
            0xb5u, 0xedu, 0x15u, 0x53u, 0x6fu, 0x89u, 0x18u, 0x62u,
            0x2cu, 0x8au, 0x23u, 0xb6u, 0x0eu, 0x24u, 0x4du, 0xadu,
            0x1eu, 0x48u, 0x9eu, 0x0du, 0x4cu, 0x87u, 0x28u, 0x35u,
        },
    };

static void ot_rram_ctrl_latch_seeds(OtRramCtrlState *s)
{
    static const unsigned SEED_INFO_PAGES[OT_RRAM_SEED_COUNT] = {
        RRAM_CREATOR_SEED_INFO_PAGE,
        RRAM_OWNER_SEED_INFO_PAGE,
    };

    for (unsigned idx = 0u; idx < OT_RRAM_SEED_COUNT; idx++) {
        const uint8_t *page =
            &s->info_storage[SEED_INFO_PAGES[idx] * OT_RRAM_INFO_PAGE_BYTES];
        bool all_ones = true;
        for (unsigned b = 0u; b < OT_RRAM_SEED_BYTES; b++) {
            if (page[b] != UINT8_MAX) {
                all_ones = false;
                break;
            }
        }
        if (all_ones) {
            memcpy(s->seeds[idx], RRAM_DEFAULT_SEEDS[idx], OT_RRAM_SEED_BYTES);
        } else {
            memcpy(s->seeds[idx], page, OT_RRAM_SEED_BYTES);
        }
    }
}

void ot_rram_ctrl_get_seed(const OtRramCtrlState *s, unsigned idx,
                           uint8_t *seed)
{
    g_assert(idx < OT_RRAM_SEED_COUNT);
    memcpy(seed, s->seeds[idx], OT_RRAM_SEED_BYTES);
}

static MemTxResult rram_ctrl_core_read(void *opaque, hwaddr addr, uint64_t *val,
                                       unsigned size, MemTxAttrs attrs)
{
    (void)size;
    (void)attrs;
    OtRramCtrlState *s = opaque;
    uint32_t val32 = 0;

    switch (addr) {
    case R_WR_FIFO * sizeof(uint32_t):
        return MEMTX_ERROR;

    case R_RD_FIFO * sizeof(uint32_t):
        if (!ot_fifo32_is_empty(&s->rd_fifo)) {
            val32 = ot_fifo32_pop(&s->rd_fifo);
            ot_rram_ctrl_update_status(s);
        }
        break;

    default:
        if (addr < sizeof(s->regs)) {
            val32 = s->regs[addr / sizeof(uint32_t)];
        }
        break;
    }

    *val = val32;
    return MEMTX_OK;
}

static MemTxResult rram_ctrl_core_write(void *opaque, hwaddr addr, uint64_t val,
                                        unsigned size, MemTxAttrs attrs)
{
    (void)attrs;
    OtRramCtrlState *s = opaque;
    uint32_t val32 = val;

    if (size != sizeof(uint32_t) &&
        !(size == 1u && addr == (R_CTRL_REGWEN * sizeof(uint32_t)))) {
        return MEMTX_ERROR;
    }

    switch (addr) {
    case R_RD_FIFO * sizeof(uint32_t):
        return MEMTX_ERROR;

    case R_WR_FIFO * sizeof(uint32_t):
        if (!FIELD_EX32(s->regs[R_CONTROL], CONTROL, START)) {
            return MEMTX_OK;
        }

        if (!ot_fifo32_is_full(&s->wr_fifo)) {
            ot_fifo32_push(&s->wr_fifo, val32);
            ot_rram_ctrl_update_status(s);
            ot_rram_ctrl_process_wr_fifo(s);
        }
        break;

    case R_CONTROL * sizeof(uint32_t):
        s->regs[R_CONTROL] = val32;
        if (FIELD_EX32(val32, CONTROL, START)) {
            s->regs[R_OP_STATUS] = 0;
            s->regs[R_ERR_CODE] = 0;
            ot_rram_ctrl_execute_op(s);
        }
        break;

    case R_INTR_STATE * sizeof(uint32_t):
        s->regs[R_INTR_STATE] &= ~(val32 & INTR_EVENT_MASK);
        ot_rram_ctrl_update_status(s);
        break;
    case R_INTR_ENABLE * sizeof(uint32_t):
        s->regs[R_INTR_ENABLE] = val32 & INTR_ALL_MASK;
        ot_rram_ctrl_update_irqs(s);
        break;
    case R_INTR_TEST * sizeof(uint32_t):
        s->regs[R_INTR_TEST] = val32 & INTR_STATUS_MASK;
        s->regs[R_INTR_STATE] |= val32 & INTR_EVENT_MASK;
        ot_rram_ctrl_update_status(s);
        break;
    case R_FIFO_LVL * sizeof(uint32_t):
        s->regs[R_FIFO_LVL] = val32 & R_FIFO_LVL_MASK;
        ot_rram_ctrl_update_status(s);
        break;
    case R_ALERT_TEST * sizeof(uint32_t):
        for (unsigned i = 0u; i < OT_RRAM_ALERT_COUNT; i++) {
            if (val32 & (1u << i)) {
                ibex_irq_set(&s->alerts[i], 1);
                ibex_irq_set(&s->alerts[i], 0);
            }
        }
        break;

    case R_FIFO_CLR * sizeof(uint32_t):
        ot_fifo32_reset(&s->wr_fifo);
        ot_fifo32_reset(&s->rd_fifo);
        ot_rram_ctrl_update_status(s);
        break;

    case R_INIT * sizeof(uint32_t):
        s->regs[R_INIT] = val32 & 1u;
        if ((val32 & 1u) && !s->lcmgr_init_done) {
            ot_rram_ctrl_latch_seeds(s);
            s->lcmgr_init_done = true;
        }
        break;

    case R_ADDR * sizeof(uint32_t):
        s->regs[R_ADDR] =
            val32 & (RRAM_HOST_SIZE - 1u) & ~(uint32_t)(sizeof(uint32_t) - 1u);
        break;

    case R_CTRL_REGWEN * sizeof(uint32_t):
    case (R_REGION_CFG_REGWEN_0 *
          sizeof(uint32_t))...(R_REGION_CFG_REGWEN_9 * sizeof(uint32_t)):
    case (R_INFO_REGWEN_0 *
          sizeof(uint32_t))...(R_INFO_REGWEN_7 * sizeof(uint32_t)):
        s->regs[addr / sizeof(uint32_t)] &= (val32 & 1u);
        break;

    default:
        if (addr < sizeof(s->regs)) {
            s->regs[addr / sizeof(uint32_t)] = val32;
        }
        break;
    }

    return MEMTX_OK;
}

static const MemoryRegionOps rram_ctrl_core_ops = {
    .read_with_attrs = rram_ctrl_core_read,
    .write_with_attrs = rram_ctrl_core_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static MemTxResult rram_ctrl_otp_read(void *opaque, hwaddr addr, uint64_t *val,
                                      unsigned size, MemTxAttrs attrs)
{
    (void)opaque;
    (void)addr;
    (void)size;
    (void)attrs;
    *val = 0;
    return MEMTX_ERROR;
}

static MemTxResult rram_ctrl_otp_write(void *opaque, hwaddr addr, uint64_t val,
                                       unsigned size, MemTxAttrs attrs)
{
    (void)opaque;
    (void)addr;
    (void)val;
    (void)size;
    (void)attrs;
    return MEMTX_ERROR;
}

static const MemoryRegionOps rram_ctrl_otp_ops = {
    .read_with_attrs = rram_ctrl_otp_read,
    .write_with_attrs = rram_ctrl_otp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void ot_rram_ctrl_load(OtRramCtrlState *s, Error **errp)
{
    if (s->blk) {
        uint64_t perm = BLK_PERM_CONSISTENT_READ |
                        (blk_supports_write_perm(s->blk) ? BLK_PERM_WRITE : 0);
        blk_set_perm(s->blk, perm, perm, errp);

        uint32_t magic = 0;
        /* NOLINTBEGIN(clang-analyzer-optin.core.EnumCastOutOfRange) */
        blk_pread(s->blk, 0, sizeof(magic), &magic, 0);

        /* "vFSH" in little endian: 'v'=0x76, 'F'=0x46, 'S'=0x53, 'H'=0x48 */
        if (magic == VFSH_MAGIC) {
            int64_t blk_len = blk_getlength(s->blk);
            uint16_t pages_per_bank = VFSH_DEFAULT_PAGES_PER_BANK;
            blk_pread(s->blk, VFSH_PAGES_PER_BANK_OFF, sizeof(pages_per_bank),
                      &pages_per_bank, 0);
            uint32_t data_size =
                (uint32_t)pages_per_bank * VFSH_BANKS_COUNT * VFSH_PAGE_BYTES;
            if (data_size == 0 || data_size > RRAM_HOST_SIZE) {
                data_size = VFSH_DEFAULT_DATA_BYTES;
            }
            int64_t read_size =
                MIN((int64_t)data_size, blk_len - VFSH_HDR_BYTES);
            if (read_size > 0) {
                blk_pread(s->blk, VFSH_HDR_BYTES, read_size, s->storage, 0);
            }

            int64_t info_base = VFSH_HDR_BYTES + (int64_t)data_size;
            if (blk_len >= info_base + (int64_t)sizeof(s->info_storage)) {
                blk_pread(s->blk, info_base, sizeof(s->info_storage),
                          s->info_storage, 0);
            }
            if (blk_len >=
                info_base + VFSH_CREATOR_INFO_OFF + (int64_t)VFSH_PAGE_BYTES) {
                blk_pread(s->blk, info_base + VFSH_CREATOR_INFO_OFF,
                          VFSH_PAGE_BYTES,
                          s->storage + RRAM_CREATOR_PARTITION_OFF, 0);
            }
        } else {
            blk_pread(s->blk, 0, RRAM_HOST_SIZE, s->storage, 0);
        }
        /* NOLINTEND(clang-analyzer-optin.core.EnumCastOutOfRange) */
    }
}

static void ot_rram_ctrl_reset(DeviceState *dev)
{
    OtRramCtrlState *s = OT_RRAM_CTRL(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_DIS] = OT_MULTIBITBOOL4_FALSE;
    s->regs[R_EXEC] = OT_MULTIBITBOOL4_FALSE;
    s->regs[R_STATUS] = R_STATUS_RD_EMPTY_MASK | R_STATUS_WR_EMPTY_MASK |
                        R_STATUS_INIT_DONE_MASK;
    s->regs[R_PHY_STATUS] = R_PHY_STATUS_PROG_NORMAL_AVAIL_MASK;
    s->regs[R_FIFO_LVL] = RRAM_FIFO_LVL_RST;
    s->regs[R_CTRL_REGWEN] = 1u;
    for (unsigned i = 0u; i < PARAM_NUM_REGIONS; i++) {
        s->regs[R_REGION_CFG_REGWEN_0 + i] = 1u;
        s->regs[R_REGION_CFG_0 + i] = RRAM_REGION_CFG_RST;
    }
    s->regs[R_DEFAULT_REGION] = RRAM_DEFAULT_REGION_RST;
    for (unsigned i = 0u; i < OT_RRAM_INFO_PAGE_COUNT; i++) {
        s->regs[R_INFO_REGWEN_0 + i] = 1u;
        s->regs[R_INFO_CFG_0 + i] = RRAM_INFO_CFG_RST;
    }
    s->regs[R_HW_INFO_CFG_OVERRIDE] = RRAM_HW_INFO_CFG_OVERRIDE_RST;

    s->last_wr_err_addr = 0;
    s->last_rd_err_addr = 0;
    s->lcmgr_init_done = false;
    ot_rram_ctrl_latch_seeds(s);

    /* rram_macro_prim_reg_top.sv: CSR0_REGWEN=1 (rw0c), CSR1=0 (rw), CSR2=0
     * (w1c) */
    s->prim_regs[R_PRIM_CSR0_REGWEN] = 1u;
    s->prim_regs[R_PRIM_CSR1] = 0u;
    s->prim_regs[R_PRIM_CSR2] = 0u;

    ot_fifo32_reset(&s->wr_fifo);
    ot_fifo32_reset(&s->rd_fifo);
    ot_rram_ctrl_update_status(s);
}

static MemTxResult rram_ctrl_prim_read(void *opaque, hwaddr addr, uint64_t *val,
                                       unsigned size, MemTxAttrs attrs)
{
    (void)size;
    (void)attrs;
    OtRramCtrlState *s = opaque;
    if (addr >= RRAM_PRIM_REG_BYTES || (addr & (sizeof(uint32_t) - 1u)) != 0u) {
        return MEMTX_ERROR;
    }
    *val = s->prim_regs[addr / sizeof(uint32_t)];
    return MEMTX_OK;
}

static MemTxResult rram_ctrl_prim_write(void *opaque, hwaddr addr, uint64_t val,
                                        unsigned size, MemTxAttrs attrs)
{
    (void)attrs;
    OtRramCtrlState *s = opaque;
    uint32_t v32 = (uint32_t)val;
    if (addr >= RRAM_PRIM_REG_BYTES || (addr & (sizeof(uint32_t) - 1u)) != 0u) {
        return MEMTX_ERROR;
    }
    /* RRAM_MACRO_PRIM_PERMIT = '{4'b0001, 4'b0011, 4'b0001} */
    if (addr == (R_PRIM_CSR1 * sizeof(uint32_t)) && size < sizeof(uint16_t)) {
        return MEMTX_ERROR;
    }
    switch (addr) {
    case R_PRIM_CSR0_REGWEN * sizeof(uint32_t):
        if ((v32 & 1u) == 0u) {
            s->prim_regs[R_PRIM_CSR0_REGWEN] = 0u;
        }
        break;
    case R_PRIM_CSR1 * sizeof(uint32_t):
        if (s->prim_regs[R_PRIM_CSR0_REGWEN] & 1u) {
            s->prim_regs[R_PRIM_CSR1] = v32 & R_PRIM_CSR1_VAL_MASK;
        }
        break;
    case R_PRIM_CSR2 * sizeof(uint32_t):
        s->prim_regs[R_PRIM_CSR2] =
            ((s->prim_regs[R_PRIM_CSR2] & ~R_PRIM_CSR2_RW_MASK) &
             ~(v32 & R_PRIM_CSR2_W1C_MASK)) |
            (v32 & R_PRIM_CSR2_RW_MASK);
        break;
    default:
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rram_ctrl_prim_ops = {
    .read_with_attrs = &rram_ctrl_prim_read,
    .write_with_attrs = &rram_ctrl_prim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1u,
        .max_access_size = 4u,
    },
};

static void ot_rram_ctrl_lc_broadcast(void *opaque, int n, int level)
{
    OtRramCtrlState *s = opaque;

    if (n == OT_FLASH_LC_RMA && level) {
        memset(s->storage, UINT8_MAX, RRAM_OTP_OFFSET);
        memset(s->info_storage, UINT8_MAX, sizeof(s->info_storage));
        memory_region_set_dirty(&s->host_iomem, 0, RRAM_OTP_OFFSET);
        if (s->blk && blk_is_writable(s->blk)) {
            /* NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange) */
            blk_pwrite(s->blk, 0, RRAM_OTP_OFFSET, s->storage, 0);
        }
    }
}

static void ot_rram_ctrl_realize(DeviceState *dev, Error **errp)
{
    OtRramCtrlState *s = OT_RRAM_CTRL(dev);

    s->storage = g_malloc0(RRAM_HOST_SIZE);
    memset(s->storage, UINT8_MAX, RRAM_HOST_SIZE);
    memset(s->info_storage, UINT8_MAX, sizeof(s->info_storage));
    ot_rram_ctrl_load(s, errp);

    memory_region_init_io(&s->core_iomem, OBJECT(dev), &rram_ctrl_core_ops, s,
                          TYPE_OT_RRAM_CTRL ".core", RRAM_CORE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->core_iomem);

    memory_region_init_ram_ptr(&s->host_iomem, OBJECT(dev),
                               TYPE_OT_RRAM_CTRL ".host", RRAM_HOST_SIZE,
                               s->storage);
    s->host_iomem.readonly = true;

    memory_region_init_io(&s->otp_iomem, OBJECT(dev), &rram_ctrl_otp_ops, s,
                          TYPE_OT_RRAM_CTRL ".otp", RRAM_OTP_SIZE);
    memory_region_add_subregion_overlap(&s->host_iomem, RRAM_OTP_OFFSET,
                                        &s->otp_iomem, 1);

    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->host_iomem);

    memory_region_init_io(&s->prim_iomem, OBJECT(dev), &rram_ctrl_prim_ops, s,
                          TYPE_OT_RRAM_CTRL ".prim", RRAM_PRIM_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->prim_iomem);

    for (unsigned i = 0u; i < OT_RRAM_IRQ_COUNT; i++) {
        ibex_sysbus_init_irq(OBJECT(s), &s->irqs[i]);
    }
    for (unsigned i = 0u; i < OT_RRAM_ALERT_COUNT; i++) {
        ibex_qdev_init_irq(OBJECT(dev), &s->alerts[i], OT_DEVICE_ALERT);
    }
    qdev_init_gpio_in_named(dev, &ot_rram_ctrl_lc_broadcast, OT_LC_BROADCAST,
                            OT_FLASH_LC_BROADCAST_COUNT);
}

static void ot_rram_ctrl_init(Object *obj)
{
    OtRramCtrlState *s = OT_RRAM_CTRL(obj);

    ot_fifo32_create(&s->wr_fifo, RRAM_INTERNAL_FIFO_CAPACITY);
    ot_fifo32_create(&s->rd_fifo, RRAM_INTERNAL_FIFO_CAPACITY);
}

static const Property ot_rram_ctrl_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtRramCtrlState, ot_id),
    DEFINE_PROP_DRIVE("drive", OtRramCtrlState, blk),
};

static void ot_rram_ctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_legacy_reset(dc, &ot_rram_ctrl_reset);
    dc->realize = ot_rram_ctrl_realize;
    device_class_set_props(dc, ot_rram_ctrl_properties);
}

static const TypeInfo ot_rram_ctrl_info = {
    .name = TYPE_OT_RRAM_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtRramCtrlState),
    .instance_init = ot_rram_ctrl_init,
    .class_init = ot_rram_ctrl_class_init,
};

static void ot_rram_ctrl_register_types(void)
{
    type_register_static(&ot_rram_ctrl_info);
}

type_init(ot_rram_ctrl_register_types)
