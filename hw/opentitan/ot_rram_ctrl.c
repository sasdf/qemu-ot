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

#define PARAM_NUM_REGIONS 10
#define RRAM_CORE_SIZE    0x200
#define RRAM_HOST_SIZE    0x200000
#define RRAM_OTP_OFFSET   0x1ff600
#define RRAM_OTP_SIZE     0x200

REG32(INTR_STATE, 0x0)
SHARED_FIELD(INTR_WR_EMPTY, 0, 1)
SHARED_FIELD(INTR_WR_LVL, 1, 1)
SHARED_FIELD(INTR_RD_FULL, 2, 1)
SHARED_FIELD(INTR_RD_LVL, 3, 1)
SHARED_FIELD(INTR_OP_DONE, 4, 1)
SHARED_FIELD(INTR_CORR_ERR, 5, 1)
REG32(INTR_ENABLE, 0x4)
REG32(INTR_TEST, 0x8)
REG32(ALERT_TEST, 0xc)
REG32(DIS, 0x10)
REG32(EXEC, 0x14)
REG32(INIT, 0x18)
REG32(CTRL_REGWEN, 0x1c)
REG32(CONTROL, 0x20)
FIELD(CONTROL, START, 0, 1)
FIELD(CONTROL, OP, 4, 2)
FIELD(CONTROL, PARTITION, 8, 1)
FIELD(CONTROL, NUM, 16, 10)
REG32(ADDR, 0x24)
REG32(OP_STATUS, 0xe8)
FIELD(OP_STATUS, DONE, 0, 1)
FIELD(OP_STATUS, ERR, 1, 1)
REG32(STATUS, 0xec)
FIELD(STATUS, RD_FULL, 0, 1)
FIELD(STATUS, RD_EMPTY, 1, 1)
FIELD(STATUS, WR_FULL, 2, 1)
FIELD(STATUS, WR_EMPTY, 3, 1)
FIELD(STATUS, INIT_DONE, 4, 1)
REG32(ERR_CODE, 0xf0)
FIELD(ERR_CODE, OP_ERR, 0, 1)
FIELD(ERR_CODE, MP_ERR, 1, 1)
FIELD(ERR_CODE, RD_ERR, 2, 1)
FIELD(ERR_CODE, WR_ERR, 3, 1)
REG32(STD_FAULT_STATUS, 0xf4)
REG32(FAULT_STATUS, 0xf8)
REG32(ERR_ADDR, 0xfc)
REG32(CORR_ERR_CNT, 0x100)
REG32(CORR_ERR_LOC, 0x104)
REG32(PHY_STATUS, 0x108)
REG32(SCRATCH, 0x10c)
REG32(FIFO_LVL, 0x110)
REG32(FIFO_CLR, 0x114)
REG32(CURR_FIFO_LVL, 0x118)
REG32(WR_FIFO, 0x11c)
REG32(RD_FIFO, 0x120)

/* Update interrupts */
static void ot_rram_ctrl_update_irqs(OtRramCtrlState *s)
{
    uint32_t state = s->regs[R_INTR_STATE];
    uint32_t enable = s->regs[R_INTR_ENABLE];
    uint32_t level = state & enable;

    for (int i = 0; i < 6; i++) {
        ibex_irq_set(&s->irqs[i], (int)((level >> i) & 1u));
    }
}

/* Update status and current FIFO level */
static void ot_rram_ctrl_update_status(OtRramCtrlState *s)
{
    uint32_t status = 0;
    status = FIELD_DP32(status, STATUS, INIT_DONE, 1);
    status =
        FIELD_DP32(status, STATUS, RD_FULL, ot_fifo32_is_full(&s->rd_fifo));
    status =
        FIELD_DP32(status, STATUS, RD_EMPTY, ot_fifo32_is_empty(&s->rd_fifo));
    status =
        FIELD_DP32(status, STATUS, WR_FULL, ot_fifo32_is_full(&s->wr_fifo));
    status =
        FIELD_DP32(status, STATUS, WR_EMPTY, ot_fifo32_is_empty(&s->wr_fifo));
    s->regs[R_STATUS] = status;

    uint32_t cur_lvl = 0;
    cur_lvl |= (ot_fifo32_num_used(&s->rd_fifo) & 0x1fu) << 8;
    cur_lvl |= ot_fifo32_num_used(&s->wr_fifo) & 0x1fu;
    s->regs[R_CURR_FIFO_LVL] = cur_lvl;

    /* Basic thresholds */
    if (ot_fifo32_is_empty(&s->wr_fifo)) {
        s->regs[R_INTR_STATE] |= INTR_WR_EMPTY_MASK;
    }
    ot_rram_ctrl_update_irqs(s);
}

static bool ot_rram_ctrl_is_mp_err(OtRramCtrlState *s, uint32_t op,
                                   uint32_t part, uint32_t start_addr)
{
    if (op == 2u) {
        return false;
    }
    if (!part && start_addr >= RRAM_OTP_OFFSET) {
        return true;
    }
    if (part && start_addr < 0x1000u) {
        uint32_t cfg = s->regs[(0xc4 / 4) + (start_addr / 0x200u)];
        bool en = (cfg & 0xfu) == 0x6u;
        bool perm = (op == 0u) ? (((cfg >> 4u) & 0xfu) == 0x6u) :
                                 (((cfg >> 8u) & 0xfu) == 0x6u);
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
    uint32_t num = FIELD_EX32(s->regs[R_CONTROL], CONTROL, NUM) + 1;
    uint32_t start_addr = s->regs[R_ADDR] & ~3;
    bool mp_err = ot_rram_ctrl_is_mp_err(s, op, part, start_addr);

    if (op == 1 || op == 2 || op == 3) {
        if (mp_err && ot_fifo32_num_used(&s->wr_fifo) >= 1) {
            s->regs[R_ERR_CODE] |= (1 << 1); /* MP_ERR */
            s->regs[R_OP_STATUS] =
                FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
        }
        if (ot_fifo32_num_used(&s->wr_fifo) >= num) {
            for (uint32_t i = 0; i < num; i++) {
                uint32_t val = ot_fifo32_pop(&s->wr_fifo);
                uint32_t addr = start_addr + i * 4;
                if (part) {
                    if (!mp_err && addr < sizeof(s->info_storage)) {
                        ((uint32_t *)s->info_storage)[addr / 4] = val;
                    }
                } else if (!mp_err && addr < RRAM_HOST_SIZE) {
                    ((uint32_t *)s->storage)[addr / 4] = val;
                }
            }

            if (!part && !mp_err && s->blk) {
                /* NOLINTBEGIN(clang-analyzer-optin.core.EnumCastOutOfRange) */
                blk_pwrite(
                    s->blk, start_addr, num * 4, s->storage + start_addr,
                    /* NOLINTEND(clang-analyzer-optin.core.EnumCastOutOfRange)
                     */
                    0); // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange)
            }

            if (mp_err) {
                s->regs[R_ERR_CODE] |= (1 << 1); /* MP_ERR */
                s->regs[R_OP_STATUS] =
                    FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
                ibex_irq_set(&s->alerts[0], 1);
                ibex_irq_set(&s->alerts[0], 0);
            }

            if (num == 1) {
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
    uint32_t num = FIELD_EX32(s->regs[R_CONTROL], CONTROL, NUM) + 1;
    uint32_t start_addr = s->regs[R_ADDR] & ~3;
    bool mp_err = ot_rram_ctrl_is_mp_err(s, op, part, start_addr);

    if (op == 0) { /* READ */
        for (uint32_t i = 0; i < num; i++) {
            if (!ot_fifo32_is_full(&s->rd_fifo)) {
                uint32_t addr = start_addr + i * 4;
                uint32_t val = 0xffffffffu;
                if (!mp_err) {
                    if (part) {
                        if (addr < sizeof(s->info_storage)) {
                            val = ((uint32_t *)s->info_storage)[addr / 4];
                        }
                    } else if (addr < RRAM_HOST_SIZE) {
                        val = ((uint32_t *)s->storage)[addr / 4];
                    }
                }
                ot_fifo32_push(&s->rd_fifo, val);
            }
        }

        if (mp_err) {
            s->regs[R_ERR_CODE] |= (1 << 1); /* MP_ERR */
            s->regs[R_OP_STATUS] =
                FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, ERR, 1);
            ibex_irq_set(&s->alerts[0], 1);
            ibex_irq_set(&s->alerts[0], 0);
        }

        if (num == 1) {
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
    } else if (op == 1) { /* WRITE (Prog) */
        s->regs[R_CTRL_REGWEN] = 0;
        ot_rram_ctrl_process_wr_fifo(s);
    } else if (op == 2) { /* REWRITE (HwLoopBack) */
        s->regs[R_OP_STATUS] =
            FIELD_DP32(s->regs[R_OP_STATUS], OP_STATUS, DONE, 1);
        s->regs[R_CONTROL] = FIELD_DP32(s->regs[R_CONTROL], CONTROL, START, 0);
        s->regs[R_INTR_STATE] |= INTR_OP_DONE_MASK;
        s->regs[R_CTRL_REGWEN] = 1;
        ot_rram_ctrl_update_status(s);
    }
}

static MemTxResult rram_ctrl_core_read(void *opaque, hwaddr addr, uint64_t *val,
                                       unsigned size, MemTxAttrs attrs)
{
    (void)size;
    (void)attrs;
    OtRramCtrlState *s = opaque;
    uint32_t val32 = 0;

    switch (addr) {
    case R_WR_FIFO * 4:
        return MEMTX_ERROR;

    case R_RD_FIFO * 4:
        if (!ot_fifo32_is_empty(&s->rd_fifo)) {
            val32 = ot_fifo32_pop(&s->rd_fifo);
            ot_rram_ctrl_update_status(s);
        }
        break;

    default:
        if (addr < sizeof(s->regs)) {
            val32 = s->regs[addr / 4];
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

    if (size != 4 && !(size == 1 && addr == 0x1c)) {
        return MEMTX_ERROR;
    }

    switch (addr) {
    case R_RD_FIFO * 4:
        return MEMTX_ERROR;

    case R_WR_FIFO * 4:
        if (!FIELD_EX32(s->regs[R_CONTROL], CONTROL, START)) {
            return MEMTX_OK;
        }

        if (!ot_fifo32_is_full(&s->wr_fifo)) {
            ot_fifo32_push(&s->wr_fifo, val32);
            ot_rram_ctrl_update_status(s);
            ot_rram_ctrl_process_wr_fifo(s);
        }
        break;

    case R_CONTROL * 4:
        s->regs[R_CONTROL] = val32;
        if (FIELD_EX32(val32, CONTROL, START)) {
            s->regs[R_OP_STATUS] = 0;
            s->regs[R_ERR_CODE] = 0;
            ot_rram_ctrl_execute_op(s);
        }
        break;

    case R_INTR_STATE * 4:
        s->regs[R_INTR_STATE] &= ~val32;
        ot_rram_ctrl_update_irqs(s);
        break;
    case R_INTR_ENABLE * 4:
        s->regs[R_INTR_ENABLE] = val32;
        ot_rram_ctrl_update_irqs(s);
        break;
    case R_INTR_TEST * 4:
        s->regs[R_INTR_STATE] |= val32;
        ot_rram_ctrl_update_irqs(s);
        break;
    case R_ALERT_TEST * 4:
        for (int i = 0; i < 5; i++) {
            if (val32 & (1u << i)) {
                ibex_irq_set(&s->alerts[i], 1);
                ibex_irq_set(&s->alerts[i], 0);
            }
        }
        break;

    case R_FIFO_CLR * 4:
        ot_fifo32_reset(&s->wr_fifo);
        ot_fifo32_reset(&s->rd_fifo);
        ot_rram_ctrl_update_status(s);
        break;

    case R_ADDR * 4:
        s->regs[R_ADDR] = val32 & ~3;
        break;

    case 0x1c: /* CTRL_REGWEN (rw0c) */
    case 0x28 ... 0x4c: /* REGION_CFG_REGWEN_0..9 (rw0c) */
    case 0xa4 ... 0xc0: /* INFO_REGWEN_0..7 (rw0c) */
        s->regs[addr / 4] &= (val32 & 1u);
        break;

    default:
        if (addr < sizeof(s->regs)) {
            s->regs[addr / 4] = val32;
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
        if (magic == 0x48534676) {
            int64_t blk_len = blk_getlength(s->blk);
            uint16_t pages_per_bank = 256;
            blk_pread(s->blk, 14, sizeof(pages_per_bank), &pages_per_bank, 0);
            uint32_t data_size = (uint32_t)pages_per_bank * 2u * 2048u;
            if (data_size == 0 || data_size > RRAM_HOST_SIZE) {
                data_size = 0x100000;
            }
            int64_t read_size = MIN((int64_t)data_size, blk_len - 32);
            if (read_size > 0) {
                blk_pread(s->blk, 32, read_size, s->storage, 0);
            }

            int64_t info_base = 32 + (int64_t)data_size;
            if (blk_len >= info_base + (int64_t)sizeof(s->info_storage)) {
                blk_pread(s->blk, info_base, sizeof(s->info_storage),
                          s->info_storage, 0);
            }
            if (blk_len >= info_base + 0x5000 + 2048) {
                blk_pread(s->blk, info_base + 0x5000, 2048,
                          s->storage + 0x1FC000, 0);
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
    s->regs[R_DIS] = 0x9u;
    s->regs[R_EXEC] = 0x9u;
    s->regs[R_STATUS] = 0x1au;
    s->regs[R_PHY_STATUS] = 0x1u;
    s->regs[R_CTRL_REGWEN] = 1u;
    for (int i = 0; i < 10; i++) {
        s->regs[(0x28 / 4) + i] = 1u;
        s->regs[(0x50 / 4) + i] = 0x99999u;
    }
    s->regs[0xa0 / 4] = 0x9999u;
    for (int i = 0; i < 8; i++) {
        s->regs[(0xa4 / 4) + i] = 1u;
        s->regs[(0xc4 / 4) + i] = 0x99999u;
    }
    s->regs[0xe4 / 4] = 0x99u;

    s->last_wr_err_addr = 0;
    s->last_rd_err_addr = 0;

    /* rram_macro_prim_reg_top.sv: CSR0_REGWEN=1 (rw0c), CSR1=0 (rw), CSR2=0
     * (w1c) */
    s->prim_regs[0] = 1u;
    s->prim_regs[1] = 0u;
    s->prim_regs[2] = 0u;

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
    if (addr >= 0x0cu || (addr & 0x3u) != 0u) {
        return MEMTX_ERROR;
    }
    *val = s->prim_regs[addr >> 2u];
    return MEMTX_OK;
}

static MemTxResult rram_ctrl_prim_write(void *opaque, hwaddr addr, uint64_t val,
                                        unsigned size, MemTxAttrs attrs)
{
    (void)attrs;
    OtRramCtrlState *s = opaque;
    uint32_t v32 = (uint32_t)val;
    if (addr >= 0x0cu || (addr & 0x3u) != 0u) {
        return MEMTX_ERROR;
    }
    /* RRAM_MACRO_PRIM_PERMIT = '{4'b0001, 4'b0011, 4'b0001} */
    if ((addr == 0x0u || addr == 0x8u) && size != 1u && size != 4u) {
        /* Subword byte mask > permit triggers reg_error if upper bytes touched;
         * standard 32-bit word accesses have size == 4 and are permitted. */
    }
    switch (addr) {
    case 0x0u: /* CSR0_REGWEN (rw0c, 1 bit) */
        if ((v32 & 1u) == 0u) {
            s->prim_regs[0] = 0u;
        }
        break;
    case 0x4u: /* CSR1 (rw, [12:0], gated by CSR0_REGWEN) */
        if (s->prim_regs[0] & 1u) {
            s->prim_regs[1] = v32 & 0x1fffu;
        }
        break;
    case 0x8u: /* CSR2: bits [7,3] (0x88) are rw; bits [6:4,2:0] (0x77) are w1c
                */
        s->prim_regs[2] =
            ((s->prim_regs[2] & ~0x88u) & ~(v32 & 0x77u)) | (v32 & 0x88u);
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
    (void)opaque;
    (void)n;
    (void)level;
}

static void ot_rram_ctrl_realize(DeviceState *dev, Error **errp)
{
    OtRramCtrlState *s = OT_RRAM_CTRL(dev);

    s->storage = g_malloc0(RRAM_HOST_SIZE);
    memset(s->storage, 0xff, RRAM_HOST_SIZE);
    memset(s->info_storage, 0xff, sizeof(s->info_storage));
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
                          TYPE_OT_RRAM_CTRL ".prim", 0x2000u);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->prim_iomem);

    for (int i = 0; i < 6; i++) {
        ibex_sysbus_init_irq(OBJECT(s), &s->irqs[i]);
    }
    for (int i = 0; i < 5; i++) {
        ibex_qdev_init_irq(OBJECT(dev), &s->alerts[i], OT_DEVICE_ALERT);
    }
    qdev_init_gpio_in_named(dev, &ot_rram_ctrl_lc_broadcast, OT_LC_BROADCAST,
                            8);
}

static void ot_rram_ctrl_init(Object *obj)
{
    OtRramCtrlState *s = OT_RRAM_CTRL(obj);

    ot_fifo32_create(&s->wr_fifo, 1024);
    ot_fifo32_create(&s->rd_fifo, 1024);
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
