/*
 * QEMU OpenTitan RRAM Controller
 *
 * Copyright lowRISC contributors.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HW_OPENTITAN_OT_RRAM_CTRL_H
#define HW_OPENTITAN_OT_RRAM_CTRL_H

#include "qemu/osdep.h"
#include "qom/object.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_fifo32.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "system/block-backend.h"

#define TYPE_OT_RRAM_CTRL "ot-rram_ctrl"
OBJECT_DECLARE_SIMPLE_TYPE(OtRramCtrlState, OT_RRAM_CTRL)

#define OT_RRAM_SEED_COUNT      2u
#define OT_RRAM_SEED_BYTES      32u
#define OT_RRAM_INFO_PAGE_COUNT 8u
#define OT_RRAM_INFO_PAGE_BYTES 512u
#define OT_RRAM_INFO_STORAGE_BYTES \
    (OT_RRAM_INFO_PAGE_COUNT * OT_RRAM_INFO_PAGE_BYTES)
#define OT_RRAM_IRQ_COUNT       6u
#define OT_RRAM_ALERT_COUNT     5u
#define OT_RRAM_CORE_SIZE       0x200u
#define OT_RRAM_CORE_REGS_COUNT (OT_RRAM_CORE_SIZE / sizeof(uint32_t))
#define OT_RRAM_PRIM_REGS_COUNT 3u

struct OtRramCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion core_iomem;
    MemoryRegion host_iomem;
    MemoryRegion otp_iomem;
    MemoryRegion prim_iomem;

    char *ot_id;
    BlockBackend *blk;
    uint8_t *storage;
    uint8_t info_storage[OT_RRAM_INFO_STORAGE_BYTES];
    uint8_t seeds[OT_RRAM_SEED_COUNT][OT_RRAM_SEED_BYTES];
    bool lcmgr_init_done;

    OtFifo32 wr_fifo;
    OtFifo32 rd_fifo;

    IbexIRQ irqs[OT_RRAM_IRQ_COUNT];
    IbexIRQ alerts[OT_RRAM_ALERT_COUNT];

    uint32_t last_wr_err_addr;
    uint32_t last_rd_err_addr;

    uint32_t regs[OT_RRAM_CORE_REGS_COUNT];
    uint32_t prim_regs[OT_RRAM_PRIM_REGS_COUNT];
};

void ot_rram_ctrl_get_seed(const OtRramCtrlState *s, unsigned idx,
                           uint8_t *seed);

#endif /* HW_OPENTITAN_OT_RRAM_CTRL_H */
