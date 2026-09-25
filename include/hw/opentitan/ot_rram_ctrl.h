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

struct OtRramCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion core_iomem;
    MemoryRegion host_iomem;
    MemoryRegion oob_iomem;
    MemoryRegion otp_iomem;
    MemoryRegion prim_iomem;

    char *ot_id;
    BlockBackend *blk;
    uint8_t *storage;
    uint8_t info_storage[8 * 512];

    OtFifo32 wr_fifo;
    OtFifo32 rd_fifo;

    IbexIRQ irqs[6];
    IbexIRQ alerts[5];

    uint32_t last_wr_err_addr;
    uint32_t last_rd_err_addr;

    uint32_t regs[0x200 / 4];
    uint32_t prim_regs[3];
};

#endif /* HW_OPENTITAN_OT_RRAM_CTRL_H */
