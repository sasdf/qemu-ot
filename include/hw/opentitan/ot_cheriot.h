/*
 * QEMU OpenTitan CHERIoT Memory Subsystem
 *
 * Copyright lowRISC contributors.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HW_OPENTITAN_OT_CHERIOT_H
#define HW_OPENTITAN_OT_CHERIOT_H

#include "qom/object.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"

#define TYPE_OT_CHERIOT "ot-cheriot"
OBJECT_DECLARE_SIMPLE_TYPE(OtCheriotState, OT_CHERIOT)

#define OT_CHERIOT_ENA "cheriot-ena"

struct OtCheriotState {
    SysBusDevice parent_obj;

    MemoryRegion regs;
    MemoryRegion revbm;
    uint8_t *revbm_data;

    IbexIRQ alert;

    char *ot_id;
    uint32_t cheriot_ena;
    uint32_t revbm_size;
};

#endif
