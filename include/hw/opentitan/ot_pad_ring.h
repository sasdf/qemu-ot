/*
 * QEMU OpenTitan Unified Pad Ring Device
 *
 * Copyright (c) 2025-2026 lowRISC contributors.
 *
 * Author(s):
 *  Emmanuel Blot <eblot@rivosinc.com>
 *  Alex Jones <alex.jones@lowrisc.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_OPENTITAN_OT_PAD_RING_H
#define HW_OPENTITAN_OT_PAD_RING_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_OT_PAD_RING "ot-pad-ring"
OBJECT_DECLARE_TYPE(OtPadRingState, OtPadRingClass, OT_PAD_RING)

/* Supported Pad Ring versions */
typedef enum OtPadRingVersion {
    OT_PAD_RING_VERSION_EG,
    OT_PAD_RING_VERSION_DJ,
    OT_PAD_RING_VERSION_COUNT,
} OtPadRingVersion;

/* Named qdev GPIO lines */
#define OT_PAD_RING_OUT  TYPE_OT_PAD_RING "-out"
#define OT_PAD_RING_IN   TYPE_OT_PAD_RING "-in"
#define OT_PAD_RING_ATTR TYPE_OT_PAD_RING "-attr"

/* Pad attribute bitmask flags for OT_PAD_RING_ATTR (matching hardware pad_attr_t) */
#define OT_PAD_ATTR_INVERT        BIT(0)
#define OT_PAD_ATTR_VIRTUAL_OD_EN BIT(1)
#define OT_PAD_ATTR_PULL_EN       BIT(2)
#define OT_PAD_ATTR_PULL_SELECT   BIT(3)
#define OT_PAD_ATTR_KEEPER_EN     BIT(4)
#define OT_PAD_ATTR_SCHMITT_EN    BIT(5)
#define OT_PAD_ATTR_OD_EN         BIT(6)
#define OT_PAD_ATTR_INPUT_DISABLE BIT(7)

/*
 * Type-safe Pad Index Enums for SoC GPIO wiring
 */
typedef enum OtEgPad {
    /* IOA0..IOA8 (0..8) */
    OT_EG_PAD_IOA0, OT_EG_PAD_IOA1, OT_EG_PAD_IOA2,
    OT_EG_PAD_IOA3, OT_EG_PAD_IOA4, OT_EG_PAD_IOA5,
    OT_EG_PAD_IOA6, OT_EG_PAD_IOA7, OT_EG_PAD_IOA8,
    /* IOB0..IOB12 (9..21) */
    OT_EG_PAD_IOB0, OT_EG_PAD_IOB1, OT_EG_PAD_IOB2, OT_EG_PAD_IOB3,
    OT_EG_PAD_IOB4, OT_EG_PAD_IOB5, OT_EG_PAD_IOB6, OT_EG_PAD_IOB7,
    OT_EG_PAD_IOB8, OT_EG_PAD_IOB9, OT_EG_PAD_IOB10, OT_EG_PAD_IOB11,
    OT_EG_PAD_IOB12,
    /* IOC0..IOC12 (22..34) */
    OT_EG_PAD_IOC0, OT_EG_PAD_IOC1, OT_EG_PAD_IOC2, OT_EG_PAD_IOC3,
    OT_EG_PAD_IOC4, OT_EG_PAD_IOC5, OT_EG_PAD_IOC6, OT_EG_PAD_IOC7,
    OT_EG_PAD_IOC8, OT_EG_PAD_IOC9, OT_EG_PAD_IOC10, OT_EG_PAD_IOC11,
    OT_EG_PAD_IOC12,
    /* IOR0..IOR13 (35..48) */
    OT_EG_PAD_IOR0, OT_EG_PAD_IOR1, OT_EG_PAD_IOR2, OT_EG_PAD_IOR3,
    OT_EG_PAD_IOR4, OT_EG_PAD_IOR5, OT_EG_PAD_IOR6, OT_EG_PAD_IOR7,
    OT_EG_PAD_IOR8, OT_EG_PAD_IOR9, OT_EG_PAD_IOR10, OT_EG_PAD_IOR11,
    OT_EG_PAD_IOR12, OT_EG_PAD_IOR13,
    /* System Pins */
    OT_EG_PAD_POR_N,
    OT_EG_PAD_COUNT,
} OtEgPad;

typedef enum OtDjPad {
    OT_DJ_PAD_GPIO0,  OT_DJ_PAD_GPIO1,  OT_DJ_PAD_GPIO2,  OT_DJ_PAD_GPIO3,
    OT_DJ_PAD_GPIO4,  OT_DJ_PAD_GPIO5,  OT_DJ_PAD_GPIO6,  OT_DJ_PAD_GPIO7,
    OT_DJ_PAD_GPIO8,  OT_DJ_PAD_GPIO9,  OT_DJ_PAD_GPIO10, OT_DJ_PAD_GPIO11,
    OT_DJ_PAD_GPIO12, OT_DJ_PAD_GPIO13, OT_DJ_PAD_GPIO14, OT_DJ_PAD_GPIO15,
    OT_DJ_PAD_GPIO16, OT_DJ_PAD_GPIO17, OT_DJ_PAD_GPIO18, OT_DJ_PAD_GPIO19,
    OT_DJ_PAD_GPIO20, OT_DJ_PAD_GPIO21, OT_DJ_PAD_GPIO22, OT_DJ_PAD_GPIO23,
    OT_DJ_PAD_GPIO24, OT_DJ_PAD_GPIO25, OT_DJ_PAD_GPIO26, OT_DJ_PAD_GPIO27,
    OT_DJ_PAD_GPIO28, OT_DJ_PAD_GPIO29, OT_DJ_PAD_GPIO30, OT_DJ_PAD_GPIO31,
    OT_DJ_PAD_POR_N,
    OT_DJ_PAD_COUNT,
} OtDjPad;

#endif /* HW_OPENTITAN_OT_PAD_RING_H */
