/*
 * QEMU OpenTitan System Reset Controller (sysrst_ctrl) device
 *
 * Copyright (c) 2025 lowRISC contributors.
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

#ifndef HW_OPENTITAN_OT_SYSRST_CTRL_H
#define HW_OPENTITAN_OT_SYSRST_CTRL_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_OT_SYSRST_CTRL "ot-sysrst_ctrl"
OBJECT_DECLARE_SIMPLE_TYPE(OtSysrstCtrlState, OT_SYSRST_CTRL)

#define OT_SYSRST_CTRL_RST_REQ  TYPE_OT_SYSRST_CTRL "-rst-req"
#define OT_SYSRST_CTRL_WKUP_REQ TYPE_OT_SYSRST_CTRL "-wkup-req"
#define OT_SYSRST_CTRL_INPUT    TYPE_OT_SYSRST_CTRL "-input"

enum OtSysrstCtrlInput {
    OT_SYSRST_CTRL_IN_AC_PRESENT = 0,
    OT_SYSRST_CTRL_IN_KEY0       = 1,
    OT_SYSRST_CTRL_IN_KEY1       = 2,
    OT_SYSRST_CTRL_IN_KEY2       = 3,
    OT_SYSRST_CTRL_IN_PWRB       = 4,
    OT_SYSRST_CTRL_IN_LID_OPEN   = 5,
    OT_SYSRST_CTRL_IN_EC_RST_L   = 6,
    OT_SYSRST_CTRL_IN_FLASH_WP_L = 7,
    OT_SYSRST_CTRL_IN_COUNT      = 8,
};

int ot_sysrst_ctrl_get_outsel_level(const OtSysrstCtrlState *s,
                                    uint32_t outsel);

#endif /* HW_OPENTITAN_OT_SYSRST_CTRL_H */
