/*
 * QEMU OpenTitan PWM device
 *
 * Copyright lowRISC contributors.
 *
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HW_OPENTITAN_OT_PWM_H
#define HW_OPENTITAN_OT_PWM_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_OT_PWM "ot-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(OtPwmState, OT_PWM)

#define OT_PWM_OUTPUT TYPE_OT_PWM "-output"

#endif /* HW_OPENTITAN_OT_PWM_H */
