/*
 * QEMU OpenTitan System Reset Controller (sysrst_ctrl) device
 *
 * Copyright lowRISC contributors.
 *
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_gpio_eg.h"
#include "hw/opentitan/ot_pinmux_eg.h"
#include "hw/opentitan/ot_rstmgr.h"
#include "hw/opentitan/ot_sysrst_ctrl.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "system/replay.h"

#define SYSRST_NUM_COMBOS           4u
#define SYSRST_AON_PERIOD_NS        4000LL
#define SYSRST_ULP_STEP_NS          5000LL
#define SYSRST_COMBO_RST_DELAY_NS   3000000LL
#define SYSRST_DIO_PAD_EC_RST_L     10u
#define SYSRST_DIO_PAD_FLASH_WP_L   11u
#define SYSRST_KEY_INTR_INPUT_COUNT 7u
#define SYSRST_KEY_INTR_EDGE_MASK   0x7fu
#define SYSRST_KEY_INTR_ALL_MASK    0x3fffu
#define SYSRST_COMBO_SEL_MASK       0x1fu
#define SYSRST_DEBOUNCE_MASK        0xffffu
#define SYSRST_PIN_ALLOW1_SHIFT     8u
#define SYSRST_POLL_YIELD_THRESHOLD 16u
#define SYSRST_POLL_YIELD_US        50
#define SYSRST_WORD_ALIGN_MASK      3u
#define SYSRST_WORD_BYTES           4u
#define SYSRST_MMIO_SIZE            0x100u

#define SYSRST_PERMIT_1B 0x1u
#define SYSRST_PERMIT_2B 0x3u
#define SYSRST_PERMIT_3B 0x7u
#define SYSRST_PERMIT_4B 0xfu

#define SYSRST_REG_MASK_32B            0xffffffffu
#define SYSRST_REG_MASK_AUTO_BLOCK_DEB 0x0001ffffu
#define SYSRST_REG_MASK_16B            0x0000ffffu
#define SYSRST_REG_MASK_KEY_INTR       0x00003fffu
#define SYSRST_REG_MASK_KEY_INVERT     0x00000fffu
#define SYSRST_REG_MASK_PIN_OUT        0x000000ffu
#define SYSRST_REG_MASK_AUTO_BLOCK_OUT 0x00000077u
#define SYSRST_REG_MASK_COMBO_SEL      0x0000001fu
#define SYSRST_REG_MASK_COMBO_OUT      0x0000000fu
#define SYSRST_REG_MASK_1B             0x00000001u

#define SYSRST_DEBOUNCE_DEFAULT_RESVAL 2000u
#define SYSRST_ULP_DEBOUNCE_RESVAL     8000u
#define SYSRST_PIN_OUT_DEFAULT_RESVAL  0x82u

enum OtSysrstCtrlPinOut {
    SYSRST_PIN_OUT_BAT_DISABLE = 0,
    SYSRST_PIN_OUT_EC_RST_L = 1,
    SYSRST_PIN_OUT_PWRB = 2,
    SYSRST_PIN_OUT_KEY0 = 3,
    SYSRST_PIN_OUT_KEY1 = 4,
    SYSRST_PIN_OUT_KEY2 = 5,
    SYSRST_PIN_OUT_Z3_WAKEUP = 6,
    SYSRST_PIN_OUT_FLASH_WP_L = 7,
    SYSRST_PIN_OUT_COUNT = 8,
};

enum OtSysrstCtrlPinInBit {
    SYSRST_PIN_IN_PWRB = 0,
    SYSRST_PIN_IN_KEY0 = 1,
    SYSRST_PIN_IN_KEY1 = 2,
    SYSRST_PIN_IN_KEY2 = 3,
    SYSRST_PIN_IN_LID_OPEN = 4,
    SYSRST_PIN_IN_AC_PRESENT = 5,
    SYSRST_PIN_IN_EC_RST_L = 6,
    SYSRST_PIN_IN_FLASH_WP_L = 7,
};

enum OtSysrstCtrlComboBit {
    SYSRST_COMBO_IN_KEY0 = 0,
    SYSRST_COMBO_IN_KEY1 = 1,
    SYSRST_COMBO_IN_KEY2 = 2,
    SYSRST_COMBO_IN_PWRB = 3,
    SYSRST_COMBO_IN_AC_PRESENT = 4,
};

enum OtSysrstCtrlKeyIntrBit {
    SYSRST_KEY_INTR_IN_PWRB = 0,
    SYSRST_KEY_INTR_IN_KEY0 = 1,
    SYSRST_KEY_INTR_IN_KEY1 = 2,
    SYSRST_KEY_INTR_IN_KEY2 = 3,
    SYSRST_KEY_INTR_IN_AC_PRESENT = 4,
    SYSRST_KEY_INTR_IN_EC_RST_L = 5,
    SYSRST_KEY_INTR_IN_FLASH_WP_L = 6,
};

enum OtSysrstCtrlOutsel {
    SYSRST_OUTSEL_BAT_DISABLE = 72u,
    SYSRST_OUTSEL_KEY0_OUT = 73u,
    SYSRST_OUTSEL_KEY1_OUT = 74u,
    SYSRST_OUTSEL_KEY2_OUT = 75u,
    SYSRST_OUTSEL_PWRB_OUT = 76u,
    SYSRST_OUTSEL_Z3_WAKEUP = 77u,
    SYSRST_OUTSEL_DIO_EC_RST_L = 100u,
    SYSRST_OUTSEL_DIO_FLASH_WP_L = 101u,
};

/* clang-format off */
REG32(INTR_STATE, 0x0u)
    FIELD(INTR_STATE, EVENT_DETECTED, 0u, 1u)
REG32(INTR_ENABLE, 0x4u)
REG32(INTR_TEST, 0x8u)
REG32(ALERT_TEST, 0xcu)
    FIELD(ALERT_TEST, FATAL_FAULT, 0u, 1u)
REG32(REGWEN, 0x10u)
    FIELD(REGWEN, EN, 0u, 1u)
REG32(EC_RST_CTL, 0x14u)
REG32(ULP_AC_DEBOUNCE_CTL, 0x18u)
REG32(ULP_LID_DEBOUNCE_CTL, 0x1cu)
REG32(ULP_PWRB_DEBOUNCE_CTL, 0x20u)
REG32(ULP_CTL, 0x24u)
REG32(ULP_STATUS, 0x28u)
REG32(WKUP_STATUS, 0x2cu)
    FIELD(WKUP_STATUS, WAKEUP_STS, 0u, 1u)
REG32(KEY_INVERT_CTL, 0x30u)
    FIELD(KEY_INVERT_CTL, KEY0_IN, 0u, 1u)
    FIELD(KEY_INVERT_CTL, KEY0_OUT, 1u, 1u)
    FIELD(KEY_INVERT_CTL, KEY1_IN, 2u, 1u)
    FIELD(KEY_INVERT_CTL, KEY1_OUT, 3u, 1u)
    FIELD(KEY_INVERT_CTL, KEY2_IN, 4u, 1u)
    FIELD(KEY_INVERT_CTL, KEY2_OUT, 5u, 1u)
    FIELD(KEY_INVERT_CTL, PWRB_IN, 6u, 1u)
    FIELD(KEY_INVERT_CTL, PWRB_OUT, 7u, 1u)
    FIELD(KEY_INVERT_CTL, AC_PRESENT, 8u, 1u)
    FIELD(KEY_INVERT_CTL, BAT_DISABLE, 9u, 1u)
    FIELD(KEY_INVERT_CTL, LID_OPEN, 10u, 1u)
    FIELD(KEY_INVERT_CTL, Z3_WAKEUP, 11u, 1u)
REG32(PIN_ALLOWED_CTL, 0x34u)
REG32(PIN_OUT_CTL, 0x38u)
REG32(PIN_OUT_VALUE, 0x3cu)
REG32(PIN_IN_VALUE, 0x40u)
REG32(KEY_INTR_CTL, 0x44u)
REG32(KEY_INTR_DEBOUNCE_CTL, 0x48u)
REG32(AUTO_BLOCK_DEBOUNCE_CTL, 0x4cu)
    FIELD(AUTO_BLOCK_DEBOUNCE_CTL, DEBOUNCE_TIMER, 0u, 16u)
    FIELD(AUTO_BLOCK_DEBOUNCE_CTL, AUTO_BLOCK_ENABLE, 16u, 1u)
REG32(AUTO_BLOCK_OUT_CTL, 0x50u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY0_OUT_SEL, 0u, 1u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY1_OUT_SEL, 1u, 1u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY2_OUT_SEL, 2u, 1u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY0_OUT_VALUE, 4u, 1u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY1_OUT_VALUE, 5u, 1u)
    FIELD(AUTO_BLOCK_OUT_CTL, KEY2_OUT_VALUE, 6u, 1u)
REG32(COM_PRE_SEL_CTL_0, 0x54u)
REG32(COM_PRE_DET_CTL_0, 0x64u)
REG32(COM_SEL_CTL_0, 0x74u)
REG32(COM_DET_CTL_0, 0x84u)
REG32(COM_OUT_CTL_0, 0x94u)
    FIELD(COM_OUT_CTL_0, BAT_DISABLE, 0u, 1u)
    FIELD(COM_OUT_CTL_0, INTERRUPT, 1u, 1u)
    FIELD(COM_OUT_CTL_0, EC_RST, 2u, 1u)
    FIELD(COM_OUT_CTL_0, RST_REQ, 3u, 1u)
REG32(COMBO_INTR_STATUS, 0xa4u)
REG32(KEY_INTR_STATUS, 0xa8u)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))
#define R_LAST_REG   (R_KEY_INTR_STATUS)
#define REGS_COUNT   (R_LAST_REG + 1u)
#define REGS_SIZE    (REGS_COUNT * sizeof(uint32_t))

struct OtSysrstCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    IbexIRQ irq;
    IbexIRQ alert;
    IbexIRQ rst_req;
    IbexIRQ wkup_req;

    uint32_t regs[REGS_COUNT];
    uint32_t pin_in_raw;
    uint32_t last_pin_in_val;
    uint32_t last_key_invert_ctl;
    uint32_t last_read_pin_in;
    uint32_t poll_count;
    bool combo_precond_valid[SYSRST_NUM_COMBOS];
    bool ulp_ac_latched;
    bool bat_disable_hw;
    bool ec_rst_l_hw;
    bool z3_wakeup_hw;
    QEMUTimer *ec_rst_timer;
    QEMUTimer *combo_rst_timer;
    QEMUTimer *ulp_timer;

    char *ot_id;
};

static uint32_t ot_sysrst_ctrl_get_pin_outputs(const OtSysrstCtrlState *s)
{
    uint32_t raw = s->pin_in_raw;
    uint32_t inv = s->regs[R_KEY_INVERT_CTL];

    uint32_t pwrb_int = ((raw >> OT_SYSRST_CTRL_IN_PWRB) & 1u) ^
                        FIELD_EX32(inv, KEY_INVERT_CTL, PWRB_IN);
    uint32_t key0_int = ((raw >> OT_SYSRST_CTRL_IN_KEY0) & 1u) ^
                        FIELD_EX32(inv, KEY_INVERT_CTL, KEY0_IN);
    uint32_t key1_int = ((raw >> OT_SYSRST_CTRL_IN_KEY1) & 1u) ^
                        FIELD_EX32(inv, KEY_INVERT_CTL, KEY1_IN);
    uint32_t key2_int = ((raw >> OT_SYSRST_CTRL_IN_KEY2) & 1u) ^
                        FIELD_EX32(inv, KEY_INVERT_CTL, KEY2_IN);

    bool ab_en = (bool)FIELD_EX32(s->regs[R_AUTO_BLOCK_DEBOUNCE_CTL],
                                  AUTO_BLOCK_DEBOUNCE_CTL, AUTO_BLOCK_ENABLE);
    bool ab_cond = ab_en && (pwrb_int == 0u);
    uint32_t ab_ctl = s->regs[R_AUTO_BLOCK_OUT_CTL];

    uint32_t hw_in[SYSRST_PIN_OUT_COUNT];
    hw_in[SYSRST_PIN_OUT_BAT_DISABLE] = s->bat_disable_hw ? 1u : 0u;
    hw_in[SYSRST_PIN_OUT_EC_RST_L] = s->ec_rst_l_hw ? 1u : 0u;
    hw_in[SYSRST_PIN_OUT_PWRB] = pwrb_int;
    hw_in[SYSRST_PIN_OUT_KEY0] = key0_int;
    if (ab_cond && FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY0_OUT_SEL)) {
        hw_in[SYSRST_PIN_OUT_KEY0] =
            FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY0_OUT_VALUE);
    }
    hw_in[SYSRST_PIN_OUT_KEY1] = key1_int;
    if (ab_cond && FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY1_OUT_SEL)) {
        hw_in[SYSRST_PIN_OUT_KEY1] =
            FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY1_OUT_VALUE);
    }
    hw_in[SYSRST_PIN_OUT_KEY2] = key2_int;
    if (ab_cond && FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY2_OUT_SEL)) {
        hw_in[SYSRST_PIN_OUT_KEY2] =
            FIELD_EX32(ab_ctl, AUTO_BLOCK_OUT_CTL, KEY2_OUT_VALUE);
    }
    hw_in[SYSRST_PIN_OUT_Z3_WAKEUP] = s->z3_wakeup_hw ? 1u : 0u;
    hw_in[SYSRST_PIN_OUT_FLASH_WP_L] = 0u;

    uint32_t out_ctl = s->regs[R_PIN_OUT_CTL];
    uint32_t out_val = s->regs[R_PIN_OUT_VALUE];
    uint32_t allow_ctl = s->regs[R_PIN_ALLOWED_CTL];

    uint32_t out = 0;
    for (unsigned b = 0; b < SYSRST_PIN_OUT_COUNT; b++) {
        bool en = (out_ctl >> b) & 1u;
        bool val = (out_val >> b) & 1u;
        bool allow0 = (allow_ctl >> b) & 1u;
        bool allow1 = (allow_ctl >> (b + SYSRST_PIN_ALLOW1_SHIFT)) & 1u;
        uint32_t bit_out = hw_in[b];
        if (en && allow0 && !val) {
            bit_out = 0u;
        } else if (en && allow1 && val) {
            bit_out = 1u;
        }
        out |= (bit_out << b);
    }

    /* Apply output inversion per sysrst_ctrl.sv lines 303-311 */
    out ^= FIELD_EX32(inv, KEY_INVERT_CTL, BAT_DISABLE)
           << SYSRST_PIN_OUT_BAT_DISABLE;
    out ^= FIELD_EX32(inv, KEY_INVERT_CTL, PWRB_OUT) << SYSRST_PIN_OUT_PWRB;
    out ^= FIELD_EX32(inv, KEY_INVERT_CTL, KEY0_OUT) << SYSRST_PIN_OUT_KEY0;
    out ^= FIELD_EX32(inv, KEY_INVERT_CTL, KEY1_OUT) << SYSRST_PIN_OUT_KEY1;
    out ^= FIELD_EX32(inv, KEY_INVERT_CTL, KEY2_OUT) << SYSRST_PIN_OUT_KEY2;
    out ^=
        FIELD_EX32(inv, KEY_INVERT_CTL, Z3_WAKEUP) << SYSRST_PIN_OUT_Z3_WAKEUP;

    return out;
}

int ot_sysrst_ctrl_get_outsel_level(const OtSysrstCtrlState *s, uint32_t outsel)
{
    if (!s) {
        return 0;
    }
    uint32_t out = ot_sysrst_ctrl_get_pin_outputs(s);
    switch (outsel) {
    case SYSRST_OUTSEL_BAT_DISABLE:
        return (int)((out >> SYSRST_PIN_OUT_BAT_DISABLE) & 1u);
    case SYSRST_OUTSEL_KEY0_OUT:
        return (int)((out >> SYSRST_PIN_OUT_KEY0) & 1u);
    case SYSRST_OUTSEL_KEY1_OUT:
        return (int)((out >> SYSRST_PIN_OUT_KEY1) & 1u);
    case SYSRST_OUTSEL_KEY2_OUT:
        return (int)((out >> SYSRST_PIN_OUT_KEY2) & 1u);
    case SYSRST_OUTSEL_PWRB_OUT:
        return (int)((out >> SYSRST_PIN_OUT_PWRB) & 1u);
    case SYSRST_OUTSEL_Z3_WAKEUP:
        return (int)((out >> SYSRST_PIN_OUT_Z3_WAKEUP) & 1u);
    case SYSRST_OUTSEL_DIO_EC_RST_L:
        return (int)((out >> SYSRST_PIN_OUT_EC_RST_L) & 1u);
    case SYSRST_OUTSEL_DIO_FLASH_WP_L:
        return (int)((out >> SYSRST_PIN_OUT_FLASH_WP_L) & 1u);
    default:
        return 0;
    }
}

static uint32_t ot_sysrst_ctrl_get_pin_in_value(const OtSysrstCtrlState *s)
{
    uint32_t raw = s->pin_in_raw;
    uint32_t out = ot_sysrst_ctrl_get_pin_outputs(s);
    uint32_t val = 0;

    val |= ((raw >> OT_SYSRST_CTRL_IN_PWRB) & 1u) << SYSRST_PIN_IN_PWRB;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY0) & 1u) << SYSRST_PIN_IN_KEY0;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY1) & 1u) << SYSRST_PIN_IN_KEY1;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY2) & 1u) << SYSRST_PIN_IN_KEY2;
    val |= ((raw >> OT_SYSRST_CTRL_IN_LID_OPEN) & 1u) << SYSRST_PIN_IN_LID_OPEN;
    val |= ((raw >> OT_SYSRST_CTRL_IN_AC_PRESENT) & 1u)
           << SYSRST_PIN_IN_AC_PRESENT;

    uint32_t ec_wire = ((raw >> OT_SYSRST_CTRL_IN_EC_RST_L) &
                        (out >> SYSRST_PIN_OUT_EC_RST_L)) &
                       1u;
    uint32_t wp_wire = ((raw >> OT_SYSRST_CTRL_IN_FLASH_WP_L) &
                        (out >> SYSRST_PIN_OUT_FLASH_WP_L)) &
                       1u;

    uint32_t ec_attr = ot_pinmux_eg_get_dio_pad_attr(SYSRST_DIO_PAD_EC_RST_L);
    uint32_t wp_attr = ot_pinmux_eg_get_dio_pad_attr(SYSRST_DIO_PAD_FLASH_WP_L);

    uint32_t ec_raw =
        (ec_attr & OT_PINMUX_PAD_ATTR_INPUT_DISABLE_MASK) ? 0u : ec_wire;
    uint32_t wp_raw =
        (wp_attr & OT_PINMUX_PAD_ATTR_INPUT_DISABLE_MASK) ? 0u : wp_wire;

    uint32_t ec_in = ec_raw ^ (ec_attr & OT_PINMUX_PAD_ATTR_INVERT_MASK);
    uint32_t wp_in = wp_raw ^ (wp_attr & OT_PINMUX_PAD_ATTR_INVERT_MASK);

    val |=
        (ec_in << SYSRST_PIN_IN_EC_RST_L) | (wp_in << SYSRST_PIN_IN_FLASH_WP_L);

    return val;
}

static void ot_sysrst_ctrl_get_inverted(uint32_t pin_in_val, uint32_t inv,
                                        uint32_t *combo_in,
                                        uint32_t *key_intr_in)
{
    uint32_t pwrb = ((pin_in_val >> SYSRST_PIN_IN_PWRB) & 1u) ^
                    FIELD_EX32(inv, KEY_INVERT_CTL, PWRB_IN);
    uint32_t key0 = ((pin_in_val >> SYSRST_PIN_IN_KEY0) & 1u) ^
                    FIELD_EX32(inv, KEY_INVERT_CTL, KEY0_IN);
    uint32_t key1 = ((pin_in_val >> SYSRST_PIN_IN_KEY1) & 1u) ^
                    FIELD_EX32(inv, KEY_INVERT_CTL, KEY1_IN);
    uint32_t key2 = ((pin_in_val >> SYSRST_PIN_IN_KEY2) & 1u) ^
                    FIELD_EX32(inv, KEY_INVERT_CTL, KEY2_IN);
    uint32_t ac = ((pin_in_val >> SYSRST_PIN_IN_AC_PRESENT) & 1u) ^
                  FIELD_EX32(inv, KEY_INVERT_CTL, AC_PRESENT);
    uint32_t ec_rst_l = (pin_in_val >> SYSRST_PIN_IN_EC_RST_L) & 1u;
    uint32_t flash_wp_l = (pin_in_val >> SYSRST_PIN_IN_FLASH_WP_L) & 1u;

    if (combo_in) {
        *combo_in =
            (key0 << SYSRST_COMBO_IN_KEY0) | (key1 << SYSRST_COMBO_IN_KEY1) |
            (key2 << SYSRST_COMBO_IN_KEY2) | (pwrb << SYSRST_COMBO_IN_PWRB) |
            (ac << SYSRST_COMBO_IN_AC_PRESENT);
    }
    if (key_intr_in) {
        *key_intr_in =
            (pwrb << SYSRST_KEY_INTR_IN_PWRB) |
            (key0 << SYSRST_KEY_INTR_IN_KEY0) |
            (key1 << SYSRST_KEY_INTR_IN_KEY1) |
            (key2 << SYSRST_KEY_INTR_IN_KEY2) |
            (ac << SYSRST_KEY_INTR_IN_AC_PRESENT) |
            (ec_rst_l << SYSRST_KEY_INTR_IN_EC_RST_L) |
            (flash_wp_l << SYSRST_KEY_INTR_IN_FLASH_WP_L);
    }
}

static void ot_sysrst_ctrl_combo_rst_timer_cb(void *opaque)
{
    OtSysrstCtrlState *s = opaque;
    ibex_irq_set(&s->rst_req, 1);
}

static void ot_sysrst_ctrl_update_outputs(OtSysrstCtrlState *s)
{
    bool intr_event_status =
        (s->regs[R_ULP_STATUS] | s->regs[R_COMBO_INTR_STATUS] |
         s->regs[R_KEY_INTR_STATUS]) != 0;
    bool test_q = (s->regs[R_INTR_TEST] & 1u) != 0;
    if (intr_event_status || test_q) {
        s->regs[R_INTR_STATE] |= R_INTR_STATE_EVENT_DETECTED_MASK;
    } else {
        s->regs[R_INTR_STATE] &= ~R_INTR_STATE_EVENT_DETECTED_MASK;
    }

    bool irq_level = (s->regs[R_INTR_STATE] & s->regs[R_INTR_ENABLE]) != 0;
    ibex_irq_set(&s->irq, (int)irq_level);

    bool wkup_level =
        (s->regs[R_WKUP_STATUS] & R_WKUP_STATUS_WAKEUP_STS_MASK) != 0;
    ibex_irq_set(&s->wkup_req, (int)wkup_level);

    ot_gpio_eg_notify_sysrst_change();
}

static void ot_sysrst_ctrl_ec_rst_timer_cb(void *opaque)
{
    OtSysrstCtrlState *s = opaque;
    s->ec_rst_l_hw = true;
    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_ulp_timer_cb(void *opaque)
{
    OtSysrstCtrlState *s = opaque;
    if (!(s->regs[R_ULP_CTL] & 1u) || s->ulp_ac_latched) {
        return;
    }
    s->ulp_ac_latched = true;
    s->z3_wakeup_hw = true;
    s->regs[R_ULP_STATUS] |= 1u;
    s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_check_events(OtSysrstCtrlState *s)
{
    uint32_t prev_pin_val = s->last_pin_in_val;
    uint32_t prev_inv = s->last_key_invert_ctl;
    uint32_t curr_pin_val = ot_sysrst_ctrl_get_pin_in_value(s);
    uint32_t curr_inv = s->regs[R_KEY_INVERT_CTL];
    s->last_pin_in_val = curr_pin_val;
    s->last_key_invert_ctl = curr_inv;

    uint32_t prev_combo_in = 0;
    uint32_t curr_combo_in = 0;
    uint32_t prev_key_in = 0;
    uint32_t curr_key_in = 0;

    ot_sysrst_ctrl_get_inverted(prev_pin_val, prev_inv, &prev_combo_in,
                                &prev_key_in);
    ot_sysrst_ctrl_get_inverted(curr_pin_val, curr_inv, &curr_combo_in,
                                &curr_key_in);

    if (s->regs[R_ULP_CTL] & 1u) {
        uint32_t prev_pwrb = ((prev_pin_val >> SYSRST_PIN_IN_PWRB) & 1u) ^
                             FIELD_EX32(prev_inv, KEY_INVERT_CTL, PWRB_IN);
        uint32_t curr_pwrb = ((curr_pin_val >> SYSRST_PIN_IN_PWRB) & 1u) ^
                             FIELD_EX32(curr_inv, KEY_INVERT_CTL, PWRB_IN);
        uint32_t prev_lid = ((prev_pin_val >> SYSRST_PIN_IN_LID_OPEN) & 1u) ^
                            FIELD_EX32(prev_inv, KEY_INVERT_CTL, LID_OPEN);
        uint32_t curr_lid = ((curr_pin_val >> SYSRST_PIN_IN_LID_OPEN) & 1u) ^
                            FIELD_EX32(curr_inv, KEY_INVERT_CTL, LID_OPEN);
        uint32_t curr_ac = ((curr_pin_val >> SYSRST_PIN_IN_AC_PRESENT) & 1u) ^
                           FIELD_EX32(curr_inv, KEY_INVERT_CTL, AC_PRESENT);

        bool pwrb_h2l = (prev_pwrb == 1u && curr_pwrb == 0u);
        bool lid_l2h = (prev_lid == 0u && curr_lid == 1u);
        bool ac_high = (curr_ac == 1u);

        uint32_t debounce =
            s->regs[R_ULP_AC_DEBOUNCE_CTL] & SYSRST_DEBOUNCE_MASK;
        if (pwrb_h2l || lid_l2h ||
            (ac_high && !s->ulp_ac_latched && debounce == 0u)) {
            s->ulp_ac_latched = true;
            s->z3_wakeup_hw = true;
            s->regs[R_ULP_STATUS] |= 1u;
            s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
        } else if (ac_high) {
            if (!s->ulp_ac_latched && !timer_pending(s->ulp_timer)) {
                timer_mod_ns(s->ulp_timer,
                             qemu_clock_get_ns(OT_VIRTUAL_CLOCK) +
                                 (int64_t)(debounce + 1u) * SYSRST_ULP_STEP_NS);
            }
        } else if (!s->ulp_ac_latched) {
            timer_del(s->ulp_timer);
        }
    }

    if (prev_combo_in == curr_combo_in && prev_key_in == curr_key_in) {
        ot_sysrst_ctrl_update_outputs(s);
        return;
    }

    bool trigger_rst = false;
    uint32_t rst_det_cyc = 0;
    for (unsigned k = 0; k < SYSRST_NUM_COMBOS; k++) {
        uint32_t pre_sel =
            s->regs[R_COM_PRE_SEL_CTL_0 + k] & SYSRST_COMBO_SEL_MASK;
        uint32_t sel = s->regs[R_COM_SEL_CTL_0 + k] & SYSRST_COMBO_SEL_MASK;
        if (!sel) {
            s->combo_precond_valid[k] = false;
            continue;
        }
        bool prev_pre_valid = (pre_sel == 0u) || s->combo_precond_valid[k];
        if (pre_sel != 0u) {
            if ((curr_combo_in & pre_sel) != 0u) {
                s->combo_precond_valid[k] = false;
            } else if ((prev_combo_in & sel) != 0u) {
                s->combo_precond_valid[k] = true;
            }
        } else {
            s->combo_precond_valid[k] = true;
        }
        bool curr_pre_valid = s->combo_precond_valid[k];

        bool prev_trig = prev_pre_valid && ((prev_combo_in & sel) == 0u);
        bool curr_trig = curr_pre_valid && ((curr_combo_in & sel) == 0u);
        uint32_t out_ctl = s->regs[R_COM_OUT_CTL_0 + k];
        if (!prev_trig && curr_trig) {
            if (out_ctl & R_COM_OUT_CTL_0_BAT_DISABLE_MASK) {
                s->bat_disable_hw = true;
            }
            if (out_ctl &
                (R_COM_OUT_CTL_0_EC_RST_MASK | R_COM_OUT_CTL_0_RST_REQ_MASK)) {
                s->ec_rst_l_hw = false;
                timer_del(s->ec_rst_timer);
            }
            if (out_ctl & R_COM_OUT_CTL_0_RST_REQ_MASK) {
                trigger_rst = true;
                rst_det_cyc = s->regs[R_COM_DET_CTL_0 + k];
            }
            if (out_ctl & R_COM_OUT_CTL_0_INTERRUPT_MASK) {
                s->regs[R_COMBO_INTR_STATUS] |= (1u << k);
                s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
            }
        } else if (prev_trig && !curr_trig) {
            if (out_ctl & R_COM_OUT_CTL_0_RST_REQ_MASK) {
                timer_del(s->combo_rst_timer);
            }
            if (out_ctl &
                (R_COM_OUT_CTL_0_EC_RST_MASK | R_COM_OUT_CTL_0_RST_REQ_MASK)) {
                uint32_t stretch = s->regs[R_EC_RST_CTL] & SYSRST_DEBOUNCE_MASK;
                if (stretch == 0u) {
                    s->ec_rst_l_hw = true;
                } else {
                    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
                    timer_mod_ns(s->ec_rst_timer,
                                 now + (int64_t)stretch * SYSRST_AON_PERIOD_NS);
                }
            }
        }
    }

    uint32_t h2l = prev_key_in & ~curr_key_in;
    uint32_t l2h = ~prev_key_in & curr_key_in;
    uint32_t key_events =
        (h2l & SYSRST_KEY_INTR_EDGE_MASK) |
        ((l2h & SYSRST_KEY_INTR_EDGE_MASK) << SYSRST_KEY_INTR_INPUT_COUNT);
    uint32_t active_events =
        key_events & (s->regs[R_KEY_INTR_CTL] & SYSRST_KEY_INTR_ALL_MASK);
    if (active_events) {
        s->regs[R_KEY_INTR_STATUS] |= active_events;
        s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
    }

    if (trigger_rst) {
        int64_t delay_ns = (rst_det_cyc > 0u) ? SYSRST_COMBO_RST_DELAY_NS : 0LL;
        timer_mod_ns(s->combo_rst_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + delay_ns);
    }

    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_input_change(void *opaque, int irq, int level)
{
    OtSysrstCtrlState *s = opaque;

    g_assert((unsigned)irq < OT_SYSRST_CTRL_IN_COUNT);

    bool prev_ec_raw = (s->pin_in_raw >> OT_SYSRST_CTRL_IN_EC_RST_L) & 1u;
    bool curr_ec_raw = level != 0;

    if (level) {
        s->pin_in_raw |= (1u << irq);
    } else {
        s->pin_in_raw &= ~(1u << irq);
    }
    s->poll_count = 0;

    if (timer_pending(s->combo_rst_timer)) {
        timer_mod_ns(s->combo_rst_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                         SYSRST_COMBO_RST_DELAY_NS);
    }

    /*
     * Per sysrst_ctrl_comboact.sv: ec_rst_l_i High -> Low transition
     * (ec_rst_l_det_pulse) asserts ec_rst_l_q = 0 and stretches it for
     * EC_RST_CTL cycles.
     */
    if (irq == OT_SYSRST_CTRL_IN_EC_RST_L && prev_ec_raw && !curr_ec_raw) {
        uint32_t stretch = s->regs[R_EC_RST_CTL] & SYSRST_DEBOUNCE_MASK;
        if (stretch > 0u) {
            s->ec_rst_l_hw = false;
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
            timer_mod_ns(s->ec_rst_timer,
                         now + (int64_t)stretch * SYSRST_AON_PERIOD_NS);
        }
    }

    ot_sysrst_ctrl_check_events(s);
}

static uint32_t ot_sysrst_ctrl_reg_mask(hwaddr reg)
{
    switch (reg) {
    case R_COM_PRE_DET_CTL_0 ...(R_COM_PRE_DET_CTL_0 + SYSRST_NUM_COMBOS - 1u):
    case R_COM_DET_CTL_0 ...(R_COM_DET_CTL_0 + SYSRST_NUM_COMBOS - 1u):
        return SYSRST_REG_MASK_32B;
    case R_AUTO_BLOCK_DEBOUNCE_CTL:
        return SYSRST_REG_MASK_AUTO_BLOCK_DEB;
    case R_EC_RST_CTL:
    case R_ULP_AC_DEBOUNCE_CTL:
    case R_ULP_LID_DEBOUNCE_CTL:
    case R_ULP_PWRB_DEBOUNCE_CTL:
    case R_PIN_ALLOWED_CTL:
    case R_KEY_INTR_DEBOUNCE_CTL:
        return SYSRST_REG_MASK_16B;
    case R_KEY_INTR_CTL:
    case R_KEY_INTR_STATUS:
        return SYSRST_REG_MASK_KEY_INTR;
    case R_KEY_INVERT_CTL:
        return SYSRST_REG_MASK_KEY_INVERT;
    case R_PIN_OUT_CTL:
    case R_PIN_OUT_VALUE:
        return SYSRST_REG_MASK_PIN_OUT;
    case R_AUTO_BLOCK_OUT_CTL:
        return SYSRST_REG_MASK_AUTO_BLOCK_OUT;
    case R_COM_PRE_SEL_CTL_0 ...(R_COM_PRE_SEL_CTL_0 + SYSRST_NUM_COMBOS - 1u):
    case R_COM_SEL_CTL_0 ...(R_COM_SEL_CTL_0 + SYSRST_NUM_COMBOS - 1u):
        return SYSRST_REG_MASK_COMBO_SEL;
    case R_COM_OUT_CTL_0 ...(R_COM_OUT_CTL_0 + SYSRST_NUM_COMBOS - 1u):
    case R_COMBO_INTR_STATUS:
        return SYSRST_REG_MASK_COMBO_OUT;
    default:
        return SYSRST_REG_MASK_1B;
    }
}

static bool ot_sysrst_ctrl_accepts(void *opaque, hwaddr addr, unsigned size,
                                   bool is_write, MemTxAttrs attrs)
{
    (void)opaque;
    (void)attrs;
    hwaddr reg = R32_OFF(addr);
    if (reg >= REGS_COUNT ||
        (addr & SYSRST_WORD_ALIGN_MASK) + size > SYSRST_WORD_BYTES) {
        return false;
    }
    if (!is_write) {
        return true;
    }
    uint32_t mask = ot_sysrst_ctrl_reg_mask(reg);
    uint8_t permit = SYSRST_PERMIT_1B;
    if (mask > SYSRST_REG_MASK_AUTO_BLOCK_DEB) {
        permit = SYSRST_PERMIT_4B;
    } else if (mask > SYSRST_REG_MASK_16B) {
        permit = SYSRST_PERMIT_3B;
    } else if (mask > SYSRST_REG_MASK_PIN_OUT) {
        permit = SYSRST_PERMIT_2B;
    }
    uint8_t reg_be =
        (uint8_t)(((1u << size) - 1u) << (addr & SYSRST_WORD_ALIGN_MASK));
    return (permit & ~reg_be) == 0u;
}

static uint64_t ot_sysrst_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    OtSysrstCtrlState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);

    if (reg == R_PIN_IN_VALUE) {
        uint32_t val32 = ot_sysrst_ctrl_get_pin_in_value(s);
        if (val32 == s->last_read_pin_in) {
            s->poll_count++;
            if (s->poll_count > SYSRST_POLL_YIELD_THRESHOLD) {
                bql_unlock();
                replay_mutex_unlock();
                g_usleep(SYSRST_POLL_YIELD_US);
                replay_mutex_lock();
                bql_lock();
                val32 = ot_sysrst_ctrl_get_pin_in_value(s);
            }
        } else {
            s->poll_count = 0;
            s->last_read_pin_in = val32;
        }
        return val32;
    }
    return (reg != R_INTR_TEST && reg != R_ALERT_TEST) ? s->regs[reg] : 0;
}

static void ot_sysrst_ctrl_write(void *opaque, hwaddr addr, uint64_t val64,
                                 unsigned size)
{
    OtSysrstCtrlState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);
    uint32_t val32 = (uint32_t)val64 & ot_sysrst_ctrl_reg_mask(reg);
    s->poll_count = 0;

    switch (reg) {
    case R_INTR_STATE:
    case R_PIN_IN_VALUE:
        break;
    case R_INTR_ENABLE:
    case R_INTR_TEST:
        s->regs[reg] = val32;
        ot_sysrst_ctrl_update_outputs(s);
        break;
    case R_ALERT_TEST:
        if (val32) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case R_REGWEN:
        s->regs[R_REGWEN] &= val32;
        break;
    case R_EC_RST_CTL:
        if (s->regs[R_REGWEN] & 1u) {
            s->regs[reg] = val32;
            if (val32 == 0u && !s->ec_rst_l_hw) {
                timer_del(s->ec_rst_timer);
                s->ec_rst_l_hw = true;
            }
            ot_sysrst_ctrl_check_events(s);
        }
        break;
    case R_ULP_CTL:
        s->regs[R_ULP_CTL] = val32;
        if (!val32) {
            s->z3_wakeup_hw = false;
            s->ulp_ac_latched = false;
            timer_del(s->ulp_timer);
        }
        ot_sysrst_ctrl_check_events(s);
        break;
    case R_PIN_OUT_CTL:
    case R_PIN_OUT_VALUE:
        s->regs[reg] = val32;
        ot_sysrst_ctrl_check_events(s);
        break;
    case R_ULP_STATUS:
    case R_WKUP_STATUS:
    case R_COMBO_INTR_STATUS:
    case R_KEY_INTR_STATUS:
        s->regs[reg] &= ~val32;
        ot_sysrst_ctrl_update_outputs(s);
        break;
    default:
        if (s->regs[R_REGWEN] & 1u) {
            s->regs[reg] = val32;
            ot_sysrst_ctrl_check_events(s);
        }
        break;
    }
}

static const MemoryRegionOps ot_sysrst_ctrl_ops = {
    .read = &ot_sysrst_ctrl_read,
    .write = &ot_sysrst_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
    .valid.min_access_size = 1u,
    .valid.max_access_size = 4u,
    .valid.accepts = &ot_sysrst_ctrl_accepts,
};

static const Property ot_sysrst_ctrl_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtSysrstCtrlState, ot_id),
};

static void ot_sysrst_ctrl_reset_enter(Object *obj, ResetType type)
{
    OtSysrstCtrlState *s = OT_SYSRST_CTRL(obj);

    if (type != RESET_TYPE_COLD && ot_rstmgr_is_low_power_exit()) {
        /*
         * SYSRST_CTRL_AON resides in DomainAonSel. A low-power wakeup from deep
         * sleep only resets Domain0Sel, preserving all AON registers and
         * hardware outputs (z3_wakeup, ulp_status, pin_out_ctl, etc.).
         */
        ibex_irq_set(&s->rst_req, 0);
        ibex_irq_set(&s->alert, 0);
        ot_sysrst_ctrl_update_outputs(s);
        return;
    }

    timer_del(s->ec_rst_timer);
    timer_del(s->combo_rst_timer);
    timer_del(s->ulp_timer);

    s->bat_disable_hw = false;
    s->ec_rst_l_hw = true;
    s->z3_wakeup_hw = false;
    s->ulp_ac_latched = false;
    memset(s->combo_precond_valid, 0, sizeof(s->combo_precond_valid));
    s->poll_count = 0;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_REGWEN] = 1u;
    s->regs[R_EC_RST_CTL] = SYSRST_DEBOUNCE_DEFAULT_RESVAL;
    s->regs[R_ULP_AC_DEBOUNCE_CTL] = SYSRST_ULP_DEBOUNCE_RESVAL;
    s->regs[R_ULP_LID_DEBOUNCE_CTL] = SYSRST_ULP_DEBOUNCE_RESVAL;
    s->regs[R_ULP_PWRB_DEBOUNCE_CTL] = SYSRST_ULP_DEBOUNCE_RESVAL;
    s->regs[R_PIN_ALLOWED_CTL] = SYSRST_PIN_OUT_DEFAULT_RESVAL;
    s->regs[R_PIN_OUT_CTL] = SYSRST_PIN_OUT_DEFAULT_RESVAL;
    s->regs[R_KEY_INTR_DEBOUNCE_CTL] = SYSRST_DEBOUNCE_DEFAULT_RESVAL;
    s->regs[R_AUTO_BLOCK_DEBOUNCE_CTL] = SYSRST_DEBOUNCE_DEFAULT_RESVAL;

    s->last_pin_in_val = ot_sysrst_ctrl_get_pin_in_value(s);
    s->last_key_invert_ctl = 0u;

    ibex_irq_set(&s->rst_req, 0);
    ibex_irq_set(&s->alert, 0);
    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_init(Object *obj)
{
    OtSysrstCtrlState *s = OT_SYSRST_CTRL(obj);

    memory_region_init_io(&s->mmio, obj, &ot_sysrst_ctrl_ops, s,
                          TYPE_OT_SYSRST_CTRL, SYSRST_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    ibex_sysbus_init_irq(obj, &s->irq);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
    ibex_qdev_init_irq(obj, &s->rst_req, OT_SYSRST_CTRL_RST_REQ);
    ibex_qdev_init_irq(obj, &s->wkup_req, OT_SYSRST_CTRL_WKUP_REQ);

    s->pin_in_raw = (1u << OT_SYSRST_CTRL_IN_EC_RST_L) |
                    (1u << OT_SYSRST_CTRL_IN_FLASH_WP_L);
    s->ec_rst_l_hw = true;
    s->ec_rst_timer =
        timer_new_ns(QEMU_CLOCK_REALTIME, &ot_sysrst_ctrl_ec_rst_timer_cb, s);
    s->combo_rst_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                      &ot_sysrst_ctrl_combo_rst_timer_cb, s);
    s->ulp_timer =
        timer_new_ns(OT_VIRTUAL_CLOCK, &ot_sysrst_ctrl_ulp_timer_cb, s);

    qdev_init_gpio_in_named(DEVICE(obj), &ot_sysrst_ctrl_input_change,
                            OT_SYSRST_CTRL_INPUT, OT_SYSRST_CTRL_IN_COUNT);
}

static void ot_sysrst_ctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_props(dc, ot_sysrst_ctrl_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    RESETTABLE_CLASS(dc)->phases.enter = &ot_sysrst_ctrl_reset_enter;
}

static const TypeInfo ot_sysrst_ctrl_info = {
    .name = TYPE_OT_SYSRST_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtSysrstCtrlState),
    .instance_init = &ot_sysrst_ctrl_init,
    .class_init = &ot_sysrst_ctrl_class_init,
};

static void ot_sysrst_ctrl_register_types(void)
{
    type_register_static(&ot_sysrst_ctrl_info);
}

type_init(ot_sysrst_ctrl_register_types);
