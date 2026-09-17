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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_gpio_eg.h"
#include "hw/opentitan/ot_rstmgr.h"
#include "hw/opentitan/ot_sysrst_ctrl.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"

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
REG32(PIN_ALLOWED_CTL, 0x34u)
REG32(PIN_OUT_CTL, 0x38u)
REG32(PIN_OUT_VALUE, 0x3cu)
REG32(PIN_IN_VALUE, 0x40u)
REG32(KEY_INTR_CTL, 0x44u)
REG32(KEY_INTR_DEBOUNCE_CTL, 0x48u)
REG32(AUTO_BLOCK_DEBOUNCE_CTL, 0x4cu)
REG32(AUTO_BLOCK_OUT_CTL, 0x50u)
REG32(COM_PRE_SEL_CTL_0, 0x54u)
REG32(COM_PRE_SEL_CTL_1, 0x58u)
REG32(COM_PRE_SEL_CTL_2, 0x5cu)
REG32(COM_PRE_SEL_CTL_3, 0x60u)
REG32(COM_PRE_DET_CTL_0, 0x64u)
REG32(COM_PRE_DET_CTL_1, 0x68u)
REG32(COM_PRE_DET_CTL_2, 0x6cu)
REG32(COM_PRE_DET_CTL_3, 0x70u)
REG32(COM_SEL_CTL_0, 0x74u)
REG32(COM_SEL_CTL_1, 0x78u)
REG32(COM_SEL_CTL_2, 0x7cu)
REG32(COM_SEL_CTL_3, 0x80u)
REG32(COM_DET_CTL_0, 0x84u)
REG32(COM_DET_CTL_1, 0x88u)
REG32(COM_DET_CTL_2, 0x8cu)
REG32(COM_DET_CTL_3, 0x90u)
REG32(COM_OUT_CTL_0, 0x94u)
    FIELD(COM_OUT_CTL_0, BAT_DISABLE, 0u, 1u)
    FIELD(COM_OUT_CTL_0, INTERRUPT, 1u, 1u)
    FIELD(COM_OUT_CTL_0, EC_RST, 2u, 1u)
    FIELD(COM_OUT_CTL_0, RST_REQ, 3u, 1u)
REG32(COM_OUT_CTL_1, 0x98u)
REG32(COM_OUT_CTL_2, 0x9cu)
REG32(COM_OUT_CTL_3, 0xa0u)
REG32(COMBO_INTR_STATUS, 0xa4u)
REG32(KEY_INTR_STATUS, 0xa8u)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))
#define R_LAST_REG   (R_KEY_INTR_STATUS)
#define REGS_COUNT   (R_LAST_REG + 1u)
#define REGS_SIZE    0x100u

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
    uint32_t last_read_pin_in;
    uint32_t poll_count;
    bool bat_disable_hw;
    bool ec_rst_l_hw;
    bool z3_wakeup_hw;
    QEMUTimer *ec_rst_timer;

    char *ot_id;
};

struct OtSysrstCtrlClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

static uint32_t ot_sysrst_ctrl_get_pin_outputs(const OtSysrstCtrlState *s)
{
    uint32_t raw = s->pin_in_raw;
    uint32_t inv = s->regs[R_KEY_INVERT_CTL];

    uint32_t pwrb_int = ((raw >> OT_SYSRST_CTRL_IN_PWRB) & 1u) ^ ((inv >> 6u) & 1u);
    uint32_t key0_int = ((raw >> OT_SYSRST_CTRL_IN_KEY0) & 1u) ^ ((inv >> 0u) & 1u);
    uint32_t key1_int = ((raw >> OT_SYSRST_CTRL_IN_KEY1) & 1u) ^ ((inv >> 2u) & 1u);
    uint32_t key2_int = ((raw >> OT_SYSRST_CTRL_IN_KEY2) & 1u) ^ ((inv >> 4u) & 1u);

    bool ab_en = (s->regs[R_AUTO_BLOCK_DEBOUNCE_CTL] >> 16u) & 1u;
    bool ab_cond = ab_en && (pwrb_int == 0u);
    uint32_t ab_ctl = s->regs[R_AUTO_BLOCK_OUT_CTL];

    uint32_t hw_in[8];
    hw_in[0] = s->bat_disable_hw ? 1u : 0u;
    hw_in[1] = s->ec_rst_l_hw ? 1u : 0u;
    hw_in[2] = pwrb_int;
    hw_in[3] = (ab_cond && ((ab_ctl >> 0u) & 1u)) ? ((ab_ctl >> 4u) & 1u) : key0_int;
    hw_in[4] = (ab_cond && ((ab_ctl >> 1u) & 1u)) ? ((ab_ctl >> 5u) & 1u) : key1_int;
    hw_in[5] = (ab_cond && ((ab_ctl >> 2u) & 1u)) ? ((ab_ctl >> 6u) & 1u) : key2_int;
    hw_in[6] = s->z3_wakeup_hw ? 1u : 0u;
    hw_in[7] = 0u; /* flash_wp_l has no passthrough input */

    uint32_t out_ctl = s->regs[R_PIN_OUT_CTL];
    uint32_t out_val = s->regs[R_PIN_OUT_VALUE];
    uint32_t allow_ctl = s->regs[R_PIN_ALLOWED_CTL];

    uint32_t out = 0;
    for (unsigned b = 0; b < 8u; b++) {
        bool en = (out_ctl >> b) & 1u;
        bool val = (out_val >> b) & 1u;
        bool allow0 = (allow_ctl >> b) & 1u;
        bool allow1 = (allow_ctl >> (b + 8u)) & 1u;
        uint32_t bit_out = (en && allow0 && !val) ? 0u :
                           (en && allow1 && val) ? 1u : hw_in[b];
        out |= (bit_out << b);
    }

    /* Apply output inversion per sysrst_ctrl.sv lines 303-311 */
    out ^= ((inv >> 9u) & 1u) << 0u;  /* bat_disable */
    out ^= ((inv >> 7u) & 1u) << 2u;  /* pwrb_out */
    out ^= ((inv >> 1u) & 1u) << 3u;  /* key0_out */
    out ^= ((inv >> 3u) & 1u) << 4u;  /* key1_out */
    out ^= ((inv >> 5u) & 1u) << 5u;  /* key2_out */
    out ^= ((inv >> 11u) & 1u) << 6u; /* z3_wakeup */

    return out;
}

int ot_sysrst_ctrl_get_outsel_level(const OtSysrstCtrlState *s, uint32_t outsel)
{
    if (!s) {
        return 0;
    }
    uint32_t out = ot_sysrst_ctrl_get_pin_outputs(s);
    switch (outsel) {
    case 72u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonBatDisable */
        return (int)((out >> 0u) & 1u);
    case 73u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonKey0Out */
        return (int)((out >> 3u) & 1u);
    case 74u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonKey1Out */
        return (int)((out >> 4u) & 1u);
    case 75u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonKey2Out */
        return (int)((out >> 5u) & 1u);
    case 76u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonPwrbOut */
        return (int)((out >> 2u) & 1u);
    case 77u: /* kTopEarlgreyPinmuxOutselSysrstCtrlAonZ3Wakeup */
        return (int)((out >> 6u) & 1u);
    case 100u: /* DIO ec_rst_l */
        return (int)((out >> 1u) & 1u);
    case 101u: /* DIO flash_wp_l */
        return (int)((out >> 7u) & 1u);
    default:
        return 0;
    }
}

static uint32_t ot_sysrst_ctrl_get_pin_in_value(const OtSysrstCtrlState *s)
{
    uint32_t raw = s->pin_in_raw;
    uint32_t out = ot_sysrst_ctrl_get_pin_outputs(s);
    uint32_t val = 0;

    val |= ((raw >> OT_SYSRST_CTRL_IN_PWRB) & 1u) << 0u;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY0) & 1u) << 1u;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY1) & 1u) << 2u;
    val |= ((raw >> OT_SYSRST_CTRL_IN_KEY2) & 1u) << 3u;
    val |= ((raw >> OT_SYSRST_CTRL_IN_LID_OPEN) & 1u) << 4u;
    val |= ((raw >> OT_SYSRST_CTRL_IN_AC_PRESENT) & 1u) << 5u;

    uint32_t ec_in = ((raw >> OT_SYSRST_CTRL_IN_EC_RST_L) & 1u) &
                     ((out >> 1u) & 1u);
    uint32_t wp_in = ((raw >> OT_SYSRST_CTRL_IN_FLASH_WP_L) & 1u) &
                     ((out >> 7u) & 1u);
    val |= (ec_in << 6u) | (wp_in << 7u);

    return val;
}

static void ot_sysrst_ctrl_get_inverted(const OtSysrstCtrlState *s,
                                        uint32_t pin_in_val,
                                        uint32_t *combo_in,
                                        uint32_t *key_intr_in)
{
    uint32_t inv = s->regs[R_KEY_INVERT_CTL];

    uint32_t pwrb = ((pin_in_val >> 0u) & 1u) ^ ((inv >> 6u) & 1u);
    uint32_t key0 = ((pin_in_val >> 1u) & 1u) ^ ((inv >> 0u) & 1u);
    uint32_t key1 = ((pin_in_val >> 2u) & 1u) ^ ((inv >> 2u) & 1u);
    uint32_t key2 = ((pin_in_val >> 3u) & 1u) ^ ((inv >> 4u) & 1u);
    uint32_t ac   = ((pin_in_val >> 5u) & 1u) ^ ((inv >> 8u) & 1u);
    uint32_t ec_rst_l   = (pin_in_val >> 6u) & 1u;
    uint32_t flash_wp_l = (pin_in_val >> 7u) & 1u;

    if (combo_in) {
        *combo_in = (key0 << 0u) | (key1 << 1u) | (key2 << 2u) |
                    (pwrb << 3u) | (ac << 4u);
    }
    if (key_intr_in) {
        *key_intr_in = (pwrb << 0u) | (key0 << 1u) | (key1 << 2u) |
                       (key2 << 3u) | (ac << 4u) | (ec_rst_l << 5u) |
                       (flash_wp_l << 6u);
    }
}

static void ot_sysrst_ctrl_update_outputs(OtSysrstCtrlState *s)
{
    bool intr_event_status = (s->regs[R_ULP_STATUS] |
                              s->regs[R_COMBO_INTR_STATUS] |
                              s->regs[R_KEY_INTR_STATUS]) != 0;
    bool test_q = (s->regs[R_INTR_TEST] & 1u) != 0;
    if (intr_event_status || test_q) {
        s->regs[R_INTR_STATE] |= R_INTR_STATE_EVENT_DETECTED_MASK;
    } else {
        s->regs[R_INTR_STATE] &= ~R_INTR_STATE_EVENT_DETECTED_MASK;
    }

    bool irq_level = (s->regs[R_INTR_STATE] & s->regs[R_INTR_ENABLE]) != 0;
    ibex_irq_set(&s->irq, (int)irq_level);

    bool wkup_level = (s->regs[R_WKUP_STATUS] & R_WKUP_STATUS_WAKEUP_STS_MASK) != 0;
    ibex_irq_set(&s->wkup_req, (int)wkup_level);

    ot_gpio_eg_notify_sysrst_change();
}

static void ot_sysrst_ctrl_ec_rst_timer_cb(void *opaque)
{
    OtSysrstCtrlState *s = opaque;
    s->ec_rst_l_hw = true;
    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_check_events(OtSysrstCtrlState *s)
{
    uint32_t prev_pin_val = s->last_pin_in_val;
    uint32_t curr_pin_val = ot_sysrst_ctrl_get_pin_in_value(s);
    s->last_pin_in_val = curr_pin_val;

    uint32_t prev_combo_in = 0;
    uint32_t curr_combo_in = 0;
    uint32_t prev_key_in = 0;
    uint32_t curr_key_in = 0;

    ot_sysrst_ctrl_get_inverted(s, prev_pin_val, &prev_combo_in, &prev_key_in);
    ot_sysrst_ctrl_get_inverted(s, curr_pin_val, &curr_combo_in, &curr_key_in);

    if (s->regs[R_ULP_CTL] & 1u) {
        uint32_t inv = s->regs[R_KEY_INVERT_CTL];
        uint32_t prev_pwrb = ((prev_pin_val >> 0u) & 1u) ^ ((inv >> 6u) & 1u);
        uint32_t curr_pwrb = ((curr_pin_val >> 0u) & 1u) ^ ((inv >> 6u) & 1u);
        uint32_t prev_lid = ((prev_pin_val >> 4u) & 1u) ^ ((inv >> 10u) & 1u);
        uint32_t curr_lid = ((curr_pin_val >> 4u) & 1u) ^ ((inv >> 10u) & 1u);
        uint32_t curr_ac = ((curr_pin_val >> 5u) & 1u) ^ ((inv >> 8u) & 1u);

        bool pwrb_h2l = (prev_pwrb == 1u && curr_pwrb == 0u);
        bool lid_l2h = (prev_lid == 0u && curr_lid == 1u);
        bool ac_high = (curr_ac == 1u);

        if (pwrb_h2l || lid_l2h || ac_high) {
            s->z3_wakeup_hw = true;
            s->regs[R_ULP_STATUS] |= 1u;
            s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
        }
    }

    if (prev_pin_val == curr_pin_val) {
        ot_sysrst_ctrl_update_outputs(s);
        return;
    }

    bool trigger_rst = false;
    for (unsigned k = 0; k < 4u; k++) {
        uint32_t sel = s->regs[R_COM_SEL_CTL_0 + k] & 0x1fu;
        if (!sel) {
            continue;
        }
        bool prev_trig = ((prev_combo_in & sel) == 0);
        bool curr_trig = ((curr_combo_in & sel) == 0);
        uint32_t out_ctl = s->regs[R_COM_OUT_CTL_0 + k];
        if (!prev_trig && curr_trig) {
            if (out_ctl & R_COM_OUT_CTL_0_BAT_DISABLE_MASK) {
                s->bat_disable_hw = true;
            }
            if (out_ctl & (R_COM_OUT_CTL_0_EC_RST_MASK |
                           R_COM_OUT_CTL_0_RST_REQ_MASK)) {
                s->ec_rst_l_hw = false;
                if (s->ec_rst_timer) {
                    timer_del(s->ec_rst_timer);
                }
            }
            if (out_ctl & R_COM_OUT_CTL_0_RST_REQ_MASK) {
                trigger_rst = true;
            }
            if (out_ctl & R_COM_OUT_CTL_0_INTERRUPT_MASK) {
                s->regs[R_COMBO_INTR_STATUS] |= (1u << k);
                s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
            }
        } else if (prev_trig && !curr_trig) {
            if (out_ctl & (R_COM_OUT_CTL_0_EC_RST_MASK |
                           R_COM_OUT_CTL_0_RST_REQ_MASK)) {
                uint32_t stretch = s->regs[R_EC_RST_CTL] & 0xffffu;
                if (stretch == 0u || !s->ec_rst_timer) {
                    s->ec_rst_l_hw = true;
                } else {
                    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
                    timer_mod_ns(s->ec_rst_timer,
                                 now + (int64_t)stretch * 4000LL);
                }
            }
        }
    }

    uint32_t h2l = prev_key_in & ~curr_key_in;
    uint32_t l2h = ~prev_key_in & curr_key_in;
    uint32_t key_events = (h2l & 0x7fu) | ((l2h & 0x7fu) << 7u);
    uint32_t active_events = key_events & (s->regs[R_KEY_INTR_CTL] & 0x3fffu);
    if (active_events) {
        s->regs[R_KEY_INTR_STATUS] |= active_events;
        s->regs[R_WKUP_STATUS] |= R_WKUP_STATUS_WAKEUP_STS_MASK;
    }

    if (trigger_rst) {
        ibex_irq_set(&s->rst_req, 1);
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

    /*
     * Per sysrst_ctrl_comboact.sv: ec_rst_l_i High -> Low transition
     * (ec_rst_l_det_pulse) asserts ec_rst_l_q = 0 and stretches it for
     * EC_RST_CTL cycles.
     */
    if (irq == OT_SYSRST_CTRL_IN_EC_RST_L && prev_ec_raw && !curr_ec_raw) {
        uint32_t stretch = s->regs[R_EC_RST_CTL] & 0xffffu;
        if (stretch > 0u && s->ec_rst_timer) {
            s->ec_rst_l_hw = false;
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
            timer_mod_ns(s->ec_rst_timer, now + (int64_t)stretch * 4000LL);
        }
    }

    ot_sysrst_ctrl_check_events(s);
}

static uint64_t ot_sysrst_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    OtSysrstCtrlState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%x\n", __func__,
                      (uint32_t)addr);
        return 0;
    }

    if (reg == R_INTR_TEST || reg == R_ALERT_TEST) {
        return 0;
    }

    if (reg == R_PIN_IN_VALUE) {
        uint32_t val32 = ot_sysrst_ctrl_get_pin_in_value(s);
        if (val32 == s->last_read_pin_in) {
            s->poll_count++;
            if (s->poll_count > 16u) {
                bql_unlock();
                g_usleep(50);
                bql_lock();
                val32 = ot_sysrst_ctrl_get_pin_in_value(s);
            }
        } else {
            s->poll_count = 0;
            s->last_read_pin_in = val32;
        }
        return val32;
    }

    return s->regs[reg];
}

static void ot_sysrst_ctrl_write(void *opaque, hwaddr addr, uint64_t val64,
                                 unsigned size)
{
    OtSysrstCtrlState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%x\n", __func__,
                      (uint32_t)addr);
        return;
    }

    s->poll_count = 0;

    switch (reg) {
    case R_INTR_STATE:
        /* INTR_STATE is RO (Status-type interrupt) */
        break;
    case R_INTR_ENABLE:
        s->regs[R_INTR_ENABLE] = val32 & 1u;
        ot_sysrst_ctrl_update_outputs(s);
        break;
    case R_INTR_TEST:
        s->regs[R_INTR_TEST] = val32 & 1u;
        ot_sysrst_ctrl_update_outputs(s);
        break;
    case R_ALERT_TEST:
        if (val32 & 1u) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case R_REGWEN:
        s->regs[R_REGWEN] &= (val32 & 1u);
        break;
    case R_EC_RST_CTL:
        if (s->regs[R_REGWEN] & 1u) {
            s->regs[reg] = val32 & 0xffffu;
            if (s->regs[reg] == 0u && !s->ec_rst_l_hw) {
                if (s->ec_rst_timer) {
                    timer_del(s->ec_rst_timer);
                }
                s->ec_rst_l_hw = true;
            }
            ot_sysrst_ctrl_check_events(s);
        }
        break;
    case R_ULP_CTL:
        s->regs[R_ULP_CTL] = val32 & 1u;
        if (!(val32 & 1u)) {
            s->z3_wakeup_hw = false;
        }
        ot_sysrst_ctrl_check_events(s);
        break;
    case R_ULP_STATUS:
    case R_COMBO_INTR_STATUS:
    case R_KEY_INTR_STATUS:
        s->regs[reg] &= ~val32;
        ot_sysrst_ctrl_update_outputs(s);
        break;
    case R_WKUP_STATUS:
        s->regs[R_WKUP_STATUS] &= ~(val32 & 1u);
        ot_sysrst_ctrl_update_outputs(s);
        break;
    case R_PIN_IN_VALUE:
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
};

static const Property ot_sysrst_ctrl_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtSysrstCtrlState, ot_id),
};

static void ot_sysrst_ctrl_reset_enter(Object *obj, ResetType type)
{
    OtSysrstCtrlClass *c = OT_SYSRST_CTRL_GET_CLASS(obj);
    OtSysrstCtrlState *s = OT_SYSRST_CTRL(obj);

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

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

    if (s->ec_rst_timer) {
        timer_del(s->ec_rst_timer);
    }

    s->bat_disable_hw = false;
    s->ec_rst_l_hw = true;
    s->z3_wakeup_hw = false;
    s->poll_count = 0;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_REGWEN] = 1u;
    s->regs[R_EC_RST_CTL] = 2000u;
    s->regs[R_ULP_AC_DEBOUNCE_CTL] = 8000u;
    s->regs[R_ULP_LID_DEBOUNCE_CTL] = 8000u;
    s->regs[R_ULP_PWRB_DEBOUNCE_CTL] = 8000u;
    s->regs[R_PIN_ALLOWED_CTL] = 0x82u;
    s->regs[R_PIN_OUT_CTL] = 0x82u;
    s->regs[R_KEY_INTR_DEBOUNCE_CTL] = 2000u;
    s->regs[R_AUTO_BLOCK_DEBOUNCE_CTL] = 2000u;

    s->last_pin_in_val = ot_sysrst_ctrl_get_pin_in_value(s);

    ibex_irq_set(&s->rst_req, 0);
    ibex_irq_set(&s->alert, 0);
    ot_sysrst_ctrl_update_outputs(s);
}

static void ot_sysrst_ctrl_init(Object *obj)
{
    OtSysrstCtrlState *s = OT_SYSRST_CTRL(obj);

    memory_region_init_io(&s->mmio, obj, &ot_sysrst_ctrl_ops, s,
                          TYPE_OT_SYSRST_CTRL, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    ibex_sysbus_init_irq(obj, &s->irq);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
    ibex_qdev_init_irq(obj, &s->rst_req, OT_SYSRST_CTRL_RST_REQ);
    ibex_qdev_init_irq(obj, &s->wkup_req, OT_SYSRST_CTRL_WKUP_REQ);

    s->pin_in_raw = (1u << OT_SYSRST_CTRL_IN_EC_RST_L) |
                    (1u << OT_SYSRST_CTRL_IN_FLASH_WP_L);
    s->ec_rst_l_hw = true;
    s->ec_rst_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                   &ot_sysrst_ctrl_ec_rst_timer_cb, s);

    qdev_init_gpio_in_named(DEVICE(obj), &ot_sysrst_ctrl_input_change,
                            OT_SYSRST_CTRL_INPUT, OT_SYSRST_CTRL_IN_COUNT);
}

static void ot_sysrst_ctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_props(dc, ot_sysrst_ctrl_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(dc);
    OtSysrstCtrlClass *sc = OT_SYSRST_CTRL_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_sysrst_ctrl_reset_enter, NULL,
                                       NULL, &sc->parent_phases);
}

static const TypeInfo ot_sysrst_ctrl_info = {
    .name = TYPE_OT_SYSRST_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtSysrstCtrlState),
    .instance_init = &ot_sysrst_ctrl_init,
    .class_init = &ot_sysrst_ctrl_class_init,
    .class_size = sizeof(OtSysrstCtrlClass),
};

static void ot_sysrst_ctrl_register_types(void)
{
    type_register_static(&ot_sysrst_ctrl_info);
}

type_init(ot_sysrst_ctrl_register_types);
