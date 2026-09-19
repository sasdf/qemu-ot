/*
 * QEMU OpenTitan ADC Controller device
 *
 * Copyright (c) 2025 lowRISC contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/opentitan/ot_adc_ctrl.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_rstmgr.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "trace.h"

#define NUM_ADC_FILTERS  8u
#define NUM_ADC_CHANNELS 2u

/* clang-format off */
REG32(INTR_STATE, 0x00u)
    FIELD(INTR_STATE, MATCH_PENDING, 0u, 1u)
REG32(INTR_ENABLE, 0x04u)
    FIELD(INTR_ENABLE, MATCH_PENDING, 0u, 1u)
REG32(INTR_TEST, 0x08u)
    FIELD(INTR_TEST, MATCH_PENDING, 0u, 1u)
REG32(ALERT_TEST, 0x0cu)
    FIELD(ALERT_TEST, FATAL_FAULT, 0u, 1u)
REG32(ADC_EN_CTL, 0x10u)
    FIELD(ADC_EN_CTL, ADC_ENABLE, 0u, 1u)
    FIELD(ADC_EN_CTL, ONESHOT_MODE, 1u, 1u)
REG32(ADC_PD_CTL, 0x14u)
    FIELD(ADC_PD_CTL, LP_MODE, 0u, 1u)
    FIELD(ADC_PD_CTL, PWRUP_TIME, 4u, 4u)
    FIELD(ADC_PD_CTL, WAKEUP_TIME, 8u, 24u)
REG32(ADC_LP_SAMPLE_CTL, 0x18u)
    FIELD(ADC_LP_SAMPLE_CTL, LP_SAMPLE_CNT, 0u, 8u)
REG32(ADC_SAMPLE_CTL, 0x1cu)
    FIELD(ADC_SAMPLE_CTL, NP_SAMPLE_CNT, 0u, 16u)
REG32(ADC_FSM_RST, 0x20u)
    FIELD(ADC_FSM_RST, RST_EN, 0u, 1u)
REG32(ADC_CHN0_FILTER_CTL_0, 0x24u)
REG32(ADC_CHN0_FILTER_CTL_1, 0x28u)
REG32(ADC_CHN0_FILTER_CTL_2, 0x2cu)
REG32(ADC_CHN0_FILTER_CTL_3, 0x30u)
REG32(ADC_CHN0_FILTER_CTL_4, 0x34u)
REG32(ADC_CHN0_FILTER_CTL_5, 0x38u)
REG32(ADC_CHN0_FILTER_CTL_6, 0x3cu)
REG32(ADC_CHN0_FILTER_CTL_7, 0x40u)
REG32(ADC_CHN1_FILTER_CTL_0, 0x44u)
REG32(ADC_CHN1_FILTER_CTL_1, 0x48u)
REG32(ADC_CHN1_FILTER_CTL_2, 0x4cu)
REG32(ADC_CHN1_FILTER_CTL_3, 0x50u)
REG32(ADC_CHN1_FILTER_CTL_4, 0x54u)
REG32(ADC_CHN1_FILTER_CTL_5, 0x58u)
REG32(ADC_CHN1_FILTER_CTL_6, 0x5cu)
REG32(ADC_CHN1_FILTER_CTL_7, 0x60u)
REG32(ADC_CHN_VAL_0, 0x64u)
REG32(ADC_CHN_VAL_1, 0x68u)
REG32(ADC_WAKEUP_CTL, 0x6cu)
REG32(FILTER_STATUS, 0x70u)
REG32(ADC_INTR_CTL, 0x74u)
REG32(ADC_INTR_STATUS, 0x78u)
REG32(ADC_FSM_STATE, 0x7cu)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))

#define R_LAST_REG (R_ADC_FSM_STATE)
#define REGS_COUNT (R_LAST_REG + 1u)
#define REGS_SIZE  (REGS_COUNT * sizeof(uint32_t))
#define REG_NAME(_reg_) \
    ((((_reg_) < REGS_COUNT) && REG_NAMES[_reg_]) ? REG_NAMES[_reg_] : "?")

#define REG_NAME_ENTRY(_reg_) [R_##_reg_] = stringify(_reg_)
static const char *REG_NAMES[REGS_COUNT] = {
    REG_NAME_ENTRY(INTR_STATE),
    REG_NAME_ENTRY(INTR_ENABLE),
    REG_NAME_ENTRY(INTR_TEST),
    REG_NAME_ENTRY(ALERT_TEST),
    REG_NAME_ENTRY(ADC_EN_CTL),
    REG_NAME_ENTRY(ADC_PD_CTL),
    REG_NAME_ENTRY(ADC_LP_SAMPLE_CTL),
    REG_NAME_ENTRY(ADC_SAMPLE_CTL),
    REG_NAME_ENTRY(ADC_FSM_RST),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_0),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_1),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_2),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_3),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_4),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_5),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_6),
    REG_NAME_ENTRY(ADC_CHN0_FILTER_CTL_7),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_0),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_1),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_2),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_3),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_4),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_5),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_6),
    REG_NAME_ENTRY(ADC_CHN1_FILTER_CTL_7),
    REG_NAME_ENTRY(ADC_CHN_VAL_0),
    REG_NAME_ENTRY(ADC_CHN_VAL_1),
    REG_NAME_ENTRY(ADC_WAKEUP_CTL),
    REG_NAME_ENTRY(FILTER_STATUS),
    REG_NAME_ENTRY(ADC_INTR_CTL),
    REG_NAME_ENTRY(ADC_INTR_STATUS),
    REG_NAME_ENTRY(ADC_FSM_STATE),
};
#undef REG_NAME_ENTRY

#define FILTER_CTL_MIN_V_SHIFT 2u
#define FILTER_CTL_MIN_V_MASK  0x3ffu
#define FILTER_CTL_COND_MASK   (1u << 12u)
#define FILTER_CTL_MAX_V_SHIFT 18u
#define FILTER_CTL_MAX_V_MASK  0x3ffu
#define FILTER_CTL_EN_MASK     (1u << 31u)
#define FILTER_CTL_REG_MASK    0x8ffc1ffcu
#define ADC_PD_CTL_MASK        0xfffffff1u
#define FSM_STATE_NP_0         12u

#define CHN_VAL_VALUE_SHIFT      2u
#define CHN_VAL_INTR_VALUE_SHIFT 18u
#define CHN_VAL_MASK             0x3ffu

#define FILTER_STATUS_TRANS_BIT  8u
#define ADC_INTR_ONESHOT_BIT     9u
#define WAKEUP_CTL_MASK          0x1ffu
#define FILTER_STATUS_MASK       0x1ffu
#define ADC_INTR_MASK            0x3ffu

struct OtAdcCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    IbexIRQ irq;
    IbexIRQ alert;
    IbexIRQ wkup;

    uint32_t *regs;
    bool trigger_q;
};

struct OtAdcCtrlClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

static void ot_adc_ctrl_update_irqs(OtAdcCtrlState *s)
{
    /*
     * In RTL (adc_ctrl_intr.sv), intr_state is a Status-type interrupt driven
     * by status_irq_value = |adc_intr_status.
     */
    if (s->regs[R_ADC_INTR_STATUS] != 0u || s->regs[R_INTR_TEST] != 0u) {
        s->regs[R_INTR_STATE] |= R_INTR_STATE_MATCH_PENDING_MASK;
    } else {
        s->regs[R_INTR_STATE] &= ~R_INTR_STATE_MATCH_PENDING_MASK;
    }

    bool irq_level = (s->regs[R_INTR_STATE] & s->regs[R_INTR_ENABLE]) != 0u;
    ibex_irq_set(&s->irq, irq_level);

    /*
     * In RTL (adc_ctrl_core.sv):
     * wkup_req_o = |(filter_status.match & adc_wakeup_ctl.match_en) ||
     *              (filter_status.trans & adc_wakeup_ctl.trans_en)
     */
    bool wkup_level =
        (s->regs[R_FILTER_STATUS] & s->regs[R_ADC_WAKEUP_CTL] & WAKEUP_CTL_MASK) != 0u;
    ibex_irq_set(&s->wkup, wkup_level);
}

#define DEFAULT_CHN0_VAL 0x155u
#define DEFAULT_CHN1_VAL 0x2aau

static bool ot_adc_ctrl_filter_match_chn(uint32_t filter_ctl, uint32_t chn_val)
{
    uint32_t min_v =
        (filter_ctl >> FILTER_CTL_MIN_V_SHIFT) & FILTER_CTL_MIN_V_MASK;
    uint32_t max_v =
        (filter_ctl >> FILTER_CTL_MAX_V_SHIFT) & FILTER_CTL_MAX_V_MASK;
    bool out_of_range = (filter_ctl & FILTER_CTL_COND_MASK) != 0u;

    if (!out_of_range) {
        return (min_v <= chn_val) && (chn_val <= max_v);
    }
    return (min_v > chn_val) || (chn_val > max_v);
}

static bool ot_adc_ctrl_pick_sample_chn(uint32_t filter_ctl, uint32_t def_val,
                                        uint32_t *out_val)
{
    if (ot_adc_ctrl_filter_match_chn(filter_ctl, def_val)) {
        *out_val = def_val;
        return true;
    }

    uint32_t min_v =
        (filter_ctl >> FILTER_CTL_MIN_V_SHIFT) & FILTER_CTL_MIN_V_MASK;
    uint32_t max_v =
        (filter_ctl >> FILTER_CTL_MAX_V_SHIFT) & FILTER_CTL_MAX_V_MASK;
    bool out_of_range = (filter_ctl & FILTER_CTL_COND_MASK) != 0u;

    if (!out_of_range) {
        if (min_v <= max_v) {
            *out_val = (min_v + max_v) / 2u;
            return true;
        }
        return false;
    }

    if (min_v > 0u) {
        *out_val = min_v - 1u;
        return true;
    }
    if (max_v < FILTER_CTL_MAX_V_MASK) {
        *out_val = max_v + 1u;
        return true;
    }
    return false;
}

static void ot_adc_ctrl_update_chn_val(OtAdcCtrlState *s, uint32_t chn0_val,
                                       uint32_t chn1_val, bool intr_we)
{
    chn0_val &= CHN_VAL_MASK;
    chn1_val &= CHN_VAL_MASK;

    uint32_t intr0 = s->regs[R_ADC_CHN_VAL_0] & (CHN_VAL_MASK << CHN_VAL_INTR_VALUE_SHIFT);
    uint32_t intr1 = s->regs[R_ADC_CHN_VAL_1] & (CHN_VAL_MASK << CHN_VAL_INTR_VALUE_SHIFT);
    if (intr_we) {
        intr0 = chn0_val << CHN_VAL_INTR_VALUE_SHIFT;
        intr1 = chn1_val << CHN_VAL_INTR_VALUE_SHIFT;
    }

    s->regs[R_ADC_CHN_VAL_0] = (chn0_val << CHN_VAL_VALUE_SHIFT) | intr0;
    s->regs[R_ADC_CHN_VAL_1] = (chn1_val << CHN_VAL_VALUE_SHIFT) | intr1;
}

void ot_adc_ctrl_trigger_wakeup(OtAdcCtrlState *s)
{
    if (!(s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ADC_ENABLE_MASK) ||
        (s->regs[R_ADC_FSM_RST] & R_ADC_FSM_RST_RST_EN_MASK)) {
        return;
    }

    uint32_t chn0_val = DEFAULT_CHN0_VAL;
    uint32_t chn1_val = DEFAULT_CHN1_VAL;

    /*
     * Select a single coherent (chn0_val, chn1_val) pair for the conversion,
     * preferring the FPGA AST default (0x155, 0x2aa) if it matches at least
     * one enabled filter, or the first satisfiable enabled filter otherwise.
     */
    bool default_matches_any = false;
    for (unsigned k = 0; k < NUM_ADC_FILTERS; k++) {
        uint32_t f0 = s->regs[R_ADC_CHN0_FILTER_CTL_0 + k];
        uint32_t f1 = s->regs[R_ADC_CHN1_FILTER_CTL_0 + k];
        bool en0 = (f0 & FILTER_CTL_EN_MASK) != 0u;
        bool en1 = (f1 & FILTER_CTL_EN_MASK) != 0u;
        if ((en0 || en1) &&
            (!en0 || ot_adc_ctrl_filter_match_chn(f0, DEFAULT_CHN0_VAL)) &&
            (!en1 || ot_adc_ctrl_filter_match_chn(f1, DEFAULT_CHN1_VAL))) {
            default_matches_any = true;
            break;
        }
    }

    if (!default_matches_any) {
        for (unsigned k = 0; k < NUM_ADC_FILTERS; k++) {
            uint32_t f0 = s->regs[R_ADC_CHN0_FILTER_CTL_0 + k];
            uint32_t f1 = s->regs[R_ADC_CHN1_FILTER_CTL_0 + k];
            bool en0 = (f0 & FILTER_CTL_EN_MASK) != 0u;
            bool en1 = (f1 & FILTER_CTL_EN_MASK) != 0u;
            if (!en0 && !en1) {
                continue;
            }
            uint32_t cand0 = DEFAULT_CHN0_VAL;
            uint32_t cand1 = DEFAULT_CHN1_VAL;
            bool ok0 = !en0 || ot_adc_ctrl_pick_sample_chn(f0, DEFAULT_CHN0_VAL, &cand0);
            bool ok1 = !en1 || ot_adc_ctrl_pick_sample_chn(f1, DEFAULT_CHN1_VAL, &cand1);
            if (ok0 && ok1) {
                chn0_val = cand0;
                chn1_val = cand1;
                break;
            }
        }
    }

    bool any_match = false;
    for (unsigned k = 0; k < NUM_ADC_FILTERS; k++) {
        uint32_t f0 = s->regs[R_ADC_CHN0_FILTER_CTL_0 + k];
        uint32_t f1 = s->regs[R_ADC_CHN1_FILTER_CTL_0 + k];
        bool en0 = (f0 & FILTER_CTL_EN_MASK) != 0u;
        bool en1 = (f1 & FILTER_CTL_EN_MASK) != 0u;

        bool match_k = (en0 || en1) &&
                       (!en0 || ot_adc_ctrl_filter_match_chn(f0, chn0_val)) &&
                       (!en1 || ot_adc_ctrl_filter_match_chn(f1, chn1_val));
        if (!match_k) {
            continue;
        }

        s->regs[R_FILTER_STATUS] |= (1u << k);
        if (s->regs[R_ADC_INTR_CTL] & (1u << k)) {
            s->regs[R_ADC_INTR_STATUS] |= (1u << k);
        }
        any_match = true;
    }

    ot_adc_ctrl_update_chn_val(s, chn0_val, chn1_val, any_match);

    if (any_match && (s->regs[R_ADC_PD_CTL] & R_ADC_PD_CTL_LP_MODE_MASK)) {
        /* Low power to normal power FSM transition occurred */
        s->regs[R_FILTER_STATUS] |= (1u << FILTER_STATUS_TRANS_BIT);
        if (s->regs[R_ADC_INTR_CTL] & (1u << FILTER_STATUS_TRANS_BIT)) {
            s->regs[R_ADC_INTR_STATUS] |= (1u << FILTER_STATUS_TRANS_BIT);
        }
    }

    ot_adc_ctrl_update_irqs(s);
}

static void ot_adc_ctrl_update_fsm(OtAdcCtrlState *s)
{
    if (s->regs[R_ADC_FSM_RST] & R_ADC_FSM_RST_RST_EN_MASK) {
        s->trigger_q = false;
        s->regs[R_ADC_FSM_STATE] = 0u;
        return;
    }

    bool adc_en = (s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ADC_ENABLE_MASK) != 0u;
    bool oneshot =
        (s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ONESHOT_MODE_MASK) != 0u;
    bool trigger_l2h = !s->trigger_q && adc_en;
    s->trigger_q = adc_en;

    if (!adc_en) {
        s->regs[R_ADC_FSM_STATE] = 0u;
        return;
    }

    if (oneshot) {
        if (trigger_l2h) {
            ot_adc_ctrl_update_chn_val(s, DEFAULT_CHN0_VAL, DEFAULT_CHN1_VAL,
                                       true);
            if (s->regs[R_ADC_INTR_CTL] & (1u << ADC_INTR_ONESHOT_BIT)) {
                s->regs[R_ADC_INTR_STATUS] |= (1u << ADC_INTR_ONESHOT_BIT);
            }
            s->regs[R_ADC_FSM_STATE] = 0u;
            ot_adc_ctrl_update_irqs(s);
        }
    } else {
        s->regs[R_ADC_FSM_STATE] = FSM_STATE_NP_0;
        ot_adc_ctrl_trigger_wakeup(s);
    }
}

static uint64_t ot_adc_ctrl_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    OtAdcCtrlState *s = opaque;
    (void)size;
    uint32_t val32;

    hwaddr reg = R32_OFF(addr);

    switch (reg) {
    case R_INTR_STATE:
    case R_INTR_ENABLE:
    case R_ADC_EN_CTL:
    case R_ADC_PD_CTL:
    case R_ADC_LP_SAMPLE_CTL:
    case R_ADC_SAMPLE_CTL:
    case R_ADC_FSM_RST:
    case R_ADC_CHN0_FILTER_CTL_0 ... R_ADC_CHN0_FILTER_CTL_7:
    case R_ADC_CHN1_FILTER_CTL_0 ... R_ADC_CHN1_FILTER_CTL_7:
    case R_ADC_CHN_VAL_0:
    case R_ADC_CHN_VAL_1:
    case R_ADC_WAKEUP_CTL:
    case R_FILTER_STATUS:
    case R_ADC_INTR_CTL:
    case R_ADC_INTR_STATUS:
    case R_ADC_FSM_STATE:
        val32 = s->regs[reg];
        break;
    case R_INTR_TEST:
    case R_ALERT_TEST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: W/O register 0x%02x (%s)\n",
                      __func__, (uint32_t)addr, REG_NAME(reg));
        val32 = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%02x\n", __func__,
                      (uint32_t)addr);
        val32 = 0;
        break;
    }

    uint32_t pc = ibex_get_current_pc();
    trace_ot_adc_ctrl_io_read_out((uint32_t)addr, REG_NAME(reg), val32, pc);

    return (uint64_t)val32;
}

static void ot_adc_ctrl_regs_write(void *opaque, hwaddr addr, uint64_t val64,
                                   unsigned size)
{
    OtAdcCtrlState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;

    hwaddr reg = R32_OFF(addr);

    uint32_t pc = ibex_get_current_pc();
    trace_ot_adc_ctrl_io_write((uint32_t)addr, REG_NAME(reg), val32, pc);

    switch (reg) {
    case R_INTR_STATE:
        /* RW1C if status source is clear */
        val32 &= R_INTR_STATE_MATCH_PENDING_MASK;
        if (s->regs[R_ADC_INTR_STATUS] == 0u) {
            s->regs[R_INTR_STATE] &= ~val32;
        }
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_INTR_ENABLE:
        val32 &= R_INTR_ENABLE_MATCH_PENDING_MASK;
        s->regs[R_INTR_ENABLE] = val32;
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_INTR_TEST:
        s->regs[R_INTR_TEST] = val32 & R_INTR_TEST_MATCH_PENDING_MASK;
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_ALERT_TEST:
        val32 &= R_ALERT_TEST_FATAL_FAULT_MASK;
        if (val32) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case R_ADC_EN_CTL:
        s->regs[R_ADC_EN_CTL] = val32 & 0x3u;
        ot_adc_ctrl_update_fsm(s);
        break;
    case R_ADC_PD_CTL:
        s->regs[R_ADC_PD_CTL] = val32 & ADC_PD_CTL_MASK;
        if ((s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ADC_ENABLE_MASK) &&
            !(s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ONESHOT_MODE_MASK)) {
            ot_adc_ctrl_update_fsm(s);
        }
        break;
    case R_ADC_LP_SAMPLE_CTL:
        s->regs[R_ADC_LP_SAMPLE_CTL] = val32 & 0xffu;
        break;
    case R_ADC_SAMPLE_CTL:
        s->regs[R_ADC_SAMPLE_CTL] = val32 & 0xffffu;
        break;
    case R_ADC_FSM_RST:
        s->regs[R_ADC_FSM_RST] = val32 & 0x1u;
        ot_adc_ctrl_update_fsm(s);
        break;
    case R_ADC_CHN0_FILTER_CTL_0 ... R_ADC_CHN0_FILTER_CTL_7:
    case R_ADC_CHN1_FILTER_CTL_0 ... R_ADC_CHN1_FILTER_CTL_7:
        s->regs[reg] = val32 & FILTER_CTL_REG_MASK;
        if ((s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ADC_ENABLE_MASK) &&
            !(s->regs[R_ADC_EN_CTL] & R_ADC_EN_CTL_ONESHOT_MODE_MASK)) {
            ot_adc_ctrl_update_fsm(s);
        }
        break;
    case R_ADC_WAKEUP_CTL:
        s->regs[R_ADC_WAKEUP_CTL] = val32 & WAKEUP_CTL_MASK;
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_FILTER_STATUS:
        /* RW1C */
        s->regs[R_FILTER_STATUS] &= ~(val32 & FILTER_STATUS_MASK);
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_ADC_INTR_CTL:
        s->regs[R_ADC_INTR_CTL] = val32 & ADC_INTR_MASK;
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_ADC_INTR_STATUS:
        /* RW1C */
        s->regs[R_ADC_INTR_STATUS] &= ~(val32 & ADC_INTR_MASK);
        ot_adc_ctrl_update_irqs(s);
        break;
    case R_ADC_CHN_VAL_0:
    case R_ADC_CHN_VAL_1:
    case R_ADC_FSM_STATE:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: R/O register 0x%02x (%s)\n",
                      __func__, (uint32_t)addr, REG_NAME(reg));
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%02x\n", __func__,
                      (uint32_t)addr);
        break;
    }
}

static const MemoryRegionOps ot_adc_ctrl_regs_ops = {
    .read = &ot_adc_ctrl_regs_read,
    .write = &ot_adc_ctrl_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
};

static void ot_adc_ctrl_reset_enter(Object *obj, ResetType type)
{
    OtAdcCtrlClass *c = OT_ADC_CTRL_GET_CLASS(obj);
    OtAdcCtrlState *s = OT_ADC_CTRL(obj);

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    ibex_irq_set(&s->irq, 0);
    ibex_irq_set(&s->alert, 0);

    if (type != RESET_TYPE_COLD && ot_rstmgr_is_low_power_exit()) {
        /*
         * ADC_CTRL_AON resides in DomainAonSel. Low-power wakeup from deep sleep
         * only resets Domain0Sel, preserving AON registers and pending status.
         */
        return;
    }

    memset(s->regs, 0, REGS_SIZE);
    s->trigger_q = false;
    s->regs[R_ADC_PD_CTL] = 0x64070u;
    s->regs[R_ADC_LP_SAMPLE_CTL] = 0x4u;
    s->regs[R_ADC_SAMPLE_CTL] = 0x9bu;
}

static void ot_adc_ctrl_reset_exit(Object *obj, ResetType type)
{
    OtAdcCtrlClass *c = OT_ADC_CTRL_GET_CLASS(obj);
    OtAdcCtrlState *s = OT_ADC_CTRL(obj);

    if (c->parent_phases.exit) {
        c->parent_phases.exit(obj, type);
    }

    ot_adc_ctrl_update_irqs(s);
}

static void ot_adc_ctrl_init(Object *obj)
{
    OtAdcCtrlState *s = OT_ADC_CTRL(obj);

    memory_region_init_io(&s->mmio, obj, &ot_adc_ctrl_regs_ops, s,
                          TYPE_OT_ADC_CTRL, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    s->regs = g_new0(uint32_t, REGS_COUNT);
    ibex_sysbus_init_irq(obj, &s->irq);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
    ibex_qdev_init_irq(obj, &s->wkup, OT_ADC_CTRL_WKUP);
}

static void ot_adc_ctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(klass);
    OtAdcCtrlClass *ac = OT_ADC_CTRL_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_adc_ctrl_reset_enter, NULL,
                                       &ot_adc_ctrl_reset_exit,
                                       &ac->parent_phases);
}

static const TypeInfo ot_adc_ctrl_info = {
    .name = TYPE_OT_ADC_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtAdcCtrlState),
    .instance_init = &ot_adc_ctrl_init,
    .class_size = sizeof(OtAdcCtrlClass),
    .class_init = &ot_adc_ctrl_class_init,
};

static void ot_adc_ctrl_register_types(void)
{
    type_register_static(&ot_adc_ctrl_info);
}

type_init(ot_adc_ctrl_register_types);
