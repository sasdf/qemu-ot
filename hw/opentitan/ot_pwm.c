/*
 * QEMU OpenTitan PWM device
 *
 * Copyright (c) 2025 Rivos, Inc.
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
#include "qemu/timer.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_gpio_eg.h"
#include "hw/opentitan/ot_pwm.h"
#include "hw/opentitan/ot_rstmgr.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"

#define PARAM_NUM_OUTPUTS 6u

/* clang-format off */
REG32(ALERT_TEST, 0x00u)
    FIELD(ALERT_TEST, FATAL_FAULT, 0u, 1u)
REG32(REGWEN, 0x04u)
    FIELD(REGWEN, EN, 0u, 1u)
REG32(CFG, 0x08u)
    FIELD(CFG, CLK_DIV, 0u, 27u)
    FIELD(CFG, DC_RESN, 27u, 4u)
    FIELD(CFG, CNTR_EN, 31u, 1u)
REG32(PWM_EN, 0x0cu)
REG32(INVERT, 0x10u)
REG32(PWM_PARAM_0, 0x14u)
REG32(PWM_PARAM_1, 0x18u)
REG32(PWM_PARAM_2, 0x1cu)
REG32(PWM_PARAM_3, 0x20u)
REG32(PWM_PARAM_4, 0x24u)
REG32(PWM_PARAM_5, 0x28u)
REG32(DUTY_CYCLE_0, 0x2cu)
REG32(DUTY_CYCLE_1, 0x30u)
REG32(DUTY_CYCLE_2, 0x34u)
REG32(DUTY_CYCLE_3, 0x38u)
REG32(DUTY_CYCLE_4, 0x3cu)
REG32(DUTY_CYCLE_5, 0x40u)
REG32(BLINK_PARAM_0, 0x44u)
REG32(BLINK_PARAM_1, 0x48u)
REG32(BLINK_PARAM_2, 0x4cu)
REG32(BLINK_PARAM_3, 0x50u)
REG32(BLINK_PARAM_4, 0x54u)
REG32(BLINK_PARAM_5, 0x58u)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))
#define R_LAST_REG   (R_BLINK_PARAM_5)
#define REGS_COUNT   (R_LAST_REG + 1u)
#define REGS_SIZE    (REGS_COUNT * sizeof(uint32_t))

struct OtPwmState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    IbexIRQ alert;

    uint32_t regs[REGS_COUNT];
    int64_t cntr_start_ns;

    char *ot_id;
};

struct OtPwmClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

void ot_pwm_shift_start_ns(OtPwmState *s, int64_t delta_ns)
{
    if (s) {
        s->cntr_start_ns += delta_ns;
    }
}

bool ot_pwm_is_active(OtPwmState *s)
{
    if (!s) {
        return false;
    }
    bool cntr_en = (bool)FIELD_EX32(s->regs[R_CFG], CFG, CNTR_EN);
    bool chan_en = (s->regs[R_PWM_EN] != 0);
    return cntr_en && chan_en;
}

bool ot_pwm_get_channel_level(OtPwmState *s, unsigned ch, int64_t now_ns)
{
    if (!s || ch >= PARAM_NUM_OUTPUTS) {
        return false;
    }

    bool cntr_en = (bool)FIELD_EX32(s->regs[R_CFG], CFG, CNTR_EN);
    bool chan_en = (bool)((s->regs[R_PWM_EN] >> ch) & 1u);
    bool invert = (bool)((s->regs[R_INVERT] >> ch) & 1u);

    if (!cntr_en || !chan_en || now_ns < s->cntr_start_ns) {
        return invert;
    }

    uint32_t clk_div = FIELD_EX32(s->regs[R_CFG], CFG, CLK_DIV);
    uint32_t dc_resn = FIELD_EX32(s->regs[R_CFG], CFG, DC_RESN);

    /* AON clock frequency in Earlgrey QEMU is 250 kHz (4000 ns per tick) */
    uint64_t beat_ns = (uint64_t)(clk_div + 1u) * 4000ULL;

    uint64_t elapsed_beats = (uint64_t)(now_ns - s->cntr_start_ns) / beat_ns;
    uint32_t beats_per_cycle = 1u << (dc_resn + 1u);
    uint64_t cycle_idx = elapsed_beats / beats_per_cycle;
    uint32_t beat_in_cycle = (uint32_t)(elapsed_beats % beats_per_cycle);
    uint32_t step = 1u << (16u - dc_resn - 1u);
    uint16_t phase_cntr = (uint16_t)(beat_in_cycle * step);

    uint32_t duty_reg = s->regs[R_DUTY_CYCLE_0 + ch];
    uint16_t duty_a = (uint16_t)(duty_reg & 0xffffu);
    uint16_t duty_b = (uint16_t)((duty_reg >> 16u) & 0xffffu);

    uint32_t param_reg = s->regs[R_PWM_PARAM_0 + ch];
    uint16_t phase_delay = (uint16_t)(param_reg & 0xffffu);
    bool htbt_en = (bool)((param_reg >> 30u) & 1u);
    bool blink_en = (bool)((param_reg >> 31u) & 1u);

    uint32_t blink_reg = s->regs[R_BLINK_PARAM_0 + ch];
    uint16_t blink_x = (uint16_t)(blink_reg & 0xffffu);
    uint16_t blink_y = (uint16_t)((blink_reg >> 16u) & 0xffffu);

    uint16_t duty_cycle_actual = duty_a;
    if (blink_en && !htbt_en) {
        uint16_t blink_sum = (uint16_t)(blink_x + blink_y + 1u);
        uint16_t blink_ctr =
            (uint16_t)(cycle_idx % ((uint64_t)blink_sum + 1ULL));
        duty_cycle_actual = (blink_ctr > blink_x) ? duty_b : duty_a;
    } else if (blink_en && htbt_en && duty_a != duty_b) {
        uint64_t htbt_steps = cycle_idx / ((uint64_t)blink_x + 1ULL);
        bool pos_htbt = (duty_a < duty_b);
        bool htbt_dir = !pos_htbt;
        uint16_t dc_htbt = duty_a;
        bool dc_wrap_q = false;
        uint64_t max_steps = MIN(htbt_steps, 4096ULL);
        for (uint64_t i = 0; i < max_steps; i++) {
            uint32_t ext = htbt_dir ?
                               ((uint32_t)dc_htbt - (uint32_t)blink_y - 1u) :
                               ((uint32_t)dc_htbt + (uint32_t)blink_y + 1u);
            bool dc_wrap = (ext >> 16u) & 1u;
            uint16_t dc_htbt_d = (uint16_t)ext;
            dc_wrap_q = pos_htbt ? (dc_wrap && !htbt_dir) :
                                   (dc_wrap && htbt_dir);
            dc_htbt = dc_htbt_d;
            if (pos_htbt && (dc_htbt >= duty_b || dc_wrap)) {
                htbt_dir = true;
            } else if (pos_htbt && dc_htbt == duty_a) {
                htbt_dir = false;
            } else if (!pos_htbt && (dc_htbt <= duty_b || dc_wrap)) {
                htbt_dir = false;
            } else if (!pos_htbt && dc_htbt == duty_a) {
                htbt_dir = true;
            }
        }
        duty_cycle_actual = !dc_wrap_q ? dc_htbt : (pos_htbt ? 0xffffu : 0u);
    }

    uint16_t dc_mask = (uint16_t)(0xffffu >> (dc_resn + 1u));
    uint16_t phase_delay_masked = phase_delay & ~dc_mask;
    uint16_t duty_cycle_masked = duty_cycle_actual & ~dc_mask;

    uint32_t off_full =
        (uint32_t)phase_delay_masked + (uint32_t)duty_cycle_masked;
    bool phase_wrap = (off_full > 0xffffu);
    uint16_t off_phase = (uint16_t)off_full;

    bool on_phase_exceeded = (phase_cntr >= phase_delay_masked);
    bool off_phase_exceeded = (phase_cntr >= off_phase);

    bool active = phase_wrap ? (on_phase_exceeded || !off_phase_exceeded) :
                               (on_phase_exceeded && !off_phase_exceeded);

    return invert ? !active : active;
}

static const uint8_t PWM_PERMIT[REGS_COUNT] = {
    [R_ALERT_TEST] = 0x1u,
    [R_REGWEN] = 0x1u,
    [R_CFG] = 0xfu,
    [R_PWM_EN] = 0x1u,
    [R_INVERT] = 0x1u,
    [R_PWM_PARAM_0] = 0xfu,
    [R_PWM_PARAM_1] = 0xfu,
    [R_PWM_PARAM_2] = 0xfu,
    [R_PWM_PARAM_3] = 0xfu,
    [R_PWM_PARAM_4] = 0xfu,
    [R_PWM_PARAM_5] = 0xfu,
    [R_DUTY_CYCLE_0] = 0xfu,
    [R_DUTY_CYCLE_1] = 0xfu,
    [R_DUTY_CYCLE_2] = 0xfu,
    [R_DUTY_CYCLE_3] = 0xfu,
    [R_DUTY_CYCLE_4] = 0xfu,
    [R_DUTY_CYCLE_5] = 0xfu,
    [R_BLINK_PARAM_0] = 0xfu,
    [R_BLINK_PARAM_1] = 0xfu,
    [R_BLINK_PARAM_2] = 0xfu,
    [R_BLINK_PARAM_3] = 0xfu,
    [R_BLINK_PARAM_4] = 0xfu,
    [R_BLINK_PARAM_5] = 0xfu,
};

static MemTxResult ot_pwm_read_with_attrs(void *opaque, hwaddr addr,
                                          uint64_t *val64, unsigned size,
                                          MemTxAttrs attrs)
{
    OtPwmState *s = opaque;
    (void)attrs;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Bad offset 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        *val64 = 0;
        return MEMTX_DECODE_ERROR;
    }

    uint32_t val32 = (reg == R_ALERT_TEST) ? 0u : s->regs[reg];
    uint32_t shift = (uint32_t)((addr & 0x3u) * 8u);
    uint32_t size_mask = size >= 4u ? UINT32_MAX : ((1u << (size * 8u)) - 1u);
    *val64 = (uint64_t)((val32 >> shift) & size_mask);
    return MEMTX_OK;
}

static MemTxResult ot_pwm_write_with_attrs(void *opaque, hwaddr addr,
                                           uint64_t val64, unsigned size,
                                           MemTxAttrs attrs)
{
    OtPwmState *s = opaque;
    (void)attrs;
    uint32_t val32 = (uint32_t)val64;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Bad offset 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        return MEMTX_DECODE_ERROR;
    }

    uint8_t reg_be = (uint8_t)(((1u << size) - 1u) << (addr & 0x3u));
    if ((PWM_PERMIT[reg] & ~reg_be) != 0u) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %s: Sub-word write error at 0x%02x (size=%u)\n",
                      __func__, s->ot_id, (uint32_t)addr, size);
        return MEMTX_DECODE_ERROR;
    }

    val32 <<= (addr & 0x3u) * 8u;

    if (reg == R_ALERT_TEST) {
        if (val32 & R_ALERT_TEST_FATAL_FAULT_MASK) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        return MEMTX_OK;
    }

    if (reg == R_REGWEN) {
        s->regs[R_REGWEN] &= (val32 & R_REGWEN_EN_MASK);
        return MEMTX_OK;
    }

    if (!s->regs[R_REGWEN]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %s: Write to locked register 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        return MEMTX_OK;
    }

    switch (reg) {
    case R_CFG: {
        bool new_en = (bool)FIELD_EX32(val32, CFG, CNTR_EN);
        s->regs[R_CFG] = val32;
        if (new_en) {
            s->cntr_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        ot_gpio_eg_notify_sysrst_change();
        break;
    }
    case R_PWM_EN: {
        uint32_t prev_en = s->regs[R_PWM_EN];
        s->regs[R_PWM_EN] = val32 & ((1u << PARAM_NUM_OUTPUTS) - 1u);
        if (!prev_en && s->regs[R_PWM_EN] &&
            FIELD_EX32(s->regs[R_CFG], CFG, CNTR_EN)) {
            s->cntr_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        ot_gpio_eg_notify_sysrst_change();
        break;
    }
    case R_INVERT:
        s->regs[R_INVERT] = val32 & ((1u << PARAM_NUM_OUTPUTS) - 1u);
        ot_gpio_eg_notify_sysrst_change();
        break;
    case R_PWM_PARAM_0 ... R_PWM_PARAM_5:
        s->regs[reg] = val32 & 0xc000ffffu;
        ot_gpio_eg_notify_sysrst_change();
        break;
    case R_DUTY_CYCLE_0 ... R_BLINK_PARAM_5:
    default:
        s->regs[reg] = val32;
        ot_gpio_eg_notify_sysrst_change();
        break;
    }

    return MEMTX_OK;
}

static const MemoryRegionOps ot_pwm_regs_ops = {
    .read_with_attrs = &ot_pwm_read_with_attrs,
    .write_with_attrs = &ot_pwm_write_with_attrs,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1u,
    .impl.max_access_size = 4u,
    .valid.min_access_size = 1u,
    .valid.max_access_size = 4u,
};

static const Property ot_pwm_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtPwmState, ot_id),
};

static void ot_pwm_reset_enter(Object *obj, ResetType type)
{
    OtPwmClass *c = OT_PWM_GET_CLASS(obj);
    OtPwmState *s = OT_PWM(obj);

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    if (type != RESET_TYPE_COLD && ot_rstmgr_is_low_power_exit()) {
        ibex_irq_set(&s->alert, 0);
        return;
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_REGWEN] = 1u;
    s->regs[R_CFG] = (7u << R_CFG_DC_RESN_SHIFT) | 0x8000u;
    for (unsigned i = 0; i < PARAM_NUM_OUTPUTS; i++) {
        s->regs[R_DUTY_CYCLE_0 + i] = 0x7fff7fffull;
    }
    s->cntr_start_ns = 0;
    ibex_irq_set(&s->alert, 0);
}

static void ot_pwm_init(Object *obj)
{
    OtPwmState *s = OT_PWM(obj);

    memory_region_init_io(&s->mmio, obj, &ot_pwm_regs_ops, s, TYPE_OT_PWM,
                          0x80u);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
}

static void ot_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    ResettableClass *rc = RESETTABLE_CLASS(klass);
    OtPwmClass *pc = OT_PWM_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_pwm_reset_enter, NULL, NULL,
                                       &pc->parent_phases);
    device_class_set_props(dc, ot_pwm_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo ot_pwm_info = {
    .name = TYPE_OT_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtPwmState),
    .instance_init = &ot_pwm_init,
    .class_size = sizeof(OtPwmClass),
    .class_init = &ot_pwm_class_init,
};

static void ot_pwm_register_types(void)
{
    type_register_static(&ot_pwm_info);
}

type_init(ot_pwm_register_types);
