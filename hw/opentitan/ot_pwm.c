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
#include "hw/opentitan/ot_pwm.h"
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
    if (beat_ns == 0) {
        beat_ns = 4000ULL;
    }

    uint64_t elapsed_beats = (uint64_t)(now_ns - s->cntr_start_ns) / beat_ns;
    uint32_t beats_per_cycle = 1u << (dc_resn + 1u);
    uint32_t beat_in_cycle = (uint32_t)(elapsed_beats % beats_per_cycle);
    uint32_t step = 1u << (16u - dc_resn - 1u);
    uint16_t phase_cntr = (uint16_t)(beat_in_cycle * step);

    uint32_t duty_reg = s->regs[R_DUTY_CYCLE_0 + ch];
    uint16_t duty_a = (uint16_t)(duty_reg & 0xffffu);

    uint32_t param_reg = s->regs[R_PWM_PARAM_0 + ch];
    uint16_t phase_delay = (uint16_t)(param_reg & 0xffffu);

    uint16_t rel_phase = (uint16_t)(phase_cntr - phase_delay);
    bool active = (rel_phase < duty_a);

    return invert ? !active : active;
}

static uint64_t ot_pwm_read(void *opaque, hwaddr addr, unsigned size)
{
    OtPwmState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Bad offset 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        return 0;
    }

    if (reg == R_ALERT_TEST) {
        return 0;
    }

    return s->regs[reg];
}

static void ot_pwm_write(void *opaque, hwaddr addr, uint64_t val64,
                         unsigned size)
{
    OtPwmState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;
    hwaddr reg = R32_OFF(addr);

    if (reg >= REGS_COUNT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Bad offset 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        return;
    }

    if (reg == R_ALERT_TEST) {
        if (val32 & R_ALERT_TEST_FATAL_FAULT_MASK) {
            ibex_irq_set(&s->alert, 1);
        }
        return;
    }

    if (reg == R_REGWEN) {
        s->regs[R_REGWEN] &= (val32 & R_REGWEN_EN_MASK);
        return;
    }

    if (!s->regs[R_REGWEN]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %s: Write to locked register 0x%02x\n", __func__,
                      s->ot_id, (uint32_t)addr);
        return;
    }

    switch (reg) {
    case R_CFG: {
        bool prev_en = (bool)FIELD_EX32(s->regs[R_CFG], CFG, CNTR_EN);
        bool new_en = (bool)FIELD_EX32(val32, CFG, CNTR_EN);
        s->regs[R_CFG] = val32;
        if (!prev_en && new_en) {
            s->cntr_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;
    }
    case R_PWM_EN: {
        uint32_t prev_en = s->regs[R_PWM_EN];
        s->regs[R_PWM_EN] = val32 & ((1u << PARAM_NUM_OUTPUTS) - 1u);
        if (!prev_en && s->regs[R_PWM_EN] &&
            FIELD_EX32(s->regs[R_CFG], CFG, CNTR_EN)) {
            s->cntr_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;
    }
    case R_INVERT:
        s->regs[R_INVERT] = val32 & ((1u << PARAM_NUM_OUTPUTS) - 1u);
        break;
    case R_PWM_PARAM_0 ... R_BLINK_PARAM_5:
        s->regs[reg] = val32;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ot_pwm_regs_ops = {
    .read = &ot_pwm_read,
    .write = &ot_pwm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
};

static const Property ot_pwm_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtPwmState, ot_id),
};

static void ot_pwm_reset(DeviceState *dev)
{
    OtPwmState *s = OT_PWM(dev);

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
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
}

static void ot_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_legacy_reset(dc, &ot_pwm_reset);
    device_class_set_props(dc, ot_pwm_properties);
}

static const TypeInfo ot_pwm_info = {
    .name = TYPE_OT_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtPwmState),
    .instance_init = &ot_pwm_init,
    .class_init = &ot_pwm_class_init,
};

static void ot_pwm_register_types(void)
{
    type_register_static(&ot_pwm_info);
}

type_init(ot_pwm_register_types);
