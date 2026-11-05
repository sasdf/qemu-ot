/*
 * QEMU OpenTitan Darjeeling GPIO device
 *
 * Copyright (c) 2023-2026 lowRISC contributors.
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
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_gpio_dj.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "trace.h"

#define PARAM_NUM_ALERTS 1u
#define PARAM_NUM_IO     32u

/* clang-format off */
REG32(INTR_STATE, 0x0u)
REG32(INTR_ENABLE, 0x4u)
REG32(INTR_TEST, 0x8u)
REG32(ALERT_TEST, 0xcu)
    FIELD(ALERT_TEST, FATAL_FAULT_ERR, 0u, 1u)
REG32(DATA_IN, 0x10u)
REG32(HW_STRAPS_DATA_IN_VALID, 0x14u)
    FIELD(HW_STRAPS_DATA_IN_VALID, VALID, 0u, 1u)
REG32(HW_STRAPS_DATA_IN, 0x18u)
REG32(DIRECT_OUT, 0x1cu)
REG32(MASKED_OUT_LOWER, 0x20u)
    SHARED_FIELD(MASKED_VALUE, 0u, 16u)
    SHARED_FIELD(MASKED_MASK, 16u, 16u)
REG32(MASKED_OUT_UPPER, 0x24u)
REG32(DIRECT_OE, 0x28u)
REG32(MASKED_OE_LOWER, 0x2cu)
REG32(MASKED_OE_UPPER, 0x30u)
REG32(INTR_CTRL_EN_RISING, 0x34u)
REG32(INTR_CTRL_EN_FALLING, 0x38u)
REG32(INTR_CTRL_EN_LVLHIGH, 0x3cu)
REG32(INTR_CTRL_EN_LVLLOW, 0x40u)
REG32(CTRL_EN_INPUT_FILTER, 0x44u)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))

#define R_LAST_REG (R_CTRL_EN_INPUT_FILTER)
#define REGS_COUNT (R_LAST_REG + 1u)
#define REGS_SIZE  (REGS_COUNT * sizeof(uint32_t))

#define ALERT_TEST_MASK (R_ALERT_TEST_FATAL_FAULT_ERR_MASK)

struct OtGpioDjState {
    SysBusDevice parent_obj;

    IbexIRQ *irqs;
    IbexIRQ *gpos;
    IbexIRQ alert;

    MemoryRegion mmio;

    uint32_t regs[REGS_COUNT];
    uint32_t data_out;
    uint32_t data_oe;

    char *ot_id;
};

struct OtGpioDjClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

static void ot_gpio_dj_update_irqs(OtGpioDjState *s)
{
    uint32_t level = s->regs[R_INTR_STATE] & s->regs[R_INTR_ENABLE];
    trace_ot_gpio_irqs(s->ot_id, s->regs[R_INTR_STATE],
                       s->regs[R_INTR_ENABLE], level);
    for (unsigned ix = 0; ix < PARAM_NUM_IO; ix++) {
        ibex_irq_set(&s->irqs[ix], (int)((level >> ix) & 0x1u));
    }
}

static void ot_gpio_dj_check_level_intr(OtGpioDjState *s)
{
    uint32_t data_in = s->regs[R_DATA_IN];
    uint32_t lvl_high = s->regs[R_INTR_CTRL_EN_LVLHIGH] & data_in;
    uint32_t lvl_low = s->regs[R_INTR_CTRL_EN_LVLLOW] & ~data_in;
    s->regs[R_INTR_STATE] |= (lvl_high | lvl_low);
}

static void ot_gpio_dj_check_edge_intr(OtGpioDjState *s, uint32_t prev)
{
    uint32_t data_in = s->regs[R_DATA_IN];
    uint32_t diff = data_in ^ prev;
    uint32_t rising = diff & data_in & s->regs[R_INTR_CTRL_EN_RISING];
    uint32_t falling = diff & ~data_in & s->regs[R_INTR_CTRL_EN_FALLING];
    s->regs[R_INTR_STATE] |= (rising | falling);
}

static void ot_gpio_dj_update_outputs(OtGpioDjState *s)
{
    for (unsigned i = 0; i < PARAM_NUM_IO; i++) {
        int val = (s->data_oe & (1u << i)) ?
                  (int)((s->data_out >> i) & 1u) : -1;
        ibex_irq_set(&s->gpos[i], val);
    }
}

static void ot_gpio_dj_in_change(void *opaque, int no, int level)
{
    OtGpioDjState *s = opaque;
    if (no < 0 || no >= PARAM_NUM_IO) {
        return;
    }
    uint32_t prev = s->regs[R_DATA_IN];
    if (level > 0) {
        s->regs[R_DATA_IN] |= (1u << no);
    } else {
        s->regs[R_DATA_IN] &= ~(1u << no);
    }
    ot_gpio_dj_check_edge_intr(s, prev);
    ot_gpio_dj_check_level_intr(s);
    ot_gpio_dj_update_irqs(s);
}

static void ot_gpio_dj_strap_en(void *opaque, int no, int level)
{
    OtGpioDjState *s = opaque;
    g_assert(no == 0);
    if (level) {
        s->regs[R_HW_STRAPS_DATA_IN] = s->regs[R_DATA_IN];
        s->regs[R_HW_STRAPS_DATA_IN_VALID] =
            R_HW_STRAPS_DATA_IN_VALID_VALID_MASK;
        trace_ot_gpio_strap_en(s->ot_id, no, (bool)level,
                               s->regs[R_HW_STRAPS_DATA_IN]);
    }
}

static bool ot_gpio_dj_accepts(void *opaque, hwaddr addr, unsigned size,
                               bool is_write, MemTxAttrs attrs)
{
    (void)opaque;
    (void)attrs;
    (void)is_write;

    if (addr == A_ALERT_TEST && size == 1u) {
        return true;
    }
    return (addr % 4u == 0) && (addr / 4u < REGS_COUNT) && size == 4u;
}

static uint64_t ot_gpio_dj_read(void *opaque, hwaddr addr, unsigned size)
{
    OtGpioDjState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);

    switch (reg) {
    case R_ALERT_TEST:
    case R_INTR_TEST:
        return 0;
    case R_DIRECT_OUT:
        return s->data_out;
    case R_DIRECT_OE:
        return s->data_oe;
    case R_MASKED_OUT_LOWER:
        return s->data_out & 0xffffu;
    case R_MASKED_OUT_UPPER:
        return (s->data_out >> 16u) & 0xffffu;
    case R_MASKED_OE_LOWER:
        return s->data_oe & 0xffffu;
    case R_MASKED_OE_UPPER:
        return (s->data_oe >> 16u) & 0xffffu;
    default:
        return s->regs[reg];
    }
}

static void ot_gpio_dj_write(void *opaque, hwaddr addr, uint64_t val64,
                             unsigned size)
{
    OtGpioDjState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);
    uint32_t val32 = (uint32_t)val64;

    switch (reg) {
    case R_INTR_STATE:
        s->regs[R_INTR_STATE] &= ~val32;
        ot_gpio_dj_check_level_intr(s);
        ot_gpio_dj_update_irqs(s);
        break;
    case R_INTR_ENABLE:
        s->regs[R_INTR_ENABLE] = val32;
        ot_gpio_dj_update_irqs(s);
        break;
    case R_INTR_TEST:
        s->regs[R_INTR_STATE] |= val32;
        ot_gpio_dj_update_irqs(s);
        break;
    case R_ALERT_TEST:
        if (val32 & ALERT_TEST_MASK) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case R_DATA_IN:
    case R_HW_STRAPS_DATA_IN:
    case R_HW_STRAPS_DATA_IN_VALID:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Write to read-only reg 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    case R_DIRECT_OUT:
        s->data_out = val32;
        ot_gpio_dj_update_outputs(s);
        break;
    case R_MASKED_OUT_LOWER: {
        uint32_t mask = (val32 >> 16u) & 0xffffu;
        uint32_t val = val32 & 0xffffu;
        s->data_out = (s->data_out & ~mask) | (val & mask);
        ot_gpio_dj_update_outputs(s);
        break;
    }
    case R_MASKED_OUT_UPPER: {
        uint32_t mask = val32 & 0xffff0000u;
        uint32_t val = (val32 & 0xffffu) << 16u;
        s->data_out = (s->data_out & ~mask) | (val & mask);
        ot_gpio_dj_update_outputs(s);
        break;
    }
    case R_DIRECT_OE:
        s->data_oe = val32;
        ot_gpio_dj_update_outputs(s);
        break;
    case R_MASKED_OE_LOWER: {
        uint32_t mask = (val32 >> 16u) & 0xffffu;
        uint32_t val = val32 & 0xffffu;
        s->data_oe = (s->data_oe & ~mask) | (val & mask);
        ot_gpio_dj_update_outputs(s);
        break;
    }
    case R_MASKED_OE_UPPER: {
        uint32_t mask = val32 & 0xffff0000u;
        uint32_t val = (val32 & 0xffffu) << 16u;
        s->data_oe = (s->data_oe & ~mask) | (val & mask);
        ot_gpio_dj_update_outputs(s);
        break;
    }
    case R_INTR_CTRL_EN_RISING:
    case R_INTR_CTRL_EN_FALLING:
    case R_INTR_CTRL_EN_LVLHIGH:
    case R_INTR_CTRL_EN_LVLLOW:
    case R_CTRL_EN_INPUT_FILTER:
        s->regs[reg] = val32;
        ot_gpio_dj_check_level_intr(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps ot_gpio_dj_regs_ops = {
    .read = &ot_gpio_dj_read,
    .write = &ot_gpio_dj_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
    .valid.accepts = &ot_gpio_dj_accepts,
};

static const Property ot_gpio_dj_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtGpioDjState, ot_id),
};

static void ot_gpio_dj_reset_enter(Object *obj, ResetType type)
{
    OtGpioDjClass *c = OT_GPIO_DJ_GET_CLASS(obj);
    OtGpioDjState *s = OT_GPIO_DJ(obj);

    trace_ot_gpio_reset(s->ot_id, "enter");

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->data_out = 0;
    s->data_oe = 0;

    ot_gpio_dj_update_outputs(s);
    ot_gpio_dj_update_irqs(s);
    ibex_irq_set(&s->alert, 0);
}

static void ot_gpio_dj_init(Object *obj)
{
    OtGpioDjState *s = OT_GPIO_DJ(obj);

    memory_region_init_io(&s->mmio, obj, &ot_gpio_dj_regs_ops, s,
                          TYPE_OT_GPIO_DJ, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    s->irqs = g_new(IbexIRQ, PARAM_NUM_IO);
    s->gpos = g_new(IbexIRQ, PARAM_NUM_IO);
    for (unsigned ix = 0; ix < PARAM_NUM_IO; ix++) {
        ibex_sysbus_init_irq(obj, &s->irqs[ix]);
    }
    ibex_qdev_init_irqs_default(obj, s->gpos, OT_GPIO_OUT, PARAM_NUM_IO, -1);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);

    qdev_init_gpio_in_named(DEVICE(obj), &ot_gpio_dj_strap_en, OT_GPIO_STRAP_EN,
                            1);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_gpio_dj_in_change, OT_GPIO_IN,
                            PARAM_NUM_IO);
}

static void ot_gpio_dj_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_props(dc, ot_gpio_dj_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(dc);
    OtGpioDjClass *gc = OT_GPIO_DJ_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_gpio_dj_reset_enter, NULL,
                                       NULL, &gc->parent_phases);
}

static const TypeInfo ot_gpio_dj_info = {
    .name = TYPE_OT_GPIO_DJ,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtGpioDjState),
    .instance_init = &ot_gpio_dj_init,
    .class_size = sizeof(OtGpioDjClass),
    .class_init = &ot_gpio_dj_class_init,
};

static void ot_gpio_dj_register_types(void)
{
    type_register_static(&ot_gpio_dj_info);
}

type_init(ot_gpio_dj_register_types);
