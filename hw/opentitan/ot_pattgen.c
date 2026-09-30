/*
 * QEMU OpenTitan Pattern Generator (PATTGEN) device
 *
 * Copyright lowRISC contributors.
 *
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_gpio_eg.h"
#include "hw/opentitan/ot_pattgen.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"

#define PARAM_NUM_CHANNELS 2u
#define PARAM_NUM_IRQS     2u
#define PATTGEN_NUM_PINS   (PARAM_NUM_CHANNELS * 2u)

#define PATTGEN_START_OFFSET_US 10000ULL
#define PATTGEN_IO_CLK_HZ       6000000ULL
#define PATTGEN_MIN_DURATION_NS 1000ULL

#define PATTGEN_PERMIT_1B 0x1u
#define PATTGEN_PERMIT_4B 0xfu

/* clang-format off */
REG32(INTR_STATE, 0x00u)
    SHARED_FIELD(INTR_DONE_CH0, 0u, 1u)
    SHARED_FIELD(INTR_DONE_CH1, 1u, 1u)
REG32(INTR_ENABLE, 0x04u)
REG32(INTR_TEST, 0x08u)
REG32(ALERT_TEST, 0x0cu)
    FIELD(ALERT_TEST, FATAL_FAULT, 0u, 1u)
REG32(CTRL, 0x10u)
    FIELD(CTRL, ENABLE_CH0, 0u, 1u)
    FIELD(CTRL, ENABLE_CH1, 1u, 1u)
    FIELD(CTRL, POLARITY_CH0, 2u, 1u)
    FIELD(CTRL, POLARITY_CH1, 3u, 1u)
    FIELD(CTRL, INACTIVE_LEVEL_PCL_CH0, 4u, 1u)
    FIELD(CTRL, INACTIVE_LEVEL_PDA_CH0, 5u, 1u)
    FIELD(CTRL, INACTIVE_LEVEL_PCL_CH1, 6u, 1u)
    FIELD(CTRL, INACTIVE_LEVEL_PDA_CH1, 7u, 1u)
REG32(PREDIV_CH0, 0x14u)
REG32(DATA_CH0_0, 0x1cu)
REG32(SIZE, 0x2cu)
    FIELD(SIZE, LEN_CH0, 0u, 6u)
    FIELD(SIZE, REPS_CH0, 6u, 10u)
    FIELD(SIZE, LEN_CH1, 16u, 6u)
    FIELD(SIZE, REPS_CH1, 22u, 10u)
/* clang-format on */

#define INTR_MASK (INTR_DONE_CH0_MASK | INTR_DONE_CH1_MASK)
#define CTRL_MASK \
    (R_CTRL_ENABLE_CH0_MASK | R_CTRL_ENABLE_CH1_MASK | \
     R_CTRL_POLARITY_CH0_MASK | R_CTRL_POLARITY_CH1_MASK | \
     R_CTRL_INACTIVE_LEVEL_PCL_CH0_MASK | R_CTRL_INACTIVE_LEVEL_PDA_CH0_MASK | \
     R_CTRL_INACTIVE_LEVEL_PCL_CH1_MASK | R_CTRL_INACTIVE_LEVEL_PDA_CH1_MASK)

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))
#define R_LAST_REG   (R_SIZE)
#define REGS_COUNT   (R_LAST_REG + 1u)
#define REGS_SIZE    (REGS_COUNT * sizeof(uint32_t))

typedef struct {
    bool active;
    bool pending_start;
    bool polarity;
    bool inactive_pcl;
    bool inactive_pda;
    uint32_t prediv;
    uint64_t data;
    uint32_t len;
    uint32_t reps;
} OtPattgenChan;

struct OtPattgenState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    IbexIRQ irqs[PARAM_NUM_IRQS];
    IbexIRQ alert;
    QEMUTimer *timer;

    OtGpioEgState *gpio;

    uint32_t regs[REGS_COUNT];
    OtPattgenChan chan[PARAM_NUM_CHANNELS];
    bool pda[PARAM_NUM_CHANNELS];
    bool pcl[PARAM_NUM_CHANNELS];

    char *ot_id;
};

static void ot_pattgen_update_irqs(OtPattgenState *s)
{
    uint32_t state = s->regs[R_INTR_STATE] & s->regs[R_INTR_ENABLE];
    for (unsigned i = 0; i < PARAM_NUM_IRQS; i++) {
        ibex_irq_set(&s->irqs[i], (int)((state >> i) & 1u));
    }
}

static void ot_pattgen_latch_channel(OtPattgenState *s, unsigned c)
{
    OtPattgenChan *ch = &s->chan[c];
    uint32_t ctrl = s->regs[R_CTRL];
    ch->polarity = (bool)((ctrl >> (R_CTRL_POLARITY_CH0_SHIFT + c)) & 1u);
    ch->inactive_pcl =
        (bool)((ctrl >> (R_CTRL_INACTIVE_LEVEL_PCL_CH0_SHIFT + 2u * c)) & 1u);
    ch->inactive_pda =
        (bool)((ctrl >> (R_CTRL_INACTIVE_LEVEL_PDA_CH0_SHIFT + 2u * c)) & 1u);
    ch->prediv = s->regs[R_PREDIV_CH0 + c];
    uint64_t lo = s->regs[R_DATA_CH0_0 + 2u * c];
    uint64_t hi = s->regs[R_DATA_CH0_0 + 2u * c + 1u];
    ch->data = lo | (hi << 32);
    uint32_t size = s->regs[R_SIZE];
    if (c == 0u) {
        ch->len = FIELD_EX32(size, SIZE, LEN_CH0);
        ch->reps = FIELD_EX32(size, SIZE, REPS_CH0);
    } else {
        ch->len = FIELD_EX32(size, SIZE, LEN_CH1);
        ch->reps = FIELD_EX32(size, SIZE, REPS_CH1);
    }
}

bool ot_pattgen_get_pin_level(OtPattgenState *s, unsigned pin_idx)
{
    if (!s || pin_idx >= PATTGEN_NUM_PINS) {
        return false;
    }
    return (pin_idx & 1u) ? s->pcl[pin_idx >> 1u] : s->pda[pin_idx >> 1u];
}

static void ot_pattgen_timer_cb(void *opaque)
{
    OtPattgenState *s = OT_PATTGEN(opaque);
    bool run_ch[PARAM_NUM_CHANNELS] = { false, false };
    uint32_t total_steps[PARAM_NUM_CHANNELS] = { 0, 0 };
    uint32_t k[PARAM_NUM_CHANNELS] = { 1, 1 };

    for (unsigned c = 0; c < PARAM_NUM_CHANNELS; c++) {
        bool en = (bool)((s->regs[R_CTRL] >> c) & 1u);
        if (en && s->chan[c].pending_start) {
            s->chan[c].pending_start = false;
            s->chan[c].active = true;
            run_ch[c] = true;
            uint32_t total_bits =
                (s->chan[c].len + 1u) * (s->chan[c].reps + 1u);
            total_steps[c] = 2u * total_bits;
            k[c] = 0;
        }
    }

    if (!run_ch[0] && !run_ch[1]) {
        return;
    }

    uint64_t base_us = (s->gpio ? ot_gpio_eg_get_total_us(s->gpio) : 0ULL) +
                       PATTGEN_START_OFFSET_US;
    uint32_t done_mask = 0;

    if (s->gpio) {
        ot_gpio_eg_set_pattgen_batch(s->gpio, true);
    }

    while ((run_ch[0] && k[0] <= total_steps[0]) ||
           (run_ch[1] && k[1] <= total_steps[1])) {
        uint64_t step_us[PARAM_NUM_CHANNELS] = { UINT64_MAX, UINT64_MAX };
        for (unsigned c = 0; c < PARAM_NUM_CHANNELS; c++) {
            if (run_ch[c] && k[c] <= total_steps[c]) {
                uint64_t t_ns =
                    ((uint64_t)k[c] * ((uint64_t)s->chan[c].prediv + 1ULL) *
                     (uint64_t)NANOSECONDS_PER_SECOND) /
                    PATTGEN_IO_CLK_HZ;
                step_us[c] = base_us + (t_ns / (uint64_t)SCALE_US);
            }
        }

        uint64_t next_us = MIN(step_us[0], step_us[1]);

        for (unsigned c = 0; c < PARAM_NUM_CHANNELS; c++) {
            if (run_ch[c] && k[c] <= total_steps[c] && step_us[c] == next_us) {
                uint32_t step = k[c];
                if (step < total_steps[c]) {
                    bool pcl_int = (step & 1u) != 0;
                    s->pcl[c] = s->chan[c].polarity ? !pcl_int : pcl_int;
                    unsigned bit_idx = (step / 2u) % (s->chan[c].len + 1u);
                    s->pda[c] = (bool)((s->chan[c].data >> bit_idx) & 1u);
                } else {
                    s->chan[c].active = false;
                    s->pcl[c] = s->chan[c].inactive_pcl;
                    s->pda[c] = s->chan[c].inactive_pda;
                    done_mask |= (1u << c);
                }
                k[c]++;
            }
        }

        if (s->gpio) {
            ot_gpio_eg_notify_pattgen_change(s->gpio, next_us);
        }
    }

    if (s->gpio) {
        ot_gpio_eg_set_pattgen_batch(s->gpio, false);
    }

    if (done_mask) {
        s->regs[R_INTR_STATE] |= done_mask;
        ot_pattgen_update_irqs(s);
    }
}

static bool ot_pattgen_accepts(void *opaque, hwaddr addr, unsigned size,
                               bool is_write, MemTxAttrs attrs)
{
    (void)opaque;
    (void)attrs;
    hwaddr reg = R32_OFF(addr);
    uint8_t reg_be =
        (uint8_t)(((1u << size) - 1u) << (addr & (sizeof(uint32_t) - 1u)));
    uint8_t permit =
        (reg >= R_PREDIV_CH0) ? PATTGEN_PERMIT_4B : PATTGEN_PERMIT_1B;
    return reg < REGS_COUNT && (!is_write || (permit & ~reg_be) == 0u);
}

static uint64_t ot_pattgen_read(void *opaque, hwaddr addr, unsigned size)
{
    OtPattgenState *s = opaque;
    (void)size;
    hwaddr reg = R32_OFF(addr);
    if (reg == R_INTR_TEST || reg == R_ALERT_TEST) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: W/O register 0x%02" HWADDR_PRIx "\n", __func__,
                      addr);
        return 0;
    }
    return s->regs[reg];
}

static void ot_pattgen_write(void *opaque, hwaddr addr, uint64_t val64,
                             unsigned size)
{
    OtPattgenState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;
    hwaddr reg = R32_OFF(addr);

    switch (reg) {
    case R_INTR_STATE:
        s->regs[R_INTR_STATE] &= ~(val32 & INTR_MASK);
        ot_pattgen_update_irqs(s);
        break;
    case R_INTR_ENABLE:
        s->regs[R_INTR_ENABLE] = val32 & INTR_MASK;
        ot_pattgen_update_irqs(s);
        break;
    case R_INTR_TEST:
        s->regs[R_INTR_STATE] |= (val32 & INTR_MASK);
        ot_pattgen_update_irqs(s);
        break;
    case R_ALERT_TEST:
        if (val32 & R_ALERT_TEST_FATAL_FAULT_MASK) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case R_CTRL:
        val32 &= CTRL_MASK;
        /* fall through */
    default: {
        uint32_t old_ctrl = s->regs[R_CTRL];
        s->regs[reg] = val32;
        bool schedule_timer = false;
        uint64_t max_duration_ns = 0;

        for (unsigned c = 0; c < PARAM_NUM_CHANNELS; c++) {
            bool was_en = (bool)((old_ctrl >> c) & 1u);
            bool now_en = (bool)((s->regs[R_CTRL] >> c) & 1u);
            if (!now_en) {
                s->chan[c].pending_start = false;
                ot_pattgen_latch_channel(s, c);
                if (s->chan[c].active) {
                    s->pda[c] = (bool)(s->chan[c].data & 1u);
                    s->pcl[c] = s->chan[c].polarity;
                } else {
                    s->pda[c] = s->chan[c].inactive_pda;
                    s->pcl[c] = s->chan[c].inactive_pcl;
                }
            } else if (!was_en && now_en) {
                ot_pattgen_latch_channel(s, c);
                s->chan[c].active = true;
                s->chan[c].pending_start = true;
                schedule_timer = true;
                uint64_t total_steps =
                    2ULL * ((uint64_t)s->chan[c].len + 1ULL) *
                    ((uint64_t)s->chan[c].reps + 1ULL);
                uint64_t dur_ns =
                    (total_steps * ((uint64_t)s->chan[c].prediv + 1ULL) *
                     (uint64_t)NANOSECONDS_PER_SECOND) /
                    PATTGEN_IO_CLK_HZ;
                max_duration_ns =
                    MAX(max_duration_ns, MAX(dur_ns, PATTGEN_MIN_DURATION_NS));
            }
        }

        if (s->gpio) {
            ot_gpio_eg_notify_pattgen_change(s->gpio, 0);
        }

        if (schedule_timer) {
            timer_mod_ns(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                       (int64_t)max_duration_ns);
        }
        break;
    }
    }
}

static const MemoryRegionOps ot_pattgen_regs_ops = {
    .read = &ot_pattgen_read,
    .write = &ot_pattgen_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4u,
        .max_access_size = 4u,
    },
    .valid = {
        .min_access_size = 1u,
        .max_access_size = 4u,
        .accepts = &ot_pattgen_accepts,
    },
};

static const Property ot_pattgen_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtPattgenState, ot_id),
    DEFINE_PROP_LINK("gpio", OtPattgenState, gpio, TYPE_OT_GPIO_EG,
                     OtGpioEgState *),
};

static void ot_pattgen_reset(DeviceState *dev)
{
    OtPattgenState *s = OT_PATTGEN(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chan, 0, sizeof(s->chan));
    for (unsigned c = 0; c < PARAM_NUM_CHANNELS; c++) {
        s->pda[c] = false;
        s->pcl[c] = false;
    }
    ot_pattgen_update_irqs(s);
    ibex_irq_set(&s->alert, 0);
}

static void ot_pattgen_init(Object *obj)
{
    OtPattgenState *s = OT_PATTGEN(obj);

    memory_region_init_io(&s->mmio, obj, &ot_pattgen_regs_ops, s,
                          TYPE_OT_PATTGEN, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    for (unsigned i = 0; i < PARAM_NUM_IRQS; i++) {
        ibex_sysbus_init_irq(obj, &s->irqs[i]);
    }
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &ot_pattgen_timer_cb, s);
}

static void ot_pattgen_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    device_class_set_legacy_reset(dc, &ot_pattgen_reset);
    device_class_set_props(dc, ot_pattgen_properties);
}

static const TypeInfo ot_pattgen_info = {
    .name = TYPE_OT_PATTGEN,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtPattgenState),
    .instance_init = &ot_pattgen_init,
    .class_init = &ot_pattgen_class_init,
};

static void ot_pattgen_register_types(void)
{
    type_register_static(&ot_pattgen_info);
}

type_init(ot_pattgen_register_types);
