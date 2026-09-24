/*
 * QEMU OpenTitan EarlGrey Analog Sensor Top device
 *
 * Copyright (c) 2023-2025 Rivos, Inc.
 *
 * Author(s):
 *  Emmanuel Blot <eblot@rivosinc.com>
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
 *
 * Note: for now, only a minimalist subset of Analog Sensor Top device is
 *       implemented in order to enable OpenTitan's ROM boot to progress
 */

#include "qemu/osdep.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "qemu/typedefs.h"
#include "qapi/error.h"
#include "hw/opentitan/ot_ast_eg.h"
#include "hw/opentitan/ot_clock_ctrl.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_noise_src.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_clock_src.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "trace.h"

/* clang-format off */
REG32(REGA0, 0x0u)
REG32(REGA1, 0x4u)
REG32(REGA2, 0x8u)
REG32(REGA3, 0xcu)
REG32(REGA4, 0x10u)
REG32(REGA5, 0x14u)
REG32(REGA6, 0x18u)
REG32(REGA7, 0x1cu)
REG32(REGA8, 0x20u)
REG32(REGA9, 0x24u)
REG32(REGA10, 0x28u)
REG32(REGA11, 0x2cu)
REG32(REGA12, 0x30u)
REG32(REGA13, 0x34u)
REG32(REGA14, 0x38u)
REG32(REGA15, 0x3cu)
REG32(REGA16, 0x40u)
REG32(REGA17, 0x44u)
REG32(REGA18, 0x48u)
REG32(REGA19, 0x4cu)
REG32(REGA20, 0x50u)
REG32(REGA21, 0x54u)
REG32(REGA22, 0x58u)
REG32(REGA23, 0x5cu)
REG32(REGA24, 0x60u)
REG32(REGA25, 0x64u)
REG32(REGA26, 0x68u)
REG32(REGA27, 0x6cu)
REG32(REGA28, 0x70u)
REG32(REGA29, 0x74u)
REG32(REGA30, 0x78u)
REG32(REGA31, 0x7cu)
REG32(REGA32, 0x80u)
REG32(REGA33, 0x84u)
REG32(REGA34, 0x88u)
REG32(REGA35, 0x8cu)
REG32(REGA36, 0x90u)
REG32(REGA37, 0x94u)
REG32(REGAL, 0x98u)
REG32(REGA38, 0x98u)
REG32(REGA39, 0x9cu)
REG32(REGA40, 0xa0u)
REG32(REGA41, 0xa4u)
REG32(REGA42, 0xa8u)
REG32(REGA43, 0xacu)
REG32(REGA44, 0xb0u)
REG32(REGA45, 0xb4u)
REG32(REGA46, 0xb8u)
REG32(REGA47, 0xbcu)
REG32(REGA48, 0xc0u)
REG32(REGA49, 0xc4u)
REG32(REGA50, 0xc8u)
REG32(REGA51, 0xccu)
REG32(REGA52, 0xd0u)
REG32(REGAL_V2, 0xd4u)
REG32(REGB0, 0x200u)
REG32(REGB1, 0x204u)
REG32(REGB2, 0x208u)
REG32(REGB3, 0x20cu)
REG32(REGB4, 0x210u)
/* clang-format on */

#define R32_OFF(_r_) ((_r_) / sizeof(uint32_t))

#define RA_V1_LAST_REG (R_REGAL)
#define REGSA_V1_COUNT (RA_V1_LAST_REG + 1u)
#define RA_LAST_REG    (R_REGAL_V2)
#define REGSA_COUNT    (RA_LAST_REG + 1u)
#define REGSA_SIZE     (REGSA_COUNT * sizeof(uint32_t))
#define REGA_NAME(_s_, _reg_) \
    (((_reg_) == ((_s_)->num_regs_a - 1u)) ? \
         "REGAL" : \
         ((((_reg_) < REGSA_COUNT) && REGA_NAMES[_reg_]) ? REGA_NAMES[_reg_] : \
                                                           "?"))

#define RB_LAST_REG (R_REGB4)
#define REGSB_COUNT (RB_LAST_REG + 1u)
#define REGSB_SIZE  (REGSB_COUNT * sizeof(uint32_t))
#define REGB_NAME(_reg_) \
    ((((_reg_) < REGSB_COUNT) && REGB_NAMES[_reg_]) ? REGB_NAMES[_reg_] : "?")

#define REGS_SIZE (A_REGB4 + sizeof(uint32_t))
#define REG_NAME(_s_, _reg_) \
    ((_reg_) >= R_REGB0 ? REGB_NAME((_reg_) - (R_REGB0)) : \
                          REGA_NAME(_s_, _reg_))

#define REG_NAME_ENTRY(_reg_) [R_##_reg_] = stringify(_reg_)
static const char REGA_NAMES[REGSA_COUNT][8U] = {
    REG_NAME_ENTRY(REGA0),  REG_NAME_ENTRY(REGA1),  REG_NAME_ENTRY(REGA2),
    REG_NAME_ENTRY(REGA3),  REG_NAME_ENTRY(REGA4),  REG_NAME_ENTRY(REGA5),
    REG_NAME_ENTRY(REGA6),  REG_NAME_ENTRY(REGA7),  REG_NAME_ENTRY(REGA8),
    REG_NAME_ENTRY(REGA9),  REG_NAME_ENTRY(REGA10), REG_NAME_ENTRY(REGA11),
    REG_NAME_ENTRY(REGA12), REG_NAME_ENTRY(REGA13), REG_NAME_ENTRY(REGA14),
    REG_NAME_ENTRY(REGA15), REG_NAME_ENTRY(REGA16), REG_NAME_ENTRY(REGA17),
    REG_NAME_ENTRY(REGA18), REG_NAME_ENTRY(REGA19), REG_NAME_ENTRY(REGA20),
    REG_NAME_ENTRY(REGA21), REG_NAME_ENTRY(REGA22), REG_NAME_ENTRY(REGA23),
    REG_NAME_ENTRY(REGA24), REG_NAME_ENTRY(REGA25), REG_NAME_ENTRY(REGA26),
    REG_NAME_ENTRY(REGA27), REG_NAME_ENTRY(REGA28), REG_NAME_ENTRY(REGA29),
    REG_NAME_ENTRY(REGA30), REG_NAME_ENTRY(REGA31), REG_NAME_ENTRY(REGA32),
    REG_NAME_ENTRY(REGA33), REG_NAME_ENTRY(REGA34), REG_NAME_ENTRY(REGA35),
    REG_NAME_ENTRY(REGA36), REG_NAME_ENTRY(REGA37), REG_NAME_ENTRY(REGA38),
    REG_NAME_ENTRY(REGA39), REG_NAME_ENTRY(REGA40), REG_NAME_ENTRY(REGA41),
    REG_NAME_ENTRY(REGA42), REG_NAME_ENTRY(REGA43), REG_NAME_ENTRY(REGA44),
    REG_NAME_ENTRY(REGA45), REG_NAME_ENTRY(REGA46), REG_NAME_ENTRY(REGA47),
    REG_NAME_ENTRY(REGA48), REG_NAME_ENTRY(REGA49), REG_NAME_ENTRY(REGA50),
    REG_NAME_ENTRY(REGA51), REG_NAME_ENTRY(REGA52),
};

static const char REGB_NAMES[REGSB_COUNT][6U] = {
    REG_NAME_ENTRY(REGB0), REG_NAME_ENTRY(REGB1), REG_NAME_ENTRY(REGB2),
    REG_NAME_ENTRY(REGB3), REG_NAME_ENTRY(REGB4),
};
#undef REG_NAME_ENTRY

#define OT_AST_EG_NOISE_4BIT_RATE 200000u /* 200 kHz (24 MHz / 120 clocks) */

typedef struct {
    char *name;
    unsigned frequency;
    IbexIRQ out;
    char *irq_name;
    const DeviceState *sink;
    bool aon;
    bool active;
} OtASTEgClock;

struct OtASTEgState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    GList *clocks; /* OtASTEgClock */

    uint32_t *regsa;
    uint32_t *regsb;

    char *cfg_topclocks;
    char *cfg_aonclocks;
    uint32_t num_regs_a;
};

struct OtASTEgClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

static const char *CFGSEP = ",";

static gint ot_ast_eg_match_clock_by_name(gconstpointer a, gconstpointer b)
{
    const OtASTEgClock *ca = a;
    const OtASTEgClock *cb = b;

    return strcmp(ca->name, cb->name);
}

static OtASTEgClock *ot_ast_eg_find_clock(OtASTEgState *s, const char *name)
{
    OtASTEgClock clock = { .name = (char *)name };

    GList *glist =
        g_list_find_custom(s->clocks, &clock, &ot_ast_eg_match_clock_by_name);

    return glist ? glist->data : NULL;
}

static void ot_ast_eg_reset_clock(gpointer data, gpointer user_data)
{
    OtASTEgState *s = user_data;
    OtASTEgClock *clk = data;
    (void)s;

    clk->active = clk->aon;
}

static void ot_ast_eg_update_clock(gpointer data, gpointer user_data)
{
    OtASTEgState *s = user_data;
    OtASTEgClock *clk = data;
    (void)s;

    unsigned frequency = clk->active ? clk->frequency : 0;
    trace_ot_ast_upate_clock(clk->name, frequency);

    ibex_irq_set(&clk->out, (int)frequency);
}

static const char *
ot_ast_eg_get_clock_source(IbexClockSrcIf *ifd, const char *name,
                           const DeviceState *sink, Error **errp)
{
    OtASTEgState *s = OT_AST_EG(ifd);

    OtASTEgClock *clk = ot_ast_eg_find_clock(s, name);
    if (!clk) {
        error_setg(errp, "%s: AST: no such clock: %s", __func__, name);
        return NULL;
    }

    if (clk->sink && clk->sink != sink) {
        error_setg(errp, "%s: AST supports a unique sink per clock: %s",
                   __func__, name);
        return NULL;
    }

    clk->sink = sink;

    return clk->irq_name;
}

static void ot_ast_eg_clock_enable(OtClockCtrlIf *dev, const char *clkname,
                                   bool enable)
{
    OtASTEgState *s = OT_AST_EG(dev);

    OtASTEgClock *clk = ot_ast_eg_find_clock(s, clkname);
    g_assert(clk);

    clk->active = enable;
    unsigned frequency = clk->active ? clk->frequency : 0;

    trace_ot_ast_upate_clock(clk->name, frequency);

    ibex_irq_set(&clk->out, (int)frequency);
}

static void ot_ast_eg_clock_ext_freq_select(OtClockCtrlIf *dev, bool enable)
{
    OtASTEgState *s = OT_AST_EG(dev);

    (void)s;

    qemu_log_mask(LOG_UNIMP, "%s: not implemented: %u\n", __func__, enable);
}

static unsigned ot_ast_eg_get_fill_rate(OtNoiseSrcIf *dev)
{
    (void)dev;

    return OT_AST_EG_NOISE_4BIT_RATE / 2u; /* 4 bits to byte */
}

static void ot_ast_eg_get_noise(OtNoiseSrcIf *dev, uint8_t *buffer,
                                size_t length)
{
    (void)dev;

    qemu_guest_getrandom_nofail((void *)buffer, length);
}

static void ot_ast_eg_parse_clocks(OtASTEgState *s, Error **errp)
{
    if (!s->cfg_topclocks) {
        error_setg(errp, "%s: topclocks config not defined", __func__);
        return;
    }

    if (!s->cfg_aonclocks) {
        error_setg(errp, "%s: aonclocks config not defined", __func__);
        return;
    }

    char *config = g_strdup(s->cfg_topclocks);
    for (char *clkbrk, *clkdesc = strtok_r(config, CFGSEP, &clkbrk); clkdesc;
         clkdesc = strtok_r(NULL, CFGSEP, &clkbrk)) {
        char clkname[16];
        unsigned clkfreq = 0;
        unsigned length = 0;
        int ret;
        /* NOLINTNEXTLINE(cert-err34-c) */
        ret = sscanf(clkdesc, "%15[a-z0-9_]:%u%n", clkname, &clkfreq, &length);
        if (ret != 2) {
            error_setg(errp, "%s: invalid clock %s format: %d", __func__,
                       clkdesc, ret);
            g_free(config);
            return;
        }
        if (clkdesc[length]) {
            error_setg(errp, "%s: trailing chars in subclock %s", __func__,
                       clkdesc);
            g_free(config);
            return;
        }

        OtASTEgClock *clk = g_new0(OtASTEgClock, 1u);
        clk->name = strdup(clkname);
        clk->frequency = clkfreq;
        clk->irq_name = g_strdup_printf("clock-out-%s", clk->name);
        ibex_qdev_init_irq(OBJECT(s), &clk->out, clk->irq_name);
        s->clocks = g_list_append(s->clocks, clk);
        trace_ot_ast_create_clock(clk->name, clk->frequency, clk->irq_name);
    }
    g_free(config);

    config = g_strdup(s->cfg_aonclocks);
    for (char *clkbrk, *clkname = strtok_r(config, CFGSEP, &clkbrk); clkname;
         clkname = strtok_r(NULL, CFGSEP, &clkbrk)) {
        OtASTEgClock *clk = ot_ast_eg_find_clock(s, clkname);
        if (!clk) {
            error_setg(errp, "%s: invalid AON clock name %s", __func__,
                       clkname);
            return;
        }

        clk->aon = true;
        clk->active = true;
    }
    g_free(config);
}


static bool ot_ast_eg_regs_accepts(void *opaque, hwaddr addr, unsigned size,
                                   bool is_write, MemTxAttrs attrs)
{
    OtASTEgState *s = opaque;
    (void)attrs;
    hwaddr reg = R32_OFF(addr);
    return (reg < s->num_regs_a || (reg >= R_REGB0 && reg <= R_REGB4)) &&
           (!is_write || size == 4u);
}

static uint64_t ot_ast_eg_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    OtASTEgState *s = opaque;
    (void)size;
    uint32_t val32;

    hwaddr reg = R32_OFF(addr);

    if (reg < s->num_regs_a - 1u) {
        val32 = s->regsa[reg];
    } else if (reg == s->num_regs_a - 1u) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s is write-only\n", __func__,
                      REG_NAME(s, reg));
        val32 = 0;
    } else if (reg >= R_REGB0 && reg <= R_REGB4) {
        val32 = s->regsb[reg - R_REGB0];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%03x\n", __func__,
                      (uint32_t)addr);
        val32 = 0;
    }

    uint32_t pc = ibex_get_current_pc();
    trace_ot_ast_io_read_out((uint32_t)addr, REG_NAME(s, reg), val32, pc);

    return (uint64_t)val32;
};

static void ot_ast_eg_regs_write(void *opaque, hwaddr addr, uint64_t val64,
                                 unsigned size)
{
    OtASTEgState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;

    hwaddr reg = R32_OFF(addr);

    uint32_t pc = ibex_get_current_pc();
    trace_ot_ast_io_write((uint32_t)addr, REG_NAME(s, reg), val32, pc);

    if (reg == R_REGA0 || reg == R_REGA1 || reg == R_REGA28) {
        /*
         * ROM startup (sw/device/silicon_creator/rom/rom_start.S) bulk-copies
         * CREATOR_SW_CFG_AST_CFG across the contiguous REGA0..REGAL range via
         * crt_section_copy when AST_INIT_EN is enabled; ignore writes to these
         * read-only registers without logging guest errors.
         */
    } else if (reg < s->num_regs_a) {
        s->regsa[reg] = val32;
    } else if (reg >= R_REGB0 && reg <= R_REGB4) {
        s->regsb[reg - R_REGB0] = val32;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%03x\n", __func__,
                      (uint32_t)addr);
    }
};

static const Property ot_ast_eg_properties[] = {
    DEFINE_PROP_STRING("topclocks", OtASTEgState, cfg_topclocks),
    DEFINE_PROP_STRING("aonclocks", OtASTEgState, cfg_aonclocks),
    DEFINE_PROP_UINT32("num-regs-a", OtASTEgState, num_regs_a, REGSA_V1_COUNT),
};

static const MemoryRegionOps ot_ast_eg_regs_ops = {
    .read = &ot_ast_eg_regs_read,
    .write = &ot_ast_eg_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
    .valid.accepts = &ot_ast_eg_regs_accepts,
};

static void ot_ast_eg_reset_enter(Object *obj, ResetType type)
{
    OtASTEgClass *c = OT_AST_EG_GET_CLASS(obj);
    OtASTEgState *s = OT_AST_EG(obj);

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    memset(s->regsa, 0, REGSA_SIZE);
    memset(s->regsb, 0, REGSB_SIZE);

    for (uint32_t i = 1u; i < s->num_regs_a && i < REGSA_COUNT; i++) {
        s->regsa[i] = i;
    }

    g_list_foreach(s->clocks, ot_ast_eg_reset_clock, s);
}

static void ot_ast_eg_reset_exit(Object *obj, ResetType type)
{
    OtASTEgClass *c = OT_AST_EG_GET_CLASS(obj);
    OtASTEgState *s = OT_AST_EG(obj);

    if (c->parent_phases.exit) {
        c->parent_phases.exit(obj, type);
    }

    g_list_foreach(s->clocks, ot_ast_eg_update_clock, s);
}

static void ot_ast_eg_realize(DeviceState *dev, Error **errp)
{
    OtASTEgState *s = OT_AST_EG(dev);
    (void)errp;

    ot_ast_eg_parse_clocks(s, &error_fatal);
}

static void ot_ast_eg_init(Object *obj)
{
    OtASTEgState *s = OT_AST_EG(obj);

    memory_region_init_io(&s->mmio, obj, &ot_ast_eg_regs_ops, s, TYPE_OT_AST_EG,
                          0x400u);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    s->regsa = g_new0(uint32_t, REGSA_COUNT);
    s->regsb = g_new0(uint32_t, REGSB_COUNT);
}

static void ot_ast_eg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    dc->realize = &ot_ast_eg_realize;
    device_class_set_props(dc, ot_ast_eg_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(klass);
    OtASTEgClass *ac = OT_AST_EG_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_ast_eg_reset_enter, NULL,
                                       &ot_ast_eg_reset_exit,
                                       &ac->parent_phases);

    IbexClockSrcIfClass *ic = IBEX_CLOCK_SRC_IF_CLASS(klass);
    ic->get_clock_source = &ot_ast_eg_get_clock_source;

    OtClockCtrlIfClass *cc = OT_CLOCK_CTRL_IF_CLASS(klass);
    cc->clock_enable = &ot_ast_eg_clock_enable;
    cc->clock_ext_freq_select = &ot_ast_eg_clock_ext_freq_select;

    OtNoiseSrcIfClass *nc = OT_NOISE_SRC_IF_CLASS(klass);
    nc->get_fill_rate = &ot_ast_eg_get_fill_rate;
    nc->get_noise = &ot_ast_eg_get_noise;
}

static const TypeInfo ot_ast_eg_info = {
    .name = TYPE_OT_AST_EG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtASTEgState),
    .instance_init = &ot_ast_eg_init,
    .class_size = sizeof(OtASTEgClass),
    .class_init = &ot_ast_eg_class_init,
    .interfaces =
        (InterfaceInfo[]){
            { TYPE_IBEX_CLOCK_SRC_IF },
            { TYPE_OT_CLOCK_CTRL_IF },
            { TYPE_OT_NOISE_SRC_IF },
            {},
        },
};

static void ot_ast_eg_register_types(void)
{
    type_register_static(&ot_ast_eg_info);
}

type_init(ot_ast_eg_register_types);
