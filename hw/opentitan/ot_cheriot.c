/*
 * QEMU OpenTitan CHERIoT Memory Subsystem
 *
 * Copyright lowRISC contributors.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author(s):
 *  OpenTitan Contributors
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
 * Copyright lowRISC contributors.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_cheriot.h"
#include "hw/opentitan/ot_common.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"

#define REG_ALERT_TEST 0x0
#define REGS_SIZE      0x4

static bool ot_cheriot_regs_accepts(void *opaque, hwaddr addr, unsigned size,
                                    bool is_write, MemTxAttrs attrs)
{
    (void)opaque;
    (void)attrs;
    unsigned reg_be = ((1u << size) - 1u) << (addr & 3u);
    if (addr < REGS_SIZE) {
        if (is_write) {
            if ((0x1u & ~reg_be) != 0) {
                return false; /* Faults */
            }
        }
        return true;
    }
    return false;
}

static uint64_t ot_cheriot_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)size;
    return 0;
}

static void ot_cheriot_regs_write(void *opaque, hwaddr addr, uint64_t val64,
                                  unsigned size)
{
    (void)size;
    OtCheriotState *s = OT_CHERIOT(opaque);
    if (addr == REG_ALERT_TEST && (val64 & 1u)) {
        ibex_irq_set(&s->alert, 1);
        ibex_irq_set(&s->alert, 0);
    }
}

static const MemoryRegionOps ot_cheriot_regs_ops = {
    .read = &ot_cheriot_regs_read,
    .write = &ot_cheriot_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.accepts = &ot_cheriot_regs_accepts,
    .impl.min_access_size = 1u,
    .impl.max_access_size = 4u,
};

static MemTxResult ot_cheriot_revbm_read(
    void *opaque, hwaddr addr, uint64_t *data, unsigned size, MemTxAttrs attrs)
{
    (void)attrs;
    OtCheriotState *s = OT_CHERIOT(opaque);
    if (s->cheriot_ena != OT_MULTIBITBOOL4_TRUE) {
        return MEMTX_ERROR; /* Trap! */
    }
    uint64_t val = 0;
    if (addr + size <= s->revbm_size) {
        memcpy(&val, s->revbm_data + addr, size);
    }
    *data = val;
    return MEMTX_OK;
}

static MemTxResult ot_cheriot_revbm_write(
    void *opaque, hwaddr addr, uint64_t value, unsigned size, MemTxAttrs attrs)
{
    (void)attrs;
    OtCheriotState *s = OT_CHERIOT(opaque);
    if (s->cheriot_ena != OT_MULTIBITBOOL4_TRUE) {
        return MEMTX_ERROR; /* Trap! */
    }
    if (addr + size <= s->revbm_size) {
        memcpy(s->revbm_data + addr, &value, size);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps ot_cheriot_revbm_ops = {
    .read_with_attrs = &ot_cheriot_revbm_read,
    .write_with_attrs = &ot_cheriot_revbm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1u,
    .impl.max_access_size = 4u,
};

static void ot_cheriot_ena_handler(void *opaque, int n, int level)
{
    (void)n;
    OtCheriotState *s = OT_CHERIOT(opaque);
    s->cheriot_ena = level;
}

static void ot_cheriot_reset(DeviceState *dev)
{
    OtCheriotState *s = OT_CHERIOT(dev);
    s->cheriot_ena = OT_MULTIBITBOOL4_FALSE;
    memset(s->revbm_data, 0, s->revbm_size);
}

static void ot_cheriot_init(Object *obj)
{
    OtCheriotState *s = OT_CHERIOT(obj);
    memory_region_init_io(&s->regs, obj, &ot_cheriot_regs_ops, s,
                          TYPE_OT_CHERIOT ".regs", REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->regs);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_cheriot_ena_handler,
                            OT_CHERIOT_ENA, 1);
}

static void ot_cheriot_realize(DeviceState *dev, Error **errp)
{
    (void)errp;
    OtCheriotState *s = OT_CHERIOT(dev);
    s->revbm_data = g_malloc0(s->revbm_size);
    memory_region_init_io(&s->revbm, OBJECT(dev), &ot_cheriot_revbm_ops, s,
                          TYPE_OT_CHERIOT ".revbm", s->revbm_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->revbm);
}

static const Property ot_cheriot_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtCheriotState, ot_id),
    DEFINE_PROP_UINT32("revbm-size", OtCheriotState, revbm_size, 3072u),
};

static void ot_cheriot_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;
    device_class_set_legacy_reset(dc, &ot_cheriot_reset);
    dc->realize = &ot_cheriot_realize;
    device_class_set_props(dc, ot_cheriot_properties);
}

static const TypeInfo ot_cheriot_info = {
    .name = TYPE_OT_CHERIOT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtCheriotState),
    .instance_init = &ot_cheriot_init,
    .class_init = &ot_cheriot_class_init,
};

static void ot_cheriot_register_types(void)
{
    type_register_static(&ot_cheriot_info);
}

type_init(ot_cheriot_register_types);
