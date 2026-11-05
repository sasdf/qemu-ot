/*
 * QEMU OpenTitan Unified Pad Ring Device
 *
 * Copyright (c) 2025-2026 lowRISC contributors.
 *
 * Author(s):
 *  Emmanuel Blot <eblot@rivosinc.com>
 *  Alex Jones <alex.jones@lowrisc.org>
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
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qobject/json-parser.h"
#include "qobject/qjson.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"
#include "qobject/qstring.h"
#include "qobject/qnum.h"
#include "qobject/qbool.h"
#include "chardev/char-fe.h"
#include "hw/opentitan/ot_common.h"
#include "hw/opentitan/ot_pad_ring.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/riscv/ibex_irq.h"
#include "trace.h"

#define OT_PAD_RING_MON_MAX_EVENTS 16384u
#define OT_PAD_RING_BB_MAX_PINS    8u

typedef enum OtPadMode {
    OT_PAD_MODE_INPUT,
    OT_PAD_MODE_PUSH_PULL,
    OT_PAD_MODE_OPEN_DRAIN,
} OtPadMode;

typedef enum OtPadPull {
    OT_PAD_PULL_NONE,
    OT_PAD_PULL_DOWN,
    OT_PAD_PULL_UP,
} OtPadPull;

typedef struct OtPadDef {
    const char *name;
    OtPadPull default_chip_pull;
} OtPadDef;

typedef struct OtPadEndpoint {
    OtPadMode mode;
    OtPadPull pull;
    bool level;
} OtPadEndpoint;

typedef struct OtPadMonSlot {
    bool active;        /* True if actively monitored */
    uint8_t sig_idx;    /* Signal index in active monitoring session */
} OtPadMonSlot;

typedef struct OtPadBbSlot {
    bool active;        /* True if monitored in current bitbang entry */
    GArray *edges;      /* Owned array of OtPadMonEvent recorded for this pad */
} OtPadBbSlot;

typedef struct OtPadSlot {
    OtPadEndpoint chip; /* Derived from OT_PAD_RING_OUT + ATTR (mode, pull, level) */
    OtPadEndpoint host; /* Updated by Proxy JSON; preserved across reset */
    bool invert;        /* OT_PAD_ATTR_INVERT */
    bool input_disable; /* OT_PAD_ATTR_INPUT_DISABLE */
    bool last_eval;     /* Last evaluated wire state for edge detection */
    OtPadMonSlot mon;   /* Passive GPIO monitoring hook */
    OtPadBbSlot bb;     /* Bitbang sampling hook with owned edge buffer */
} OtPadSlot;

typedef struct OtPadMonEvent {
    uint8_t signal_index;
    bool edge;          /* true = Rising/High, false = Falling/Low */
    uint64_t timestamp; /* Virtual timestamp in ns (mon) or tick index (bb) */
} OtPadMonEvent;

typedef enum OtWireStrength {
    OT_WIRE_DRIVE_LOW  = -9, /* Strong 0: beats 2 * DRIVE_HIGH (+6) */
    OT_WIRE_PULL_LOW   = -1, /* Weak 0 */
    OT_WIRE_HIZ        =  0, /* Undriven & unpulled (floating) */
    OT_WIRE_PULL_HIGH  = +1, /* Weak 1 */
    OT_WIRE_DRIVE_HIGH = +3, /* Strong 1: beats 2 * PULL_LOW (-2) */
} OtWireStrength;

typedef enum {
    BB_ENTRY_WRITE,
    BB_ENTRY_BOTH,
    BB_ENTRY_DELAY,
    BB_ENTRY_AWAIT,
} OtPadRingBbType;

typedef struct OtPadRingBbEntry {
    OtPadRingBbType type;
    uint8_t *data;
    size_t data_len;
    uint32_t clock_ticks;
    uint8_t mask;
    uint8_t pattern;
} OtPadRingBbEntry;

struct OtPadRingState {
    DeviceState parent_obj;

    /* Properties */
    char *ot_id;
    uint8_t version;
    CharFrontend chr;

    /* Pad definition & slots */
    const OtPadDef *pad_defs;
    unsigned pad_count;
    OtPadSlot *slots;
    GHashTable *pad_map;

    /* Outbound lines to internal IP / pinmux / rstmgr */
    IbexIRQ *pad_in;

    /* JSON Streaming Parser */
    JSONMessageParser parser;

    /* Floating noise simulation */
    QEMUTimer *float_noise_timer;

    /* Monitoring */
    GArray *mon_events;
    bool mon_overflow;

    /* Analytical Bitbang Engine State */
    QEMUTimer *bb_timer;
    uint64_t bb_clock_ns;
    int64_t bb_entry_start_ns;
    unsigned bb_pins[OT_PAD_RING_BB_MAX_PINS];
    unsigned bb_npins;
    GArray *bb_entries;
    unsigned bb_entry_idx;
    uint32_t bb_tick;
    bool bb_running;
    bool bb_query_pending;
};

struct OtPadRingClass {
    DeviceClass parent_class;
    ResettablePhases parent_phases;
};

static const OtPadDef OT_EG_PAD_DEFS[OT_EG_PAD_COUNT] = {
    [OT_EG_PAD_IOA0]  = { "IOA0",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA1]  = { "IOA1",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA2]  = { "IOA2",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA3]  = { "IOA3",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA4]  = { "IOA4",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA5]  = { "IOA5",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA6]  = { "IOA6",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA7]  = { "IOA7",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOA8]  = { "IOA8",  OT_PAD_PULL_NONE },

    [OT_EG_PAD_IOB0]  = { "IOB0",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB1]  = { "IOB1",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB2]  = { "IOB2",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB3]  = { "IOB3",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB4]  = { "IOB4",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB5]  = { "IOB5",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB6]  = { "IOB6",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB7]  = { "IOB7",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB8]  = { "IOB8",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB9]  = { "IOB9",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB10] = { "IOB10", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB11] = { "IOB11", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOB12] = { "IOB12", OT_PAD_PULL_NONE },

    [OT_EG_PAD_IOC0]  = { "IOC0",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC1]  = { "IOC1",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC2]  = { "IOC2",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC3]  = { "IOC3",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC4]  = { "IOC4",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC5]  = { "IOC5",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC6]  = { "IOC6",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC7]  = { "IOC7",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC8]  = { "IOC8",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC9]  = { "IOC9",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC10] = { "IOC10", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC11] = { "IOC11", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOC12] = { "IOC12", OT_PAD_PULL_NONE },

    [OT_EG_PAD_IOR0]  = { "IOR0",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR1]  = { "IOR1",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR2]  = { "IOR2",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR3]  = { "IOR3",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR4]  = { "IOR4",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR5]  = { "IOR5",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR6]  = { "IOR6",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR7]  = { "IOR7",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR8]  = { "IOR8",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR9]  = { "IOR9",  OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR10] = { "IOR10", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR11] = { "IOR11", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR12] = { "IOR12", OT_PAD_PULL_NONE },
    [OT_EG_PAD_IOR13] = { "IOR13", OT_PAD_PULL_NONE },

    [OT_EG_PAD_POR_N] = { "POR_N", OT_PAD_PULL_UP },
};

static const OtPadDef OT_DJ_PAD_DEFS[OT_DJ_PAD_COUNT] = {
    [OT_DJ_PAD_GPIO0]  = { "GPIO0",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO1]  = { "GPIO1",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO2]  = { "GPIO2",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO3]  = { "GPIO3",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO4]  = { "GPIO4",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO5]  = { "GPIO5",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO6]  = { "GPIO6",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO7]  = { "GPIO7",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO8]  = { "GPIO8",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO9]  = { "GPIO9",  OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO10] = { "GPIO10", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO11] = { "GPIO11", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO12] = { "GPIO12", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO13] = { "GPIO13", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO14] = { "GPIO14", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO15] = { "GPIO15", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO16] = { "GPIO16", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO17] = { "GPIO17", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO18] = { "GPIO18", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO19] = { "GPIO19", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO20] = { "GPIO20", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO21] = { "GPIO21", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO22] = { "GPIO22", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO23] = { "GPIO23", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO24] = { "GPIO24", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO25] = { "GPIO25", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO26] = { "GPIO26", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO27] = { "GPIO27", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO28] = { "GPIO28", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO29] = { "GPIO29", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO30] = { "GPIO30", OT_PAD_PULL_NONE },
    [OT_DJ_PAD_GPIO31] = { "GPIO31", OT_PAD_PULL_NONE },

    [OT_DJ_PAD_POR_N]  = { "POR_N",  OT_PAD_PULL_UP },
};

static void ot_pad_update(OtPadRingState *s, unsigned pad);

static int ot_pad_ring_find_pin(OtPadRingState *s, const char *name)
{
    if (!name) {
        return -1;
    }
    gpointer val = g_hash_table_lookup(s->pad_map, name);
    if (val != NULL) {
        return GPOINTER_TO_INT(val) - 1;
    }
    return -1;
}

static int ot_pad_endpoint_strength(const OtPadEndpoint *ep)
{
    if (ep->mode == OT_PAD_MODE_PUSH_PULL) {
        return ep->level ? OT_WIRE_DRIVE_HIGH : OT_WIRE_DRIVE_LOW;
    }
    if (ep->mode == OT_PAD_MODE_OPEN_DRAIN && !ep->level) {
        return OT_WIRE_DRIVE_LOW;
    }
    if (ep->pull == OT_PAD_PULL_UP) {
        return OT_WIRE_PULL_HIGH;
    }
    if (ep->pull == OT_PAD_PULL_DOWN) {
        return OT_WIRE_PULL_LOW;
    }
    return OT_WIRE_HIZ;
}

static bool ot_pad_eval_wire(const OtPadSlot *slot)
{
    OtPadEndpoint phys_chip = slot->chip;
    phys_chip.level ^= slot->invert;
    int net = ot_pad_endpoint_strength(&phys_chip) * 2 +
              ot_pad_endpoint_strength(&slot->host);
    return (net == 0) ? g_random_boolean() : (net > 0);
}

static bool ot_pad_eval_in(const OtPadSlot *slot)
{
    bool raw = slot->input_disable ? false : ot_pad_eval_wire(slot);
    return raw ^ slot->invert;
}

static void ot_pad_ring_mon_stop(OtPadRingState *s)
{
    for (unsigned i = 0; i < s->pad_count; i++) {
        s->slots[i].mon.active = false;
    }
    g_array_set_size(s->mon_events, 0);
}

static void ot_pad_ring_mon_update(OtPadRingState *s, unsigned pad, bool wire)
{
    OtPadSlot *slot = &s->slots[pad];
    if (s->mon_events->len >= OT_PAD_RING_MON_MAX_EVENTS) {
        s->mon_overflow = true;
        /* Stop edge detection immediately on buffer overrun per trait specification */
        ot_pad_ring_mon_stop(s);
    } else {
        OtPadMonEvent ev = {
            .signal_index = slot->mon.sig_idx,
            .edge = wire, /* true = Rising, false = Falling */
            .timestamp = (uint64_t)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        };
        g_array_append_val(s->mon_events, ev);
    }
}

static void ot_pad_ring_bb_update(OtPadRingState *s, unsigned pad, bool wire);

static void ot_pad_update(OtPadRingState *s, unsigned pad)
{
    OtPadSlot *slot = &s->slots[pad];
    bool wire = ot_pad_eval_wire(slot);

    /* Edge transition detection */
    if (wire != slot->last_eval) {
        slot->last_eval = wire;
        trace_ot_pad_ring_wire_change(s->ot_id, s->pad_defs[pad].name, wire);
        if (slot->mon.active) {
            ot_pad_ring_mon_update(s, pad, wire);
        }
        if (slot->bb.active && s->bb_running) {
            ot_pad_ring_bb_update(s, pad, wire);
        }
    }

    /* Drive resolved input to SoC */
    bool in_val = slot->input_disable ? false : wire;
    in_val ^= slot->invert;
    ibex_irq_set(&s->pad_in[pad], (int)in_val);
}

static void ot_pad_ring_float_noise_timer_cb(void *opaque)
{
    OtPadRingState *s = opaque;
    for (unsigned i = 0; i < s->pad_count; i++) {
        ot_pad_update(s, i);
    }
    timer_mod_ns(s->float_noise_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000LL);
}

static void ot_pad_ring_core_out_handler(void *opaque, int irq, int level)
{
    OtPadRingState *s = opaque;
    if (irq < 0 || irq >= (int)s->pad_count) {
        return;
    }
    OtPadSlot *slot = &s->slots[irq];

    trace_ot_pad_ring_core_out(s->ot_id, s->pad_defs[irq].name, level != 0);

    if (level < 0) {
        slot->chip.mode = OT_PAD_MODE_INPUT;
        slot->chip.level = false;
    } else {
        slot->chip.mode = OT_PAD_MODE_PUSH_PULL;
        slot->chip.level = (level != 0);
    }

    ot_pad_update(s, (unsigned)irq);
}

static void ot_pad_ring_core_attr_handler(void *opaque, int irq, int level)
{
    OtPadRingState *s = opaque;
    if (irq < 0 || irq >= (int)s->pad_count) {
        return;
    }
    OtPadSlot *slot = &s->slots[irq];
    uint32_t attr = (uint32_t)level;

    trace_ot_pad_ring_core_attr(s->ot_id, s->pad_defs[irq].name, attr);

    slot->invert = (attr & OT_PAD_ATTR_INVERT) != 0;
    slot->input_disable = (attr & OT_PAD_ATTR_INPUT_DISABLE) != 0;

    if (attr & OT_PAD_ATTR_PULL_EN) {
        slot->chip.pull = (attr & OT_PAD_ATTR_PULL_SELECT) ?
                          OT_PAD_PULL_UP : OT_PAD_PULL_DOWN;
    } else {
        slot->chip.pull = s->pad_defs[irq].default_chip_pull;
    }

    ot_pad_update(s, (unsigned)irq);
}

static void ot_pad_ring_send_response(OtPadRingState *s, const char *json_str)
{
    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        return;
    }
    gchar *line = g_strdup_printf("%s\n", json_str);
    int len = (int)strlen(line);
    qemu_chr_fe_write_all(&s->chr, (const uint8_t *)line, len);
    g_free(line);
}

static void G_GNUC_PRINTF(2, 3) ot_pad_ring_send_error(OtPadRingState *s, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    gchar *desc = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    gchar *resp = g_strdup_printf(
        "{\"Res\":{\"Err\":{\"description\":\"%s\",\"backtrace\":\"<disabled>\",\"error\":null}}}",
        desc);
    ot_pad_ring_send_response(s, resp);
    g_free(resp);
    g_free(desc);
}

static void ot_pad_ring_bb_clean_entries(OtPadRingState *s)
{
    for (unsigned i = 0; i < s->bb_entries->len; i++) {
        OtPadRingBbEntry *e = &g_array_index(s->bb_entries, OtPadRingBbEntry, i);
        g_free(e->data);
    }
    g_array_set_size(s->bb_entries, 0);
}

static inline uint64_t ot_pad_ring_bb_get_cur_tick(const OtPadRingState *s)
{
    int64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now_ns - s->bb_entry_start_ns;
    return elapsed_ns > 0 ? (uint64_t)elapsed_ns / s->bb_clock_ns : 0;
}

static inline int64_t ot_pad_ring_bb_tick_to_ns(const OtPadRingState *s, uint64_t tick)
{
    return s->bb_entry_start_ns + (int64_t)(tick * s->bb_clock_ns);
}

static inline void ot_pad_ring_bb_record_pad_edge(OtPadBbSlot *bb, bool wire, uint64_t tick)
{
    GArray *edges = bb->edges;

    if (edges->len > 0) {
        OtPadMonEvent *last_ev = &g_array_index(edges, OtPadMonEvent, edges->len - 1);
        if (last_ev->timestamp == tick) {
            last_ev->edge = wire;
            return;
        }
    }

    OtPadMonEvent ev = {
        .signal_index = 0,
        .edge = wire,
        .timestamp = tick,
    };
    g_array_append_val(edges, ev);
}

static void ot_pad_ring_bb_set_monitoring(OtPadRingState *s, bool enable)
{
    for (unsigned b = 0; b < s->bb_npins; b++) {
        unsigned pad = s->bb_pins[b];
        OtPadSlot *slot = &s->slots[pad];
        slot->bb.active = enable;
        if (enable) {
            bool wire = ot_pad_eval_wire(slot);
            slot->last_eval = wire;
            g_array_set_size(slot->bb.edges, 0);
            ot_pad_ring_bb_record_pad_edge(&slot->bb, wire, 0);
        }
    }
}

static void ot_pad_ring_bb_send_query_done(OtPadRingState *s)
{
    GString *resp = g_string_new("{\"Res\":{\"Ok\":{\"GpioBitbanging\":{\"QueryDone\":{\"entries\":[");
    for (unsigned i = 0; i < s->bb_entries->len; i++) {
        OtPadRingBbEntry *e = &g_array_index(s->bb_entries, OtPadRingBbEntry, i);
        if (i > 0) {
            g_string_append_c(resp, ',');
        }
        switch (e->type) {
        case BB_ENTRY_WRITE:
            g_string_append(resp, "\"Write\"");
            break;
        case BB_ENTRY_BOTH:
            g_string_append(resp, "{\"Both\":{\"data\":[");
            for (size_t k = 0; k < e->data_len; k++) {
                if (k > 0) {
                    g_string_append_c(resp, ',');
                }
                g_string_append_printf(resp, "%u", (unsigned)e->data[k]);
            }
            g_string_append(resp, "]}}");
            break;
        case BB_ENTRY_DELAY:
            g_string_append(resp, "\"Delay\"");
            break;
        case BB_ENTRY_AWAIT:
            g_string_append(resp, "\"Await\"");
            break;
        }
    }
    g_string_append(resp, "]}}}}}");
    ot_pad_ring_send_response(s, resp->str);
    g_string_free(resp, TRUE);
}

static void ot_pad_ring_bb_stop(OtPadRingState *s)
{
    timer_del(s->bb_timer);
    s->bb_running = false;
    ot_pad_ring_bb_set_monitoring(s, false);
    if (s->bb_query_pending) {
        s->bb_query_pending = false;
        ot_pad_ring_bb_send_query_done(s);
    }
}

static uint32_t ot_pad_ring_bb_find_next_drive_tick(const OtPadRingBbEntry *e, uint32_t cur_tick)
{
    uint32_t next_tick = cur_tick + 1;
    while (next_tick < e->data_len && e->data[next_tick] == e->data[cur_tick]) {
        next_tick++;
    }
    return next_tick;
}

static void ot_pad_ring_bb_drive_sample(OtPadRingState *s, uint8_t byte)
{
    for (unsigned b = 0; b < s->bb_npins; b++) {
        unsigned pad = s->bb_pins[b];
        bool bit = (byte >> b) & 1;
        bool mode_change = false;
        if (s->slots[pad].host.mode == OT_PAD_MODE_INPUT) {
            s->slots[pad].host.mode = OT_PAD_MODE_PUSH_PULL;
            mode_change = true;
        }
        if (s->slots[pad].host.level != bit || mode_change) {
            s->slots[pad].host.level = bit;
            ot_pad_update(s, pad);
        }
    }
}

static uint8_t ot_pad_ring_bb_sample(const OtPadRingState *s)
{
    uint8_t sample = 0;
    for (unsigned b = 0; b < s->bb_npins; b++) {
        unsigned pad = s->bb_pins[b];
        if (ot_pad_eval_wire(&s->slots[pad])) {
            sample |= (1u << b);
        }
    }
    return sample;
}

static void ot_pad_ring_bb_reconstruct_both(OtPadRingState *s, OtPadRingBbEntry *e)
{
    /* Seeded at t = 0 from edges[0] (recorded at tick 0 when monitoring starts) */
    bool cur_level[OT_PAD_RING_BB_MAX_PINS] = { false };
    size_t ev_idx[OT_PAD_RING_BB_MAX_PINS] = { 0 };

    for (uint32_t t = 0; t < e->data_len; t++) {
        uint8_t sample = 0;
        for (unsigned b = 0; b < s->bb_npins; b++) {
            unsigned pad = s->bb_pins[b];
            GArray *edges = s->slots[pad].bb.edges;
            if (ev_idx[b] < edges->len) {
                OtPadMonEvent *ev = &g_array_index(edges, OtPadMonEvent, ev_idx[b]);
                if (ev->timestamp == t) {
                    cur_level[b] = ev->edge;
                    ev_idx[b]++;
                }
            }
            if (cur_level[b]) {
                sample |= (1u << b);
            }
        }
        e->data[t] = sample;
    }
}

static void ot_pad_ring_bb_start_entry(OtPadRingState *s);

static void ot_pad_ring_bb_advance_entry(OtPadRingState *s)
{
    OtPadRingBbEntry *cur_e = &g_array_index(s->bb_entries, OtPadRingBbEntry, s->bb_entry_idx);
    if (cur_e->type == BB_ENTRY_BOTH) {
        ot_pad_ring_bb_reconstruct_both(s, cur_e);
    }

    s->bb_entry_idx++;

    if (s->bb_entry_idx >= s->bb_entries->len) {
        ot_pad_ring_bb_stop(s);
    } else {
        ot_pad_ring_bb_start_entry(s);
    }
}

static void ot_pad_ring_bb_update(OtPadRingState *s, unsigned pad, bool wire)
{
    OtPadSlot *slot = &s->slots[pad];
    OtPadRingBbEntry *e = &g_array_index(s->bb_entries, OtPadRingBbEntry, s->bb_entry_idx);

    if (e->type == BB_ENTRY_BOTH) {
        uint64_t tick = ot_pad_ring_bb_get_cur_tick(s);
        if (tick < e->data_len) {
            ot_pad_ring_bb_record_pad_edge(&slot->bb, wire, tick);
        }
    } else if (e->type == BB_ENTRY_AWAIT) {
        if ((ot_pad_ring_bb_sample(s) & e->mask) == e->pattern) {
            ot_pad_ring_bb_advance_entry(s);
        }
    }
}

static void ot_pad_ring_bb_step(OtPadRingState *s)
{
    if (!s->bb_running) {
        return;
    }

    OtPadRingBbEntry *e = &g_array_index(s->bb_entries, OtPadRingBbEntry, s->bb_entry_idx);
    s->bb_entry_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                           (int64_t)(s->bb_tick * s->bb_clock_ns);

    switch (e->type) {
    case BB_ENTRY_WRITE:
    case BB_ENTRY_BOTH:
        if (s->bb_tick < e->data_len) {
            ot_pad_ring_bb_drive_sample(s, e->data[s->bb_tick]);
            s->bb_tick = ot_pad_ring_bb_find_next_drive_tick(e, s->bb_tick);
            timer_mod_ns(s->bb_timer, ot_pad_ring_bb_tick_to_ns(s, s->bb_tick));
        } else {
            ot_pad_ring_bb_advance_entry(s);
        }
        break;

    case BB_ENTRY_DELAY:
        if (s->bb_tick < e->clock_ticks) {
            s->bb_tick = e->clock_ticks;
            timer_mod_ns(s->bb_timer, ot_pad_ring_bb_tick_to_ns(s, s->bb_tick));
        } else {
            ot_pad_ring_bb_advance_entry(s);
        }
        break;

    case BB_ENTRY_AWAIT:
        timer_del(s->bb_timer);
        if ((ot_pad_ring_bb_sample(s) & e->mask) == e->pattern) {
            ot_pad_ring_bb_advance_entry(s);
        }
        break;
    }
}

static void ot_pad_ring_bb_start_entry(OtPadRingState *s)
{
    OtPadRingBbEntry *e = &g_array_index(s->bb_entries, OtPadRingBbEntry, s->bb_entry_idx);
    s->bb_tick = 0;
    ot_pad_ring_bb_set_monitoring(s, e->type == BB_ENTRY_BOTH || e->type == BB_ENTRY_AWAIT);
    ot_pad_ring_bb_step(s);
}

static void ot_pad_ring_bb_timer_cb(void *opaque)
{
    OtPadRingState *s = opaque;
    ot_pad_ring_bb_step(s);
}

static void ot_pad_ring_handle_gpio(OtPadRingState *s, QDict *gpio_dict)
{
    if (!gpio_dict) {
        ot_pad_ring_send_error(s, "Malformed Gpio request");
        return;
    }
    const char *id = qdict_get_try_str(gpio_dict, "id");
    int pin_idx = ot_pad_ring_find_pin(s, id);
    if (pin_idx < 0) {
        ot_pad_ring_send_error(s, "Unknown pin '%s'", id ? id : "<null>");
        return;
    }
    QObject *cmd_obj = qdict_get(gpio_dict, "command");
    if (!cmd_obj) {
        ot_pad_ring_send_error(s, "Missing Gpio command");
        return;
    }
    if (qobject_type(cmd_obj) == QTYPE_QSTRING) {
        const char *cmd_str = qstring_get_str(qobject_to(QString, cmd_obj));
        if (strcmp(cmd_str, "Read") == 0) {
            bool val = ot_pad_eval_wire(&s->slots[pin_idx]);
            gchar *resp = g_strdup_printf("{\"Res\":{\"Ok\":{\"Gpio\":{\"Read\":{\"value\":%s}}}}}",
                                          val ? "true" : "false");
            ot_pad_ring_send_response(s, resp);
            g_free(resp);
            return;
        }
    }
    if (qobject_type(cmd_obj) == QTYPE_QDICT) {
        QDict *cmd_dict = qobject_to(QDict, cmd_obj);
        if (qdict_haskey(cmd_dict, "Write")) {
            QDict *w = qdict_get_qdict(cmd_dict, "Write");
            bool logic = qdict_get_bool(w, "logic");
            s->slots[pin_idx].host.level = logic;
            if (s->slots[pin_idx].host.mode == OT_PAD_MODE_INPUT) {
                s->slots[pin_idx].host.mode = OT_PAD_MODE_PUSH_PULL;
            }
            ot_pad_update(s, (unsigned)pin_idx);
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Gpio\":\"Write\"}}}");
            return;
        }
        if (qdict_haskey(cmd_dict, "SetMode")) {
            QDict *m = qdict_get_qdict(cmd_dict, "SetMode");
            const char *mode = qdict_get_try_str(m, "mode");
            if (g_strcmp0(mode, "Input") == 0) {
                s->slots[pin_idx].host.mode = OT_PAD_MODE_INPUT;
            } else if (g_strcmp0(mode, "PushPull") == 0) {
                s->slots[pin_idx].host.mode = OT_PAD_MODE_PUSH_PULL;
            } else if (g_strcmp0(mode, "OpenDrain") == 0) {
                s->slots[pin_idx].host.mode = OT_PAD_MODE_OPEN_DRAIN;
            }
            ot_pad_update(s, (unsigned)pin_idx);
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Gpio\":\"SetMode\"}}}");
            return;
        }
        if (qdict_haskey(cmd_dict, "SetPullMode")) {
            QDict *p = qdict_get_qdict(cmd_dict, "SetPullMode");
            const char *pull = qdict_get_try_str(p, "pull");
            if (g_strcmp0(pull, "PullUp") == 0) {
                s->slots[pin_idx].host.pull = OT_PAD_PULL_UP;
            } else if (g_strcmp0(pull, "PullDown") == 0) {
                s->slots[pin_idx].host.pull = OT_PAD_PULL_DOWN;
            } else if (g_strcmp0(pull, "None") == 0) {
                s->slots[pin_idx].host.pull = OT_PAD_PULL_NONE;
            }
            ot_pad_update(s, (unsigned)pin_idx);
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Gpio\":\"SetPullMode\"}}}");
            return;
        }
        if (qdict_haskey(cmd_dict, "MultiSet")) {
            QDict *ms = qdict_get_qdict(cmd_dict, "MultiSet");
            const char *mode = qdict_get_try_str(ms, "mode");
            const char *pull = qdict_get_try_str(ms, "pull");
            QObject *val_obj = qdict_get(ms, "value");

            if (mode) {
                if (g_strcmp0(mode, "Input") == 0) {
                    s->slots[pin_idx].host.mode = OT_PAD_MODE_INPUT;
                } else if (g_strcmp0(mode, "PushPull") == 0) {
                    s->slots[pin_idx].host.mode = OT_PAD_MODE_PUSH_PULL;
                } else if (g_strcmp0(mode, "OpenDrain") == 0) {
                    s->slots[pin_idx].host.mode = OT_PAD_MODE_OPEN_DRAIN;
                }
            } else if (val_obj &&
                       s->slots[pin_idx].host.mode == OT_PAD_MODE_INPUT) {
                s->slots[pin_idx].host.mode = OT_PAD_MODE_PUSH_PULL;
            }
            if (pull) {
                if (g_strcmp0(pull, "PullUp") == 0) {
                    s->slots[pin_idx].host.pull = OT_PAD_PULL_UP;
                } else if (g_strcmp0(pull, "PullDown") == 0) {
                    s->slots[pin_idx].host.pull = OT_PAD_PULL_DOWN;
                } else if (g_strcmp0(pull, "None") == 0) {
                    s->slots[pin_idx].host.pull = OT_PAD_PULL_NONE;
                }
            }
            if (val_obj && qobject_type(val_obj) == QTYPE_QBOOL) {
                s->slots[pin_idx].host.level = qbool_get_bool(qobject_to(QBool, val_obj));
            }
            ot_pad_update(s, (unsigned)pin_idx);
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Gpio\":\"MultiSet\"}}}");
            return;
        }
    }
    ot_pad_ring_send_error(s, "Unknown Gpio command");
}

static void ot_pad_ring_handle_gpio_monitoring(OtPadRingState *s, QDict *mon_dict)
{
    if (!mon_dict) {
        ot_pad_ring_send_error(s, "Malformed GpioMonitoring request");
        return;
    }
    QObject *cmd_obj = qdict_get(mon_dict, "command");
    if (!cmd_obj) {
        ot_pad_ring_send_error(s, "Missing GpioMonitoring command");
        return;
    }
    if (qobject_type(cmd_obj) == QTYPE_QSTRING) {
        const char *cmd_str = qstring_get_str(qobject_to(QString, cmd_obj));
        if (strcmp(cmd_str, "GetClockNature") == 0) {
            ot_pad_ring_send_response(s,
                "{\"Res\":{\"Ok\":{\"GpioMonitoring\":{\"GetClockNature\":{\"resp\":{\"Wallclock\":{\"resolution\":1000000000,\"offset\":null}}}}}}}");
            return;
        }
    }
    if (qobject_type(cmd_obj) == QTYPE_QDICT) {
        QDict *cmd_dict = qobject_to(QDict, cmd_obj);
        if (qdict_haskey(cmd_dict, "Start")) {
            QDict *st = qdict_get_qdict(cmd_dict, "Start");
            QList *pins = qdict_get_qlist(st, "pins");
            if (!pins) {
                ot_pad_ring_send_error(s, "Missing pins array in Monitoring Start");
                return;
            }

            /* Unconditionally clear overflow and reset previous monitor pins */
            s->mon_overflow = false;
            ot_pad_ring_mon_stop(s);

            GString *init_lvls = g_string_new("");
            const QListEntry *entry;
            unsigned idx = 0;
            QLIST_FOREACH_ENTRY(pins, entry) {
                const char *pname = qstring_get_str(qobject_to(QString, qlist_entry_obj(entry)));
                int pidx = ot_pad_ring_find_pin(s, pname);
                if (pidx < 0) {
                    g_string_free(init_lvls, TRUE);
                    ot_pad_ring_send_error(s, "Unknown monitoring pin '%s'", pname);
                    return;
                }
                s->slots[pidx].mon.active = true;
                s->slots[pidx].mon.sig_idx = (uint8_t)idx++;
                bool wire = ot_pad_eval_wire(&s->slots[pidx]);
                s->slots[pidx].last_eval = wire;
                if (init_lvls->len > 0) {
                    g_string_append_c(init_lvls, ',');
                }
                g_string_append(init_lvls, wire ? "true" : "false");
            }
            gchar *resp = g_strdup_printf(
                "{\"Res\":{\"Ok\":{\"GpioMonitoring\":{\"Start\":{\"resp\":{\"initial_levels\":[%s]}}}}}}",
                init_lvls->str);
            ot_pad_ring_send_response(s, resp);
            g_free(resp);
            g_string_free(init_lvls, TRUE);
            return;
        }
        if (qdict_haskey(cmd_dict, "Read")) {
            if (s->mon_overflow) {
                ot_pad_ring_send_error(s, "Buffer overrun");
                return;
            }

            QDict *rd = qdict_get_qdict(cmd_dict, "Read");
            bool cont = qdict_get_try_bool(rd, "continue_monitoring", true);
            int64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            GString *ev_str = g_string_new("");
            for (unsigned i = 0; i < s->mon_events->len; i++) {
                OtPadMonEvent *ev = &g_array_index(s->mon_events, OtPadMonEvent, i);
                if (ev_str->len > 0) {
                    g_string_append_c(ev_str, ',');
                }
                g_string_append_printf(ev_str,
                    "{\"signal_index\":%u,\"edge\":\"%s\",\"timestamp\":%" PRIu64 "}",
                    (unsigned)ev->signal_index, ev->edge ? "Rising" : "Falling", ev->timestamp);
            }
            gchar *resp = g_strdup_printf(
                "{\"Res\":{\"Ok\":{\"GpioMonitoring\":{\"Read\":{\"resp\":{\"events\":[%s],\"timestamp\":%" PRId64 "}}}}}}",
                ev_str->str, now_ns);
            ot_pad_ring_send_response(s, resp);
            g_free(resp);
            g_string_free(ev_str, TRUE);

            if (!cont) {
                ot_pad_ring_mon_stop(s);
            } else {
                g_array_set_size(s->mon_events, 0);
            }
            return;
        }
    }
    ot_pad_ring_send_error(s, "Unknown GpioMonitoring command");
}

static void ot_pad_ring_bb_query(OtPadRingState *s)
{
    if (s->bb_running) {
        s->bb_query_pending = true;
        return;
    }

    ot_pad_ring_bb_send_query_done(s);
}

static void ot_pad_ring_handle_gpio_bitbanging(OtPadRingState *s, QDict *bb_dict)
{
    if (!bb_dict) {
        ot_pad_ring_send_error(s, "Malformed GpioBitbanging request");
        return;
    }
    QObject *cmd_obj = qdict_get(bb_dict, "command");
    if (!cmd_obj) {
        ot_pad_ring_send_error(s, "Missing GpioBitbanging command");
        return;
    }
    if (qobject_type(cmd_obj) == QTYPE_QSTRING) {
        const char *cmd_str = qstring_get_str(qobject_to(QString, cmd_obj));
        if (strcmp(cmd_str, "Query") == 0) {
            ot_pad_ring_bb_query(s);
            return;
        }
    }
    if (qobject_type(cmd_obj) == QTYPE_QDICT) {
        QDict *cmd_dict = qobject_to(QDict, cmd_obj);
        if (qdict_haskey(cmd_dict, "Start")) {
            QDict *st = qdict_get_qdict(cmd_dict, "Start");
            QList *pins = qdict_get_qlist(st, "pins");
            uint64_t clock_ns = (uint64_t)qdict_get_try_int(st, "clock_ns", 1000);
            QList *entries = qdict_get_qlist(st, "entries");
            if (!pins || !entries) {
                ot_pad_ring_send_error(s, "Missing pins or entries in Bitbang Start");
                return;
            }
            if (clock_ns == 0) {
                ot_pad_ring_send_error(s, "Invalid bitbang clock_ns 0");
                return;
            }

            ot_pad_ring_bb_stop(s);
            ot_pad_ring_bb_clean_entries(s);

            s->bb_clock_ns = clock_ns;
            s->bb_npins = 0;
            const QListEntry *pentry;
            QLIST_FOREACH_ENTRY(pins, pentry) {
                if (s->bb_npins >= OT_PAD_RING_BB_MAX_PINS) {
                    ot_pad_ring_send_error(s, "Too many bitbang pins (max %u)",
                                           OT_PAD_RING_BB_MAX_PINS);
                    return;
                }
                const char *pname = qstring_get_str(qobject_to(QString, qlist_entry_obj(pentry)));
                int pidx = ot_pad_ring_find_pin(s, pname);
                if (pidx < 0) {
                    ot_pad_ring_send_error(s, "Unknown bitbang pin '%s'", pname);
                    return;
                }
                s->bb_pins[s->bb_npins++] = (unsigned)pidx;
            }

            const QListEntry *eentry;
            QLIST_FOREACH_ENTRY(entries, eentry) {
                QDict *ed = qobject_to(QDict, qlist_entry_obj(eentry));
                OtPadRingBbEntry item = { 0 };
                if (qdict_haskey(ed, "Write")) {
                    QDict *w = qdict_get_qdict(ed, "Write");
                    QList *dl = qdict_get_qlist(w, "data");
                    item.type = BB_ENTRY_WRITE;
                    item.data_len = qlist_empty(dl) ? 0 : qlist_size(dl);
                    item.data = g_new0(uint8_t, item.data_len);
                    const QListEntry *de;
                    size_t di = 0;
                    QLIST_FOREACH_ENTRY(dl, de) {
                        item.data[di++] = (uint8_t)qnum_get_uint(qobject_to(QNum, qlist_entry_obj(de)));
                    }
                } else if (qdict_haskey(ed, "Both")) {
                    QDict *b = qdict_get_qdict(ed, "Both");
                    QList *dl = qdict_get_qlist(b, "data");
                    item.type = BB_ENTRY_BOTH;
                    item.data_len = qlist_empty(dl) ? 0 : qlist_size(dl);
                    item.data = g_new0(uint8_t, item.data_len);
                    const QListEntry *de;
                    size_t di = 0;
                    QLIST_FOREACH_ENTRY(dl, de) {
                        item.data[di++] = (uint8_t)qnum_get_uint(qobject_to(QNum, qlist_entry_obj(de)));
                    }
                } else if (qdict_haskey(ed, "Delay")) {
                    QDict *d = qdict_get_qdict(ed, "Delay");
                    item.type = BB_ENTRY_DELAY;
                    item.clock_ticks = (uint32_t)qdict_get_int(d, "clock_ticks");
                } else if (qdict_haskey(ed, "Await")) {
                    QDict *a = qdict_get_qdict(ed, "Await");
                    item.type = BB_ENTRY_AWAIT;
                    item.mask = (uint8_t)qdict_get_int(a, "mask");
                    item.pattern = (uint8_t)qdict_get_int(a, "pattern");
                }
                g_array_append_val(s->bb_entries, item);
            }

            if (s->bb_entries->len > 0) {
                s->bb_entry_idx = 0;
                s->bb_running = true;
                ot_pad_ring_bb_start_entry(s);
            }
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"GpioBitbanging\":\"Start\"}}}");
            return;
        }
    }
    ot_pad_ring_send_error(s, "Unknown GpioBitbanging command");
}

static void ot_pad_ring_handle_proxy(OtPadRingState *s, QObject *proxy_obj)
{
    if (qobject_type(proxy_obj) == QTYPE_QSTRING) {
        const char *str = qstring_get_str(qobject_to(QString, proxy_obj));
        if (strcmp(str, "Provides") == 0) {
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Proxy\":{\"Provides\":{\"provides_map\":{}}}}}}");
            return;
        }
    }
    ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":{\"Proxy\":\"Bootstrap\"}}}");
}

static void ot_pad_ring_json_emit(void *opaque, QObject *obj, Error *err)
{
    OtPadRingState *s = opaque;

    if (err) {
        error_free(err);
        return;
    }
    if (!obj) {
        return;
    }

    QDict *root = qobject_to(QDict, obj);
    if (!root) {
        qobject_unref(obj);
        return;
    }

    QObject *req_obj = qdict_get(root, "Req");
    if (!req_obj) {
        ot_pad_ring_send_error(s, "Missing Req in request");
        qobject_unref(obj);
        return;
    }

    if (qobject_type(req_obj) == QTYPE_QSTRING) {
        const char *cmd = qstring_get_str(qobject_to(QString, req_obj));
        trace_ot_pad_ring_cmd(s->ot_id, cmd);
        if (strcmp(cmd, "GetCapabilities") == 0) {
            ot_pad_ring_send_response(s,
                "{\"Res\":{\"Ok\":{\"GetCapabilities\":{\"capabilities\":[\"GPIO\",\"GPIO_MONITORING\",\"GPIO_BITBANGING\"]}}}}");
        } else if (strcmp(cmd, "ApplyDefaultConfiguration") == 0) {
            ot_pad_ring_send_response(s, "{\"Res\":{\"Ok\":\"ApplyDefaultConfiguration\"}}");
        } else {
            ot_pad_ring_send_error(s, "Unsupported scalar command %s", cmd);
        }
        qobject_unref(obj);
        return;
    }

    if (qobject_type(req_obj) == QTYPE_QDICT) {
        QDict *req_dict = qobject_to(QDict, req_obj);
        if (qdict_haskey(req_dict, "Gpio")) {
            ot_pad_ring_handle_gpio(s, qdict_get_qdict(req_dict, "Gpio"));
        } else if (qdict_haskey(req_dict, "GpioMonitoring")) {
            ot_pad_ring_handle_gpio_monitoring(s, qdict_get_qdict(req_dict, "GpioMonitoring"));
        } else if (qdict_haskey(req_dict, "GpioBitbanging")) {
            ot_pad_ring_handle_gpio_bitbanging(s, qdict_get_qdict(req_dict, "GpioBitbanging"));
        } else if (qdict_haskey(req_dict, "Proxy")) {
            ot_pad_ring_handle_proxy(s, qdict_get(req_dict, "Proxy"));
        } else {
            ot_pad_ring_send_error(s, "Unsupported request category");
        }
        qobject_unref(obj);
        return;
    }

    ot_pad_ring_send_error(s, "Invalid Req type");
    qobject_unref(obj);
}

static int ot_pad_ring_chr_can_receive(void *opaque)
{
    return 1024;
}

static void ot_pad_ring_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    OtPadRingState *s = opaque;
    json_message_parser_feed(&s->parser, (const char *)buf, size);
}

static void ot_pad_ring_chr_event(void *opaque, QEMUChrEvent event)
{
    OtPadRingState *s = opaque;
    switch (event) {
    case CHR_EVENT_OPENED:
        break;
    case CHR_EVENT_CLOSED:
        json_message_parser_destroy(&s->parser);
        json_message_parser_init(&s->parser, ot_pad_ring_json_emit, s, NULL);
        s->bb_query_pending = false;
        ot_pad_ring_bb_stop(s);
        ot_pad_ring_bb_clean_entries(s);
        ot_pad_ring_mon_stop(s);
        s->mon_overflow = false;
        break;
    default:
        break;
    }
}

static void ot_pad_ring_reset_enter(Object *obj, ResetType type)
{
    OtPadRingClass *c = OT_PAD_RING_GET_CLASS(obj);
    OtPadRingState *s = OT_PAD_RING(obj);

    trace_ot_pad_ring_reset(s->ot_id, "enter");

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    /* Reset chip-side endpoint state; preserve host-side configurations and monitoring */
    for (unsigned i = 0; i < s->pad_count; i++) {
        s->slots[i].chip.mode = OT_PAD_MODE_INPUT;
        s->slots[i].chip.pull = s->pad_defs[i].default_chip_pull;
        s->slots[i].chip.level = false;
        s->slots[i].invert = false;
        s->slots[i].input_disable = false;
        s->slots[i].last_eval = false;
        s->pad_in[i].level = -1;
    }
}

static void ot_pad_ring_reset_hold(Object *obj, ResetType type)
{
    OtPadRingClass *c = OT_PAD_RING_GET_CLASS(obj);
    OtPadRingState *s = OT_PAD_RING(obj);

    trace_ot_pad_ring_reset(s->ot_id, "hold");

    if (c->parent_phases.hold) {
        c->parent_phases.hold(obj, type);
    }

    for (unsigned i = 0; i < s->pad_count; i++) {
        ibex_irq_set(&s->pad_in[i], (int)ot_pad_eval_in(&s->slots[i]));
    }
}

static void ot_pad_ring_realize(DeviceState *dev, Error **errp)
{
    OtPadRingState *s = OT_PAD_RING(dev);
    (void)errp;

    g_assert(s->version < OT_PAD_RING_VERSION_COUNT);

    if (s->version == OT_PAD_RING_VERSION_EG) {
        s->pad_defs = OT_EG_PAD_DEFS;
        s->pad_count = OT_EG_PAD_COUNT;
    } else {
        s->pad_defs = OT_DJ_PAD_DEFS;
        s->pad_count = OT_DJ_PAD_COUNT;
    }

    s->pad_map = g_hash_table_new(g_str_hash, g_str_equal);
    for (unsigned i = 0; i < s->pad_count; i++) {
        g_hash_table_insert(s->pad_map, (gpointer)s->pad_defs[i].name,
                            GINT_TO_POINTER((int)i + 1));
    }

    s->slots = g_new0(OtPadSlot, s->pad_count);
    for (unsigned i = 0; i < s->pad_count; i++) {
        s->slots[i].chip.mode = OT_PAD_MODE_INPUT;
        s->slots[i].chip.pull = s->pad_defs[i].default_chip_pull;
        s->slots[i].host.mode = OT_PAD_MODE_INPUT;
        s->slots[i].host.pull = OT_PAD_PULL_NONE;
        s->slots[i].last_eval = ot_pad_eval_wire(&s->slots[i]);
        s->slots[i].bb.edges = g_array_new(FALSE, FALSE, sizeof(OtPadMonEvent));
    }

    qdev_init_gpio_in_named(dev, &ot_pad_ring_core_out_handler,
                            OT_PAD_RING_OUT, s->pad_count);
    qdev_init_gpio_in_named(dev, &ot_pad_ring_core_attr_handler,
                            OT_PAD_RING_ATTR, s->pad_count);

    s->pad_in = g_new0(IbexIRQ, s->pad_count);
    ibex_qdev_init_irqs_default(OBJECT(dev), s->pad_in,
                                OT_PAD_RING_IN, s->pad_count, -1);

    json_message_parser_init(&s->parser, ot_pad_ring_json_emit, s, NULL);

    s->float_noise_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, ot_pad_ring_float_noise_timer_cb, s);
    s->bb_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, ot_pad_ring_bb_timer_cb, s);
    s->mon_events = g_array_new(FALSE, FALSE, sizeof(OtPadMonEvent));
    s->bb_entries = g_array_new(FALSE, FALSE, sizeof(OtPadRingBbEntry));

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, ot_pad_ring_chr_can_receive,
                                 ot_pad_ring_chr_receive,
                                 ot_pad_ring_chr_event,
                                 NULL, s, NULL, true);
    }

    timer_mod_ns(s->float_noise_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000LL);
}

static void ot_pad_ring_unrealize(DeviceState *dev)
{
    OtPadRingState *s = OT_PAD_RING(dev);

    json_message_parser_destroy(&s->parser);
    timer_free(s->float_noise_timer);
    timer_free(s->bb_timer);
    ot_pad_ring_bb_clean_entries(s);
    g_array_free(s->bb_entries, TRUE);
    g_array_free(s->mon_events, TRUE);
    for (unsigned i = 0; i < s->pad_count; i++) {
        g_array_free(s->slots[i].bb.edges, TRUE);
    }
    g_hash_table_destroy(s->pad_map);
    g_free(s->slots);
    g_free(s->pad_in);
}

static const Property ot_pad_ring_properties[] = {
    DEFINE_PROP_STRING(OT_COMMON_DEV_ID, OtPadRingState, ot_id),
    DEFINE_PROP_UINT8("version", OtPadRingState, version, OT_PAD_RING_VERSION_EG),
    DEFINE_PROP_CHR("chardev", OtPadRingState, chr),
};

static void ot_pad_ring_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    dc->realize = &ot_pad_ring_realize;
    dc->unrealize = &ot_pad_ring_unrealize;
    device_class_set_props(dc, ot_pad_ring_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(dc);
    OtPadRingClass *pc = OT_PAD_RING_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_pad_ring_reset_enter,
                                       &ot_pad_ring_reset_hold,
                                       NULL,
                                       &pc->parent_phases);
}

static const TypeInfo ot_pad_ring_info = {
    .name = TYPE_OT_PAD_RING,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(OtPadRingState),
    .class_init = &ot_pad_ring_class_init,
    .class_size = sizeof(OtPadRingClass)
};

static void ot_pad_ring_register_types(void)
{
    type_register_static(&ot_pad_ring_info);
}

type_init(ot_pad_ring_register_types);
