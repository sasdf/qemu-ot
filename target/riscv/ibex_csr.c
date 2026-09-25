/*
 * QEMU LowRisc Ibex core features
 *
 * Copyright (c) 2023-2024 Rivos, Inc.
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
/* NOLINTBEGIN(misc-header-include-cycle) */
#include "cpu.h"
/* NOLINTEND(misc-header-include-cycle) */

#if !defined(CONFIG_USER_ONLY)

/* Custom CSRs */
#define CSR_CPUCTRLSTS 0x7c0
#define CSR_SECURESEED 0x7c1

#define CPUCTRLSTS_ICACHE_ENABLE     0x001
#define CPUCTRLSTS_DATA_IND_TIMING   0x002
#define CPUCTRLSTS_DUMMY_INSTR_EN    0x004
#define CPUCTRLSTS_DUMMY_INSTR_MASK  0x038
#define CPUCTRLSTS_SYNC_EXC_SEEN     0x040
#define CPUCTRLSTS_DOUBLE_FAULT_SEEN 0x080
#define CPUCTRLSTS_IC_SCR_KEY_VALID  0x100

static RISCVException read_cpuctrlsts(CPURISCVState *env, int csrno,
                                      target_ulong *val)
{
    (void)csrno;
    *val = (env->ic_scr_key_inval_cnt ? 0 : CPUCTRLSTS_IC_SCR_KEY_VALID) |
           env->cpuctrlsts;
    env->ic_scr_key_inval_cnt = 0;
    return RISCV_EXCP_NONE;
}

static RISCVException write_cpuctrlsts(CPURISCVState *env, int csrno,
                                       target_ulong val, uintptr_t ra)
{
    (void)csrno;
    (void)ra;
    /*
     * In ibex_cs_registers.sv (lines 701-704, 1656-1657 in v1; 874-877,
     * 1967-1968 in v2), cpuctrlsts_part_q[7:0] (including double_fault_seen at
     * bit 7 and sync_exc_seen at bit 6) is directly written from
     * csr_wdata_int[7:0], while bit 8 (ic_scr_key_valid) is read-only.
     */
    env->cpuctrlsts = val & 0xffu;
    env->sync_exc_seen = (val & CPUCTRLSTS_SYNC_EXC_SEEN) != 0;
    return RISCV_EXCP_NONE;
}

static RISCVException read_secureseed(CPURISCVState *env, int csrno,
                                      target_ulong *val)
{
    (void)env;
    (void)csrno;
    /*
     * "Seed values are not actually stored in a register and so reads to this
     * register will always return zero."
     */
    *val = 0;
    return RISCV_EXCP_NONE;
}

static RISCVException write_secureseed(CPURISCVState *env, int csrno,
                                       target_ulong val, uintptr_t ra)
{
    (void)env;
    (void)csrno;
    (void)val;
    (void)ra;
    return RISCV_EXCP_NONE;
}

static RISCVException any(CPURISCVState *env, int csrno)
{
    (void)env;
    (void)csrno;
    /*
     *  unfortunately, this predicate is not public, so duplicate the standard
     *  implementation
     */
    return RISCV_EXCP_NONE;
}

static RISCVException read_mtvec(CPURISCVState *env, int csrno,
                                 target_ulong *val)
{
    (void)csrno;
    *val = env->mtvec;

    return RISCV_EXCP_NONE;
}

static RISCVException write_mtvec(CPURISCVState *env, int csrno,
                                  target_ulong val, uintptr_t ra)
{
    (void)csrno;
    (void)ra;
    /* bits [1:0] encode mode; Ibex only supports 1 = vectored */
    if ((val & 3u) != 1u) {
        qemu_log_mask(LOG_UNIMP,
                      "CSR_MTVEC: reserved mode not supported 0x" TARGET_FMT_lx
                      "\n",
                      val);
        /* WARL, Ibex will tie any invalid mode writes to 0b01 (vectored) */
        val &= ~3u;
        val |= 1u;
    }

    /* bits [7:2] are always 0, address should be aligned in 256 bytes */
    env->mtvec = val & ~0xFCu;

    return RISCV_EXCP_NONE;
}

void riscv_cpu_set_mcounteren_writable(CPUState *cs, bool writable);
void riscv_cpu_set_mcounteren_writable(CPUState *cs, bool writable)
{
    if (cs) {
        RISCV_CPU(cs)->env.mcounteren_writable_disabled = !writable;
    }
}

static RISCVException read_mcounteren(CPURISCVState *env, int csrno,
                                      target_ulong *val)
{
    (void)csrno;
    *val = env->mcounteren;
    return RISCV_EXCP_NONE;
}

static RISCVException write_mcounteren(CPURISCVState *env, int csrno,
                                       target_ulong val, uintptr_t ra)
{
    (void)csrno;
    (void)ra;
    if (env->mcounteren_writable_disabled) {
        return RISCV_EXCP_NONE;
    }
    /*
     * Earlgrey top_earlgrey.sv sets RvCoreIbexMHPMCounterNum = 2,
     * giving mcounteren WARL mask 0x0000001d (CY, IR, HPM3, HPM4; TM=0).
     */
    env->mcounteren = val & 0x1du;
    return RISCV_EXCP_NONE;
}

#endif /* !defined(CONFIG_USER_ONLY) */

const RISCVCSR ibex_csr_list[] = {
#if !defined(CONFIG_USER_ONLY)
    {
        .csrno = CSR_MTVEC,
        .csr_ops = { "mtvec", any, &read_mtvec, &write_mtvec },
    },
    {
        .csrno = CSR_MCOUNTEREN,
        .csr_ops = { "mcounteren", any, &read_mcounteren, &write_mcounteren },
    },
    {
        .csrno = CSR_CPUCTRLSTS,
        .csr_ops = { "cpuctrlsts", any, &read_cpuctrlsts, &write_cpuctrlsts },
    },
    {
        .csrno = CSR_SECURESEED,
        .csr_ops = { "secureseed", any, &read_secureseed, &write_secureseed },
    },
#endif /* !defined(CONFIG_USER_ONLY) */
    {},
};
