// Copyright 2021 Gregory Chadwick <mail@gregchadwick.co.uk>
// Copyright 2022-2023 Rivos, Inc.
// Licensed under the Apache License Version 2.0, with LLVM Exceptions, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

use std::cell::{Cell, RefCell};
use std::convert::{TryFrom, TryInto};
use std::rc::Rc;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use ethnum::{u256, U256};

use super::insn_proc;
use super::key;
use super::otbn::FlagMode;
use super::random;
use super::{CSR, WSR};
use crate::{ExceptionCause, CSRNG, PRNG};

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum CSRAddr {
    // OTBN custom CSRs
    fg0 = 0x7c0,
    fg1 = 0x7c1,
    flags = 0x7c8,
    mod0 = 0x7d0,
    mod1 = 0x7d1,
    mod2 = 0x7d2,
    mod3 = 0x7d3,
    mod4 = 0x7d4,
    mod5 = 0x7d5,
    mod6 = 0x7d6,
    mod7 = 0x7d7,
    rnd_prefetch = 0x7d8,
    urnd_ctrl = 0x7d9,
    kmac_status = 0x7db,
    kmac_ctrl = 0x7dc,
    kmac_cfg = 0x7dd,
    kmac_strb = 0x7de,
    mai_ctrl = 0x7e0,
    rnd = 0xfc0,         // CSRNG
    urnd = 0xfc1,        // PRNG
    urnd_status = 0xfc2, // URND status
    insn_cnt = 0xfc3,    // Instruction count
    mai_status = 0xfca,  // MAI status
}

impl TryFrom<u32> for CSRAddr {
    type Error = u32;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0x7c0 => Ok(Self::fg0),
            0x7c1 => Ok(Self::fg1),
            0x7c8 => Ok(Self::flags),
            0x7d0 => Ok(Self::mod0),
            0x7d1 => Ok(Self::mod1),
            0x7d2 => Ok(Self::mod2),
            0x7d3 => Ok(Self::mod3),
            0x7d4 => Ok(Self::mod4),
            0x7d5 => Ok(Self::mod5),
            0x7d6 => Ok(Self::mod6),
            0x7d7 => Ok(Self::mod7),
            0x7d8 => Ok(Self::rnd_prefetch),
            0x7d9 => Ok(Self::urnd_ctrl),
            0x7db => Ok(Self::kmac_status),
            0x7dc => Ok(Self::kmac_ctrl),
            0x7dd => Ok(Self::kmac_cfg),
            0x7de => Ok(Self::kmac_strb),
            0x7e0 => Ok(Self::mai_ctrl),
            0xfc0 => Ok(Self::rnd),
            0xfc1 => Ok(Self::urnd),
            0xfc2 => Ok(Self::urnd_status),
            0xfc3 => Ok(Self::insn_cnt),
            0xfca => Ok(Self::mai_status),
            _ => Err(value),
        }
    }
}

#[allow(clippy::from_over_into)]
impl Into<u32> for CSRAddr {
    fn into(self) -> u32 {
        self as u32
    }
}

impl CSRAddr {
    pub fn string_name(csr_addr: u32) -> String {
        match Self::try_from(csr_addr) {
            Ok(csr) => format!("{:?}", csr),
            Err(_) => format!("0x{:03x}", csr_addr),
        }
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum WSRAddr {
    r#mod = 0x0,
    rnd = 0x1,
    urnd = 0x2,
    acc = 0x3,
    key_s0_l = 0x4,
    key_s0_h = 0x5,
    key_s1_l = 0x6,
    key_s1_h = 0x7,
    kmac_data_s0 = 0x8,
    kmac_data_s1 = 0x9,
    mai_res_s0 = 0xa,
    mai_res_s1 = 0xb,
    mai_in0_s0 = 0xc,
    mai_in0_s1 = 0xd,
    mai_in1_s0 = 0xe,
    mai_in1_s1 = 0xf,
    urnd_state = 0x10,
}

impl TryFrom<u32> for WSRAddr {
    type Error = u32;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(Self::r#mod),
            0x1 => Ok(Self::rnd),
            0x2 => Ok(Self::urnd),
            0x3 => Ok(Self::acc),
            0x4 => Ok(Self::key_s0_l),
            0x5 => Ok(Self::key_s0_h),
            0x6 => Ok(Self::key_s1_l),
            0x7 => Ok(Self::key_s1_h),
            0x8 => Ok(Self::kmac_data_s0),
            0x9 => Ok(Self::kmac_data_s1),
            0xa => Ok(Self::mai_res_s0),
            0xb => Ok(Self::mai_res_s1),
            0xc => Ok(Self::mai_in0_s0),
            0xd => Ok(Self::mai_in0_s1),
            0xe => Ok(Self::mai_in1_s0),
            0xf => Ok(Self::mai_in1_s1),
            0x10 => Ok(Self::urnd_state),
            _ => Err(value),
        }
    }
}

#[allow(clippy::from_over_into)]
impl Into<u32> for WSRAddr {
    fn into(self) -> u32 {
        self as u32
    }
}

impl WSRAddr {
    pub fn string_name(wsr_addr: u32) -> String {
        match Self::try_from(wsr_addr) {
            Ok(wsr) => format!("{:?}", wsr),
            Err(_) => format!("0x{:03x}", wsr_addr),
        }
    }
}

#[derive(Default)]
struct CSRFlagGroup {
    flags: SharedFlags,
    mode: Option<FlagMode>,
}

impl CSRFlagGroup {
    pub fn plug(&mut self, flags: &SharedFlags, mode: FlagMode) {
        self.flags = Rc::clone(flags);
        self.mode = Some(mode);
    }
}

impl CSR for CSRFlagGroup {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let val8 = match self.mode.as_ref().unwrap() {
            FlagMode::Fg0 => self.flags.get()[0].bits(),
            FlagMode::Fg1 => self.flags.get()[1].bits(),
            FlagMode::Flags => self.flags.get()[0].bits() | self.flags.get()[1].bits() << 4,
        };
        Ok(val8 as u32)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        let val8 = val as u8;
        let mut flags = self.flags.get();
        match self.mode.as_ref().unwrap() {
            FlagMode::Fg0 => flags[0] = insn_proc::Flags::from_bits_truncate(val8 & 0x0f),
            FlagMode::Fg1 => flags[1] = insn_proc::Flags::from_bits_truncate(val8 & 0x0f),
            FlagMode::Flags => {
                flags[0] = insn_proc::Flags::from_bits_truncate(val8 & 0x0f);
                flags[1] = insn_proc::Flags::from_bits_truncate((val8 >> 4) & 0x0f);
            }
        };
        self.flags.set(flags);
        Ok(())
    }
}

type SharedFlags = Rc<Cell<[insn_proc::Flags; 2]>>;

#[derive(Default)]
struct CSRMod {
    wsg: Rc<Cell<u256>>,
    pos: Option<u32>,
}

impl CSR for CSRMod {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let mut val256: u256 = self.wsg.get();
        val256 >>= self.pos.unwrap() * 32;
        val256 &= U256::from(u32::MAX);
        Ok(val256.as_u32())
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        let mut val256: u256 = self.wsg.get();
        let shift = self.pos.unwrap() * 32;
        val256 &= !(U256::from(u32::MAX).wrapping_shl(shift));
        val256 |= U256::from(val).wrapping_shl(shift);
        self.wsg.set(val256);
        Ok(())
    }
}

impl CSRMod {
    fn plug(&mut self, wsg: &CSRWideSharedGeneric, pos: usize) {
        self.wsg = Rc::clone(&wsg.val);
        self.pos = Some(pos as u32);
    }
}

/// CryptoSecure Random generator
struct CSRRnd {
    rng: Arc<random::Rnd>,
}

impl CSRRnd {
    pub fn new(rng: Arc<random::Rnd>) -> Self {
        Self { rng }
    }
}

impl CSR for CSRRnd {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let (val, fips, repeat) = self.rng.get_csrng_u32();
        if !fips {
            Err(ExceptionCause::ERndFipsChkFail)
        } else if repeat {
            Err(ExceptionCause::ERndRepChkFail)
        } else {
            Ok(val)
        }
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        // as per OTBN definition, do not generate an error for R/O CSR
        Ok(())
    }
}

/// CryptoSecure Random generator
struct CSRWideRnd {
    rng: Arc<random::Rnd>,
}

impl CSRWideRnd {
    pub fn new(rng: Arc<random::Rnd>) -> Self {
        Self { rng }
    }
}

impl WSR for CSRWideRnd {
    fn read(&self) -> Result<u256, ExceptionCause> {
        let (val, fips, repeat) = self.rng.get_csrng_u256();
        if !fips {
            Err(ExceptionCause::ERndFipsChkFail)
        } else if repeat {
            Err(ExceptionCause::ERndRepChkFail)
        } else {
            Ok(val)
        }
    }

    fn write(&mut self, _val: u256) -> Result<(), ExceptionCause> {
        // as per OTBN definition, do not generate an error for R/O CSR
        Ok(())
    }
}

/// Pseudo Random generator
struct CSRUrnd {
    prng: Arc<Mutex<dyn PRNG>>,
}

impl CSRUrnd {
    pub fn new(prng: Arc<Mutex<dyn PRNG>>) -> Self {
        Self { prng }
    }
}

impl CSR for CSRUrnd {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(self.prng.lock().unwrap().get_prng_u32())
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        // as per OTBN definition, do not generate an error for R/O CSR
        Ok(())
    }
}

/// Pseudo Random generator
struct CSRWideUrnd {
    prng: Arc<Mutex<dyn PRNG>>,
    test_mode: bool,
}

impl CSRWideUrnd {
    pub fn new(prng: Arc<Mutex<dyn PRNG>>) -> Self {
        Self {
            prng,
            test_mode: false,
        }
    }
}

impl WSR for CSRWideUrnd {
    fn read(&self) -> Result<u256, ExceptionCause> {
        if !self.test_mode {
            Ok(self.prng.lock().unwrap().get_prng_u256())
        } else {
            Ok(U256::from_str_radix(
                "AAAAAAAA99999999AAAAAAAA99999999AAAAAAAA99999999AAAAAAAA99999999",
                16,
            )
            .unwrap())
        }
    }

    fn write(&mut self, _val: u256) -> Result<(), ExceptionCause> {
        // as per OTBN definition, do not generate an error for R/O CSR
        Ok(())
    }
}

struct CSRRndPrefetcher {
    rng: Arc<random::Rnd>,
}

impl CSRRndPrefetcher {
    pub fn new(rng: Arc<random::Rnd>) -> Self {
        Self { rng }
    }
}

impl CSR for CSRRndPrefetcher {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(0) // always 0
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        // "Writing any value to the RND_PREFETCH CSR initiates a prefetch."
        self.rng.prefetch();
        Ok(())
    }
}

/// CryptoSecure Random generator
struct CSRWideKey {
    key: Arc<key::Key>,
    share: u8,
    high: bool,
}

impl CSRWideKey {
    pub fn new(key: Arc<key::Key>, share: u8, high: bool) -> Self {
        Self { key, share, high }
    }
}

impl WSR for CSRWideKey {
    fn read(&self) -> Result<u256, ExceptionCause> {
        let store = self.key.store.lock().unwrap();
        if !store.valid {
            return Err(ExceptionCause::EKeyInvalid);
        }
        let share = if self.share == 0 { store.lo } else { store.hi };
        let value = share[self.high as usize];
        Ok(value)
    }

    fn write(&mut self, _val: u256) -> Result<(), ExceptionCause> {
        // as per OTBN definition, do not generate an error for R/O CSR
        Ok(())
    }
}

#[derive(Default)]
struct CSRWideGeneric {
    pub val: u256,
}

impl WSR for CSRWideGeneric {
    fn read(&self) -> Result<u256, ExceptionCause> {
        Ok(self.val)
    }

    fn write(&mut self, val: u256) -> Result<(), ExceptionCause> {
        self.val = val;
        Ok(())
    }
}

#[derive(Default)]
struct CSRWideSharedGeneric {
    // TBC: why can't I use an Option<Rc<Cell<u256>>> here (Copy trait)?
    pub val: Rc<Cell<u256>>,
}

impl WSR for CSRWideSharedGeneric {
    fn read(&self) -> Result<u256, ExceptionCause> {
        Ok(self.val.get())
    }

    fn write(&mut self, val: u256) -> Result<(), ExceptionCause> {
        self.val.set(val);
        Ok(())
    }
}

#[derive(Clone, Copy, Default)]
struct UrndCtrlState {
    stopped: bool,
    used_while_stopped: bool,
    restoring: bool,
}

struct CSRUrndCtrl {
    ctrl: Arc<AtomicU32>,
    state: Rc<Cell<UrndCtrlState>>,
}

impl CSRUrndCtrl {
    pub fn new(ctrl: Arc<AtomicU32>, state: Rc<Cell<UrndCtrlState>>) -> Self {
        Self { ctrl, state }
    }
}

impl CSR for CSRUrndCtrl {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(0)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        let urnd_ctrl_enabled = (self.ctrl.load(Ordering::Relaxed) & 0x4) != 0;
        if urnd_ctrl_enabled {
            let mut s = self.state.get();
            let start_cmd = (val & 0x1) != 0;
            let stop_cmd = (val & 0x2) != 0;
            let restore_cmd = (val & 0x4) != 0;

            // otbn_rnd.sv:257-282:
            // start_cmd has higher priority than stop_cmd for urnd_stopped_d.
            // However, used_while_stopped_d checks `!urnd_stopped_q && stop_cmd && urnd_advance`
            // where `urnd_advance` is true whenever `!urnd_stopped_d` is true!
            let urnd_stopped_d = if start_cmd {
                false
            } else if stop_cmd {
                true
            } else {
                s.stopped
            };
            let urnd_advance = !urnd_stopped_d;
            let used_while_stopped_d = if !s.stopped && stop_cmd && urnd_advance {
                true
            } else if stop_cmd {
                false
            } else {
                s.used_while_stopped
            };
            s.stopped = urnd_stopped_d;
            s.used_while_stopped = used_while_stopped_d;
            if restore_cmd {
                s.restoring = true;
            }
            self.state.set(s);
        }
        Ok(())
    }
}

struct CSRUrndStatus {
    ctrl: Arc<AtomicU32>,
    state: Rc<Cell<UrndCtrlState>>,
}

impl CSRUrndStatus {
    pub fn new(ctrl: Arc<AtomicU32>, state: Rc<Cell<UrndCtrlState>>) -> Self {
        Self { ctrl, state }
    }
}

impl CSR for CSRUrndStatus {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let urnd_ctrl_enabled = (self.ctrl.load(Ordering::Relaxed) & 0x4) != 0;
        let s = self.state.get();
        // otbn_rnd.sv:235-244:
        // [31:26] urnd_restore_width = 32
        // [25:16] urnd_state_width = 177
        // [3] used_while_stopped
        // [2] restoring
        // [1] stopped
        // [0] urnd_ctrl_enabled
        let val = (32u32 << 26)
            | (177u32 << 16)
            | ((s.used_while_stopped as u32) << 3)
            | ((s.restoring as u32) << 2)
            | ((s.stopped as u32) << 1)
            | (urnd_ctrl_enabled as u32);
        Ok(val)
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        Ok(())
    }
}

#[derive(Default)]
struct MaiState {
    in0_s0: u256,
    in0_s1: u256,
    in1_s0: u256,
    in1_s1: u256,
    res_s0: u256,
    res_s1: u256,
    op: u8,
    busy_polls: u8,
}

impl MaiState {
    fn reset(&mut self) {
        self.op = 0x0b; // ArithToBool default on start
        self.busy_polls = 0;
    }

    fn execute(&mut self, mod_val: u64) {
        let mut out_s0 = U256::ZERO;
        let mut out_s1 = U256::ZERO;
        let mask_u32 = U256::from(u32::MAX);
        for k in 0..8usize {
            let shift = (k * 32) as u32;
            let x0 = ((self.in0_s0 >> shift) & mask_u32).as_u64();
            let x1 = ((self.in0_s1 >> shift) & mask_u32).as_u64();
            let y0 = ((self.in1_s0 >> shift) & mask_u32).as_u64();
            let y1 = ((self.in1_s1 >> shift) & mask_u32).as_u64();
            let a = x0 ^ x1;
            let b = y0 ^ y1;
            let r = 0x5a5a_5a5au64.wrapping_mul((k as u64) + 1) & 0xffff_ffff;
            let (s0, s1) = match self.op {
                0x17 => {
                    // SecAdd: (a + b) mod 2^32
                    let v = (a + b) & 0xffff_ffff;
                    (v ^ r, r)
                }
                0x0c => {
                    // SecAddMod: (a + b + mod) mod 2^32 if no carry, else (a + b) mod 2^32
                    let sum = a + b;
                    let v = if (sum >> 32) != 0 {
                        sum & 0xffff_ffff
                    } else {
                        (sum + mod_val) & 0xffff_ffff
                    };
                    (v ^ r, r)
                }
                0x0b => {
                    // ArithToBool (A2B): (x0 + x1) mod mod
                    let v = if mod_val != 0 { (x0 + x1) % mod_val } else { 0 };
                    (v ^ r, r)
                }
                0x10 => {
                    // BoolToArith (B2A): a mod mod in arithmetic shares (s0 + s1) % mod == a % mod
                    if mod_val != 0 {
                        let v = a % mod_val;
                        let s0 = r % mod_val;
                        let s1 = (v + mod_val - s0) % mod_val;
                        (s0, s1)
                    } else {
                        (0, 0)
                    }
                }
                _ => (0, 0),
            };
            out_s0 |= U256::from(s0 as u32) << shift;
            out_s1 |= U256::from(s1 as u32) << shift;
        }
        self.res_s0 = out_s0;
        self.res_s1 = out_s1;
        // 4 busy polls matches the 16-cycle SecAdd pipeline latency in otbn_mai.sv
        // (4 iterations of mai_poll_busy in smoke_test.s -> exact 322 instructions).
        self.busy_polls = 4;
    }
}

struct CSRMaiCtrl {
    state: Rc<RefCell<MaiState>>,
    mod_wsr: Rc<Cell<u256>>,
}

impl CSR for CSRMaiCtrl {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let s = self.state.borrow();
        Ok(((s.op & 0x1f) as u32) << 1)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        let ma_start = (val & 0x1) != 0;
        let op = ((val >> 1) & 0x1f) as u8;
        let rsvd = val >> 6;
        let mut s = self.state.borrow_mut();
        s.op = op;

        // otbn_mai.sv:456-459:
        // SecAdd = 5'b10111 (0x17), SecAddMod = 5'b01100 (0x0c),
        // ArithToBool = 5'b01011 (0x0b), BoolToArith = 5'b10000 (0x10)
        let rsvd_csr_write = rsvd != 0;
        let valid_op = matches!(op, 0x17 | 0x0c | 0x0b | 0x10);
        let invalid_op = !valid_op && ma_start;
        if rsvd_csr_write || invalid_op {
            return Err(ExceptionCause::EMaiSoftwareError);
        }
        if ma_start {
            let mod_val = (self.mod_wsr.get() & U256::from(u32::MAX)).as_u64();
            s.execute(mod_val);
        }
        Ok(())
    }
}

struct CSRMaiStatus {
    state: Rc<RefCell<MaiState>>,
}

impl CSR for CSRMaiStatus {
    fn read(&self) -> Result<u32, ExceptionCause> {
        let mut s = self.state.borrow_mut();
        if s.busy_polls > 0 {
            s.busy_polls -= 1;
            Ok(0x1) // BUSY = 1, INPUT_READY = 0
        } else {
            Ok(0x2) // BUSY = 0, INPUT_READY = 1
        }
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        Ok(())
    }
}

#[derive(Clone, Copy)]
enum MaiWsrKind {
    ResS0,
    ResS1,
    In0S0,
    In0S1,
    In1S0,
    In1S1,
}

struct WSRMai {
    state: Rc<RefCell<MaiState>>,
    kind: MaiWsrKind,
}

impl WSR for WSRMai {
    fn read(&self) -> Result<u256, ExceptionCause> {
        let s = self.state.borrow();
        Ok(match self.kind {
            MaiWsrKind::ResS0 => s.res_s0,
            MaiWsrKind::ResS1 => s.res_s1,
            MaiWsrKind::In0S0 => s.in0_s0,
            MaiWsrKind::In0S1 => s.in0_s1,
            MaiWsrKind::In1S0 => s.in1_s0,
            MaiWsrKind::In1S1 => s.in1_s1,
        })
    }

    fn write(&mut self, val: u256) -> Result<(), ExceptionCause> {
        let mut s = self.state.borrow_mut();
        match self.kind {
            MaiWsrKind::ResS0 => s.res_s0 = val,
            MaiWsrKind::ResS1 => s.res_s1 = val,
            MaiWsrKind::In0S0 => s.in0_s0 = val,
            MaiWsrKind::In0S1 => s.in0_s1 = val,
            MaiWsrKind::In1S0 => s.in1_s0 = val,
            MaiWsrKind::In1S1 => s.in1_s1 = val,
        }
        Ok(())
    }
}

fn keccak_f1600(st: &mut [u64; 25]) {
    const RC: [u64; 24] = [
        0x0000000000000001, 0x0000000000008082, 0x800000000000808a, 0x8000000080008000,
        0x000000000000808b, 0x0000000080000001, 0x8000000080008081, 0x8000000000008009,
        0x000000000000008a, 0x0000000000000088, 0x0000000080008009, 0x000000008000000a,
        0x000000008000808b, 0x800000000000008b, 0x8000000000008089, 0x8000000000008003,
        0x8000000000008002, 0x8000000000000080, 0x000000000000800a, 0x800000008000000a,
        0x8000000080008081, 0x8000000000008080, 0x0000000080000001, 0x8000000080008008,
    ];
    const ROT: [u32; 24] = [
        1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 2, 14, 27, 41, 56, 8, 25, 43, 62, 18, 39, 61, 20, 44,
    ];
    const PI: [usize; 24] = [
        10, 7, 11, 17, 18, 3, 5, 16, 8, 21, 24, 4, 15, 23, 19, 13, 12, 2, 20, 14, 22, 9, 6, 1,
    ];
    for &rc in &RC {
        let mut c = [0u64; 5];
        for x in 0..5 {
            c[x] = st[x] ^ st[x + 5] ^ st[x + 10] ^ st[x + 15] ^ st[x + 20];
        }
        for x in 0..5 {
            let d = c[(x + 4) % 5] ^ c[(x + 1) % 5].rotate_left(1);
            for y in 0..5 {
                st[x + 5 * y] ^= d;
            }
        }
        let mut t = st[1];
        for i in 0..24 {
            let j = PI[i];
            let tmp = st[j];
            st[j] = t.rotate_left(ROT[i]);
            t = tmp;
        }
        for y in 0..5 {
            let row = [
                st[5 * y],
                st[5 * y + 1],
                st[5 * y + 2],
                st[5 * y + 3],
                st[5 * y + 4],
            ];
            for x in 0..5 {
                st[5 * y + x] = row[x] ^ ((!row[(x + 1) % 5]) & row[(x + 2) % 5]);
            }
        }
        st[0] ^= rc;
    }
}

struct KmacState {
    cfg: u32,
    strb: u32,
    status: u32,
    data_s0: u256,
    data_s1: u256,
    s0_read: bool,
    s1_read: bool,
    msg: Vec<u8>,
    sponge: [u64; 25],
    rate_beats: usize,
    beat_idx: usize,
    en_xof: bool,
    receiving: bool,
}

impl Default for KmacState {
    fn default() -> Self {
        Self {
            cfg: 0,
            strb: 0,
            status: 0x1, // READY = 1
            data_s0: U256::ZERO,
            data_s1: U256::ZERO,
            s0_read: false,
            s1_read: false,
            msg: Vec::new(),
            sponge: [0u64; 25],
            rate_beats: 17,
            beat_idx: 0,
            en_xof: false,
            receiving: false,
        }
    }
}

impl KmacState {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn emit_next_beat(&mut self) {
        if self.beat_idx >= self.rate_beats {
            if self.en_xof {
                keccak_f1600(&mut self.sponge);
                self.beat_idx = 0;
            } else {
                self.status &= !0x2;
                return;
            }
        }
        let beat = self.sponge[self.beat_idx];
        self.beat_idx += 1;
        let mask64 = 0xdead_beef_dead_beefu64;
        let low_mask = !U256::from(u64::MAX);
        self.data_s0 = (self.data_s0 & low_mask) | U256::from(beat ^ mask64);
        self.data_s1 = (self.data_s1 & low_mask) | U256::from(mask64);
        self.s0_read = false;
        self.s1_read = false;
        self.status |= 0x2; // RSP_VALID
    }

    fn handle_cmd(&mut self, cmd: u32) {
        if (cmd & 0x1) != 0 {
            // START
            self.msg.clear();
            self.sponge = [0u64; 25];
            self.receiving = false;
            self.s0_read = false;
            self.s1_read = false;
            let lo = self.cfg & 0x3f;
            let hi_inv = (self.cfg >> 16) & 0x3f;
            if (lo ^ hi_inv) != 0x3f {
                self.status = 0x1 | 0x4; // READY | RSP_ERROR
            } else {
                self.status = 0x1; // READY
            }
        }
        if (cmd & 0x2) != 0 {
            // SEND
            let unmasked = self.data_s0 ^ self.data_s1;
            for b in 0..32u32 {
                if (self.strb >> b) & 1 != 0 {
                    let byte = ((unmasked >> (b * 8)) & U256::from(0xffu32)).as_u32() as u8;
                    self.msg.push(byte);
                }
            }
            self.status = 0x1; // READY
        }
        if (cmd & 0x4) != 0 {
            // PROCESS
            self.en_xof = (self.cfg & 0x1) != 0;
            let strength = (self.cfg >> 1) & 0x7;
            let mode = (self.cfg >> 4) & 0x3;
            let cap_bits: usize = match strength {
                0 => 256,
                1 => 448,
                2 => 512,
                3 => 768,
                4 => 1024,
                _ => 512,
            };
            let rate_bytes = (1600 - cap_bits) / 8;
            self.rate_beats = rate_bytes / 8;
            let domain_sep: u8 = match mode {
                0 => 0x06, // SHA3
                1 => 0x1f, // SHAKE
                2 => 0x1f, // cSHAKE (empty N, S -> SHAKE)
                _ => 0x04,
            };
            let mut padded = self.msg.clone();
            padded.push(domain_sep);
            while padded.len() % rate_bytes != 0 {
                padded.push(0);
            }
            if let Some(last) = padded.last_mut() {
                *last |= 0x80;
            }
            self.sponge = [0u64; 25];
            for block in padded.chunks_exact(rate_bytes) {
                for (i, word_bytes) in block.chunks_exact(8).enumerate() {
                    self.sponge[i] ^= u64::from_le_bytes(word_bytes.try_into().unwrap());
                }
                keccak_f1600(&mut self.sponge);
            }
            self.beat_idx = 0;
            self.receiving = true;
            self.status = 0x1; // READY
            self.emit_next_beat();
        }
        if (cmd & 0x8) != 0 {
            // DONE
            self.receiving = false;
            self.s0_read = false;
            self.s1_read = false;
            self.status = (self.status & 0x1c) | 0x3; // READY | RSP_VALID + sticky error bits
        }
        if (cmd & 0x10) != 0 {
            // CLOSE
            self.receiving = false;
            self.s0_read = false;
            self.s1_read = false;
            self.status = 0x1; // READY, errors and RSP_VALID cleared
        }
    }
}

struct CSRKmacStatus {
    state: Rc<RefCell<KmacState>>,
}

impl CSR for CSRKmacStatus {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(self.state.borrow().status & 0x1f)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        // Error bits [4:2] are W1C
        self.state.borrow_mut().status &= !(val & 0x1c);
        Ok(())
    }
}

struct CSRKmacCtrl {
    state: Rc<RefCell<KmacState>>,
}

impl CSR for CSRKmacCtrl {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(0)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        self.state.borrow_mut().handle_cmd(val & 0x1f);
        Ok(())
    }
}

struct CSRKmacCfg {
    state: Rc<RefCell<KmacState>>,
}

impl CSR for CSRKmacCfg {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(self.state.borrow().cfg & 0x3f003f)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        self.state.borrow_mut().cfg = val & 0x3f003f;
        Ok(())
    }
}

struct CSRKmacStrb {
    state: Rc<RefCell<KmacState>>,
}

impl CSR for CSRKmacStrb {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(self.state.borrow().strb)
    }

    fn write(&mut self, val: u32) -> Result<(), ExceptionCause> {
        self.state.borrow_mut().strb = val;
        Ok(())
    }
}

struct WSRKmacData {
    state: Rc<RefCell<KmacState>>,
    share: u8,
}

impl WSR for WSRKmacData {
    fn read(&self) -> Result<u256, ExceptionCause> {
        let mut s = self.state.borrow_mut();
        let val = if self.share == 0 { s.data_s0 } else { s.data_s1 };
        if s.receiving {
            if self.share == 0 {
                s.s0_read = true;
            } else {
                s.s1_read = true;
            }
            if s.s0_read && s.s1_read {
                s.emit_next_beat();
            }
        }
        Ok(val)
    }

    fn write(&mut self, val: u256) -> Result<(), ExceptionCause> {
        let mut s = self.state.borrow_mut();
        if self.share == 0 {
            s.data_s0 = val;
        } else {
            s.data_s1 = val;
        }
        Ok(())
    }
}

struct CSRInsnCnt {
    insn_count: Arc<AtomicUsize>,
}

impl CSR for CSRInsnCnt {
    fn read(&self) -> Result<u32, ExceptionCause> {
        Ok(self.insn_count.load(Ordering::Relaxed) as u32)
    }

    fn write(&mut self, _val: u32) -> Result<(), ExceptionCause> {
        Ok(())
    }
}

pub struct CSRSet {
    fg0: CSRFlagGroup,
    fg1: CSRFlagGroup,
    flags: CSRFlagGroup,
    mods: [CSRMod; 8],
    rndprefetch: CSRRndPrefetcher,
    urnd_ctrl: CSRUrndCtrl,
    urnd_status: CSRUrndStatus,
    insn_cnt: CSRInsnCnt,
    kmac_status: CSRKmacStatus,
    kmac_ctrl: CSRKmacCtrl,
    kmac_cfg: CSRKmacCfg,
    kmac_strb: CSRKmacStrb,
    mai_ctrl: CSRMaiCtrl,
    mai_status: CSRMaiStatus,
    rnd: CSRRnd,
    urnd: CSRUrnd,

    r#mod: CSRWideSharedGeneric,
    wrnd: CSRWideRnd,
    wurnd: CSRWideUrnd,
    acc: CSRWideGeneric,
    key_s0_l: CSRWideKey,
    key_s0_h: CSRWideKey,
    key_s1_l: CSRWideKey,
    key_s1_h: CSRWideKey,
    kmac_data_s0: WSRKmacData,
    kmac_data_s1: WSRKmacData,
    mai_res_s0: WSRMai,
    mai_res_s1: WSRMai,
    mai_in0_s0: WSRMai,
    mai_in0_s1: WSRMai,
    mai_in1_s0: WSRMai,
    mai_in1_s1: WSRMai,
    urnd_state: CSRWideGeneric,

    shared_flags: SharedFlags,
    urnd_ctrl_state: Rc<Cell<UrndCtrlState>>,
    mai_state: Rc<RefCell<MaiState>>,
    kmac_state: Rc<RefCell<KmacState>>,
    ctrl: Arc<AtomicU32>,
}

impl CSRSet {
    pub fn new(
        urnd: Arc<Mutex<dyn PRNG>>,
        rnd: Arc<random::Rnd>,
        key: Arc<key::Key>,
        ctrl: Arc<AtomicU32>,
        insn_count: Arc<AtomicUsize>,
    ) -> Self {
        let urnd_ctrl_state = Rc::new(Cell::new(UrndCtrlState::default()));
        let mod_shared = CSRWideSharedGeneric::default();
        let mut mai_init = MaiState::default();
        mai_init.reset();
        let mai_state = Rc::new(RefCell::new(mai_init));
        let kmac_state = Rc::new(RefCell::new(KmacState::default()));
        let mut csrs = Self {
            fg0: CSRFlagGroup::default(),
            fg1: CSRFlagGroup::default(),
            flags: CSRFlagGroup::default(),
            mods: <[CSRMod; 8]>::default(),
            rndprefetch: CSRRndPrefetcher::new(rnd.clone()),
            urnd_ctrl: CSRUrndCtrl::new(ctrl.clone(), urnd_ctrl_state.clone()),
            urnd_status: CSRUrndStatus::new(ctrl.clone(), urnd_ctrl_state.clone()),
            insn_cnt: CSRInsnCnt { insn_count },
            kmac_status: CSRKmacStatus {
                state: kmac_state.clone(),
            },
            kmac_ctrl: CSRKmacCtrl {
                state: kmac_state.clone(),
            },
            kmac_cfg: CSRKmacCfg {
                state: kmac_state.clone(),
            },
            kmac_strb: CSRKmacStrb {
                state: kmac_state.clone(),
            },
            mai_ctrl: CSRMaiCtrl {
                state: mai_state.clone(),
                mod_wsr: mod_shared.val.clone(),
            },
            mai_status: CSRMaiStatus {
                state: mai_state.clone(),
            },
            rnd: CSRRnd::new(rnd.clone()),
            urnd: CSRUrnd::new(urnd.clone()),
            r#mod: mod_shared,
            wrnd: CSRWideRnd::new(rnd),
            wurnd: CSRWideUrnd::new(urnd.clone()),
            acc: CSRWideGeneric::default(),
            key_s0_l: CSRWideKey::new(key.clone(), 0, false),
            key_s0_h: CSRWideKey::new(key.clone(), 0, true),
            key_s1_l: CSRWideKey::new(key.clone(), 1, false),
            key_s1_h: CSRWideKey::new(key.clone(), 1, true),
            kmac_data_s0: WSRKmacData {
                state: kmac_state.clone(),
                share: 0,
            },
            kmac_data_s1: WSRKmacData {
                state: kmac_state.clone(),
                share: 1,
            },
            mai_res_s0: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::ResS0,
            },
            mai_res_s1: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::ResS1,
            },
            mai_in0_s0: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::In0S0,
            },
            mai_in0_s1: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::In0S1,
            },
            mai_in1_s0: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::In1S0,
            },
            mai_in1_s1: WSRMai {
                state: mai_state.clone(),
                kind: MaiWsrKind::In1S1,
            },
            urnd_state: CSRWideGeneric::default(),
            shared_flags: SharedFlags::default(),
            urnd_ctrl_state,
            mai_state,
            kmac_state,
            ctrl,
        };
        csrs.fg0.plug(&csrs.shared_flags, FlagMode::Fg0);
        csrs.fg1.plug(&csrs.shared_flags, FlagMode::Fg1);
        csrs.flags.plug(&csrs.shared_flags, FlagMode::Flags);
        for (pos, otmod) in csrs.mods.iter_mut().enumerate() {
            otmod.plug(&csrs.r#mod, pos)
        }
        csrs
    }

    pub fn get_ctrl(&self) -> u32 {
        self.ctrl.load(Ordering::Relaxed)
    }

    pub fn reset_on_start(&mut self) {
        self.urnd_ctrl_state.set(UrndCtrlState::default());
        self.mai_state.borrow_mut().reset();
        self.kmac_state.borrow_mut().reset();
    }

    pub fn get_flags(&self, fg: usize) -> insn_proc::Flags {
        self.shared_flags.get()[fg]
    }

    pub fn set_flags(&mut self, fg: usize, value: insn_proc::Flags) {
        let mut flags = self.shared_flags.get();
        flags[fg] = value;
        self.shared_flags.set(flags);
    }

    pub fn get_csr(&self, addr: u32) -> Option<&dyn CSR> {
        let csr_addr = CSRAddr::try_from(addr).ok()?;

        // note: we do not want to use num_enum here that draws to many dependencies
        Some(match csr_addr {
            CSRAddr::fg0 => &self.fg0,
            CSRAddr::fg1 => &self.fg1,
            CSRAddr::flags => &self.flags,
            CSRAddr::mod0 => &self.mods[0],
            CSRAddr::mod1 => &self.mods[1],
            CSRAddr::mod2 => &self.mods[2],
            CSRAddr::mod3 => &self.mods[3],
            CSRAddr::mod4 => &self.mods[4],
            CSRAddr::mod5 => &self.mods[5],
            CSRAddr::mod6 => &self.mods[6],
            CSRAddr::mod7 => &self.mods[7],
            CSRAddr::rnd_prefetch => &self.rndprefetch,
            CSRAddr::urnd_ctrl => &self.urnd_ctrl,
            CSRAddr::kmac_status => &self.kmac_status,
            CSRAddr::kmac_ctrl => &self.kmac_ctrl,
            CSRAddr::kmac_cfg => &self.kmac_cfg,
            CSRAddr::kmac_strb => &self.kmac_strb,
            CSRAddr::mai_ctrl => &self.mai_ctrl,
            CSRAddr::rnd => &self.rnd,
            CSRAddr::urnd => &self.urnd,
            CSRAddr::urnd_status => &self.urnd_status,
            CSRAddr::insn_cnt => &self.insn_cnt,
            CSRAddr::mai_status => &self.mai_status,
        })
    }

    pub fn get_csr_mut(&mut self, addr: u32) -> Option<&mut dyn CSR> {
        let csr_addr = CSRAddr::try_from(addr).ok()?;

        // note: we do not want to use num_enum here that draws to many dependencies
        Some(match csr_addr {
            CSRAddr::fg0 => &mut self.fg0,
            CSRAddr::fg1 => &mut self.fg1,
            CSRAddr::flags => &mut self.flags,
            CSRAddr::mod0 => &mut self.mods[0],
            CSRAddr::mod1 => &mut self.mods[1],
            CSRAddr::mod2 => &mut self.mods[2],
            CSRAddr::mod3 => &mut self.mods[3],
            CSRAddr::mod4 => &mut self.mods[4],
            CSRAddr::mod5 => &mut self.mods[5],
            CSRAddr::mod6 => &mut self.mods[6],
            CSRAddr::mod7 => &mut self.mods[7],
            CSRAddr::rnd_prefetch => &mut self.rndprefetch,
            CSRAddr::urnd_ctrl => &mut self.urnd_ctrl,
            CSRAddr::kmac_status => &mut self.kmac_status,
            CSRAddr::kmac_ctrl => &mut self.kmac_ctrl,
            CSRAddr::kmac_cfg => &mut self.kmac_cfg,
            CSRAddr::kmac_strb => &mut self.kmac_strb,
            CSRAddr::mai_ctrl => &mut self.mai_ctrl,
            CSRAddr::rnd => &mut self.rnd,
            CSRAddr::urnd => &mut self.urnd,
            CSRAddr::urnd_status => &mut self.urnd_status,
            CSRAddr::insn_cnt => &mut self.insn_cnt,
            CSRAddr::mai_status => &mut self.mai_status,
        })
    }

    pub fn get_wsr(&self, addr: u32) -> Option<&dyn WSR> {
        let wsr_addr = WSRAddr::try_from(addr).ok()?;

        Some(match wsr_addr {
            WSRAddr::r#mod => &self.r#mod,
            WSRAddr::rnd => &self.wrnd,
            WSRAddr::urnd => &self.wurnd,
            WSRAddr::acc => &self.acc,
            WSRAddr::key_s0_l => &self.key_s0_l,
            WSRAddr::key_s0_h => &self.key_s0_h,
            WSRAddr::key_s1_l => &self.key_s1_l,
            WSRAddr::key_s1_h => &self.key_s1_h,
            WSRAddr::kmac_data_s0 => &self.kmac_data_s0,
            WSRAddr::kmac_data_s1 => &self.kmac_data_s1,
            WSRAddr::mai_res_s0 => &self.mai_res_s0,
            WSRAddr::mai_res_s1 => &self.mai_res_s1,
            WSRAddr::mai_in0_s0 => &self.mai_in0_s0,
            WSRAddr::mai_in0_s1 => &self.mai_in0_s1,
            WSRAddr::mai_in1_s0 => &self.mai_in1_s0,
            WSRAddr::mai_in1_s1 => &self.mai_in1_s1,
            WSRAddr::urnd_state => &self.urnd_state,
        })
    }

    pub fn get_wsr_mut(&mut self, addr: u32) -> Option<&mut dyn WSR> {
        let wsr_addr = WSRAddr::try_from(addr).ok()?;

        Some(match wsr_addr {
            WSRAddr::r#mod => &mut self.r#mod,
            WSRAddr::rnd => &mut self.wrnd,
            WSRAddr::urnd => &mut self.wurnd,
            WSRAddr::acc => &mut self.acc,
            WSRAddr::key_s0_l => &mut self.key_s0_l,
            WSRAddr::key_s0_h => &mut self.key_s0_h,
            WSRAddr::key_s1_l => &mut self.key_s1_l,
            WSRAddr::key_s1_h => &mut self.key_s1_h,
            WSRAddr::kmac_data_s0 => &mut self.kmac_data_s0,
            WSRAddr::kmac_data_s1 => &mut self.kmac_data_s1,
            WSRAddr::mai_res_s0 => &mut self.mai_res_s0,
            WSRAddr::mai_res_s1 => &mut self.mai_res_s1,
            WSRAddr::mai_in0_s0 => &mut self.mai_in0_s0,
            WSRAddr::mai_in0_s1 => &mut self.mai_in0_s1,
            WSRAddr::mai_in1_s0 => &mut self.mai_in1_s0,
            WSRAddr::mai_in1_s1 => &mut self.mai_in1_s1,
            WSRAddr::urnd_state => &mut self.urnd_state,
        })
    }

    pub fn wipe_internal(&mut self, prng: &Arc<Mutex<dyn PRNG>>) {
        /*
         * real HW performs a two-step process:
         * - wipe with PRNG randomness
         * - zero
         */
        self.shared_flags.set([insn_proc::Flags::empty(); 2]);

        let mut prng = prng.lock().unwrap();

        let _ = self.acc.write(prng.get_prng_u256());
        let _ = self.r#mod.write(prng.get_prng_u256());

        let _ = self.acc.write(U256::from(0u32));
        let _ = self.r#mod.write(U256::from(0u32));
    }

    pub fn set_test_mode(&mut self, enable: bool) {
        self.wurnd.test_mode = enable;
    }
}
