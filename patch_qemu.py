import os
import re

file_path = "hw/opentitan/otbn/otbn/src/insn_decode.rs"
with open(file_path, "r") as f:
    data = f.read()

# Swap bn.mulvl and bn.mulv mapping
data = re.sub(
    r"\(false, false\) => Some\(processor.process_bn_mulvl\(dec_insn\)\),\n\s*\(false, true\) => Some\(processor.process_bn_mulv\(dec_insn\)\),",
    r"(false, false) => Some(processor.process_bn_mulv(dec_insn)),\n                (false, true) => Some(processor.process_bn_mulvl(dec_insn)),",
    data
)

# Swap bn.mulvm and bn.mulvml mapping
data = re.sub(
    r"\(true, false\) => Some\(processor.process_bn_mulvm\(dec_insn\)\),\n\s*\(true, true\) => Some\(processor.process_bn_mulvml\(dec_insn\)\),",
    r"(true, false) => Some(processor.process_bn_mulvm(dec_insn)),\n                (true, true) => Some(processor.process_bn_mulvml(dec_insn)),", # Wait, I already fixed it previously maybe? No this is mainline!
    data
)
data = re.sub(
    r"\(true, false\) => Some\(processor.process_bn_mulvml\(dec_insn\)\),\n\s*\(true, true\) => Some\(processor.process_bn_mulvm\(dec_insn\)\),",
    r"(true, false) => Some(processor.process_bn_mulvm(dec_insn)),\n                (true, true) => Some(processor.process_bn_mulvml(dec_insn)),",
    data
)

with open(file_path, "w") as f:
    f.write(data)

file_path = "hw/opentitan/otbn/otbn/src/insn_exec.rs"
with open(file_path, "r") as f:
    data = f.read()

# Fix bn_mulvl to have lane extraction and signed math
data = re.sub(
    r"fn process_bn_mulvl\(&mut self, dec_insn: insn_format::RType\) -> Self::InstructionResult \{\n\s*let a = self.hart_state.read_wide_register\(dec_insn.rs1\)\?;\n\s*let b = self.hart_state.read_wide_register\(dec_insn.rs2\)\?;\n\s*let mut res = ethnum::U256::ZERO;\n\s*for i in 0..8 {\n\s*let wa = \(a >> \(i \* 32\) as u32\).as_u32\(\);\n\s*let wb = \(b >> \(i \* 32\) as u32\).as_u32\(\);\n\s*let prod = \(wa as u64\) \* \(wb as u64\);\n\s*res \|= ethnum::U256::from\(prod as u32\) << \(i \* 32\) as u32;\n\s*}",
    r"""fn process_bn_mulvl(&mut self, dec_insn: insn_format::RType) -> Self::InstructionResult {
        let a = self.hart_state.read_wide_register(dec_insn.rs1)?;
        let b = self.hart_state.read_wide_register(dec_insn.rs2)?;
        let is_lane = (dec_insn.funct7 & 0b0000_0100) != 0;
        let lane_idx = (dec_insn.funct7 >> 3) & 0b111;
        let mut res = ethnum::U256::ZERO;
        for i in 0..8 {
            let wa = (a >> (i * 32) as u32).as_u32();
            let wb = if is_lane { (b >> (lane_idx * 32) as u32).as_u32() } else { (b >> (i * 32) as u32).as_u32() };
            let prod = (wa as i32 as i64) * (wb as i32 as i64);
            res |= ethnum::U256::from(prod as u32) << (i * 32) as u32;
        }""",
    data
)

# Fix bn_mulv to have signed math
data = re.sub(
    r"fn process_bn_mulv\(&mut self, dec_insn: insn_format::RType\) -> Self::InstructionResult \{\n\s*let a = self.hart_state.read_wide_register\(dec_insn.rs1\)\?;\n\s*let b = self.hart_state.read_wide_register\(dec_insn.rs2\)\?;\n\s*let mut res = ethnum::U256::ZERO;\n\s*for i in 0..8 {\n\s*let wa = \(a >> \(i \* 32\) as u32\).as_u32\(\);\n\s*let wb = \(b >> \(i \* 32\) as u32\).as_u32\(\);\n\s*let prod = \(wa as u64\) \* \(wb as u64\);\n\s*res \|= ethnum::U256::from\(\(prod >> 32\) as u32\) << \(i \* 32\) as u32;\n\s*}",
    r"""fn process_bn_mulv(&mut self, dec_insn: insn_format::RType) -> Self::InstructionResult {
        let a = self.hart_state.read_wide_register(dec_insn.rs1)?;
        let b = self.hart_state.read_wide_register(dec_insn.rs2)?;
        let is_lane = (dec_insn.funct7 & 0b0000_0100) != 0;
        let lane_idx = (dec_insn.funct7 >> 3) & 0b111;
        let mut res = ethnum::U256::ZERO;
        for i in 0..8 {
            let wa = (a >> (i * 32) as u32).as_u32();
            let wb = if is_lane { (b >> (lane_idx * 32) as u32).as_u32() } else { (b >> (i * 32) as u32).as_u32() };
            let prod = (wa as i32 as i64) * (wb as i32 as i64);
            res |= ethnum::U256::from((prod >> 32) as u32) << (i * 32) as u32;
        }""",
    data
)

# Strip out conditional mod reduction from process_bn_mulvml
data = re.sub(
    r"let mut final_res_64 = \(res_128 >> 32\) as u64;\n\s*if final_res_64 >= q as u64 \{\n\s*final_res_64 = final_res_64.wrapping_sub\(q as u64\);\n\s*\}",
    r"let final_res_64 = (res_128 >> 32) as u64;",
    data
)

with open(file_path, "w") as f:
    f.write(data)
