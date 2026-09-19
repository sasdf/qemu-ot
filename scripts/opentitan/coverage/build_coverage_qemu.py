#!/usr/bin/env python3
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Build an isolated coverage-instrumented QEMU binary in the agent's Bazel output_base
(<output_base>/qemu-cov-root/build/qemu-system-riscv32) without modifying <qemu>/build/
(keeping normal day-to-day builds 100% untouched).

Usage:
  scripts/opentitan/coverage/build_coverage_qemu.py <qemu_dir> <opentitan_dir>

Then run tests with coverage enabled via:
  ./bazelisk.sh test --test_env=OT_QEMU_COVERAGE=1 //sw/device/tests/...
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

EARLGREY_SOURCE_PATTERNS = [
    "hw/opentitan/ot_",
    "hw/riscv/ot_earlgrey.c",
    "hw/riscv/ibex_",
    "hw/misc/pulp_rv_dm.c",
    "hw/jtag/tap_ctrl_rbb.c",
    "hw/block/m25p80",
    "hw/nvram/eeprom_at24c.c",
    "hw/intc/sifive_plic.c",
]

EXCLUDE_PATTERNS = [
    "_dj.c",
    "ot_keymgr_dpe.c",
    "ot_mbx.c",
    "ot_dma.c",
    "ot_soc_proxy.c",
    "ot_soc_dbg_ctrl.c",
]


def is_earlgrey_model_source(src_path: str) -> bool:
    if any(ex in src_path for ex in EXCLUDE_PATTERNS):
        return False
    return any(pat in src_path for pat in EARLGREY_SOURCE_PATTERNS)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("qemu_dir", nargs="?", default="/root/src/qemu",
                        help="Path to QEMU source repository")
    parser.add_argument("ot_dir", nargs="?", default=None,
                        help="Path to OpenTitan repository (used to locate Bazel output_base)")
    parser.add_argument("--cov-root", default=None,
                        help="Explicit directory for isolated coverage build")
    args, _ = parser.parse_known_args()

    qemu_dir = os.path.abspath(args.qemu_dir)
    ot_dir = os.path.abspath(args.ot_dir) if args.ot_dir else None

    normal_build_dir = os.path.join(qemu_dir, "build")
    if not os.path.exists(os.path.join(normal_build_dir, "build.ninja")):
        raise SystemExit(f"Missing normal build directory {normal_build_dir}/build.ninja")

    out_base = None
    if ot_dir and os.path.exists(ot_dir):
        out_base = subprocess.check_output(
            ["./bazelisk.sh", "info", "output_base"], cwd=ot_dir, text=True
        ).strip()

    if args.cov_root:
        cov_root = os.path.abspath(args.cov_root)
    elif out_base:
        cov_root = os.path.join(out_base, "qemu-cov-root")
    else:
        m = re.search(r"/workspaces/([^/]+)", qemu_dir)
        agent_id = m.group(1) if m else f"uid-{os.getuid()}"
        cov_root = os.path.join("/tmp", f"qemu-cov-{agent_id}")

    cov_build_dir = os.path.join(cov_root, "build")
    os.makedirs(cov_build_dir, exist_ok=True)

    for entry in os.listdir(qemu_dir):
        if entry in ("build", "build-cov"):
            continue
        dst = os.path.join(cov_root, entry)
        src = os.path.join(qemu_dir, entry)
        if not os.path.lexists(dst):
            os.symlink(src, dst)

    subprocess.check_call([
        "rsync", "-a",
        "--exclude=*.gcda",
        "--exclude=*.gcno",
        "--exclude=qemu-system-riscv32",
        "--exclude=build.ninja",
        normal_build_dir + "/",
        cov_build_dir + "/",
    ])
    shutil.copy2(
        os.path.join(normal_build_dir, "build.ninja"),
        os.path.join(cov_build_dir, "build.ninja"),
    )

    cov_ninja = os.path.join(cov_build_dir, "build.ninja")
    with open(cov_ninja, "r") as f:
        lines = f.readlines()

    out_lines = []
    instrumented_objs = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r"^build (\S+\.c\.o): c_COMPILER (\S+)", line)
        if m and is_earlgrey_model_source(m.group(2)):
            obj_rel = m.group(1)
            instrumented_objs.append(obj_rel)
            out_lines.append(line)
            i += 1
            while i < len(lines) and (lines[i].startswith(" ") or lines[i].startswith("\t")):
                sub = lines[i]
                if sub.lstrip().startswith("ARGS =") and "--coverage" not in sub:
                    sub = sub.rstrip("\n") + " --coverage\n"
                out_lines.append(sub)
                i += 1
            continue
        if line.startswith("build qemu-system-riscv32: c_LINKER_RSP"):
            out_lines.append(line)
            i += 1
            while i < len(lines) and (lines[i].startswith(" ") or lines[i].startswith("\t")):
                sub = lines[i]
                if sub.lstrip().startswith("LINK_ARGS =") and "--coverage" not in sub:
                    sub = sub.rstrip("\n") + " --coverage\n"
                out_lines.append(sub)
                i += 1
            continue
        out_lines.append(line)
        i += 1

    with open(cov_ninja, "w") as f:
        f.writelines(out_lines)

    for obj_rel in instrumented_objs:
        obj_full = os.path.join(cov_build_dir, obj_rel)
        gcno_full = obj_full[:-2] + ".gcno"
        if os.path.exists(obj_full) and not os.path.exists(gcno_full):
            os.remove(obj_full)

    print(f"Building coverage-instrumented QEMU in {cov_build_dir} "
          f"({len(instrumented_objs)} Earlgrey model objects)...")
    subprocess.check_call(["ninja", "-C", cov_build_dir, "qemu-system-riscv32"])
    cov_bin_path = os.path.join(cov_build_dir, "qemu-system-riscv32")

    if out_base:
        symlink_path = os.path.join(out_base, "qemu-system-riscv32-cov")
        if os.path.lexists(symlink_path):
            os.remove(symlink_path)
        os.symlink(cov_bin_path, symlink_path)
        print(f"Linked coverage binary -> {symlink_path}")

    print(f"Coverage binary ready: {cov_bin_path}")
    print("Normal day-to-day build untouched: ./bazelisk.sh test ...")
    print("Run with coverage enabled:         ./bazelisk.sh test --test_env=OT_QEMU_COVERAGE=1 ...")


if __name__ == "__main__":
    main()
