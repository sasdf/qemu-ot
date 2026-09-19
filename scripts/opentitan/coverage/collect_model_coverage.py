#!/usr/bin/env python3
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Collect and merge per-test gcov .gcda outputs from Bazel's TEST_UNDECLARED_OUTPUTS_DIR
(bazel-testlogs/**/test.outputs/gcov) and extract uncovered code chunks across all
QEMU Earlgrey hardware models.

All intermediate merge directories and default outputs are isolated inside the agent's
Bazel output_base (<output_base>/qemu-cov-report/) to prevent multi-agent collisions.
"""
import argparse
import concurrent.futures
import glob
import json
import os
import re
import shutil
import subprocess
import tempfile
import zipfile
from typing import Dict, List, Tuple

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


def discover_gcov_dirs(ot_dir: str, work_dir: str) -> List[str]:
    testlogs_dir = os.path.join(ot_dir, "bazel-testlogs")
    if not os.path.exists(testlogs_dir):
        return []
    gcov_dirs = []
    unzip_idx = 0
    for root, dirs, files in os.walk(testlogs_dir, followlinks=True):
        if os.path.basename(root) == "gcov" and os.path.basename(os.path.dirname(root)) == "test.outputs":
            if any(f.endswith(".gcda") for f in files):
                gcov_dirs.append(root)
        elif os.path.basename(root) == "test.outputs" and "gcov" not in dirs and "outputs.zip" in files:
            zip_path = os.path.join(root, "outputs.zip")
            try:
                with zipfile.ZipFile(zip_path, "r") as zf:
                    gcda_members = [n for n in zf.namelist() if n.endswith(".gcda")]
                    if gcda_members:
                        dest = os.path.join(work_dir, f"unzipped_{unzip_idx}")
                        unzip_idx += 1
                        os.makedirs(dest, exist_ok=True)
                        for m in gcda_members:
                            data = zf.read(m)
                            with open(os.path.join(dest, os.path.basename(m)), "wb") as out_f:
                                out_f.write(data)
                        gcov_dirs.append(dest)
            except Exception:
                pass
    return sorted(set(gcov_dirs))


def merge_two_dirs(args: Tuple[str, str, str]) -> str:
    d1, d2, out_d = args
    os.makedirs(out_d, exist_ok=True)
    subprocess.check_call(["gcov-tool", "merge", d1, d2, "-o", out_d])
    return out_d


def parallel_tree_merge(dirs: List[str], merge_root: str) -> str:
    if not dirs:
        empty = os.path.join(merge_root, "empty")
        os.makedirs(empty, exist_ok=True)
        return empty
    if len(dirs) == 1:
        return dirs[0]

    current = list(dirs)
    level = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=32) as pool:
        while len(current) > 1:
            next_level = []
            tasks = []
            for i in range(0, len(current), 2):
                if i + 1 < len(current):
                    out_d = os.path.join(merge_root, f"lvl_{level}_{i // 2}")
                    tasks.append((current[i], current[i + 1], out_d))
                else:
                    next_level.append(current[i])
            merged = list(pool.map(merge_two_dirs, tasks))
            next_level.extend(merged)
            current = next_level
            level += 1
    return current[0]


def run_gcov_on_obj(args: Tuple[str, str, str]) -> Dict:
    eval_build_dir, obj_rel, qemu_dir = args
    raw = subprocess.check_output(
        ["gcov", "-j", "-t", obj_rel],
        cwd=eval_build_dir,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    data = json.loads(raw)
    results = {}
    for f_entry in data.get("files", []):
        f_path = f_entry.get("file", "")
        rel_path = f_path[3:] if f_path.startswith("../") else f_path
        if not is_earlgrey_model_source(rel_path):
            continue
        src_abs = os.path.join(qemu_dir, rel_path)
        src_lines = []
        if os.path.exists(src_abs):
            with open(src_abs, "r", errors="replace") as sf:
                src_lines = sf.readlines()

        line_map = {}
        func_for_line = {}
        for l_info in f_entry.get("lines", []):
            ln = l_info["line_number"]
            cnt = l_info["count"]
            line_map[ln] = max(line_map.get(ln, 0), cnt)
            if l_info.get("function_name"):
                func_for_line[ln] = l_info["function_name"]

        funcs = []
        for fn in f_entry.get("functions", []):
            funcs.append({
                "name": fn["name"],
                "start_line": fn["start_line"],
                "end_line": fn.get("end_line", fn["start_line"]),
                "execution_count": fn["execution_count"],
            })

        exec_lines = sorted(line_map.keys())
        covered_lines = [ln for ln in exec_lines if line_map[ln] > 0]
        uncovered_lines = [ln for ln in exec_lines if line_map[ln] == 0]

        # Group contiguous or near-contiguous uncovered executable lines (gap <= 3 non-covered lines) into chunks
        chunks = []
        if uncovered_lines:
            start = uncovered_lines[0]
            prev = start
            chunk_unexec = [start]
            for ln in uncovered_lines[1:]:
                # Merge into same chunk if no covered executable line lies between prev and ln and gap <= 4
                has_covered_between = any(
                    (prev < k < ln) and line_map[k] > 0 for k in exec_lines
                )
                if not has_covered_between and (ln - prev) <= 4:
                    chunk_unexec.append(ln)
                    prev = ln
                else:
                    chunks.append((start, prev, chunk_unexec))
                    start = ln
                    prev = ln
                    chunk_unexec = [ln]
            chunks.append((start, prev, chunk_unexec))

        formatted_chunks = []
        for c_start, c_end, c_lines in chunks:
            c_funcs = sorted({func_for_line.get(ln, "") for ln in c_lines if func_for_line.get(ln)})
            snippet = ""
            if src_lines:
                lo = max(1, c_start - 1)
                hi = min(len(src_lines), c_end + 1)
                snippet = "".join(
                    f"{idx}: {src_lines[idx - 1]}" for idx in range(lo, hi + 1)
                )
            formatted_chunks.append({
                "start_line": c_start,
                "end_line": c_end,
                "uncovered_executable_lines": len(c_lines),
                "functions": c_funcs,
                "snippet": snippet,
            })

        results[rel_path] = {
            "file": rel_path,
            "total_executable_lines": len(exec_lines),
            "covered_executable_lines": len(covered_lines),
            "uncovered_executable_lines": len(uncovered_lines),
            "line_coverage_pct": round(100.0 * len(covered_lines) / len(exec_lines), 2) if exec_lines else 100.0,
            "total_functions": len(funcs),
            "covered_functions": sum(1 for fn in funcs if fn["execution_count"] > 0),
            "uncovered_functions": [fn["name"] for fn in funcs if fn["execution_count"] == 0],
            "function_coverage_pct": round(
                100.0 * sum(1 for fn in funcs if fn["execution_count"] > 0) / len(funcs), 2
            ) if funcs else 100.0,
            "uncovered_chunks": formatted_chunks,
        }
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qemu-dir", default="/root/src/qemu",
                        help="Path to QEMU source repository")
    parser.add_argument("--ot-dir", default="/root/src/opentitan",
                        help="Path to OpenTitan workspace containing bazel-testlogs")
    parser.add_argument("--cov-build-dir", default=None,
                        help="Path to coverage build dir containing .gcno files (defaults to <output_base>/qemu-cov-root/build)")
    parser.add_argument("--out-dir", default=None,
                        help="Directory to write uncovered_chunks.json and summary.md (defaults to <output_base>/qemu-cov-report)")
    parser.add_argument("--out-json", default=None,
                        help="Optional explicit path for uncovered_chunks.json")
    parser.add_argument("--out-summary", default=None,
                        help="Optional explicit path for summary.md")
    args = parser.parse_args()

    qemu_dir = os.path.abspath(args.qemu_dir)
    ot_dir = os.path.abspath(args.ot_dir)
    out_base = subprocess.check_output(
        ["./bazelisk.sh", "info", "output_base"], cwd=ot_dir, text=True
    ).strip()

    cov_build_dir = (
        os.path.abspath(args.cov_build_dir)
        if args.cov_build_dir
        else os.path.join(out_base, "qemu-cov-root", "build")
    )
    if not os.path.exists(cov_build_dir):
        raise SystemExit(f"Coverage build dir {cov_build_dir} not found. Run build_coverage_qemu.py first.")

    report_dir = (
        os.path.abspath(args.out_dir)
        if args.out_dir
        else os.path.join(out_base, "qemu-cov-report")
    )
    os.makedirs(report_dir, exist_ok=True)

    with tempfile.TemporaryDirectory(dir=report_dir, prefix="merge_") as tmp_work:
        gcov_dirs = discover_gcov_dirs(ot_dir, tmp_work)
        print(f"Discovered {len(gcov_dirs)} per-test gcov output directories in {ot_dir}/bazel-testlogs")
        merged_gcda_dir = parallel_tree_merge(gcov_dirs, os.path.join(tmp_work, "tree"))

        eval_build = os.path.join(tmp_work, "eval_build")
        for sub in ("libsystem.a.p", "libqemu-riscv32-softmmu.a.p"):
            src_sub = os.path.join(cov_build_dir, sub)
            dst_sub = os.path.join(eval_build, sub)
            os.makedirs(dst_sub, exist_ok=True)
            for gcno in glob.glob(os.path.join(src_sub, "*.gcno")):
                base = os.path.basename(gcno)
                os.symlink(gcno, os.path.join(dst_sub, base))
                gcda_name = base[:-5] + ".gcda"
                merged_gcda = os.path.join(merged_gcda_dir, gcda_name)
                if os.path.exists(merged_gcda):
                    shutil.copy2(merged_gcda, os.path.join(dst_sub, gcda_name))

        obj_targets = []
        for sub in ("libsystem.a.p", "libqemu-riscv32-softmmu.a.p"):
            for gcno in sorted(glob.glob(os.path.join(eval_build, sub, "*.gcno"))):
                obj_rel = os.path.join(sub, os.path.basename(gcno)[:-5] + ".o")
                obj_targets.append((eval_build, obj_rel, qemu_dir))

        all_files: Dict[str, Dict] = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=16) as pool:
            for file_dict in pool.map(run_gcov_on_obj, obj_targets):
                all_files.update(file_dict)

    sorted_files = sorted(
        all_files.values(),
        key=lambda x: (-x["uncovered_executable_lines"], x["file"]),
    )
    tot_exec = sum(f["total_executable_lines"] for f in sorted_files)
    tot_cov = sum(f["covered_executable_lines"] for f in sorted_files)
    tot_uncov = sum(f["uncovered_executable_lines"] for f in sorted_files)
    tot_fn = sum(f["total_functions"] for f in sorted_files)
    tot_fn_cov = sum(f["covered_functions"] for f in sorted_files)
    tot_chunks = sum(len(f["uncovered_chunks"]) for f in sorted_files)

    summary_payload = {
        "test_gcov_dirs_merged": len(gcov_dirs),
        "total_files": len(sorted_files),
        "total_executable_lines": tot_exec,
        "covered_executable_lines": tot_cov,
        "uncovered_executable_lines": tot_uncov,
        "line_coverage_pct": round(100.0 * tot_cov / tot_exec, 2) if tot_exec else 100.0,
        "total_functions": tot_fn,
        "covered_functions": tot_fn_cov,
        "function_coverage_pct": round(100.0 * tot_fn_cov / tot_fn, 2) if tot_fn else 100.0,
        "total_uncovered_chunks": tot_chunks,
        "files": sorted_files,
    }

    json_path = os.path.abspath(args.out_json) if args.out_json else os.path.join(report_dir, "uncovered_chunks.json")
    md_path = os.path.abspath(args.out_summary) if args.out_summary else os.path.join(report_dir, "summary.md")
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    os.makedirs(os.path.dirname(md_path), exist_ok=True)

    with open(json_path, "w") as jf:
        json.dump(summary_payload, jf, indent=2)

    md_lines = [
        "# QEMU Earlgrey Hardware Model Coverage Summary",
        "",
        f"- **Tests Merged (`TEST_UNDECLARED_OUTPUTS_DIR/gcov`)**: `{len(gcov_dirs)}`",
        f"- **Total Earlgrey Model Files**: `{len(sorted_files)}`",
        f"- **Executable Lines**: `{tot_cov} / {tot_exec}` (`{summary_payload['line_coverage_pct']}%`), Uncovered: `{tot_uncov}`",
        f"- **Functions**: `{tot_fn_cov} / {tot_fn}` (`{summary_payload['function_coverage_pct']}%`)",
        f"- **Total Uncovered Chunks**: `{tot_chunks}`",
        "",
        "| File | Line Coverage | Covered / Total Lines | Uncovered Lines | Function Coverage | Uncovered Chunks |",
        "| :--- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for f in sorted_files:
        md_lines.append(
            f"| `{f['file']}` | `{f['line_coverage_pct']}%` | "
            f"`{f['covered_executable_lines']} / {f['total_executable_lines']}` | "
            f"`{f['uncovered_executable_lines']}` | "
            f"`{f['covered_functions']} / {f['total_functions']} ({f['function_coverage_pct']}%)` | "
            f"`{len(f['uncovered_chunks'])}` |"
        )
    with open(md_path, "w") as mf:
        mf.write("\n".join(md_lines) + "\n")

    print(f"Wrote JSON report:    {json_path}")
    print(f"Wrote Markdown table: {md_path}")
    print(f"Overall Coverage:     {tot_cov}/{tot_exec} lines ({summary_payload['line_coverage_pct']}%), "
          f"{tot_fn_cov}/{tot_fn} functions ({summary_payload['function_coverage_pct']}%), "
          f"{tot_chunks} uncovered chunks")


if __name__ == "__main__":
    main()
