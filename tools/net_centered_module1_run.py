#!/usr/bin/env python3
"""Run and verify NC module 1 (read/state rebuild/demand statistics only)."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import net_centered_reference as reference  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_tsv(path: Path):
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return [], []
    header = lines[0].split("\t")
    rows = [line.split("\t") for line in lines[1:] if line]
    return header, rows


def compare_tsv(cpp_path: Path, ref_path: Path, key_columns):
    cpp_header, cpp_rows = read_tsv(cpp_path)
    ref_header, ref_rows = read_tsv(ref_path)
    if cpp_header != ref_header:
        return {"pass": False, "header_cpp": cpp_header, "header_reference": ref_header}
    cpp_by_key = {tuple(row[index] for index in key_columns): row for row in cpp_rows}
    ref_by_key = {tuple(row[index] for index in key_columns): row for row in ref_rows}
    mismatches = []
    for key in sorted(set(cpp_by_key) | set(ref_by_key)):
        cpp_row = cpp_by_key.get(key)
        ref_row = ref_by_key.get(key)
        if cpp_row is None or ref_row is None:
            mismatches.append(
                {"key": list(key), "cpp": cpp_row, "reference": ref_row, "reason": "missing row"}
            )
            continue
        differences = [
            {"column": cpp_header[index], "cpp": cpp_row[index], "reference": ref_row[index]}
            for index in range(len(cpp_header))
            if cpp_row[index] != ref_row[index]
        ]
        if differences:
            mismatches.append({"key": list(key), "differences": differences})
    return {
        "pass": not mismatches,
        "rows_cpp": len(cpp_rows),
        "rows_reference": len(ref_rows),
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
    }


def compare_summary(cpp_summary, ref_summary):
    keys = [
        "boards",
        "nodes",
        "nets",
        "replicated_nodes",
        "replica_locations",
        "duplicate_pin_nets",
        "v2_cost",
        "sum_d_e",
        "sum_cross_sink_count",
        "cross_board_nets",
        "max_d_e",
        "top_d_nets",
    ]
    differences = []
    for key in keys:
        if cpp_summary.get(key) != ref_summary.get(key):
            differences.append(
                {"key": key, "cpp": cpp_summary.get(key), "reference": ref_summary.get(key)}
            )
    return {"pass": not differences, "differences": differences}


def parse_checks(path: Path):
    header, rows = read_tsv(path)
    if header[:3] != ["check", "status", "detail"]:
        raise RuntimeError(f"unexpected checks header in {path}")
    return [dict(zip(header, row)) for row in rows]


def run_existing_evaluator(case: str, placement: str):
    command = [
        sys.executable,
        str(ROOT / "tools" / "evaluate_partition_wthd.py"),
        case,
        placement,
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        return {"pass": False, "error": completed.stderr.strip(), "command": command}
    result = json.loads(completed.stdout)
    return {"pass": True, "result": result}


def compare_existing_evaluator(cpp_dir: Path, evaluator_result):
    if not evaluator_result.get("pass"):
        return {"pass": False, "error": evaluator_result.get("error")}
    result = evaluator_result["result"]
    cpp_summary = json.loads((cpp_dir / "summary.json").read_text(encoding="utf-8"))
    usage_header, usage_rows = read_tsv(cpp_dir / "usage.tsv")
    comm_column = usage_header.index("comm")
    usage_columns = [usage_header.index(f"usage{dim}") for dim in range(8)]
    usage = [[int(row[index]) for index in usage_columns] for row in usage_rows]
    comm = [int(row[comm_column]) for row in usage_rows]
    differences = []
    if result["wthd"] != cpp_summary["v2_cost"]:
        differences.append(
            {"field": "wthd", "evaluator": result["wthd"], "cpp": cpp_summary["v2_cost"]}
        )
    if result["resources"] != usage:
        differences.append({"field": "resources"})
    if result["communication"] != comm:
        differences.append({"field": "communication"})
    if not result["legal"]:
        differences.append({"field": "legal", "evaluator": result["legal"]})
    return {
        "pass": not differences,
        "differences": differences,
        "evaluator_wthd": result["wthd"],
        "evaluator_legal": result["legal"],
    }


def compare_expected(summary, nets_rows, expected):
    differences = []
    for key, value in expected.get("summary", {}).items():
        if summary.get(key) != value:
            differences.append({"field": f"summary.{key}", "expected": value, "actual": summary.get(key)})
    by_net = {str(row["net_id"]): row for row in nets_rows}
    for net_id, fields in expected.get("nets", {}).items():
        row = by_net.get(str(net_id))
        if row is None:
            differences.append({"field": f"nets.{net_id}", "reason": "missing"})
            continue
        for key, value in fields.items():
            if str(row.get(key)) != str(value):
                differences.append(
                    {
                        "field": f"nets.{net_id}.{key}",
                        "expected": value,
                        "actual": row.get(key),
                    }
                )
    return {"pass": not differences, "differences": differences}


def write_report(path: Path, identity, checks, comparisons, semantic_rows, top_nets, blockers):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# NC module 1 verification\n\n")
        stream.write("## Identity\n\n")
        stream.write("| item | value |\n|---|---|\n")
        for key, value in identity.items():
            if isinstance(value, (dict, list)):
                value = json.dumps(value, ensure_ascii=False)
            stream.write(f"| {key} | `{value}` |\n")
        stream.write("\n## State checks\n\n")
        stream.write("| check | status | detail |\n|---|---|---|\n")
        for check in checks:
            stream.write(
                f"| {check['check']} | {check['status']} | {check['detail']} |\n"
            )
        stream.write("\n## Cross-checks\n\n")
        stream.write("| comparison | pass | details |\n|---|---|---|\n")
        for name, result in comparisons.items():
            details = ""
            if isinstance(result, dict):
                details = result.get("error", "")
                if not details and "differences" in result:
                    details = json.dumps(result["differences"][:3], ensure_ascii=False)
            stream.write(f"| {name} | {bool(result.get('pass'))} | {details} |\n")
        stream.write("\n## Semantic differences\n\n")
        stream.write(f"Rows: {len(semantic_rows)}\n\n")
        if semantic_rows:
            stream.write(
                "| net_id | source_board | d_e | v2_target_count | delta | duplicate | "
                "sink_replica | source_replica | nc_c_counts | native_target_pins |\n"
                "|---|---|---|---|---|---|---|---|---|---|\n"
            )
            for row in semantic_rows[:12]:
                stream.write(
                    f"| {row['net_id']} | {row['source_board']} | {row['d_e']} | "
                    f"{row['v2_target_count']} | {int(row['d_e']) - int(row['v2_target_count'])} | "
                    f"{row['duplicate_pin_count']} | {row['has_sink_replica']} | "
                    f"{row['has_source_replica']} | {row['nc_c_counts']} | "
                    f"{row['native_target_pins']} |\n"
                )
        stream.write("\n## Top d_e nets\n\n")
        stream.write(
            "| net_id | source_node | source_board | d_e | cross_sink_count | "
            "v2_target_count | v2_cost | duplicate_pin_count |\n"
            "|---|---|---|---|---|---|---|---|\n"
        )
        for row in top_nets:
            stream.write(
                f"| {row['net_id']} | {row['source_node']} | {row['source_board']} | "
                f"{row['d_e']} | {row['cross_sink_count']} | {row['v2_target_count']} | "
                f"{row['v2_cost']} | {row['duplicate_pin_count']} |\n"
            )
        stream.write("\n## Blockers\n\n")
        if blockers:
            for blocker in blockers:
                stream.write(f"- {blocker}\n")
        else:
            stream.write("- None for module 1 scope.\n")
        stream.write(
            "\nModule 1 only reads and rebuilds state; it performs no Move search and "
            "does not modify the input placement.\n"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case")
    parser.add_argument("--placement")
    parser.add_argument("--snapshot")
    parser.add_argument("--out", required=True)
    parser.add_argument("--cpp", required=True)
    parser.add_argument("--expected")
    parser.add_argument("--allow-known-comm-cache-diff", action="store_true")
    args = parser.parse_args()

    placement_mode = args.placement is not None
    if placement_mode == (args.snapshot is not None):
        raise SystemExit("exactly one of --placement or --snapshot is required")
    if placement_mode and not args.case:
        raise SystemExit("--case is required for placement mode")

    out = Path(args.out)
    cpp_dir = out / "cpp"
    ref_dir = out / "reference"
    out.mkdir(parents=True, exist_ok=True)
    cpp_dir.mkdir(parents=True, exist_ok=True)
    ref_dir.mkdir(parents=True, exist_ok=True)

    cpp_command = [args.cpp, "--out", str(cpp_dir)]
    if placement_mode:
        cpp_command += ["--case", args.case, "--placement", args.placement]
    else:
        cpp_command += ["--snapshot", args.snapshot]
        if args.case:
            cpp_command += ["--case", args.case]
    cpp_run = subprocess.run(cpp_command, capture_output=True, text=True, check=False)
    if cpp_run.returncode != 0:
        raise SystemExit(f"C++ module failed:\n{cpp_run.stderr}")

    if placement_mode:
        data = reference.read_case(Path(args.case))
        data["placements"] = reference.V.read_placements(
            Path(args.placement), data["ids"], data["fpga_ids"]
        )
    else:
        data = reference.parse_snapshot(
            Path(args.snapshot), Path(args.case) if args.case else None
        )
    result = reference.compute_reference(data)
    result["summary"].update(
        input_mode="snapshot" if args.snapshot else "placement",
        case_dir=args.case or "",
        placement=args.placement or "",
        snapshot=args.snapshot or "",
        snapshot_meta_cost=(
            data["snapshot"]["meta_cost"] if data["snapshot"] is not None else None
        ),
    )
    reference.write_tsvs(ref_dir, data, result)

    cpp_summary = json.loads((cpp_dir / "summary.json").read_text(encoding="utf-8"))
    ref_summary = json.loads((ref_dir / "summary.json").read_text(encoding="utf-8"))
    checks = parse_checks(cpp_dir / "state_checks.tsv")

    comparisons = {
        "cpp_vs_reference_nets": compare_tsv(cpp_dir / "nets.tsv", ref_dir / "nets.tsv", [0]),
        "cpp_vs_reference_nodes": compare_tsv(
            cpp_dir / "nodes.tsv", ref_dir / "nodes.tsv", [0]
        ),
        "cpp_vs_reference_usage": compare_tsv(
            cpp_dir / "usage.tsv", ref_dir / "usage.tsv", [0]
        ),
        "cpp_vs_reference_target_pins": compare_tsv(
            cpp_dir / "target_pins.tsv", ref_dir / "target_pins.tsv", [0, 1]
        ),
        "cpp_vs_reference_semantic_differences": compare_tsv(
            cpp_dir / "semantic_differences.tsv",
            ref_dir / "semantic_differences.tsv",
            [0],
        ),
        "cpp_vs_reference_summary": compare_summary(cpp_summary, ref_summary),
    }

    if placement_mode:
        evaluator = run_existing_evaluator(args.case, args.placement)
        comparisons["cpp_vs_existing_evaluator"] = compare_existing_evaluator(
            cpp_dir, evaluator
        )

    semantic_header, semantic_data = read_tsv(cpp_dir / "semantic_differences.tsv")
    semantic_rows = [dict(zip(semantic_header, row)) for row in semantic_data]
    nets_header, nets_data = read_tsv(cpp_dir / "nets.tsv")
    nets_rows = [dict(zip(nets_header, row)) for row in nets_data]

    if args.expected:
        expected = json.loads(Path(args.expected).read_text(encoding="utf-8"))
        comparisons["fixture_expected"] = compare_expected(
            ref_summary, nets_rows, expected
        )

    hard_checks = {
        "placement_valid",
        "resource_capacity",
        "communication_capacity",
        "hop_reachable",
        "v2_net_costs",
        "v2_total_cost",
        "native_target_pins_recompute",
    }
    blockers = []
    for check in checks:
        if check["status"] == "FAIL" and check["check"] in hard_checks:
            blockers.append(f"{check['check']}: {check['detail']}")
        if (
            check["status"] == "FAIL"
            and check["check"].startswith("snapshot_")
            and not args.allow_known_comm_cache_diff
        ):
            blockers.append(f"{check['check']}: {check['detail']}")

    for name, result in comparisons.items():
        if not result.get("pass"):
            blockers.append(f"{name} failed")

    case_files = []
    if args.case:
        for name in ("design.are", "design.net", "design.info", "design.topo"):
            path = Path(args.case) / name
            if path.exists():
                case_files.append({"path": str(path), "sha256": sha256(path)})
    identity = {
        "module": 1,
        "scope": "read/state rebuild/demand statistics only; no Move search",
        "input_mode": "snapshot" if args.snapshot else "placement",
        "case": args.case or "",
        "placement": args.placement or "",
        "snapshot": args.snapshot or "",
        "case_files": case_files,
        "placement_sha256": sha256(Path(args.placement)) if args.placement else None,
        "snapshot_sha256": sha256(Path(args.snapshot)) if args.snapshot else None,
        "cpp_source_sha256": sha256(ROOT / "tools" / "net_centered_module1.cpp"),
        "reference_source_sha256": sha256(ROOT / "tools" / "net_centered_reference.py"),
        "demand_semantics": "strict sink master labels; duplicate sink node IDs deduplicated",
        "v2_semantics": "sink master+replicas minus source master+replicas",
        "budget": "not used by module 1",
        "initial_v2_wthd": cpp_summary["v2_cost"],
    }
    known_native_cache_differences = [
        check
        for check in checks
        if check["status"] == "FAIL" and check["check"].startswith("snapshot_")
    ]
    verification = {
        "identity": identity,
        "state_checks": checks,
        "comparisons": comparisons,
        "semantic_difference_rows": len(semantic_rows),
        "known_native_cache_differences": known_native_cache_differences,
        "allowed_known_comm_cache_diff": args.allow_known_comm_cache_diff,
        "blockers": blockers,
        "pass": not blockers,
    }
    (out / "identity.json").write_text(
        json.dumps(identity, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (out / "comparison.json").write_text(
        json.dumps(comparisons, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (out / "verification.json").write_text(
        json.dumps(verification, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_report(
        out / "REPORT.md",
        identity,
        checks,
        comparisons,
        semantic_rows,
        ref_summary["top_d_nets"],
        blockers,
    )
    print(
        f"module1 verification pass={verification['pass']} "
        f"blockers={len(blockers)} semantic_rows={len(semantic_rows)}"
    )
    if blockers:
        for blocker in blockers:
            print(f"BLOCKER: {blocker}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
