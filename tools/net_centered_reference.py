#!/usr/bin/env python3
"""Independent reference for NC module 1.

This script does not call the C++ RePart implementation. It reads either:
  - a case directory plus design.fpga.out placement, or
  - a REFINEMENT_SNAPSHOT text file,

then rebuilds the placement/usage/communication state and computes:
  - strict sink-master demand (c_{e,b}, d_e, cross-board sink count),
  - V2 target boards and V2 WTHD,
  - native target-pin counts (occurrence/replica semantics),
  - per-node distinct incident-net degree.

The existing tools/replication_thd_validator.py is used only for the
placement-mode parse and V2 per-net evaluation; it is independent of the C++
module under test. Snapshot parsing is implemented here from the documented
text format.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]


def load_validator():
    path = ROOT / "tools" / "replication_thd_validator.py"
    spec = importlib.util.spec_from_file_location("nc_reference_validator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


V = load_validator()


def read_case(case_dir: Path):
    names, ids = V.read_nodes(case_dir / "design.are")
    board_names, fpga_ids = V.read_info(case_dir / "design.info")
    limit, distances = V.read_topology(case_dir / "design.topo", fpga_ids)
    nets = V.read_nets(case_dir / "design.net", ids)

    weights: list[list[int]] = []
    for line in (case_dir / "design.are").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields:
            weights.append([int(value) for value in fields[1:]])
    capacities: list[list[int]] = []
    comm_caps: list[int] = []
    for line in (case_dir / "design.info").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        comm_caps.append(int(fields[1]))
        capacities.append([int(value) for value in fields[2:]])
    return dict(
        names=names,
        ids=ids,
        board_names=board_names,
        fpga_ids=fpga_ids,
        limit=limit,
        distances=distances,
        nets=nets,
        weights=weights,
        capacities=capacities,
        comm_caps=comm_caps,
        snapshot=None,
    )


def parse_snapshot(snapshot_path: Path, case_dir: Path | None):
    lines = snapshot_path.read_text(encoding="utf-8").splitlines()
    meta_cost = -1
    board_count = -1
    limit = -1
    for line in lines:
        fields = line.split()
        if fields and fields[0] == "META":
            meta_cost = int(fields[1])
            board_count = int(fields[2])
            limit = int(fields[3])
            break
    if board_count < 0:
        raise ValueError("snapshot lacks META")

    board_names = [f"B{board}" for board in range(board_count)]
    fpga_ids = {name: board for board, name in enumerate(board_names)}
    distances = [[-1] * board_count for _ in range(board_count)]
    capacities = [[0] * 8 for _ in range(board_count)]
    comm_caps = [0] * board_count
    snapshot_usage = [[0] * 8 for _ in range(board_count)]
    snapshot_comm = [0] * board_count
    node_labels: dict[int, int] = {}
    node_weights: dict[int, list[int]] = {}
    node_replicas: dict[int, set[int]] = {}
    net_weight: dict[int, int] = {}
    net_source_part: dict[int, int] = {}
    net_pins: dict[int, list[int]] = {}

    for line in lines:
        fields = line.split()
        if not fields:
            continue
        kind = fields[0]
        if kind == "META":
            continue
        if kind == "BOARD":
            board = int(fields[1])
            comm_caps[board] = int(fields[2])
            capacities[board] = [int(value) for value in fields[3:11]]
        elif kind == "HOP":
            board = int(fields[1])
            distances[board] = [int(value) for value in fields[2 : 2 + board_count]]
        elif kind == "USAGE":
            board = int(fields[1])
            snapshot_usage[board] = [int(value) for value in fields[2:10]]
        elif kind == "COMM":
            board = int(fields[1])
            snapshot_comm[board] = int(fields[2])
        elif kind == "NODE":
            node = int(fields[1])
            node_labels[node] = int(fields[2])
            node_weights[node] = [int(value) for value in fields[3:11]]
            count = int(fields[11])
            node_replicas[node] = {int(value) for value in fields[12 : 12 + count]}
        elif kind == "NET":
            net = int(fields[1])
            net_weight[net] = int(fields[2])
            net_source_part[net] = int(fields[3])
            net_pins[net] = [int(value) for value in fields[4:]]

    node_count = max(node_labels) + 1 if node_labels else 0
    if case_dir is not None:
        names: list[str] = []
        for line in (case_dir / "design.are").read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if fields:
                names.append(fields[0])
        if len(names) == node_count:
            node_names = names
        else:
            node_names = [f"n{node}" for node in range(node_count)]
    else:
        node_names = [f"n{node}" for node in range(node_count)]
    ids = {name: node for node, name in enumerate(node_names)}

    placements = [V.NodePlacement() for _ in range(node_count)]
    for node, label in node_labels.items():
        placements[node].master_fpga = label
        placements[node].replica_fpgas = set(node_replicas.get(node, set()))

    nets = []
    for net in sorted(net_pins):
        pins = net_pins[net]
        nets.append(V.Net(net, pins[0], tuple(pins[1:]), net_weight[net], ""))

    weights = [node_weights[node] for node in range(node_count)]
    return dict(
        names=node_names,
        ids=ids,
        board_names=board_names,
        fpga_ids=fpga_ids,
        limit=limit,
        distances=distances,
        nets=nets,
        weights=weights,
        capacities=capacities,
        comm_caps=comm_caps,
        placements=placements,
        snapshot=dict(
            meta_cost=meta_cost,
            usage=snapshot_usage,
            comm=snapshot_comm,
            source_part=net_source_part,
            weight=net_weight,
            pins=net_pins,
        ),
    )


def nonzero_counts(values):
    parts = [f"{index}:{value}" for index, value in enumerate(values) if value > 0]
    return ";".join(parts) if parts else "-"


def compute_reference(data):
    names = data["names"]
    distances = data["distances"]
    nets = data["nets"]
    weights = data["weights"]
    capacities = data["capacities"]
    comm_caps = data["comm_caps"]
    placements = data["placements"]
    board_count = len(data["board_names"])

    contributions, totals = V.evaluate(nets, placements, distances)
    by_net = {item.net_id: item for item in contributions}

    # Independent V2 targets, used to check the validator contribution.
    target_sets: dict[int, list[int]] = {}
    for net in nets:
        source = placements[net.source]
        targets = set()
        for sink in net.sinks:
            sink_place = placements[sink]
            targets.add(sink_place.master_fpga)
            targets.update(sink_place.replica_fpgas)
        targets.discard(source.master_fpga)
        targets -= source.replica_fpgas
        target_sets[net.net_id] = sorted(targets)

    node_nets: list[set[int]] = [set() for _ in weights]
    net_rows = []
    sum_d = 0
    sum_cross = 0
    cross_board_nets = 0
    max_d = 0
    duplicate_nets = 0
    v2_total = 0
    all_costs_valid = True

    for net in nets:
        source_place = placements[net.source]
        source_board = source_place.master_fpga
        source_replicas = set(source_place.replica_fpgas)
        node_nets[net.source].add(net.net_id)

        unique_sinks = []
        seen_sinks = set()
        for sink in net.sinks:
            node_nets[sink].add(net.net_id)
            if sink in seen_sinks:
                continue
            seen_sinks.add(sink)
            unique_sinks.append(sink)

        c_by_board = [0] * board_count
        cross_sink = 0
        has_sink_replica = False
        for sink in unique_sinks:
            sink_place = placements[sink]
            c_by_board[sink_place.master_fpga] += 1
            if sink_place.master_fpga != source_board:
                cross_sink += 1
            if sink_place.replica_fpgas:
                has_sink_replica = True
        d_e = sum(
            1
            for board, count in enumerate(c_by_board)
            if board != source_board and count > 0
        )

        native_pins = [0] * board_count
        for sink in net.sinks:
            sink_place = placements[sink]
            if sink_place.master_fpga not in source_replicas:
                native_pins[sink_place.master_fpga] += 1
            for replica in sink_place.replica_fpgas:
                if replica not in source_replicas:
                    native_pins[replica] += 1

        v2_targets = target_sets[net.net_id]
        contribution_targets = list(by_net[net.net_id].target_fpgas)
        if v2_targets != contribution_targets:
            raise AssertionError(
                f"net {net.net_id}: independent targets {v2_targets} != validator "
                f"{contribution_targets}"
            )
        v2_target_count = len(v2_targets)
        hops = [distances[source_board][board] for board in v2_targets]
        v2_valid = all(hop >= 0 for hop in hops)
        if v2_valid:
            v2_cost = sum(hops) * net.weight
            if v2_cost != by_net[net.net_id].weighted:
                raise AssertionError(
                    f"net {net.net_id}: independent V2 {v2_cost} != validator "
                    f"{by_net[net.net_id].weighted}"
                )
            v2_total += v2_cost
        else:
            v2_cost = -1
            all_costs_valid = False

        duplicate_pins = len(net.sinks) - len(unique_sinks)
        rows = dict(
            net_id=net.net_id,
            source=net.source,
            source_name=names[net.source] if net.source < len(names) else f"n{net.source}",
            source_board=source_board,
            unique_sink_count=len(unique_sinks),
            cross_sink_count=cross_sink,
            d_e=d_e,
            c_by_board=c_by_board,
            native_pins=native_pins,
            v2_targets=v2_targets,
            v2_target_count=v2_target_count,
            v2_cost=v2_cost,
            v2_valid=v2_valid,
            duplicate_pin_count=duplicate_pins,
            has_sink_replica=has_sink_replica,
            has_source_replica=bool(source_place.replica_fpgas),
            native_source_board_pins=native_pins[source_board],
        )
        net_rows.append(rows)
        sum_d += d_e
        sum_cross += cross_sink
        if d_e > 0:
            cross_board_nets += 1
        max_d = max(max_d, d_e)
        if duplicate_pins > 0:
            duplicate_nets += 1

    usage = [[0] * 8 for _ in range(board_count)]
    replicated_nodes = 0
    replica_locations = 0
    for node, placement in enumerate(placements):
        boards = {placement.master_fpga, *placement.replica_fpgas}
        if placement.replica_fpgas:
            replicated_nodes += 1
            replica_locations += len(placement.replica_fpgas)
        for board in boards:
            for dim in range(8):
                usage[board][dim] += weights[node][dim]

    comm = [0] * board_count
    for net in nets:
        source_place = placements[net.source]
        source_board = source_place.master_fpga
        targets = set(target_sets[net.net_id])
        if targets:
            comm[source_board] += net.weight
            for board in targets:
                comm[board] += net.weight

    node_rows = []
    for node, placement in enumerate(placements):
        node_rows.append(
            dict(
                node_id=node,
                name=names[node] if node < len(names) else f"n{node}",
                board=placement.master_fpga,
                replicas=sorted(placement.replica_fpgas),
                net_degree=len(node_nets[node]),
            )
        )

    top = sorted(net_rows, key=lambda row: (-row["d_e"], row["net_id"]))[:10]
    summary = dict(
        boards=board_count,
        nodes=len(placements),
        nets=len(nets),
        replicated_nodes=replicated_nodes,
        replica_locations=replica_locations,
        duplicate_pin_nets=duplicate_nets,
        v2_cost=v2_total if all_costs_valid else -1,
        sum_d_e=sum_d,
        sum_cross_sink_count=sum_cross,
        cross_board_nets=cross_board_nets,
        max_d_e=max_d,
        top_d_nets=[
            dict(
                net_id=row["net_id"],
                source_node=row["source"],
                source_board=row["source_board"],
                d_e=row["d_e"],
                cross_sink_count=row["cross_sink_count"],
                v2_target_count=row["v2_target_count"],
                v2_cost=row["v2_cost"],
                duplicate_pin_count=row["duplicate_pin_count"],
            )
            for row in top
        ],
    )

    snapshot = data.get("snapshot")
    snapshot_comparison = None
    if snapshot is not None:
        usage_diffs = []
        for board in range(board_count):
            for dim in range(8):
                if usage[board][dim] != snapshot["usage"][board][dim]:
                    usage_diffs.append(
                        dict(
                            board=board,
                            dim=dim,
                            reference=usage[board][dim],
                            snapshot=snapshot["usage"][board][dim],
                        )
                    )
        comm_diffs = []
        for board in range(board_count):
            if comm[board] != snapshot["comm"][board]:
                comm_diffs.append(
                    dict(
                        board=board,
                        reference=comm[board],
                        snapshot=snapshot["comm"][board],
                    )
                )
        source_diffs = []
        for net in nets:
            expected = snapshot["source_part"].get(net.net_id)
            if expected is None:
                continue
            if placements[net.source].master_fpga != expected:
                source_diffs.append(
                    dict(
                        net_id=net.net_id,
                        reference=placements[net.source].master_fpga,
                        snapshot=expected,
                    )
                )
        snapshot_comparison = dict(
            meta_cost=snapshot["meta_cost"],
            v2_cost=v2_total if all_costs_valid else -1,
            meta_matches_v2=(snapshot["meta_cost"] == v2_total and all_costs_valid),
            usage_diffs=usage_diffs,
            comm_diffs=comm_diffs,
            source_part_diffs=source_diffs,
        )

    return dict(
        nets=net_rows,
        nodes=node_rows,
        usage=usage,
        comm=comm,
        capacities=capacities,
        comm_caps=comm_caps,
        snapshot=snapshot,
        snapshot_comparison=snapshot_comparison,
        summary=summary,
    )


def write_tsvs(out_dir: Path, data, result):
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "nets.tsv").open("w", encoding="utf-8", newline="") as stream:
        stream.write(
            "net_id\tsource_node\tsource_node_name\tsource_board\tunique_sink_count"
            "\tcross_sink_count\td_e\tnc_c_counts\tnative_target_pins"
            "\tv2_target_boards\tv2_target_count\tv2_cost\tduplicate_pin_count"
            "\tnc_vs_v2_delta\thas_sink_replica\thas_source_replica"
            "\tnative_source_board_pins\n"
        )
        for row in result["nets"]:
            stream.write(
                f"{row['net_id']}\t{row['source']}\t{row['source_name']}\t"
                f"{row['source_board']}\t{row['unique_sink_count']}\t"
                f"{row['cross_sink_count']}\t{row['d_e']}\t"
                f"{nonzero_counts(row['c_by_board'])}\t"
                f"{nonzero_counts(row['native_pins'])}\t"
                f"{','.join(map(str, row['v2_targets']))}\t"
                f"{row['v2_target_count']}\t{row['v2_cost']}\t"
                f"{row['duplicate_pin_count']}\t"
                f"{row['d_e'] - row['v2_target_count']}\t"
                f"{1 if row['has_sink_replica'] else 0}\t"
                f"{1 if row['has_source_replica'] else 0}\t"
                f"{row['native_source_board_pins']}\n"
            )

    with (out_dir / "nodes.tsv").open("w", encoding="utf-8", newline="") as stream:
        stream.write("node_id\tnode_name\tboard\treplicas\tnet_degree\n")
        for row in result["nodes"]:
            stream.write(
                f"{row['node_id']}\t{row['name']}\t{row['board']}\t"
                f"{','.join(map(str, row['replicas']))}\t{row['net_degree']}\n"
            )

    snapshot = result["snapshot"]
    with (out_dir / "usage.tsv").open("w", encoding="utf-8", newline="") as stream:
        header = ["board", "comm", "native_comm"]
        header += [f"usage{dim}" for dim in range(8)]
        header += [f"capacity{dim}" for dim in range(8)]
        if snapshot is not None:
            header += ["snapshot_comm"]
            header += [f"snapshot_usage{dim}" for dim in range(8)]
        stream.write("\t".join(header) + "\n")
        for board in range(len(data["board_names"])):
            values = [
                str(board),
                str(result["comm"][board]),
                str(data["comm_caps"][board]),
            ]
            values += [str(value) for value in result["usage"][board]]
            values += [str(value) for value in data["capacities"][board]]
            if snapshot is not None:
                values += [str(snapshot["comm"][board])]
                values += [str(value) for value in snapshot["usage"][board]]
            stream.write("\t".join(values) + "\n")

    with (out_dir / "target_pins.tsv").open("w", encoding="utf-8", newline="") as stream:
        stream.write("net_id\tboard\tnative_count\treference_count\n")
        for row in result["nets"]:
            for board, count in enumerate(row["native_pins"]):
                stream.write(f"{row['net_id']}\t{board}\t{count}\t{count}\n")

    with (out_dir / "semantic_differences.tsv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        stream.write(
            "net_id\tsource_node\tsource_board\tunique_sink_count\tcross_sink_count"
            "\td_e\tv2_target_count\tnc_vs_v2_delta\tduplicate_pin_count"
            "\tnative_source_board_pins\thas_sink_replica\thas_source_replica"
            "\tnc_c_counts\tnative_target_pins\n"
        )
        for row in result["nets"]:
            differs = (
                row["d_e"] != row["v2_target_count"]
                or row["duplicate_pin_count"] > 0
                or row["has_sink_replica"]
                or row["has_source_replica"]
            )
            if not differs:
                continue
            stream.write(
                f"{row['net_id']}\t{row['source']}\t{row['source_board']}\t"
                f"{row['unique_sink_count']}\t{row['cross_sink_count']}\t"
                f"{row['d_e']}\t{row['v2_target_count']}\t"
                f"{row['d_e'] - row['v2_target_count']}\t"
                f"{row['duplicate_pin_count']}\t{row['native_source_board_pins']}\t"
                f"{1 if row['has_sink_replica'] else 0}\t"
                f"{1 if row['has_source_replica'] else 0}\t"
                f"{nonzero_counts(row['c_by_board'])}\t"
                f"{nonzero_counts(row['native_pins'])}\n"
            )

    with (out_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(result["summary"], stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    if result["snapshot_comparison"] is not None:
        with (out_dir / "snapshot_comparison.json").open(
            "w", encoding="utf-8"
        ) as stream:
            json.dump(result["snapshot_comparison"], stream, ensure_ascii=False, indent=2)
            stream.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case")
    parser.add_argument("--placement")
    parser.add_argument("--snapshot")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if bool(args.placement) == bool(args.snapshot):
        raise SystemExit("exactly one of --placement or --snapshot is required")
    if args.placement and not args.case:
        raise SystemExit("--case is required for placement mode")

    if args.snapshot:
        data = parse_snapshot(Path(args.snapshot), Path(args.case) if args.case else None)
    else:
        data = read_case(Path(args.case))
        data["placements"] = V.read_placements(
            Path(args.placement), data["ids"], data["fpga_ids"]
        )

    result = compute_reference(data)
    result["summary"]["input_mode"] = "snapshot" if args.snapshot else "placement"
    result["summary"]["case_dir"] = args.case or ""
    result["summary"]["placement"] = args.placement or ""
    result["summary"]["snapshot"] = args.snapshot or ""
    result["summary"]["snapshot_meta_cost"] = (
        data["snapshot"]["meta_cost"] if data["snapshot"] is not None else None
    )
    write_tsvs(Path(args.out), data, result)
    print(
        "reference nodes={nodes} nets={nets} boards={boards} v2_cost={v2_cost} "
        "sum_d_e={sum_d_e}".format(**result["summary"])
    )


if __name__ == "__main__":
    main()
