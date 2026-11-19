#!/usr/bin/env python3
"""Diagnostic only: compare communication accounting variants on a snapshot."""
import sys
from pathlib import Path


def load_snapshot(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    meta = None
    boards = 0
    comm_snapshot = []
    node_label = {}
    node_replicas = {}
    net_weight = {}
    net_pins = {}
    for line in lines:
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "META":
            meta = (int(fields[1]), int(fields[2]), int(fields[3]))
            boards = int(fields[2])
            comm_snapshot = [0] * boards
        elif fields[0] == "COMM":
            comm_snapshot[int(fields[1])] = int(fields[2])
        elif fields[0] == "NODE":
            node = int(fields[1])
            node_label[node] = int(fields[2])
            count = int(fields[11])
            node_replicas[node] = {int(value) for value in fields[12 : 12 + count]}
        elif fields[0] == "NET":
            net = int(fields[1])
            net_weight[net] = int(fields[2])
            net_pins[net] = [int(value) for value in fields[4:]]
    return meta, boards, comm_snapshot, node_label, node_replicas, net_weight, net_pins


def compute(variant, boards, node_label, node_replicas, net_weight, net_pins):
    comm = [0] * boards
    for net, pins in net_pins.items():
        source = pins[0]
        source_board = node_label[source]
        source_replicas = node_replicas[source]
        targets = set()
        for sink in pins[1:]:
            board = node_label[sink]
            replicas = node_replicas[sink]
            if variant in ("v2", "no_source_cover", "all_locations"):
                targets.add(board)
                targets.update(replicas)
            elif variant == "master":
                targets.add(board)
            elif variant == "target_pins":
                # Native target_pins semantics: sink master+replicas, excluding
                # source replicas, but retaining the source board if a sink is there.
                if board not in source_replicas:
                    targets.add(board)
                for replica in replicas:
                    if replica not in source_replicas:
                        targets.add(replica)
        if variant == "v2":
            targets.discard(source_board)
            targets -= source_replicas
        elif variant == "master":
            targets.discard(source_board)
        elif variant == "no_source_cover":
            targets.discard(source_board)
        elif variant == "target_pins":
            targets.discard(source_board)
        elif variant == "all_locations":
            targets.add(source_board)
            targets.update(source_replicas)
        weight = net_weight[net]
        if variant == "all_locations":
            for board in targets:
                comm[board] += weight
        elif targets:
            comm[source_board] += weight
            for board in targets:
                comm[board] += weight
    return comm


def main():
    path = sys.argv[1]
    meta, boards, snapshot_comm, labels, replicas, weights, pins = load_snapshot(path)
    print("meta", meta, "snapshot_sum", sum(snapshot_comm))
    for variant in ("v2", "master", "no_source_cover", "target_pins", "all_locations"):
        comm = compute(variant, boards, labels, replicas, weights, pins)
        differences = [
            (board, comm[board], snapshot_comm[board])
            for board in range(boards)
            if comm[board] != snapshot_comm[board]
        ]
        print(variant, "diffs", len(differences), "first", differences[:10])


if __name__ == "__main__":
    main()
