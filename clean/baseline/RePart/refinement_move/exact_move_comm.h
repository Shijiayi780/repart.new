#pragma once

// Exact single-node Move communication delta under the verified static
// communication definition:
//
//   C_{e,b}(P) = weight(e) if b is the source master board and the net has
//                at least one destination board outside the source replicas;
//                weight(e) for each destination board outside the source
//                master/replicas.
//
// Moving u only changes the contribution of nets incident to u. The delta is
// therefore the sum of the per-board before/after contribution differences of
// N(u); no other net is scanned.
//
// This header deliberately does not use target_pins_in_part or the historical
// incremental branch. It is the correctness-first Move path.

#include <algorithm>
#include <set>
#include <vector>

#include "../datastructure/hypergraph.h"

namespace exact_move_comm {

inline std::vector<int> net_contribution(const Hypergraph& g,
                                         const Hyperedge& edge,
                                         int projected_node = -1,
                                         int projected_board = -1) {
    const int board_count = g.fpga_manager.fpga_num;
    std::vector<int> contribution(board_count, 0);
    if (edge.deleted) return contribution;

    const int source = g._incidence_array[edge._begin];
    const int source_board = (source == projected_node)
                                 ? projected_board
                                 : g.hypernodes[source].label;
    const auto& source_replicas =
        g.hypernodes[source].replication_fpga_labels;

    std::vector<int> mark(board_count, -1);
    std::vector<int> targets;
    auto add_target = [&](int board) {
        if (board < 0 || board >= board_count) return;
        if (mark[board] == edge.id) return;
        mark[board] = edge.id;
        targets.push_back(board);
    };

    for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
        const int sink = g._incidence_array[i];
        const int sink_board = (sink == projected_node)
                                   ? projected_board
                                   : g.hypernodes[sink].label;
        add_target(sink_board);
        for (int replica : g.hypernodes[sink].replication_fpga_labels) {
            add_target(replica);
        }
    }

    bool has_destination = false;
    for (int board : targets) {
        if (board == source_board) continue;
        if (source_replicas.count(board)) continue;
        has_destination = true;
        break;
    }
    if (!has_destination) return contribution;

    contribution[source_board] += edge.weight;
    for (int board : targets) {
        if (board == source_board) continue;
        if (source_replicas.count(board)) continue;
        contribution[board] += edge.weight;
    }
    return contribution;
}

inline std::vector<int> move_delta(const Hypergraph& g, int u, int to) {
    const int board_count = g.fpga_manager.fpga_num;
    std::vector<int> delta(board_count, 0);
    std::set<int> incident;
    for (int net : g.hypernodes[u].incident_nets_as_source) {
        if (!g.hyperedges[net].deleted) incident.insert(net);
    }
    for (int net : g.hypernodes[u].incident_nets_as_target) {
        if (!g.hyperedges[net].deleted) incident.insert(net);
    }
    for (int net : incident) {
        const Hyperedge& edge = g.hyperedges[net];
        const std::vector<int> before = net_contribution(g, edge);
        const std::vector<int> after =
            net_contribution(g, edge, u, to);
        for (int board = 0; board < board_count; ++board) {
            delta[board] += after[board] - before[board];
        }
    }
    return delta;
}

inline bool feasible(const Hypergraph& g, const std::vector<int>& comm,
                     int u, int to, std::vector<int>& delta) {
    delta = move_delta(g, u, to);
    for (int board = 0; board < g.fpga_manager.fpga_num; ++board) {
        const long long value =
            static_cast<long long>(comm[board]) + delta[board];
        if (value < 0) return false;
        if (value > g.fpga_manager.fpga_info[board][0]) return false;
    }
    return true;
}

inline void apply(std::vector<int>& comm, const std::vector<int>& delta) {
    for (std::size_t board = 0; board < comm.size(); ++board) {
        comm[board] += delta[board];
    }
}

}  // namespace exact_move_comm
