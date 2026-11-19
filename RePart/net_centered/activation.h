#pragma once

// Net-centered first-batch activation.
//
// NC-A: only logical nodes of the center Net (maximum d_e) may enter the
//       first Move heap.
// NC-B: center Net plus one-hop Net neighbors (A_ef > 0); alpha = 0; nodes are
//       deduplicated.
//
// The filter only controls heap entry. Gain and resource/communication legality
// are unchanged. The guided pass is a separate KWayFMRefiner object; after it
// ends, the original unfiltered refinement chain is rebuilt from the current
// partition.

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../datastructure/hypergraph.h"

namespace nc_activation {

struct ActivationFilter {
    std::vector<char> allowed;
    std::vector<char> inserted;
    int batch_nodes = 0;
    int inserted_nodes = 0;

    bool allows(int node) const {
        return node >= 0 && node < static_cast<int>(allowed.size()) &&
               allowed[node] != 0;
    }

    void markInserted(int node) {
        if (node < 0 || node >= static_cast<int>(inserted.size())) return;
        if (inserted[node]) return;
        inserted[node] = 1;
        ++inserted_nodes;
    }
};

struct Batch {
    bool has_center = false;
    int center_net = -1;
    int center_d = 0;
    ActivationFilter filter;
};

inline std::vector<int> net_demand(const Hypergraph& g) {
    const int board_count = g.fpga_manager.fpga_num;
    std::vector<int> demand(g.hyperedges.size(), 0);
    std::vector<int> sink_mark(g.hypernodes.size(), -1);
    std::vector<int> board_mark(board_count, -1);
    for (const auto& edge : g.hyperedges) {
        if (edge.deleted) continue;
        const int source = g._incidence_array[edge._begin];
        const int source_board = g.hypernodes[source].label;
        int count = 0;
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const int sink = g._incidence_array[i];
            if (sink_mark[sink] == edge.id) continue;
            sink_mark[sink] = edge.id;
            const int sink_board = g.hypernodes[sink].label;
            if (sink_board == source_board) continue;
            if (board_mark[sink_board] == edge.id) continue;
            board_mark[sink_board] = edge.id;
            ++count;
        }
        demand[edge.id] = count;
    }
    return demand;
}

inline int center_net(const std::vector<int>& demand) {
    int best = -1;
    int best_demand = 0;
    for (int net = 0; net < static_cast<int>(demand.size()); ++net) {
        if (demand[net] > best_demand) {
            best_demand = demand[net];
            best = net;
        }
    }
    return best;
}

inline Batch build(const Hypergraph& g, int mode) {
    Batch batch;
    const int node_count = static_cast<int>(g.hypernodes.size());
    batch.filter.allowed.assign(node_count, 0);
    batch.filter.inserted.assign(node_count, 0);
    const std::vector<int> demand = net_demand(g);
    const int center = center_net(demand);
    if (center < 0) return batch;
    batch.has_center = true;
    batch.center_net = center;
    batch.center_d = demand[center];

    std::set<int> nodes;
    auto add_net_nodes = [&](int net) {
        const auto& edge = g.hyperedges[net];
        if (edge.deleted) return;
        for (int i = edge._begin; i < edge._begin + edge._size; ++i) {
            nodes.insert(g._incidence_array[i]);
        }
    };
    add_net_nodes(center);

    if (mode == 2) {
        std::unordered_map<int, double> association;
        for (int node : nodes) {
            std::set<int> incident;
            for (int net : g.hypernodes[node].incident_nets_as_source) {
                if (!g.hyperedges[net].deleted) incident.insert(net);
            }
            for (int net : g.hypernodes[node].incident_nets_as_target) {
                if (!g.hyperedges[net].deleted) incident.insert(net);
            }
            const int degree = static_cast<int>(incident.size());
            if (degree <= 1) continue;
            const double contribution = 1.0 / static_cast<double>(degree - 1);
            for (int net : incident) {
                if (net == center) continue;
                association[net] += contribution;
            }
        }
        for (const auto& entry : association) {
            if (entry.second > 0.0) add_net_nodes(entry.first);
        }
    }

    for (int node : nodes) {
        if (node >= 0 && node < node_count) batch.filter.allowed[node] = 1;
    }
    batch.filter.batch_nodes = static_cast<int>(nodes.size());
    return batch;
}

inline int mode_from_env() {
    const char* value = std::getenv("REPART_NC_MODE");
    if (value == nullptr) return 0;
    const std::string mode(value);
    if (mode == "A" || mode == "a" || mode == "1") return 1;
    if (mode == "B" || mode == "b" || mode == "2") return 2;
    return 0;
}

inline const char* mode_name(int mode) {
    if (mode == 1) return "A";
    if (mode == 2) return "B";
    return "off";
}

inline void log_batch(const Batch& batch, int mode, int exploration, int layer,
                      int accepted_moves, int gain, double seconds) {
    std::ostringstream line;
    line << "NC_BATCH"
         << " mode=" << mode_name(mode)
         << " exploration=" << exploration
         << " layer=" << layer
         << " center_net=" << batch.center_net
         << " center_d=" << batch.center_d
         << " batch_nodes=" << batch.filter.batch_nodes
         << " inserted_nodes=" << batch.filter.inserted_nodes
         << " accepted_moves=" << accepted_moves
         << " gain=" << gain
         << " seconds=" << seconds;
    std::cout << line.str() << std::endl;

    const char* path = std::getenv("REPART_NC_LOG");
    if (path == nullptr) return;
    std::ofstream out(path, std::ios::app);
    if (!out) return;
    out.seekp(0, std::ios::end);
    if (out.tellp() == 0) {
        out << "mode\texploration\tlayer\tcenter_net\tcenter_d\tbatch_nodes"
               "\tinserted_nodes\taccepted_moves\tgain\tseconds\n";
    }
    out << mode_name(mode) << '\t' << exploration << '\t' << layer << '\t'
        << batch.center_net << '\t' << batch.center_d << '\t'
        << batch.filter.batch_nodes << '\t' << batch.filter.inserted_nodes
        << '\t' << accepted_moves << '\t' << gain << '\t' << seconds << '\n';
}

}  // namespace nc_activation
