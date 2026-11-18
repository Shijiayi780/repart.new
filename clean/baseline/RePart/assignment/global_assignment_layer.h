#pragma once
#include "global_assignment_score.h"
#include <map>
#include <limits>
#include <utility>

namespace global_assignment_score {
// Normalize the same Hop sum used by A::sort_fpgas; unreachable entries
// retain A's 2*max_hop penalty. This is a score adaptation, not native A code.
inline std::vector<double> a_fpga_heat(const std::vector<std::vector<int>>& hops,
                                     int max_hop) {
    if (max_hop < 0) throw std::invalid_argument("negative Hop limit");
    std::vector<double> sums(hops.size(), 0.0);
    for (std::size_t a=0;a<hops.size();++a) {
        if (hops[a].size()!=hops.size()) throw std::invalid_argument("Hop matrix shape");
        for (std::size_t b=0;b<hops.size();++b) {
            if (a==b) continue;
            sums[a] += hops[a][b] == -1 ? 2.0*max_hop : hops[a][b];
        }
    }
    if (sums.empty()) return {};
    const double minimum = *std::min_element(sums.begin(),sums.end());
    std::vector<double> heat;
    for (double sum : sums) heat.push_back(sum == 0 ? 1.0 : minimum/sum);
    return heat;
}

struct LayerData {
    std::vector<int> nodes;
    std::vector<std::map<int, std::int64_t>> neighbors;
    std::vector<double> strengths;
    std::vector<int> degrees;
    Parameters parameters;
};

// Mirrors AssignmentSolver::rate_connection using target incident Net sets.
// The returned data and parameters belong to this layer, not a DFS branch.
template<class Graph, class Manager>
LayerData build_layer(const Graph& graph, const Manager& manager) {
    LayerData out;
    out.neighbors.resize(graph.hypernodes.size());
    for (std::size_t u = 0; u < graph.hypernodes.size(); ++u) {
        const auto& node = graph.hypernodes[u];
        if (node.deleted) continue;
        out.nodes.push_back(static_cast<int>(u));
        for (int edge_id : node.incident_nets_as_target) {
            const auto& edge = graph.hyperedges.at(edge_id);
            if (edge.deleted) continue;
            const int source = graph._incidence_array.at(edge._begin);
            if (source == static_cast<int>(u)) continue;
            if (graph.hypernodes.at(source).deleted)
                throw std::invalid_argument("live Net has deleted driver");
            if (edge.weight < 0) throw std::invalid_argument("negative Net weight");
            auto add = [&](int a, int b) {
                auto& weight = out.neighbors[a][b];
                if (weight > std::numeric_limits<std::int64_t>::max() - edge.weight)
                    throw std::overflow_error("connection weight overflow");
                weight += edge.weight;
            };
            add(source, static_cast<int>(u));
            add(static_cast<int>(u), source);
        }
    }
    // Node order and map iteration are deterministic, independent of hash order.
    for (int u : out.nodes) {
        double strength = 0;
        for (const auto& entry : out.neighbors[u]) strength += double(entry.second);
        out.strengths.push_back(strength);
    }
    for (std::size_t a = 0; a < manager.hop_distances.size(); ++a) {
        int degree = 0;
        for (std::size_t b = 0; b < manager.hop_distances.size(); ++b)
            if (a != b && manager.hop_distances.at(a).at(b) == 1) ++degree;
        out.degrees.push_back(degree);
    }
    out.parameters = trial_defaults(out.strengths, out.degrees);
    return out;
}

inline std::vector<AssignedNeighbor> assigned_neighbors(
        const LayerData& layer, int node, int target,
        const std::map<int,int>& assignment,
        const std::vector<std::vector<int>>& hops) {
    std::vector<AssignedNeighbor> result;
    for (const auto& [v, weight] : layer.neighbors.at(node)) {
        const auto placed = assignment.find(v);
        if (placed != assignment.end())
            result.push_back({double(weight), placed->second,
                              hops.at(target).at(placed->second)});
    }
    return result;
}
} // namespace global_assignment_score
