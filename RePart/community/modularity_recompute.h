#pragma once

#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>
#include "incidence_bipartite.h"

namespace cf {
struct RecomputedCommunity {
    int label;
    double internal_weight = 0.0; // L_C: physical internal edges once.
    double degree_sum = 0.0;      // K_C: self-loops contribute twice.
};
struct RecomputedModularity {
    std::optional<double> q; // nullopt means W=0, not Q=0.
    double total_edge_weight = 0.0;
    std::vector<double> vertex_degree;
    std::vector<RecomputedCommunity> communities;
};

inline RecomputedModularity recomputeModularity(
    const std::vector<IncidenceEdge>& edges,
    const std::vector<unsigned char>& active,
    const std::vector<int>& labels) {
    if (labels.size() != active.size())
        throw std::invalid_argument("MODULARITY_LABEL_COUNT");
    RecomputedModularity result;
    result.vertex_degree.assign(active.size(), 0.0);
    std::map<int, std::size_t> label_to_slot;
    std::vector<std::size_t> slot(active.size(), 0);
    for (std::size_t x = 0; x < active.size(); ++x) {
        if (!active[x]) continue;
        if (labels[x] < 0) throw std::invalid_argument("MODULARITY_ACTIVE_LABEL");
        const auto entry = label_to_slot.emplace(labels[x], result.communities.size());
        if (entry.second) result.communities.push_back({labels[x], 0.0, 0.0});
        slot[x] = entry.first->second;
    }
    for (const auto& edge : edges) {
        const auto u = edge.node, v = edge.net;
        if (u >= active.size() || v >= active.size() || !active[u] || !active[v])
            throw std::invalid_argument("MODULARITY_EDGE_ENDPOINT");
        if (!std::isfinite(edge.weight) || edge.weight < 0.0)
            throw std::invalid_argument("MODULARITY_EDGE_WEIGHT");
        result.total_edge_weight += edge.weight;
        result.vertex_degree[u] += edge.weight;
        result.vertex_degree[v] += edge.weight; // Also execute when u==v.
        if (slot[u] == slot[v]) result.communities[slot[u]].internal_weight += edge.weight;
        if (!std::isfinite(result.total_edge_weight) ||
            !std::isfinite(result.vertex_degree[u]) || !std::isfinite(result.vertex_degree[v]))
            throw std::overflow_error("MODULARITY_WEIGHT_OVERFLOW");
    }
    for (std::size_t x = 0; x < active.size(); ++x) {
        if (active[x]) result.communities[slot[x]].degree_sum += result.vertex_degree[x];
    }
    if (result.total_edge_weight == 0.0) return result;
    double q = 0.0;
    for (const auto& community : result.communities) {
        if (!std::isfinite(community.degree_sum))
            throw std::overflow_error("MODULARITY_DEGREE_OVERFLOW");
        const double degree_fraction = (community.degree_sum / result.total_edge_weight) * 0.5;
        q += community.internal_weight / result.total_edge_weight - degree_fraction * degree_fraction;
    }
    if (!std::isfinite(q)) throw std::overflow_error("MODULARITY_Q_NONFINITE");
    result.q = q;
    return result;
}
} // namespace cf
