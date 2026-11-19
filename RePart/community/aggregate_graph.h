#pragma once

#include <map>
#include <set>
#include <utility>
#include "incidence_bipartite.h"

namespace cf {
struct AggregatedCommunityGraph {
    // Generic undirected graph in the existing storage; no bipartite metadata.
    IncidenceBipartite graph;
    std::vector<int> old_to_new; // Inactive old slots remain -1.
    std::vector<std::size_t> anchors;
};

inline AggregatedCommunityGraph aggregateCommunityGraph(
    const IncidenceBipartite& old_graph, const std::vector<int>& labels,
    const std::vector<std::size_t>& old_anchors) {
    const auto n = old_graph.active.size();
    if (labels.size() != n || old_anchors.size() != n)
        throw std::invalid_argument("AGGREGATE_INPUT_SHAPE");
    std::map<int, std::size_t> community_key;
    std::set<std::size_t> unique_anchors;
    for (std::size_t v = 0; v < n; ++v) {
        if (!old_graph.active[v]) continue;
        if (labels[v] < 0) throw std::invalid_argument("AGGREGATE_ACTIVE_LABEL");
        if (!unique_anchors.insert(old_anchors[v]).second)
            throw std::invalid_argument("AGGREGATE_DUPLICATE_ANCHOR");
        const auto entry = community_key.emplace(labels[v], old_anchors[v]);
        if (!entry.second) entry.first->second = std::min(entry.first->second, old_anchors[v]);
    }
    std::vector<std::pair<std::size_t, int>> ordered;
    for (const auto& entry : community_key) ordered.push_back({entry.second, entry.first});
    std::sort(ordered.begin(), ordered.end()); // Stable community keys, not label values.
    if (ordered.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::overflow_error("AGGREGATE_LABEL_RANGE");
    AggregatedCommunityGraph result;
    result.old_to_new.assign(n, -1);
    std::map<int, int> community_to_new;
    for (std::size_t c = 0; c < ordered.size(); ++c) {
        community_to_new.emplace(ordered[c].second, static_cast<int>(c));
        result.anchors.push_back(ordered[c].first);
    }
    for (std::size_t v = 0; v < n; ++v)
        if (old_graph.active[v]) result.old_to_new[v] = community_to_new.at(labels[v]);

    std::map<std::pair<std::size_t, std::size_t>, double> coarse_weight;
    for (const auto& edge : old_graph.edges) { // Exactly one pass in physical-edge order.
        const auto u = edge.node, v = edge.net;
        if (u >= n || v >= n || !old_graph.active[u] || !old_graph.active[v])
            throw std::invalid_argument("AGGREGATE_EDGE_ENDPOINT");
        if (!std::isfinite(edge.weight) || edge.weight < 0.0)
            throw std::invalid_argument("AGGREGATE_EDGE_WEIGHT");
        const auto p = static_cast<std::size_t>(result.old_to_new[u]);
        const auto q = static_cast<std::size_t>(result.old_to_new[v]);
        auto& weight = coarse_weight[{std::min(p, q), std::max(p, q)}];
        weight += edge.weight; // Preserve input addition order; retain zero-weight keys.
        if (!std::isfinite(weight)) throw std::overflow_error("AGGREGATE_EDGE_OVERFLOW");
    }
    auto& graph = result.graph;
    graph.active.assign(ordered.size(), 1);
    graph.adjacency.resize(ordered.size());
    graph.degree.assign(ordered.size(), 0.0);
    for (const auto& entry : coarse_weight) { // Lexicographic canonical endpoint order.
        const auto p = entry.first.first, q = entry.first.second;
        const double weight = entry.second;
        graph.edges.push_back({p, q, weight});
        graph.total_edge_weight += weight; // One physical weight, including a loop.
        if (p == q) {
            const double diagonal = 2.0 * weight;
            if (!std::isfinite(diagonal)) throw std::overflow_error("AGGREGATE_LOOP_OVERFLOW");
            graph.adjacency[p].push_back({p, diagonal});
            graph.degree[p] += diagonal;
        } else {
            graph.adjacency[p].push_back({q, weight});
            graph.adjacency[q].push_back({p, weight});
            graph.degree[p] += weight;
            graph.degree[q] += weight;
        }
        if (!std::isfinite(graph.total_edge_weight) ||
            !std::isfinite(graph.degree[p]) || !std::isfinite(graph.degree[q]))
            throw std::overflow_error("AGGREGATE_SUM_OVERFLOW");
    }
    return result;
}
} // namespace cf
