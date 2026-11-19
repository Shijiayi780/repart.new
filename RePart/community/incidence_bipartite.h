#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include "../datastructure/hypergraph.h"

namespace cf {
struct IncidenceEdge {
    std::size_t node, net;
    double weight; // One physical undirected edge; initial graph has no loops.
};
struct IncidenceArc {
    std::size_t target;
    double weight;
};
struct IncidenceBipartite {
    std::size_t node_slots = 0;
    std::vector<unsigned char> active;
    std::vector<IncidenceEdge> edges;
    std::vector<std::vector<IncidenceArc>> adjacency;
    std::vector<double> degree;
    std::vector<std::size_t> raw_pin_count, unique_pin_count;
    double total_edge_weight = 0.0;
};

inline IncidenceBipartite buildIncidenceBipartite(const Hypergraph& hg) {
    IncidenceBipartite graph;
    const std::size_t n = hg.hypernodes.size(), m = hg.hyperedges.size();
    if (m > std::numeric_limits<std::size_t>::max() - n)
        throw std::overflow_error("INCIDENCE_VERTEX_COUNT_OVERFLOW");
    graph.node_slots = n;
    graph.active.assign(n + m, 0);
    graph.adjacency.resize(n + m);
    graph.degree.assign(n + m, 0.0);
    graph.raw_pin_count.assign(m, 0);
    graph.unique_pin_count.assign(m, 0);
    for (std::size_t u = 0; u < n; ++u)
        graph.active[u] = !hg.hypernodes[u].deleted;

    for (std::size_t e = 0; e < m; ++e) {
        const auto& net = hg.hyperedges[e];
        if (net.deleted) continue;
        const std::string context = " net=" + std::to_string(e);
        if (net._size == 0) throw std::invalid_argument("EMPTY_NET" + context);
        if (net._begin < 0 || net._size < 0)
            throw std::invalid_argument("INVALID_INCIDENCE_RANGE" + context);
        const auto begin = static_cast<std::size_t>(net._begin);
        const auto count = static_cast<std::size_t>(net._size);
        if (begin > hg._incidence_array.size() || count > hg._incidence_array.size() - begin)
            throw std::invalid_argument("INVALID_INCIDENCE_RANGE" + context);
        if (net.weight < 0) throw std::invalid_argument("NEGATIVE_NET_WEIGHT" + context);
        std::vector<int> endpoints;
        endpoints.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const int u = hg._incidence_array[begin + i];
            if (u < 0 || static_cast<std::size_t>(u) >= n || hg.hypernodes[u].deleted)
                throw std::invalid_argument("INVALID_ACTIVE_ENDPOINT" + context);
            endpoints.push_back(u);
        }
        std::sort(endpoints.begin(), endpoints.end());
        endpoints.erase(std::unique(endpoints.begin(), endpoints.end()), endpoints.end());
        const double weight = static_cast<double>(net.weight) / static_cast<double>(endpoints.size());
        if (!std::isfinite(weight)) throw std::overflow_error("NONFINITE_INCIDENCE_WEIGHT" + context);
        const std::size_t net_vertex = n + e;
        graph.active[net_vertex] = 1;
        graph.raw_pin_count[e] = count;
        graph.unique_pin_count[e] = endpoints.size();
        for (const int endpoint : endpoints) {
            const auto u = static_cast<std::size_t>(endpoint);
            graph.edges.push_back({u, net_vertex, weight});
            graph.adjacency[u].push_back({net_vertex, weight});
            graph.adjacency[net_vertex].push_back({u, weight});
            graph.degree[u] += weight;
            graph.degree[net_vertex] += weight;
            graph.total_edge_weight += weight;
        }
        if (!std::isfinite(graph.degree[net_vertex]) || !std::isfinite(graph.total_edge_weight))
            throw std::overflow_error("NONFINITE_INCIDENCE_SUM" + context);
    }
    return graph;
}
} // namespace cf
