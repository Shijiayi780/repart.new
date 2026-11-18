#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "flow_preparation.h"

namespace cf {

constexpr std::size_t kNoFlowVertex =
    std::numeric_limits<std::size_t>::max();

enum class LawlerVertexKind : std::uint8_t {
    Source,
    Sink,
    MovableNode,
    NetIn,
    NetOut
};

struct LawlerVertex {
    LawlerVertexKind kind;
    // Original node ID for MovableNode, net ID for NetIn/NetOut, otherwise
    // kNoFlowVertex.
    std::size_t object;
};

struct LawlerArc {
    std::size_t from;
    std::size_t to;
    CutWeight capacity;
};

struct LawlerGadgetMapping {
    std::size_t net;
    std::size_t in_vertex;
    std::size_t out_vertex;
};

struct CapacityBounds {
    CutWeight finite_sum;
    CutWeight infinity;
};

struct PairLawlerNetwork {
    int community_i;
    int community_j;
    std::size_t source;
    std::size_t sink;
    std::vector<LawlerVertex> vertices;
    // Fixed Ci/Cj nodes map to source/sink, movable nodes map to their own
    // vertices, and nodes outside this pair map to kNoFlowVertex.
    std::vector<std::size_t> node_to_vertex;
    std::vector<LawlerGadgetMapping> gadgets;
    std::vector<LawlerArc> arcs;
    CutWeight constant_cut;
    CutWeight finite_sum;
    CutWeight infinity;
};

// Compute S and the strictly dominating capacity INF=S+1 using exact
// uint64_t arithmetic. This helper is public so the otherwise unreachable
// uint64_t overflow boundary can be tested without constructing billions of
// int-weight hyperedges.
inline CapacityBounds makeCapacityBounds(
    const std::vector<CutWeight>& finite_capacities) {
    CutWeight total = 0;
    for (std::size_t index = 0; index < finite_capacities.size(); ++index) {
        const CutWeight capacity = finite_capacities[index];
        if (capacity > std::numeric_limits<CutWeight>::max() - total)
            throw std::overflow_error(
                "LAWLER_FINITE_SUM_OVERFLOW index=" +
                std::to_string(index));
        total += capacity;
    }
    if (total == std::numeric_limits<CutWeight>::max())
        throw std::overflow_error("LAWLER_INFINITY_OVERFLOW");
    return {total, total + 1};
}

namespace lawler_network_detail {

inline bool sameEndpoint(const CorridorEndpoint& left,
                         const CorridorEndpoint& right) {
    return left.node == right.node && left.community == right.community &&
           left.movable == right.movable;
}

inline bool sameNet(const CorridorNet& left, const CorridorNet& right) {
    if (left.net != right.net || left.weight != right.weight ||
        left.endpoints.size() != right.endpoints.size())
        return false;
    for (std::size_t index = 0; index < left.endpoints.size(); ++index) {
        if (!sameEndpoint(left.endpoints[index], right.endpoints[index]))
            return false;
    }
    return true;
}

inline bool sameCorridor(const PairCorridor& left,
                         const PairCorridor& right) {
    if (left.community_i != right.community_i ||
        left.community_j != right.community_j ||
        left.boundary_weight != right.boundary_weight ||
        left.anchor_i != right.anchor_i || left.anchor_j != right.anchor_j ||
        left.boundary_nodes != right.boundary_nodes ||
        left.movable_nodes != right.movable_nodes ||
        left.fixed_i != right.fixed_i || left.fixed_j != right.fixed_j ||
        left.related_nets.size() != right.related_nets.size())
        return false;
    for (std::size_t index = 0; index < left.related_nets.size(); ++index) {
        if (!sameNet(left.related_nets[index], right.related_nets[index]))
            return false;
    }
    return true;
}

inline void checkedConstantAdd(CutWeight& total, CutWeight weight,
                               std::size_t net) {
    if (weight > std::numeric_limits<CutWeight>::max() - total)
        throw std::overflow_error("LAWLER_CONSTANT_SUM_OVERFLOW net=" +
                                  std::to_string(net));
    total += weight;
}

inline bool isCut(const std::vector<std::size_t>& endpoints,
                  const std::vector<int>& community_labels) {
    const int first = community_labels[endpoints.front()];
    for (std::size_t index = 1; index < endpoints.size(); ++index) {
        if (community_labels[endpoints[index]] != first) return true;
    }
    return false;
}

inline bool hasThirdCommunity(const CorridorNet& net, int community_i,
                              int community_j) {
    for (const CorridorEndpoint& endpoint : net.endpoints) {
        if (endpoint.community != community_i &&
            endpoint.community != community_j)
            return true;
    }
    return false;
}

}  // namespace lawler_network_detail

// Build the directed Lawler network for one current C2 pair corridor. The
// input hypergraph, labels, and corridor are read-only. The complete-graph
// constant contains every currently cut net that is unaffected by U_ij plus
// every related net with a fixed third-community endpoint.
inline PairLawlerNetwork buildPairLawlerNetwork(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels,
    const PairCorridor& corridor) {
    const PairCorridor rebuilt = buildPairCorridor(
        hypergraph, community_labels, corridor.community_i,
        corridor.community_j);
    if (!lawler_network_detail::sameCorridor(corridor, rebuilt))
        throw std::invalid_argument("LAWLER_CORRIDOR_STATE_MISMATCH");

    std::vector<const CorridorNet*> related_by_net(
        hypergraph.hyperedges.size(), nullptr);
    std::vector<const CorridorNet*> variable_nets;
    std::vector<CutWeight> finite_capacities;
    CutWeight constant_cut = 0;

    for (const CorridorNet& net : corridor.related_nets) {
        related_by_net[net.net] = &net;
        if (lawler_network_detail::hasThirdCommunity(
                net, corridor.community_i, corridor.community_j)) {
            lawler_network_detail::checkedConstantAdd(
                constant_cut, net.weight, net.net);
        } else {
            variable_nets.push_back(&net);
            finite_capacities.push_back(net.weight);
        }
    }

    // Nets outside E_ij contain no movable node, so their current cut status
    // is invariant under every feasible binary assignment of this corridor.
    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        if (hypergraph.hyperedges[edge].deleted || related_by_net[edge] != nullptr)
            continue;
        const std::vector<std::size_t> endpoints =
            flow_preparation_detail::activeNetEndpoints(hypergraph, edge);
        if (!lawler_network_detail::isCut(endpoints, community_labels))
            continue;
        lawler_network_detail::checkedConstantAdd(
            constant_cut,
            static_cast<CutWeight>(hypergraph.hyperedges[edge].weight), edge);
    }

    const CapacityBounds bounds = makeCapacityBounds(finite_capacities);

    PairLawlerNetwork result;
    result.community_i = corridor.community_i;
    result.community_j = corridor.community_j;
    result.source = 0;
    result.sink = 1;
    result.vertices.push_back(
        {LawlerVertexKind::Source, kNoFlowVertex});
    result.vertices.push_back(
        {LawlerVertexKind::Sink, kNoFlowVertex});
    result.node_to_vertex.assign(hypergraph.hypernodes.size(), kNoFlowVertex);
    result.constant_cut = constant_cut;
    result.finite_sum = bounds.finite_sum;
    result.infinity = bounds.infinity;

    for (const std::size_t node : corridor.fixed_i)
        result.node_to_vertex[node] = result.source;
    for (const std::size_t node : corridor.fixed_j)
        result.node_to_vertex[node] = result.sink;
    for (const std::size_t node : corridor.movable_nodes) {
        const std::size_t vertex = result.vertices.size();
        result.vertices.push_back({LawlerVertexKind::MovableNode, node});
        result.node_to_vertex[node] = vertex;
    }

    for (const CorridorNet* net : variable_nets) {
        const std::size_t in_vertex = result.vertices.size();
        result.vertices.push_back({LawlerVertexKind::NetIn, net->net});
        const std::size_t out_vertex = result.vertices.size();
        result.vertices.push_back({LawlerVertexKind::NetOut, net->net});
        result.gadgets.push_back({net->net, in_vertex, out_vertex});
        result.arcs.push_back({in_vertex, out_vertex, net->weight});

        for (const CorridorEndpoint& endpoint : net->endpoints) {
            const std::size_t vertex = result.node_to_vertex[endpoint.node];
            if (vertex == kNoFlowVertex)
                throw std::logic_error(
                    "LAWLER_UNMAPPED_PAIR_ENDPOINT net=" +
                    std::to_string(net->net) + " node=" +
                    std::to_string(endpoint.node));
            result.arcs.push_back({vertex, in_vertex, result.infinity});
            result.arcs.push_back({out_vertex, vertex, result.infinity});
        }
    }
    return result;
}

}  // namespace cf
