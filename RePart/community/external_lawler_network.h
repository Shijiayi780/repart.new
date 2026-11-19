#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "external_objective.h"
#include "lawler_network.h"

namespace cf {

struct ExternalTerminalCost {
    std::size_t node;
    ExternalDistance to_community_i;
    ExternalDistance to_community_j;
    CutWeight quantized_i;
    CutWeight quantized_j;
};

struct ExternalPairLawlerNetwork {
    PairCorridor corridor;
    PairLawlerNetwork network;
    std::vector<ExternalTerminalCost> terminal_costs;
    long double kappa = 0.0L;
    std::size_t informative_corridor_nodes = 0;
    CutWeight capacity_scale = kExternalCapacityScaleV0;
};

namespace external_lawler_detail {

inline CutWeight checkedMultiply(CutWeight left, CutWeight right,
                                 const char* message) {
    if (left != 0 && right > std::numeric_limits<CutWeight>::max() / left)
        throw std::overflow_error(message);
    return left * right;
}

inline void checkedAdd(CutWeight& target, CutWeight value,
                       const char* message) {
    if (value > std::numeric_limits<CutWeight>::max() - target)
        throw std::overflow_error(message);
    target += value;
}

inline CutWeight quantize(long double value) {
    if (!(value >= 0.0L) || !std::isfinite(value))
        throw std::invalid_argument("EXTERNAL_CAPACITY_INVALID_VALUE");
    const long double scaled =
        value * static_cast<long double>(kExternalCapacityScaleV0);
    if (scaled >
        static_cast<long double>(std::numeric_limits<CutWeight>::max()))
        throw std::overflow_error("EXTERNAL_CAPACITY_QUANTIZE_OVERFLOW");
    return static_cast<CutWeight>(std::round(scaled));
}

}  // namespace external_lawler_detail

// Build D's proxy network.  Every cut arc and every unary arc is represented
// in the same 10^6 fixed-point unit; INF is recomputed after unary insertion.
inline ExternalPairLawlerNetwork buildExternalPairLawlerNetwork(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    const std::vector<int>& community_labels,
    const PairCorridor& corridor) {
    PairLawlerNetwork base =
        buildPairLawlerNetwork(hypergraph, community_labels, corridor);
    const auto snapshot_i = external_objective_detail::buildCommunitySnapshot(
        hypergraph, workspace, community_labels, corridor.community_i);
    const auto snapshot_j = external_objective_detail::buildCommunitySnapshot(
        hypergraph, workspace, community_labels, corridor.community_j);

    ExternalPairLawlerNetwork result;
    result.corridor = corridor;
    result.network = base;
    result.terminal_costs.reserve(corridor.movable_nodes.size());
    for (const std::size_t node : corridor.movable_nodes) {
        const ExternalDistance distance_i = frozenExternalDistance(
            hypergraph, workspace, community_labels, node, snapshot_i);
        const ExternalDistance distance_j = frozenExternalDistance(
            hypergraph, workspace, community_labels, node, snapshot_j);
        if (distance_i.informative || distance_j.informative)
            ++result.informative_corridor_nodes;
        result.terminal_costs.push_back(
            {node, distance_i, distance_j, 0, 0});
    }

    const std::size_t denominator =
        std::max<std::size_t>(1, result.informative_corridor_nodes);
    result.kappa = static_cast<long double>(base.finite_sum) /
                   static_cast<long double>(denominator);

    CutWeight finite_sum = 0;
    std::vector<unsigned char> infinite_arc(result.network.arcs.size(), 0);
    for (std::size_t index = 0; index < result.network.arcs.size(); ++index) {
        LawlerArc& arc = result.network.arcs[index];
        if (arc.capacity == base.infinity) {
            infinite_arc[index] = 1;
            continue;
        }
        arc.capacity = external_lawler_detail::checkedMultiply(
            arc.capacity, kExternalCapacityScaleV0,
            "EXTERNAL_BASE_CAPACITY_OVERFLOW");
        external_lawler_detail::checkedAdd(
            finite_sum, arc.capacity, "EXTERNAL_FINITE_SUM_OVERFLOW");
    }

    for (ExternalTerminalCost& terminal : result.terminal_costs) {
        terminal.quantized_i = external_lawler_detail::quantize(
            kExternalLambdaV0 * result.kappa * terminal.to_community_i.distance);
        terminal.quantized_j = external_lawler_detail::quantize(
            kExternalLambdaV0 * result.kappa * terminal.to_community_j.distance);
        external_lawler_detail::checkedAdd(
            finite_sum, terminal.quantized_i,
            "EXTERNAL_TERMINAL_SUM_OVERFLOW");
        external_lawler_detail::checkedAdd(
            finite_sum, terminal.quantized_j,
            "EXTERNAL_TERMINAL_SUM_OVERFLOW");
    }
    if (finite_sum == std::numeric_limits<CutWeight>::max())
        throw std::overflow_error("EXTERNAL_INFINITY_OVERFLOW");
    const CutWeight infinity = finite_sum + 1;
    for (std::size_t index = 0; index < result.network.arcs.size(); ++index)
        if (infinite_arc[index] != 0)
            result.network.arcs[index].capacity = infinity;

    // source->A is cut when A is assigned to Cj; A->sink is cut for Ci.
    for (const ExternalTerminalCost& terminal : result.terminal_costs) {
        const std::size_t vertex = result.network.node_to_vertex.at(terminal.node);
        if (vertex == kNoFlowVertex)
            throw std::logic_error("EXTERNAL_TERMINAL_UNMAPPED_NODE");
        result.network.arcs.push_back(
            {result.network.source, vertex, terminal.quantized_j});
        result.network.arcs.push_back(
            {vertex, result.network.sink, terminal.quantized_i});
    }
    result.network.finite_sum = finite_sum;
    result.network.infinity = infinity;
    result.network.constant_cut = external_lawler_detail::checkedMultiply(
        base.constant_cut, kExternalCapacityScaleV0,
        "EXTERNAL_CONSTANT_CAPACITY_OVERFLOW");
    return result;
}

inline ExternalPairLawlerNetwork buildExternalPairLawlerNetwork(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    const std::vector<int>& community_labels,
    int community_a, int community_b) {
    const PairCorridor corridor = buildPairCorridor(
        hypergraph, community_labels, community_a, community_b);
    return buildExternalPairLawlerNetwork(
        hypergraph, workspace, community_labels, corridor);
}

}  // namespace cf
