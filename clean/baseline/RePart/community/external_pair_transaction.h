#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "external_lawler_network.h"
#include "single_pair_transaction.h"

namespace cf {

enum class ExternalPairTransactionStatus {
    Accepted,
    RejectedNonDecreasingObjective,
    SkippedEmptyCorridor,
    SkippedNoMovement,
    CutOnlyFallbackAccepted,
    CutOnlyFallbackNotAccepted
};

inline const char* externalPairTransactionStatusName(
    ExternalPairTransactionStatus status) {
    switch (status) {
        case ExternalPairTransactionStatus::Accepted:
            return "ACCEPTED";
        case ExternalPairTransactionStatus::RejectedNonDecreasingObjective:
            return "REJECTED_NON_DECREASING_J";
        case ExternalPairTransactionStatus::SkippedEmptyCorridor:
            return "SKIPPED_EMPTY_CORRIDOR";
        case ExternalPairTransactionStatus::SkippedNoMovement:
            return "SKIPPED_NO_MOVEMENT";
        case ExternalPairTransactionStatus::CutOnlyFallbackAccepted:
            return "CUT_ONLY_FALLBACK_ACCEPTED";
        case ExternalPairTransactionStatus::CutOnlyFallbackNotAccepted:
            return "CUT_ONLY_FALLBACK_NOT_ACCEPTED";
    }
    return "UNKNOWN";
}

struct ExternalPairTransactionResult {
    ExternalPairTransactionStatus status;
    std::string reason;
    bool accepted = false;
    bool used_cut_only_fallback = false;
    std::size_t proposal_moved_nodes = 0;
    std::size_t committed_moved_nodes = 0;
    CutWeight cut_before = 0;
    CutWeight cut_after = 0;
    long double normalized_cut_before = 0.0L;
    long double normalized_cut_after = 0.0L;
    std::optional<long double> phi_before;
    std::optional<long double> phi_after;
    std::optional<long double> objective_before;
    std::optional<long double> objective_after;
    std::size_t comparable_before = 0;
    std::size_t comparable_after = 0;
    CutWeight max_flow = 0;
    CutWeight proxy_energy = 0;
    long double kappa = 0.0L;
    std::size_t informative_corridor_nodes = 0;
};

namespace external_pair_transaction_detail {

inline CutWeight checkedProxyAdd(CutWeight target, CutWeight value) {
    if (value > std::numeric_limits<CutWeight>::max() - target)
        throw std::overflow_error("EXTERNAL_PROXY_ENERGY_OVERFLOW");
    return target + value;
}

inline ExternalPairTransactionResult fallbackResult(
    const PairTransactionResult& fallback,
    const ExternalPairObjective& before) {
    const bool accepted = fallback.status == PairTransactionStatus::Accepted;
    ExternalPairTransactionResult result;
    result.status = accepted
        ? ExternalPairTransactionStatus::CutOnlyFallbackAccepted
        : ExternalPairTransactionStatus::CutOnlyFallbackNotAccepted;
    result.reason = externalPairTransactionStatusName(result.status);
    result.accepted = accepted;
    result.used_cut_only_fallback = true;
    result.proposal_moved_nodes = fallback.proposal_moved_nodes;
    result.committed_moved_nodes = fallback.committed_moved_nodes;
    result.cut_before = fallback.cut_before;
    result.cut_after = fallback.cut_after;
    result.normalized_cut_before = before.normalized_cut;
    result.comparable_before = before.comparable_members;
    result.phi_before = before.phi;
    result.objective_before = before.objective;
    result.max_flow = fallback.max_flow;
    result.proxy_energy = single_pair_transaction_detail::checkedCompleteObjective(
        fallback.max_flow, fallback.constant_cut);
    return result;
}

}  // namespace external_pair_transaction_detail

inline ExternalPairTransactionResult executeExternalSinglePairTransaction(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    std::vector<int>& community_labels,
    int community_a, int community_b) {
    const PairCorridor corridor = buildPairCorridor(
        hypergraph, community_labels, community_a, community_b);
    const ExternalPairObjective before = recomputeExternalPairObjective(
        hypergraph, workspace, community_labels, corridor.community_i,
        corridor.community_j);

    // No real external comparison means D has no signal for this pair.  Run
    // the original C transaction from the untouched entry state.
    if (!before.phi)
        return external_pair_transaction_detail::fallbackResult(
            executeSinglePairTransaction(hypergraph, community_labels,
                                         community_a, community_b),
            before);

    const ExternalPairLawlerNetwork prepared =
        buildExternalPairLawlerNetwork(hypergraph, workspace,
                                       community_labels, corridor);
    const DeterministicMinCutResult solved =
        solveDeterministicMinCut(prepared.network);
    if (solved.max_flow != solved.cut_capacity)
        throw std::logic_error("EXTERNAL_FLOW_CUT_MISMATCH");

    std::vector<int> proposal = community_labels;
    std::vector<unsigned char> movable_mask(community_labels.size(), 0);
    std::size_t moved = 0;
    for (const std::size_t node : corridor.movable_nodes) {
        const std::size_t vertex = prepared.network.node_to_vertex.at(node);
        if (vertex == kNoFlowVertex ||
            prepared.network.vertices.at(vertex).kind !=
                LawlerVertexKind::MovableNode)
            throw std::logic_error("EXTERNAL_MOVABLE_MAPPING_MISMATCH");
        movable_mask[node] = 1;
        const int target = solved.source_reachable.at(vertex) != 0
            ? corridor.community_i
            : corridor.community_j;
        if (proposal[node] != target) ++moved;
        proposal[node] = target;
    }
    for (std::size_t node = 0; node < proposal.size(); ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        if (proposal[node] < 0)
            throw std::logic_error("EXTERNAL_ILLEGAL_PROPOSAL_LABEL");
        if (movable_mask[node] == 0 &&
            proposal[node] != community_labels[node])
            throw std::logic_error("EXTERNAL_OUTSIDE_CORRIDOR_CHANGED");
    }
    if (proposal[corridor.anchor_i] != community_labels[corridor.anchor_i] ||
        proposal[corridor.anchor_j] != community_labels[corridor.anchor_j])
        throw std::logic_error("EXTERNAL_ANCHOR_CHANGED");

    const CutWeight cut_after = recomputeCut(hypergraph, proposal);
    CutWeight proxy = external_lawler_detail::checkedMultiply(
        cut_after, kExternalCapacityScaleV0,
        "EXTERNAL_PROXY_CUT_OVERFLOW");
    for (const ExternalTerminalCost& terminal : prepared.terminal_costs) {
        proxy = external_pair_transaction_detail::checkedProxyAdd(
            proxy, proposal[terminal.node] == corridor.community_i
                       ? terminal.quantized_i
                       : terminal.quantized_j);
    }
    const CutWeight solved_complete =
        single_pair_transaction_detail::checkedCompleteObjective(
            solved.max_flow, prepared.network.constant_cut);
    if (proxy != solved_complete)
        throw std::logic_error("EXTERNAL_PROXY_OBJECTIVE_MISMATCH");

    ExternalPairObjective after = recomputeExternalPairObjective(
        hypergraph, workspace, proposal, corridor.community_i,
        corridor.community_j);
    if (!after.phi)
        return external_pair_transaction_detail::fallbackResult(
            executeSinglePairTransaction(hypergraph, community_labels,
                                         community_a, community_b),
            before);

    ExternalPairTransactionResult result;
    result.proposal_moved_nodes = moved;
    result.cut_before = before.raw_cut;
    result.cut_after = before.raw_cut;
    result.normalized_cut_before = before.normalized_cut;
    result.normalized_cut_after = after.normalized_cut;
    result.phi_before = before.phi;
    result.phi_after = after.phi;
    result.objective_before = before.objective;
    result.objective_after = after.objective;
    result.comparable_before = before.comparable_members;
    result.comparable_after = after.comparable_members;
    result.max_flow = solved.max_flow;
    result.proxy_energy = proxy;
    result.kappa = prepared.kappa;
    result.informative_corridor_nodes =
        prepared.informative_corridor_nodes;

    if (corridor.movable_nodes.empty()) {
        result.status = ExternalPairTransactionStatus::SkippedEmptyCorridor;
    } else if (moved == 0) {
        result.status = ExternalPairTransactionStatus::SkippedNoMovement;
    } else if (!(*after.objective < *before.objective)) {
        result.status =
            ExternalPairTransactionStatus::RejectedNonDecreasingObjective;
    } else {
        result.status = ExternalPairTransactionStatus::Accepted;
        result.accepted = true;
        result.committed_moved_nodes = moved;
        result.cut_after = cut_after;
        community_labels.swap(proposal);
    }
    result.reason = externalPairTransactionStatusName(result.status);
    return result;
}

}  // namespace cf
