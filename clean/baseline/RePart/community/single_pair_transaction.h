#pragma once

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "deterministic_dinic.h"

namespace cf {

enum class PairTransactionStatus {
    Accepted,
    RejectedNonDecreasing,
    SkippedNoPair,
    SkippedEmptyCorridor,
    SkippedNoMovement
};

inline const char* pairTransactionStatusName(PairTransactionStatus status) {
    switch (status) {
        case PairTransactionStatus::Accepted:
            return "ACCEPTED";
        case PairTransactionStatus::RejectedNonDecreasing:
            return "REJECTED_NON_DECREASING";
        case PairTransactionStatus::SkippedNoPair:
            return "SKIPPED_NO_PAIR";
        case PairTransactionStatus::SkippedEmptyCorridor:
            return "SKIPPED_EMPTY_CORRIDOR";
        case PairTransactionStatus::SkippedNoMovement:
            return "SKIPPED_NO_MOVEMENT";
    }
    return "UNKNOWN";
}

// This mapping is deliberately explicit. Community IDs are not ordered by
// value: C2 orients a pair by its current anchors, and C3 maps that oriented
// community_i to source and community_j to sink.
struct PairSideCommunityMapping {
    std::size_t source_vertex;
    std::size_t sink_vertex;
    int source_community;
    int sink_community;
};

struct PreparedSinglePairTransaction {
    PairCorridor corridor;
    PairLawlerNetwork network;
    PairSideCommunityMapping side_mapping;
};

struct PairTransactionResult {
    PairTransactionStatus status;
    std::string reason;
    int source_community;
    int sink_community;
    std::size_t proposal_moved_nodes;
    std::size_t committed_moved_nodes;
    CutWeight cut_before;
    CutWeight cut_after;
    CutWeight max_flow;
    CutWeight constant_cut;
};

namespace single_pair_transaction_detail {

inline bool sameVertex(const LawlerVertex& left, const LawlerVertex& right) {
    return left.kind == right.kind && left.object == right.object;
}

inline bool sameArc(const LawlerArc& left, const LawlerArc& right) {
    return left.from == right.from && left.to == right.to &&
           left.capacity == right.capacity;
}

inline bool sameGadget(const LawlerGadgetMapping& left,
                       const LawlerGadgetMapping& right) {
    return left.net == right.net && left.in_vertex == right.in_vertex &&
           left.out_vertex == right.out_vertex;
}

inline bool sameNetwork(const PairLawlerNetwork& left,
                        const PairLawlerNetwork& right) {
    if (left.community_i != right.community_i ||
        left.community_j != right.community_j || left.source != right.source ||
        left.sink != right.sink ||
        left.node_to_vertex != right.node_to_vertex ||
        left.constant_cut != right.constant_cut ||
        left.finite_sum != right.finite_sum ||
        left.infinity != right.infinity ||
        left.vertices.size() != right.vertices.size() ||
        left.gadgets.size() != right.gadgets.size() ||
        left.arcs.size() != right.arcs.size())
        return false;
    for (std::size_t index = 0; index < left.vertices.size(); ++index) {
        if (!sameVertex(left.vertices[index], right.vertices[index]))
            return false;
    }
    for (std::size_t index = 0; index < left.gadgets.size(); ++index) {
        if (!sameGadget(left.gadgets[index], right.gadgets[index]))
            return false;
    }
    for (std::size_t index = 0; index < left.arcs.size(); ++index) {
        if (!sameArc(left.arcs[index], right.arcs[index])) return false;
    }
    return true;
}

inline bool sameMapping(const PairSideCommunityMapping& left,
                        const PairSideCommunityMapping& right) {
    return left.source_vertex == right.source_vertex &&
           left.sink_vertex == right.sink_vertex &&
           left.source_community == right.source_community &&
           left.sink_community == right.sink_community;
}

inline CutWeight checkedCompleteObjective(CutWeight max_flow,
                                          CutWeight constant_cut) {
    if (constant_cut > std::numeric_limits<CutWeight>::max() - max_flow)
        throw std::overflow_error("PAIR_TX_OBJECTIVE_SUM_OVERFLOW");
    return max_flow + constant_cut;
}

inline PairTransactionResult result(PairTransactionStatus status,
                                    int source_community,
                                    int sink_community,
                                    std::size_t proposal_moved_nodes,
                                    std::size_t committed_moved_nodes,
                                    CutWeight cut_before,
                                    CutWeight cut_after,
                                    CutWeight max_flow,
                                    CutWeight constant_cut) {
    return {status,
            pairTransactionStatusName(status),
            source_community,
            sink_community,
            proposal_moved_nodes,
            committed_moved_nodes,
            cut_before,
            cut_after,
            max_flow,
            constant_cut};
}

}  // namespace single_pair_transaction_detail

inline PairSideCommunityMapping makePairSideCommunityMapping(
    const PairLawlerNetwork& network) {
    return {network.source, network.sink, network.community_i,
            network.community_j};
}

// Construct a current C2/C3 snapshot and its explicit terminal-to-community
// mapping. No labels or hypergraph fields are modified.
inline PreparedSinglePairTransaction prepareSinglePairTransaction(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels,
    int community_a,
    int community_b) {
    PairCorridor corridor = buildPairCorridor(
        hypergraph, community_labels, community_a, community_b);
    PairLawlerNetwork network = buildPairLawlerNetwork(
        hypergraph, community_labels, corridor);
    PairSideCommunityMapping mapping =
        makePairSideCommunityMapping(network);
    return {std::move(corridor), std::move(network), mapping};
}

// Verify that every prepared C2/C3 field and the explicit side mapping still
// describe the current H,c state. A mismatch is a hard error, never a skip.
inline void validatePreparedSinglePairTransaction(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels,
    const PreparedSinglePairTransaction& prepared) {
    PairCorridor current_corridor;
    try {
        current_corridor = buildPairCorridor(
            hypergraph, community_labels, prepared.corridor.community_i,
            prepared.corridor.community_j);
    } catch (const std::exception& error) {
        throw std::invalid_argument(
            std::string("PAIR_TX_CORRIDOR_STATE_MISMATCH cause=") +
            error.what());
    }
    if (!lawler_network_detail::sameCorridor(prepared.corridor,
                                              current_corridor))
        throw std::invalid_argument("PAIR_TX_CORRIDOR_STATE_MISMATCH");

    const PairLawlerNetwork current_network = buildPairLawlerNetwork(
        hypergraph, community_labels, current_corridor);
    if (!single_pair_transaction_detail::sameNetwork(prepared.network,
                                                      current_network))
        throw std::invalid_argument("PAIR_TX_NETWORK_STATE_MISMATCH");

    const PairSideCommunityMapping current_mapping =
        makePairSideCommunityMapping(current_network);
    if (!single_pair_transaction_detail::sameMapping(prepared.side_mapping,
                                                      current_mapping))
        throw std::invalid_argument("PAIR_TX_SIDE_MAPPING_MISMATCH");
}

// Validate a C4 result, build the proposal on a private label copy, recompute
// both complete objectives through C1, and commit with one swap only after all
// invariants pass. This stage is public so tests can inject damaged solve
// evidence and verify that no partial commit is possible; the normal entry
// point below always obtains solved from C4 itself.
inline PairTransactionResult executeSolvedSinglePairTransaction(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels,
    const PreparedSinglePairTransaction& prepared,
    const DeterministicMinCutResult& solved) {
    validatePreparedSinglePairTransaction(hypergraph, community_labels,
                                          prepared);
    const PairCorridor& corridor = prepared.corridor;
    const PairLawlerNetwork& network = prepared.network;
    const PairSideCommunityMapping& mapping = prepared.side_mapping;

    if (solved.source_reachable.size() != network.vertices.size() ||
        solved.source_reachable[network.source] == 0 ||
        solved.source_reachable[network.sink] != 0)
        throw std::invalid_argument("PAIR_TX_INVALID_CUT_SIDE");
    if (solved.max_flow != solved.cut_capacity)
        throw std::logic_error("PAIR_TX_FLOW_CUT_MISMATCH");

    const CutWeight cut_before = recomputeCut(hypergraph, community_labels);
    std::vector<int> proposal = community_labels;
    std::vector<unsigned char> movable_mask(community_labels.size(), 0);
    std::size_t proposal_moved_nodes = 0;

    for (const std::size_t node : corridor.movable_nodes) {
        if (node >= proposal.size())
            throw std::logic_error("PAIR_TX_MOVABLE_NODE_RANGE");
        const std::size_t vertex = network.node_to_vertex[node];
        if (vertex == kNoFlowVertex || vertex >= network.vertices.size() ||
            network.vertices[vertex].kind != LawlerVertexKind::MovableNode ||
            network.vertices[vertex].object != node)
            throw std::logic_error("PAIR_TX_MOVABLE_MAPPING_MISMATCH node=" +
                                   std::to_string(node));
        movable_mask[node] = 1;
        const int proposed_community =
            solved.source_reachable[vertex] != 0
                ? mapping.source_community
                : mapping.sink_community;
        if (proposal[node] != proposed_community) ++proposal_moved_nodes;
        proposal[node] = proposed_community;
    }

    // Prove that the proposal changed only U_ij. This also covers both anchors,
    // every third-community node, and every other pair-internal fixed node.
    for (std::size_t node = 0; node < proposal.size(); ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        if (proposal[node] < 0)
            throw std::logic_error("PAIR_TX_ILLEGAL_PROPOSAL_LABEL node=" +
                                   std::to_string(node));
        if (movable_mask[node] == 0 &&
            proposal[node] != community_labels[node])
            throw std::logic_error("PAIR_TX_OUTSIDE_CORRIDOR_CHANGED node=" +
                                   std::to_string(node));
        if (movable_mask[node] != 0 &&
            proposal[node] != mapping.source_community &&
            proposal[node] != mapping.sink_community)
            throw std::logic_error("PAIR_TX_PROPOSAL_OUTSIDE_PAIR node=" +
                                   std::to_string(node));
    }
    if (proposal[corridor.anchor_i] !=
            community_labels[corridor.anchor_i] ||
        proposal[corridor.anchor_j] != community_labels[corridor.anchor_j])
        throw std::logic_error("PAIR_TX_ANCHOR_CHANGED");

    const CutWeight cut_after = recomputeCut(hypergraph, proposal);
    const CutWeight expected_after =
        single_pair_transaction_detail::checkedCompleteObjective(
            solved.max_flow, network.constant_cut);
    if (cut_after != expected_after)
        throw std::logic_error(
            "PAIR_TX_OBJECTIVE_MISMATCH cut_after=" +
            std::to_string(cut_after) + " flow_plus_constant=" +
            std::to_string(expected_after));
    if (cut_after > cut_before)
        throw std::logic_error(
            "PAIR_TX_WORSE_THAN_FEASIBLE_CURRENT cut_before=" +
            std::to_string(cut_before) + " cut_after=" +
            std::to_string(cut_after));

    if (corridor.movable_nodes.empty())
        return single_pair_transaction_detail::result(
            PairTransactionStatus::SkippedEmptyCorridor,
            mapping.source_community, mapping.sink_community, 0, 0,
            cut_before, cut_after, solved.max_flow, network.constant_cut);
    if (proposal_moved_nodes == 0)
        return single_pair_transaction_detail::result(
            PairTransactionStatus::SkippedNoMovement,
            mapping.source_community, mapping.sink_community, 0, 0,
            cut_before, cut_after, solved.max_flow, network.constant_cut);
    if (cut_after == cut_before)
        return single_pair_transaction_detail::result(
            PairTransactionStatus::RejectedNonDecreasing,
            mapping.source_community, mapping.sink_community,
            proposal_moved_nodes, 0, cut_before, cut_after, solved.max_flow,
            network.constant_cut);

    community_labels.swap(proposal);
    return single_pair_transaction_detail::result(
        PairTransactionStatus::Accepted, mapping.source_community,
        mapping.sink_community, proposal_moved_nodes, proposal_moved_nodes,
        cut_before, cut_after, solved.max_flow, network.constant_cut);
}

inline PairTransactionResult executePreparedSinglePairTransaction(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels,
    const PreparedSinglePairTransaction& prepared) {
    validatePreparedSinglePairTransaction(hypergraph, community_labels,
                                          prepared);
    const DeterministicMinCutResult solved =
        solveDeterministicMinCut(prepared.network);
    return executeSolvedSinglePairTransaction(
        hypergraph, community_labels, prepared, solved);
}

inline PairTransactionResult executeSinglePairTransaction(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels,
    int community_a,
    int community_b) {
    const PreparedSinglePairTransaction prepared =
        prepareSinglePairTransaction(hypergraph, community_labels,
                                     community_a, community_b);
    return executePreparedSinglePairTransaction(hypergraph, community_labels,
                                                prepared);
}

// Execute only the first pair in C2's frozen deterministic order. This helper
// exists for the no-pair path and future lifecycle wiring; it is not a sweep.
inline PairTransactionResult executeFirstSinglePairTransaction(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels) {
    const std::vector<CommunityPair> pairs =
        buildCommunityPairs(hypergraph, community_labels);
    if (pairs.empty()) {
        const CutWeight cut = recomputeCut(hypergraph, community_labels);
        return single_pair_transaction_detail::result(
            PairTransactionStatus::SkippedNoPair, -1, -1, 0, 0, cut, cut, 0,
            0);
    }
    return executeSinglePairTransaction(
        hypergraph, community_labels, pairs.front().community_i,
        pairs.front().community_j);
}

}  // namespace cf

