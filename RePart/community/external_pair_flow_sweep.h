#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "external_pair_transaction.h"
#include "pair_flow_sweep.h"

namespace cf {

struct ExternalPairFlowSweepRecord {
    std::size_t frozen_index;
    CommunityPair frozen_pair;
    PairCorridor current_corridor;
    std::string status;
    std::optional<ExternalPairTransactionResult> transaction;
};

struct ExternalPairFlowSweepResult {
    PairFlowSweepStatus status = PairFlowSweepStatus::Completed;
    std::string reason = "COMPLETED";
    std::vector<CommunityPair> frozen_pairs;
    std::vector<ExternalPairFlowSweepRecord> records;
    std::size_t accepted_pairs = 0;
    std::size_t rejected_pairs = 0;
    std::size_t disappeared_pairs = 0;
    std::size_t empty_corridor_pairs = 0;
    std::size_t no_movement_pairs = 0;
    std::size_t fallback_pairs = 0;
    std::size_t total_proposal_moved_nodes = 0;
    std::size_t total_committed_moved_nodes = 0;
    CutWeight cut_before = 0;
    CutWeight cut_after = 0;
};

namespace external_pair_sweep_detail {

inline void addCount(std::size_t& target, std::size_t value,
                     const char* message) {
    if (value > std::numeric_limits<std::size_t>::max() - target)
        throw std::overflow_error(message);
    target += value;
}

}  // namespace external_pair_sweep_detail

inline ExternalPairFlowSweepResult executeExternalPairFlowSweep(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels) {
    ExternalPairFlowSweepResult result;
    result.cut_before = recomputeCut(hypergraph, community_labels);
    result.cut_after = result.cut_before;
    result.frozen_pairs = buildCommunityPairs(hypergraph, community_labels);
    if (result.frozen_pairs.empty()) {
        result.status = PairFlowSweepStatus::SkippedNoPair;
        result.reason = pairFlowSweepStatusName(result.status);
        return result;
    }

    const ExternalProfileWorkspace workspace =
        buildExternalProfileWorkspace(hypergraph);
    result.records.reserve(result.frozen_pairs.size());
    CutWeight running_cut = result.cut_before;
    for (std::size_t index = 0; index < result.frozen_pairs.size(); ++index) {
        const CommunityPair frozen = result.frozen_pairs[index];
        PairCorridor corridor = buildPairCorridor(
            hypergraph, community_labels, frozen.community_i,
            frozen.community_j);
        if (corridor.boundary_weight == 0) {
            ++result.disappeared_pairs;
            result.records.push_back(
                {index, frozen, std::move(corridor),
                 "SKIPPED_DISAPPEARED_PAIR", std::nullopt});
            continue;
        }

        ExternalPairTransactionResult transaction =
            executeExternalSinglePairTransaction(
                hypergraph, workspace, community_labels,
                frozen.community_i, frozen.community_j);
        if (transaction.cut_before != running_cut)
            throw std::logic_error("EXTERNAL_SWEEP_CUT_BEFORE_MISMATCH");
        if (transaction.accepted) {
            ++result.accepted_pairs;
            running_cut = transaction.cut_after;
        } else {
            ++result.rejected_pairs;
            if (transaction.cut_after != transaction.cut_before)
                throw std::logic_error("EXTERNAL_SWEEP_REJECT_CHANGED_CUT");
        }
        if (transaction.used_cut_only_fallback) ++result.fallback_pairs;
        if (transaction.status ==
            ExternalPairTransactionStatus::SkippedEmptyCorridor)
            ++result.empty_corridor_pairs;
        if (transaction.status ==
            ExternalPairTransactionStatus::SkippedNoMovement)
            ++result.no_movement_pairs;
        external_pair_sweep_detail::addCount(
            result.total_proposal_moved_nodes,
            transaction.proposal_moved_nodes,
            "EXTERNAL_SWEEP_PROPOSAL_COUNT_OVERFLOW");
        external_pair_sweep_detail::addCount(
            result.total_committed_moved_nodes,
            transaction.committed_moved_nodes,
            "EXTERNAL_SWEEP_COMMITTED_COUNT_OVERFLOW");
        result.records.push_back(
            {index, frozen, std::move(corridor), transaction.reason,
             std::move(transaction)});
    }
    result.cut_after = recomputeCut(hypergraph, community_labels);
    if (result.cut_after != running_cut)
        throw std::logic_error("EXTERNAL_SWEEP_FINAL_CUT_MISMATCH");
    return result;
}

}  // namespace cf
