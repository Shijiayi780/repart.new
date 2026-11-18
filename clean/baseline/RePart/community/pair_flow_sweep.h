#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "single_pair_transaction.h"

namespace cf {

enum class PairFlowSweepStatus {
    Completed,
    SkippedNoPair
};

inline const char* pairFlowSweepStatusName(PairFlowSweepStatus status) {
    switch (status) {
        case PairFlowSweepStatus::Completed:
            return "COMPLETED";
        case PairFlowSweepStatus::SkippedNoPair:
            return "SKIPPED_NO_PAIR";
    }
    return "UNKNOWN";
}

enum class PairFlowSweepRecordStatus {
    Accepted,
    RejectedNonDecreasing,
    SkippedDisappearedPair,
    SkippedEmptyCorridor,
    SkippedNoMovement
};

inline const char* pairFlowSweepRecordStatusName(
    PairFlowSweepRecordStatus status) {
    switch (status) {
        case PairFlowSweepRecordStatus::Accepted:
            return "ACCEPTED";
        case PairFlowSweepRecordStatus::RejectedNonDecreasing:
            return "REJECTED_NON_DECREASING";
        case PairFlowSweepRecordStatus::SkippedDisappearedPair:
            return "SKIPPED_DISAPPEARED_PAIR";
        case PairFlowSweepRecordStatus::SkippedEmptyCorridor:
            return "SKIPPED_EMPTY_CORRIDOR";
        case PairFlowSweepRecordStatus::SkippedNoMovement:
            return "SKIPPED_NO_MOVEMENT";
    }
    return "UNKNOWN";
}

struct PairFlowSweepRecord {
    std::size_t frozen_index;
    CommunityPair frozen_pair;
    // Complete C2 reconstruction at the moment this frozen pair is reached.
    // It may differ from frozen_pair in weight and anchors because prior
    // accepted transactions are already visible.
    PairCorridor current_corridor;
    PairFlowSweepRecordStatus status;
    std::string reason;
    std::optional<PairTransactionResult> transaction;
    std::size_t proposal_moved_nodes;
    std::size_t committed_moved_nodes;
    CutWeight cut_before;
    CutWeight cut_after;
};

struct PairFlowSweepResult {
    PairFlowSweepStatus status;
    std::string reason;
    std::vector<CommunityPair> frozen_pairs;
    std::vector<PairFlowSweepRecord> records;
    std::size_t accepted_pairs;
    std::size_t rejected_pairs;
    std::size_t disappeared_pairs;
    std::size_t empty_corridor_pairs;
    std::size_t no_movement_pairs;
    std::size_t total_proposal_moved_nodes;
    std::size_t total_committed_moved_nodes;
    CutWeight cut_before;
    CutWeight cut_after;
};

namespace pair_flow_sweep_detail {

inline void checkedCountAdd(std::size_t& total, std::size_t increment,
                            const char* field) {
    if (increment > std::numeric_limits<std::size_t>::max() - total)
        throw std::overflow_error(std::string("PAIR_SWEEP_COUNT_OVERFLOW field=") +
                                  field);
    total += increment;
}

inline PairFlowSweepRecordStatus mapTransactionStatus(
    PairTransactionStatus status) {
    switch (status) {
        case PairTransactionStatus::Accepted:
            return PairFlowSweepRecordStatus::Accepted;
        case PairTransactionStatus::RejectedNonDecreasing:
            return PairFlowSweepRecordStatus::RejectedNonDecreasing;
        case PairTransactionStatus::SkippedEmptyCorridor:
            return PairFlowSweepRecordStatus::SkippedEmptyCorridor;
        case PairTransactionStatus::SkippedNoMovement:
            return PairFlowSweepRecordStatus::SkippedNoMovement;
        case PairTransactionStatus::SkippedNoPair:
            throw std::logic_error("PAIR_SWEEP_UNEXPECTED_SINGLE_PAIR_NO_PAIR");
    }
    throw std::logic_error("PAIR_SWEEP_UNKNOWN_TRANSACTION_STATUS");
}

inline void countRecord(PairFlowSweepResult& result,
                        PairFlowSweepRecordStatus status) {
    switch (status) {
        case PairFlowSweepRecordStatus::Accepted:
            checkedCountAdd(result.accepted_pairs, 1, "accepted_pairs");
            return;
        case PairFlowSweepRecordStatus::RejectedNonDecreasing:
            checkedCountAdd(result.rejected_pairs, 1, "rejected_pairs");
            return;
        case PairFlowSweepRecordStatus::SkippedDisappearedPair:
            checkedCountAdd(result.disappeared_pairs, 1,
                            "disappeared_pairs");
            return;
        case PairFlowSweepRecordStatus::SkippedEmptyCorridor:
            checkedCountAdd(result.empty_corridor_pairs, 1,
                            "empty_corridor_pairs");
            return;
        case PairFlowSweepRecordStatus::SkippedNoMovement:
            checkedCountAdd(result.no_movement_pairs, 1,
                            "no_movement_pairs");
            return;
    }
    throw std::logic_error("PAIR_SWEEP_UNKNOWN_RECORD_STATUS");
}

}  // namespace pair_flow_sweep_detail

// Run exactly one deterministic sweep. The pair list is built once and kept
// unchanged. Every entry is reached once in that frozen order, while its C2/C3
// input is rebuilt from the labels left by all earlier accepted entries.
inline PairFlowSweepResult executePairFlowSweep(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels) {
    const CutWeight initial_cut = recomputeCut(hypergraph, community_labels);
    PairFlowSweepResult result{
        PairFlowSweepStatus::Completed,
        pairFlowSweepStatusName(PairFlowSweepStatus::Completed),
        buildCommunityPairs(hypergraph, community_labels),
        {},
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        initial_cut,
        initial_cut};

    if (result.frozen_pairs.empty()) {
        result.status = PairFlowSweepStatus::SkippedNoPair;
        result.reason = pairFlowSweepStatusName(result.status);
        return result;
    }

    result.records.reserve(result.frozen_pairs.size());
    CutWeight running_cut = initial_cut;
    for (std::size_t index = 0; index < result.frozen_pairs.size(); ++index) {
        const CommunityPair frozen_pair = result.frozen_pairs[index];
        PairCorridor current_corridor = buildPairCorridor(
            hypergraph, community_labels, frozen_pair.community_i,
            frozen_pair.community_j);

        if (current_corridor.boundary_weight == 0) {
            const PairFlowSweepRecordStatus status =
                PairFlowSweepRecordStatus::SkippedDisappearedPair;
            pair_flow_sweep_detail::countRecord(result, status);
            result.records.push_back(
                {index,
                 frozen_pair,
                 std::move(current_corridor),
                 status,
                 pairFlowSweepRecordStatusName(status),
                 std::nullopt,
                 0,
                 0,
                 running_cut,
                 running_cut});
            continue;
        }

        const PairTransactionResult transaction =
            executeSinglePairTransaction(
                hypergraph, community_labels, frozen_pair.community_i,
                frozen_pair.community_j);
        if (transaction.cut_before != running_cut)
            throw std::logic_error(
                "PAIR_SWEEP_CUT_TRACE_BEFORE_MISMATCH index=" +
                std::to_string(index));

        const PairFlowSweepRecordStatus status =
            pair_flow_sweep_detail::mapTransactionStatus(transaction.status);
        if (status == PairFlowSweepRecordStatus::Accepted) {
            if (transaction.cut_after >= transaction.cut_before)
                throw std::logic_error(
                    "PAIR_SWEEP_ACCEPT_NOT_STRICT index=" +
                    std::to_string(index));
        } else if (transaction.cut_after != transaction.cut_before) {
            throw std::logic_error(
                "PAIR_SWEEP_NON_ACCEPT_CHANGED_CUT index=" +
                std::to_string(index));
        }

        pair_flow_sweep_detail::countRecord(result, status);
        pair_flow_sweep_detail::checkedCountAdd(
            result.total_proposal_moved_nodes,
            transaction.proposal_moved_nodes,
            "total_proposal_moved_nodes");
        pair_flow_sweep_detail::checkedCountAdd(
            result.total_committed_moved_nodes,
            transaction.committed_moved_nodes,
            "total_committed_moved_nodes");
        running_cut = transaction.cut_after;
        result.records.push_back(
            {index,
             frozen_pair,
             std::move(current_corridor),
             status,
             pairFlowSweepRecordStatusName(status),
             transaction,
             transaction.proposal_moved_nodes,
             transaction.committed_moved_nodes,
             transaction.cut_before,
             transaction.cut_after});
    }

    if (result.records.size() != result.frozen_pairs.size())
        throw std::logic_error("PAIR_SWEEP_RECORD_COUNT_MISMATCH");
    const CutWeight final_cut = recomputeCut(hypergraph, community_labels);
    if (final_cut != running_cut)
        throw std::logic_error("PAIR_SWEEP_FINAL_CUT_TRACE_MISMATCH");
    result.cut_after = final_cut;
    return result;
}

}  // namespace cf

