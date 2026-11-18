#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "external_pair_flow_sweep.h"

namespace cf {

enum class ExternalSweepCutTransactionStatus {
    Accepted,
    RejectedCutCeiling,
    SkippedNoMovement
};

inline const char* externalSweepCutTransactionStatusName(
    ExternalSweepCutTransactionStatus status) {
    switch (status) {
        case ExternalSweepCutTransactionStatus::Accepted:
            return "ACCEPTED";
        case ExternalSweepCutTransactionStatus::RejectedCutCeiling:
            return "REJECTED_CUT_CEILING";
        case ExternalSweepCutTransactionStatus::SkippedNoMovement:
            return "SKIPPED_NO_MOVEMENT";
    }
    return "UNKNOWN";
}

struct ExternalSweepCutTransactionResult {
    ExternalSweepCutTransactionStatus status =
        ExternalSweepCutTransactionStatus::SkippedNoMovement;
    std::string reason = "SKIPPED_NO_MOVEMENT";
    bool accepted = false;
    CutWeight cut_ceiling = 0;
    CutWeight proposal_cut = 0;
    std::size_t proposed_moved_nodes = 0;
    std::size_t committed_moved_nodes = 0;
    ExternalPairFlowSweepResult sweep;
};

inline ExternalSweepCutTransactionResult finalizeExternalSweepProposal(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels,
    std::vector<int> proposal,
    ExternalPairFlowSweepResult sweep) {
    if (community_labels.size() != hypergraph.hypernodes.size() ||
        proposal.size() != hypergraph.hypernodes.size())
        throw std::invalid_argument("EXTERNAL_SWEEP_TRANSACTION_LABEL_SHAPE");

    ExternalSweepCutTransactionResult result;
    result.cut_ceiling = recomputeCut(hypergraph, community_labels);
    if (sweep.cut_before != result.cut_ceiling)
        throw std::logic_error("EXTERNAL_SWEEP_TRANSACTION_ENTRY_CUT_MISMATCH");

    for (std::size_t node = 0; node < proposal.size(); ++node) {
        if (hypergraph.hypernodes[node].deleted) {
            if (proposal[node] != community_labels[node])
                throw std::logic_error(
                    "EXTERNAL_SWEEP_TRANSACTION_DELETED_LABEL_CHANGED");
            continue;
        }
        if (community_labels[node] < 0 || proposal[node] < 0)
            throw std::invalid_argument(
                "EXTERNAL_SWEEP_TRANSACTION_ACTIVE_LABEL");
        if (proposal[node] != community_labels[node])
            ++result.proposed_moved_nodes;
    }

    result.proposal_cut = recomputeCut(hypergraph, proposal);
    if (sweep.cut_after != result.proposal_cut)
        throw std::logic_error(
            "EXTERNAL_SWEEP_TRANSACTION_PROPOSAL_CUT_MISMATCH");
    result.sweep = std::move(sweep);

    if (result.proposed_moved_nodes == 0) {
        result.status = ExternalSweepCutTransactionStatus::SkippedNoMovement;
    } else if (result.proposal_cut > result.cut_ceiling) {
        result.status = ExternalSweepCutTransactionStatus::RejectedCutCeiling;
    } else {
        result.status = ExternalSweepCutTransactionStatus::Accepted;
        result.accepted = true;
        result.committed_moved_nodes = result.proposed_moved_nodes;
        community_labels.swap(proposal);
    }
    result.reason = externalSweepCutTransactionStatusName(result.status);
    return result;
}

inline ExternalSweepCutTransactionResult
executeExternalPairFlowSweepWithCutCeiling(
    const Hypergraph& hypergraph,
    std::vector<int>& community_labels) {
    std::vector<int> proposal = community_labels;
    ExternalPairFlowSweepResult sweep =
        executeExternalPairFlowSweep(hypergraph, proposal);
    return finalizeExternalSweepProposal(
        hypergraph, community_labels, std::move(proposal), std::move(sweep));
}

}  // namespace cf
