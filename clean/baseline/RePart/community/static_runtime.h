#pragma once
#include "detector_bridge.h"
#include "flow_lifecycle_state.h"
#include "external_pair_flow_sweep.h"
#include "external_sweep_cut_transaction.h"
#include "pair_flow_sweep.h"
#include "../datastructure/hypergraph.h"
#include <chrono>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cf_bridge {
enum class CommunityMode {
    Disabled,
    StaticOnly,
    CutOnlyFlow,
    ExternalFlow,
    ExternalFlowCutCeiling
};

inline const char* communityModeName(CommunityMode mode) {
    switch (mode) {
        case CommunityMode::Disabled:
            return "DISABLED";
        case CommunityMode::StaticOnly:
            return "B";
        case CommunityMode::CutOnlyFlow:
            return "C";
        case CommunityMode::ExternalFlow:
            return "D";
        case CommunityMode::ExternalFlowCutCeiling:
            return "E";
    }
    return "UNKNOWN";
}

struct ScheduledSweepAudit {
    cf::FlowInvocationRecord invocation;
    cf::PairFlowSweepStatus status;
    std::size_t frozen_pairs;
    std::size_t accepted_pairs;
    std::size_t rejected_pairs;
    std::size_t disappeared_pairs;
    std::size_t empty_corridor_pairs;
    std::size_t no_movement_pairs;
    std::size_t fallback_pairs;
    std::size_t proposal_moved_nodes;
    std::size_t committed_moved_nodes;
    cf::CutWeight cut_before;
    cf::CutWeight cut_after;
    double elapsed_seconds;
    bool next_attempt_expected = true;
    bool next_attempt_observed = false;
    std::size_t contractions_after_flow = 0;
    bool recovery_relieved_stall = false;
};

struct ActiveSweepResult {
    cf::PairFlowSweepStatus status;
    std::size_t frozen_pairs;
    std::size_t accepted_pairs;
    std::size_t rejected_pairs;
    std::size_t disappeared_pairs;
    std::size_t empty_corridor_pairs;
    std::size_t no_movement_pairs;
    std::size_t fallback_pairs;
    std::size_t total_proposal_moved_nodes;
    std::size_t total_committed_moved_nodes;
    cf::CutWeight cut_before;
    cf::CutWeight cut_after;
};

enum class RecoveryFlowAction {
    SkippedStateAlreadyAttempted,
    CompletedWithoutAcceptance,
    RunOneOriginalAttempt
};

inline const char* recoveryFlowActionName(RecoveryFlowAction action) {
    switch (action) {
        case RecoveryFlowAction::SkippedStateAlreadyAttempted:
            return "RECOVERY_SKIP_STATE_ALREADY_ATTEMPTED";
        case RecoveryFlowAction::CompletedWithoutAcceptance:
            return "RECOVERY_NO_ACCEPTED_PAIR";
        case RecoveryFlowAction::RunOneOriginalAttempt:
            return "RECOVERY_RUN_ONE_ORIGINAL_ATTEMPT";
    }
    return "UNKNOWN";
}

// One instance per FullCoarsener run. B keeps the detected labels unchanged;
// C/D use the scheduled lifecycle. E runs one atomic cut-ceiling transaction
// after detection and before the first original coarsening attempt.
struct StaticCommunityRun {
    bool enabled = false;
    CommunityMode mode = CommunityMode::Disabled;
    std::ostream* log = nullptr;
    std::vector<int> labels;
    std::size_t detector_calls = 0, attempts = 0, retries = 0;
    std::size_t contractions = 0, cross_candidates = 0, positive_cross_candidates = 0;
    std::size_t scheduled_initial_calls = 0;
    std::size_t scheduled_periodic_calls = 0;
    std::size_t scheduled_target_skips = 0;
    std::size_t recovery_flow_calls = 0;
    std::size_t recovery_state_already_attempted_skips = 0;
    std::size_t recovery_no_acceptance_stalls = 0;
    std::size_t recovery_retry_successes = 0;
    std::size_t recovery_retry_failures = 0;
    std::size_t external_cut_ceiling_calls = 0;
    bool stalled = false;
    cf::ScheduledFlowState scheduled_state;
    cf::FlowAttemptedStates attempted_flow_states;
    std::vector<ScheduledSweepAudit> scheduled_sweeps;
    std::vector<ScheduledSweepAudit> recovery_sweeps;
    std::optional<std::size_t> scheduled_sweep_awaiting_attempt;
    std::optional<std::size_t> recovery_sweep_awaiting_attempt;
    std::optional<cf::ExternalSweepCutTransactionResult>
        external_cut_ceiling_result;

    void enableStaticOnly() {
        enabled = true;
        mode = CommunityMode::StaticOnly;
    }

    void enableCutOnlyFlow() {
        enabled = true;
        mode = CommunityMode::CutOnlyFlow;
    }

    void enableExternalFlow() {
        enabled = true;
        mode = CommunityMode::ExternalFlow;
    }

    void enableExternalFlowCutCeiling() {
        enabled = true;
        mode = CommunityMode::ExternalFlowCutCeiling;
    }

    bool cutOnlyFlowEnabled() const {
        return enabled && mode == CommunityMode::CutOnlyFlow;
    }

    bool externalFlowEnabled() const {
        return enabled && mode == CommunityMode::ExternalFlow;
    }

    bool externalFlowCutCeilingEnabled() const {
        return enabled && mode == CommunityMode::ExternalFlowCutCeiling;
    }

    bool flowEnabled() const {
        return cutOnlyFlowEnabled() || externalFlowEnabled();
    }

    template<class... Args> void record(const char* event, const Args&... args) const {
        if (!log) return;
        *log << event;
        ((*log << '\t' << args), ...);
        *log << '\n';
    }

    void initialize(const Hypergraph& graph) {
        if (detector_calls != 0) throw std::logic_error("INITIAL_ONCE detector called twice");
        const auto begin = std::chrono::steady_clock::now();
        auto detected = detect(graph);
        if (detected.node_labels.size() != graph.hypernodes.size())
            throw std::logic_error("detector label storage mismatch");
        for (std::size_t v = 0; v < graph.hypernodes.size(); ++v)
            if (!graph.hypernodes[v].deleted && detected.node_labels[v] < 0)
                throw std::logic_error("active node missing detector label");
        labels = std::move(detected.node_labels);
        ++detector_calls;
        record("detector", detector_calls, "INITIAL_ONCE",
            std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count());
        std::map<int, std::size_t> sizes;
        for (std::size_t v = 0; v < labels.size(); ++v) {
            record("label", v, labels[v]);
            if (!graph.hypernodes[v].deleted) ++sizes[labels[v]];
        }
        for (const auto& entry : sizes) record("community_size", entry.first, entry.second);
        for (const auto& level : detected.levels) {
            record("detector_level", level.level, level.vertices, level.edges,
                level.communities, level.accepted_moves, level.rounds,
                level.last_round_moves ? std::to_string(*level.last_round_moves) : "NA",
                level.action);
            if (level.q_before && level.q_after)
                record("detector_q", level.level, *level.q_before, *level.q_after);
        }
    }

    void runInitialScheduledFlow(const Hypergraph& graph) {
        requireFlow("SCHEDULED_INITIAL_WITHOUT_FLOW_MODE");
        const cf::ScheduledFlowAction action =
            cf::beginScheduledFlowAfterDetection(
                scheduled_state,
                static_cast<std::size_t>(graph._num_vertices_current));
        if (action != cf::ScheduledFlowAction::RunInitial)
            throw std::logic_error("SCHEDULED_INITIAL_ACTION_MISMATCH");
        runScheduledSweep(graph, cf::FlowInvocationKind::ScheduledInitial);
    }

    void runInitialExternalCutCeilingFlow(const Hypergraph& graph) {
        if (!externalFlowCutCeilingEnabled())
            throw std::logic_error("EXTERNAL_CUT_CEILING_WITHOUT_E_MODE");
        if (detector_calls != 1)
            throw std::logic_error("EXTERNAL_CUT_CEILING_REQUIRES_DETECTION");
        if (external_cut_ceiling_calls != 0 ||
            external_cut_ceiling_result)
            throw std::logic_error("EXTERNAL_CUT_CEILING_INITIAL_ONCE");

        const auto begin = std::chrono::steady_clock::now();
        cf::ExternalSweepCutTransactionResult result =
            cf::executeExternalPairFlowSweepWithCutCeiling(graph, labels);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        checkedIncrement(external_cut_ceiling_calls,
                         "EXTERNAL_CUT_CEILING_COUNT_OVERFLOW");
        const std::size_t call_id = external_cut_ceiling_calls;
        recordExternalSweep(result.sweep, call_id);
        record("external_cut_ceiling_result", call_id, "INITIAL_ONCE",
               cf::externalSweepCutTransactionStatusName(result.status),
               "accepted", result.accepted, "cut_ceiling",
               result.cut_ceiling, "proposal_cut", result.proposal_cut,
               "proposal_moved", result.proposed_moved_nodes,
               "committed_moved", result.committed_moved_nodes,
               "sweep_status",
               cf::pairFlowSweepStatusName(result.sweep.status), "pairs",
               result.sweep.frozen_pairs.size(), "sweep_accepted",
               result.sweep.accepted_pairs, "sweep_rejected",
               result.sweep.rejected_pairs, "seconds", elapsed);
        external_cut_ceiling_result = std::move(result);
    }

    void runPendingScheduledFlow(const Hypergraph& graph,
                                 std::size_t target_nodes) {
        requireFlow("SCHEDULED_PERIODIC_WITHOUT_FLOW_MODE");
        const cf::ScheduledFlowAction action =
            cf::takeScheduledActionBeforeCandidateBatch(
                scheduled_state,
                static_cast<std::size_t>(graph._num_vertices_current),
                target_nodes);
        if (action == cf::ScheduledFlowAction::None) return;
        if (action == cf::ScheduledFlowAction::SkipTargetReached)
            throw std::logic_error("SCHEDULED_TARGET_CHECK_ORDER_VIOLATION");
        if (action != cf::ScheduledFlowAction::RunPeriodic)
            throw std::logic_error("SCHEDULED_PERIODIC_ACTION_MISMATCH");
        runScheduledSweep(graph, cf::FlowInvocationKind::ScheduledPeriodic);
    }

    void handleReachedTarget(std::size_t active_nodes,
                             std::size_t target_nodes) {
        requireFlow("SCHEDULED_TARGET_WITHOUT_FLOW_MODE");
        if (!scheduled_state.initialized) return;
        const cf::ScheduledFlowAction action =
            cf::takeScheduledActionBeforeCandidateBatch(
                scheduled_state, active_nodes, target_nodes);
        if (action == cf::ScheduledFlowAction::None) return;
        if (action != cf::ScheduledFlowAction::SkipTargetReached)
            throw std::logic_error("SCHEDULED_RAN_AFTER_TARGET");
        checkedIncrement(scheduled_target_skips,
                         "SCHEDULED_TARGET_SKIP_OVERFLOW");
        record("scheduled_skip", cf::scheduledFlowActionName(action),
               "nodes", active_nodes, "target", target_nodes);
    }

    cf::ScheduledTriggerResult noteCoarseningAttempt(
        std::size_t active_nodes, std::size_t actual_contractions) {
        requireFlow("SCHEDULED_ATTEMPT_WITHOUT_FLOW_MODE");
        if (scheduled_sweep_awaiting_attempt &&
            recovery_sweep_awaiting_attempt)
            throw std::logic_error("MULTIPLE_FLOWS_AWAITING_ONE_ATTEMPT");
        if (scheduled_sweep_awaiting_attempt) {
            ScheduledSweepAudit& audit =
                scheduled_sweeps.at(*scheduled_sweep_awaiting_attempt);
            if (audit.next_attempt_observed)
                throw std::logic_error("FLOW_ATTEMPT_ALREADY_BOUND");
            audit.next_attempt_observed = true;
            audit.contractions_after_flow = actual_contractions;
            record("flow_next_attempt", audit.invocation.call_id,
                   cf::flowInvocationKindName(audit.invocation.kind),
                   "contractions", actual_contractions,
                   "nodes", active_nodes);
            scheduled_sweep_awaiting_attempt.reset();
        } else if (recovery_sweep_awaiting_attempt) {
            ScheduledSweepAudit& audit =
                recovery_sweeps.at(*recovery_sweep_awaiting_attempt);
            if (!audit.next_attempt_expected || audit.next_attempt_observed)
                throw std::logic_error("RECOVERY_ATTEMPT_BINDING_MISMATCH");
            audit.next_attempt_observed = true;
            audit.contractions_after_flow = actual_contractions;
            audit.recovery_relieved_stall = actual_contractions > 0;
            if (audit.recovery_relieved_stall)
                checkedIncrement(recovery_retry_successes,
                                 "RECOVERY_SUCCESS_COUNT_OVERFLOW");
            else
                checkedIncrement(recovery_retry_failures,
                                 "RECOVERY_FAILURE_COUNT_OVERFLOW");
            record("flow_next_attempt", audit.invocation.call_id,
                   cf::flowInvocationKindName(audit.invocation.kind),
                   "contractions", actual_contractions,
                   "nodes", active_nodes);
            record("recovery_outcome", audit.invocation.call_id,
                   audit.recovery_relieved_stall ?
                       "RECOVERY_RELIEVED_STALL" :
                       "RECOVERY_RETRY_FAILED");
            recovery_sweep_awaiting_attempt.reset();
        }
        const cf::ScheduledTriggerResult trigger =
            cf::noteCoarseningAttempt(scheduled_state, active_nodes,
                                      actual_contractions);
        record("scheduled_state", "counted", trigger.counted_successful_layer,
               "node_reduction", trigger.node_reduction,
               "layer_limit", trigger.layer_limit,
               "pending", trigger.pending,
               "threshold", trigger.node_threshold,
               "B", trigger.successful_layers,
               "nodes", active_nodes,
               "contractions", actual_contractions);
        return trigger;
    }

    RecoveryFlowAction runRecoveryAfterRetryExhausted(
        const Hypergraph& graph) {
        requireFlow("RECOVERY_WITHOUT_FLOW_MODE");
        if (scheduled_sweep_awaiting_attempt ||
            recovery_sweep_awaiting_attempt)
            throw std::logic_error("FLOW_MISSING_NEXT_ATTEMPT_AUDIT");

        const cf::FlowInputState input = buildCurrentFlowInput(graph);
        if (attempted_flow_states.contains(input)) {
            checkedIncrement(recovery_state_already_attempted_skips,
                             "RECOVERY_SKIP_COUNT_OVERFLOW");
            record("recovery_skip",
                   recoveryFlowActionName(
                       RecoveryFlowAction::SkippedStateAlreadyAttempted),
                   "state_hash", cf::flowInputStateHash(input),
                   "nodes", graph._num_vertices_current);
            return RecoveryFlowAction::SkippedStateAlreadyAttempted;
        }

        const cf::FlowInvocationRecord invocation =
            attempted_flow_states.registerBeforeCall(
                cf::FlowInvocationKind::Recovery, input);
        const auto begin = std::chrono::steady_clock::now();
        const ActiveSweepResult result =
            runActiveSweep(graph, invocation.call_id);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        checkedIncrement(recovery_flow_calls,
                         "RECOVERY_CALL_COUNT_OVERFLOW");

        recovery_sweeps.push_back(
            {invocation,
             result.status,
             result.frozen_pairs,
             result.accepted_pairs,
             result.rejected_pairs,
             result.disappeared_pairs,
             result.empty_corridor_pairs,
             result.no_movement_pairs,
             result.fallback_pairs,
             result.total_proposal_moved_nodes,
             result.total_committed_moved_nodes,
             result.cut_before,
             result.cut_after,
             elapsed});
        ScheduledSweepAudit& audit = recovery_sweeps.back();
        audit.next_attempt_expected = result.accepted_pairs > 0;
        record("flow_call", invocation.call_id,
               cf::flowInvocationKindName(invocation.kind), "state_hash",
               invocation.state_hash, "state_bucket", invocation.state_bucket,
               "state_index", invocation.attempted_state_index,
               "state_repeated", invocation.state_was_already_attempted);
        record("flow_result", invocation.call_id,
               cf::pairFlowSweepStatusName(result.status), "pairs",
               result.frozen_pairs, "accepted", result.accepted_pairs,
               "rejected", result.rejected_pairs, "disappeared",
               result.disappeared_pairs, "empty_corridor",
               result.empty_corridor_pairs, "no_movement",
               result.no_movement_pairs, "proposal_moved",
               result.total_proposal_moved_nodes, "fallback",
               result.fallback_pairs, "committed_moved",
               result.total_committed_moved_nodes, "cut_before",
               result.cut_before, "cut_after", result.cut_after,
               "seconds", elapsed);

        if (result.accepted_pairs == 0) {
            checkedIncrement(recovery_no_acceptance_stalls,
                             "RECOVERY_NO_ACCEPT_COUNT_OVERFLOW");
            record("recovery_decision", invocation.call_id,
                   recoveryFlowActionName(
                       RecoveryFlowAction::CompletedWithoutAcceptance));
            return RecoveryFlowAction::CompletedWithoutAcceptance;
        }

        recovery_sweep_awaiting_attempt = recovery_sweeps.size() - 1;
        record("recovery_decision", invocation.call_id,
               recoveryFlowActionName(
                   RecoveryFlowAction::RunOneOriginalAttempt));
        return RecoveryFlowAction::RunOneOriginalAttempt;
    }

    void finish(int nodes, int target, int threshold) const {
        if (externalFlowCutCeilingEnabled())
            record("coarsen_end", stalled ? "COMMUNITY_STALL" : "TARGET_REACHED",
                nodes, target, detector_calls, attempts, retries, threshold, contractions,
                cross_candidates, positive_cross_candidates, "flow_calls",
                external_cut_ceiling_calls, "initial_cut_ceiling",
                external_cut_ceiling_calls, "scheduled_initial", 0,
                "scheduled_periodic", 0, "recovery_flow", 0);
        else if (flowEnabled())
            record("coarsen_end", stalled ? "COMMUNITY_STALL" : "TARGET_REACHED",
                nodes, target, detector_calls, attempts, retries, threshold, contractions,
                cross_candidates, positive_cross_candidates, "flow_calls",
                attempted_flow_states.invocationCount(), "scheduled_initial",
                scheduled_initial_calls, "scheduled_periodic",
                scheduled_periodic_calls, "scheduled_target_skips",
                scheduled_target_skips, "recovery_flow", recovery_flow_calls,
                "recovery_state_skips",
                recovery_state_already_attempted_skips,
                "recovery_no_accept", recovery_no_acceptance_stalls,
                "recovery_retry_success", recovery_retry_successes,
                "recovery_retry_failure", recovery_retry_failures);
        else
            record("coarsen_end", stalled ? "COMMUNITY_STALL" : "TARGET_REACHED",
                nodes, target, detector_calls, attempts, retries, threshold, contractions,
                cross_candidates, positive_cross_candidates, "flow_calls", 0);
        if (log) log->flush();
    }

private:
    static void checkedIncrement(std::size_t& value, const char* message) {
        if (value == std::numeric_limits<std::size_t>::max())
            throw std::overflow_error(message);
        ++value;
    }

    void requireFlow(const char* message) const {
        if (!flowEnabled()) throw std::logic_error(message);
    }

    void runScheduledSweep(const Hypergraph& graph,
                           cf::FlowInvocationKind kind) {
        if (kind == cf::FlowInvocationKind::Recovery)
            throw std::logic_error("RECOVERY_MUST_USE_RECOVERY_ENTRY");
        if (scheduled_sweep_awaiting_attempt ||
            recovery_sweep_awaiting_attempt)
            throw std::logic_error("FLOW_MISSING_NEXT_ATTEMPT_AUDIT");

        const cf::FlowInputState input = buildCurrentFlowInput(graph);
        const cf::FlowInvocationRecord invocation =
            attempted_flow_states.registerBeforeCall(kind, input);
        const auto begin = std::chrono::steady_clock::now();
        const ActiveSweepResult result =
            runActiveSweep(graph, invocation.call_id);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();

        if (kind == cf::FlowInvocationKind::ScheduledInitial)
            checkedIncrement(scheduled_initial_calls,
                             "SCHEDULED_INITIAL_COUNT_OVERFLOW");
        else
            checkedIncrement(scheduled_periodic_calls,
                             "SCHEDULED_PERIODIC_COUNT_OVERFLOW");

        scheduled_sweeps.push_back(
            {invocation,
             result.status,
             result.frozen_pairs,
             result.accepted_pairs,
             result.rejected_pairs,
             result.disappeared_pairs,
             result.empty_corridor_pairs,
             result.no_movement_pairs,
             result.fallback_pairs,
             result.total_proposal_moved_nodes,
             result.total_committed_moved_nodes,
             result.cut_before,
             result.cut_after,
             elapsed});
        scheduled_sweep_awaiting_attempt = scheduled_sweeps.size() - 1;
        record("flow_call", invocation.call_id,
               cf::flowInvocationKindName(kind), "state_hash",
               invocation.state_hash, "state_bucket", invocation.state_bucket,
               "state_index", invocation.attempted_state_index,
               "state_repeated", invocation.state_was_already_attempted);
        record("flow_result", invocation.call_id,
               cf::pairFlowSweepStatusName(result.status), "pairs",
               result.frozen_pairs, "accepted", result.accepted_pairs,
               "rejected", result.rejected_pairs, "disappeared",
               result.disappeared_pairs, "empty_corridor",
               result.empty_corridor_pairs, "no_movement",
               result.no_movement_pairs, "proposal_moved",
               result.total_proposal_moved_nodes, "fallback",
               result.fallback_pairs, "committed_moved",
               result.total_committed_moved_nodes, "cut_before",
               result.cut_before, "cut_after", result.cut_after,
               "seconds", elapsed);
    }

    cf::FlowInputState buildCurrentFlowInput(const Hypergraph& graph) const {
        if (externalFlowEnabled())
            return cf::buildExternalFlowV0InputState(graph, labels);
        return cf::buildCutOnlyV0FlowInputState(graph, labels);
    }

    static std::string externalMetricText(
        const std::optional<long double>& value) {
        if (!value) return "NA";
        std::ostringstream stream;
        stream << std::setprecision(
            std::numeric_limits<long double>::max_digits10) << *value;
        return stream.str();
    }

    static std::string externalMetricText(long double value) {
        return externalMetricText(std::optional<long double>(value));
    }

    void recordExternalSweep(
        const cf::ExternalPairFlowSweepResult& result,
        std::size_t call_id) const {
        for (const cf::ExternalPairFlowSweepRecord& entry : result.records) {
            if (!entry.transaction) {
                record("external_pair", call_id, entry.frozen_index,
                       entry.frozen_pair.community_i,
                       entry.frozen_pair.community_j, entry.status,
                       "boundary_weight", entry.current_corridor.boundary_weight,
                       "movable", entry.current_corridor.movable_nodes.size(),
                       "fallback", 0, "proposal_moved", 0,
                       "committed_moved", 0, "raw_cut_before", "NA",
                       "raw_cut_after", "NA", "informative", "NA",
                       "coverage", "NA",
                       "kappa", "NA", "proxy", "NA", "max_flow", "NA",
                       "normalized_cut_before", "NA",
                       "normalized_cut_after", "NA", "phi_before", "NA",
                       "phi_after", "NA", "J_before", "NA", "J_after", "NA");
                continue;
            }
            const cf::ExternalPairTransactionResult& transaction =
                *entry.transaction;
            const std::size_t movable =
                entry.current_corridor.movable_nodes.size();
            const std::optional<long double> coverage = movable == 0
                ? std::nullopt
                : std::optional<long double>(
                      static_cast<long double>(
                          transaction.informative_corridor_nodes) /
                      static_cast<long double>(movable));
            record("external_pair", call_id, entry.frozen_index,
                   entry.frozen_pair.community_i,
                   entry.frozen_pair.community_j, entry.status,
                   "boundary_weight", entry.current_corridor.boundary_weight,
                   "movable", movable, "fallback",
                   transaction.used_cut_only_fallback, "proposal_moved",
                   transaction.proposal_moved_nodes, "committed_moved",
                   transaction.committed_moved_nodes, "raw_cut_before",
                   transaction.cut_before, "raw_cut_after",
                   transaction.cut_after, "informative",
                   transaction.informative_corridor_nodes, "coverage",
                   externalMetricText(coverage), "kappa",
                   externalMetricText(transaction.kappa), "proxy",
                   transaction.proxy_energy, "max_flow", transaction.max_flow,
                   "normalized_cut_before",
                   externalMetricText(transaction.normalized_cut_before),
                   "normalized_cut_after",
                   externalMetricText(transaction.normalized_cut_after),
                   "phi_before", externalMetricText(transaction.phi_before),
                   "phi_after", externalMetricText(transaction.phi_after),
                   "J_before", externalMetricText(transaction.objective_before),
                   "J_after", externalMetricText(transaction.objective_after));
        }
    }

    ActiveSweepResult runActiveSweep(const Hypergraph& graph,
                                     std::size_t call_id) {
        if (externalFlowEnabled()) {
            const cf::ExternalPairFlowSweepResult result =
                cf::executeExternalPairFlowSweep(graph, labels);
            recordExternalSweep(result, call_id);
            return {result.status, result.frozen_pairs.size(),
                    result.accepted_pairs, result.rejected_pairs,
                    result.disappeared_pairs, result.empty_corridor_pairs,
                    result.no_movement_pairs, result.fallback_pairs,
                    result.total_proposal_moved_nodes,
                    result.total_committed_moved_nodes,
                    result.cut_before, result.cut_after};
        }
        const cf::PairFlowSweepResult result =
            cf::executePairFlowSweep(graph, labels);
        return {result.status, result.frozen_pairs.size(),
                result.accepted_pairs, result.rejected_pairs,
                result.disappeared_pairs, result.empty_corridor_pairs,
                result.no_movement_pairs, 0,
                result.total_proposal_moved_nodes,
                result.total_committed_moved_nodes,
                result.cut_before, result.cut_after};
    }
};
}
