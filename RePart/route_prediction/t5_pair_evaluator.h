#pragma once

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "t5_pair_space.h"

namespace route_prediction_v1 {

enum class T5PairClass { ArithmeticOverflow, StructuralConflict, HardIllegal,
                         THDBudgetViolation, NonImprovingObjective,
                         PairAcceptable };

struct T5Objective {
    std::int64_t umax=0, q_u=0, lmax=0, q_l=0, wthd=0;
};

struct T5Interactions {
    std::optional<std::int64_t> wthd, uthd, umax, q_u, lmax, q_l;
};

struct T5PairEvaluation {
    T5PairCandidate pair;
    T5PairClass classification=T5PairClass::StructuralConflict;
    MetricAvailability metric_availability=MetricAvailability::Complete;
    std::string arithmetic_overflow_diagnostic;
    std::vector<int> congestion_incident_nets, repair_incident_nets,
                     shared_incident_nets, affected_nets;
    std::map<int,int> projection_count_by_net;
    std::size_t old_connection_count=0, new_connection_count=0;
    std::set<ConnectionKey> added_keys, deleted_keys, unchanged_keys;
    std::size_t changed_key_count=0;
    std::optional<std::vector<std::int64_t>> delta_l, l_prime;
    std::vector<std::int64_t> delta_u, u_prime;
    std::optional<T3LoadMetrics> l_metrics, u_metrics;
    std::optional<std::int64_t> delta_uthd, delta_wthd, uthd_prime, wthd_prime;
    int unaffected_maximum_hop=0, new_affected_maximum_hop=0;
    std::optional<int> global_maximum_hop_prime;
    std::size_t unreachable_new_connection_count=0;
    bool resource_legal=true, aggregate_communication_legal=true;
    std::vector<T3ResourceVector> resource_usage_prime;
    std::vector<std::int64_t> delta_communication, communication_usage_prime;
    T3RejectReason first_hard_reject=T3RejectReason::None;
    std::set<T3RejectReason> all_hard_violations;
    T5Interactions interaction;
};

struct T5PairFullRebuildResult {
    T5PairEvaluation evaluation;
    std::vector<std::vector<Connection>> all_net_connections;
};

T5PairEvaluation evaluateT5Pair(const T3StateCache&, const T5PairCandidate&,
                                std::int64_t budget,
                                const T5Objective& baseline_objective,
                                const T3Evaluation* congestion_single=nullptr,
                                const T3Evaluation* repair_single=nullptr);
T5PairFullRebuildResult rebuildT5PairFromScratch(
    const T3StateCache&, const T5PairCandidate&, std::int64_t budget,
    const T5Objective& baseline_objective);
bool t5PairEvaluationEquivalent(const T5PairEvaluation&,
                                const T5PairEvaluation&, std::string* mismatch);

// Public comparator wrapper used by tests: lexicographic order on
// (umax, q_u, lmax, q_l, wthd). Implemented in the corresponding .cpp.
bool t5ObjectiveLess(const T5Objective&, const T5Objective&);

} // namespace route_prediction_v1
