#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "connection_projector.h"
#include "mandatory_path_metrics.h"
#include "t2_structural_candidates.h"

namespace route_prediction_v1 {

using T3ResourceVector = std::array<std::int64_t, 8>;
using MasterMoveAction = T2ActionKey;

enum class T3Class {
    HardIllegal,
    NotMandatoryImproving,
    StandaloneAcceptable,
    PairEligible
};

enum class T3RejectReason {
    None,
    PositionViolation,
    ResourceViolation,
    TopologyUnreachable,
    MaximumHopViolation,
    AggregateCommunicationViolation
};

enum class MetricAvailability {
    Complete,
    NotEvaluatedInvalidAction,
    UndefinedDueToUnreachable
};

enum class T3EvaluationStatus { Success, ArithmeticOverflow };

struct RawDeterministicRoute {
    Connection connection;
    bool topology_reachable = false;
    std::vector<int> fpga_path;
    std::vector<int> physical_link_ids;
    int hop_count = 0;
    bool exceeds_maximum_hop = false;
};

RawDeterministicRoute routeDeterministicWithoutHopGate(
    const Connection& connection, const FpgaTopology& topology,
    int maximum_hop);

struct T3LoadMetrics {
    std::int64_t maximum = 0;
    std::int64_t squared_sum = 0;
    std::int64_t sum = 0;
    long double normalized_concentration = 0.0L;
};

struct T3Constraints {
    std::vector<T3ResourceVector> node_resources;
    std::vector<T3ResourceVector> fpga_resource_capacities;
    std::vector<std::int64_t> fpga_communication_capacities;
    int maximum_hop = 0;
    std::int64_t thd_budget = 0;
};

struct T3StateCache {
    CaseData data;
    FpgaTopology topology;
    MandatoryPathTable mandatory_paths;
    T3Constraints constraints;
    std::vector<std::vector<Connection>> net_connections_current;
    std::map<ConnectionKey, RawDeterministicRoute> current_routes;
    std::vector<std::vector<int>> incident_nets;
    std::vector<std::int64_t> baseline_l;
    std::vector<std::int64_t> baseline_u;
    T3LoadMetrics baseline_l_metrics;
    T3LoadMetrics baseline_u_metrics;
    std::int64_t baseline_uthd = 0;
    std::int64_t baseline_wthd = 0;
    std::vector<int> per_net_maximum_hop;
    std::vector<T3ResourceVector> baseline_resource_usage;
    std::vector<std::int64_t> baseline_communication_usage;
    std::map<int, int> current_projection_count_by_net;

    T3StateCache(const CaseData& input_data,
                 const FpgaTopology& input_topology,
                 const T3Constraints& input_constraints);
};

struct T3Evaluation {
    T2ActionKey action;
    int source_master_fpga = -1;
    int target_fpga = -1;
    std::size_t t2_provenance_count = 0;
    T3EvaluationStatus status = T3EvaluationStatus::Success;
    std::string arithmetic_overflow_diagnostic;
    std::optional<T3Class> classification;
    MetricAvailability metric_availability = MetricAvailability::Complete;
    bool t2_invariant_violation = false;
    bool l_metrics_valid = false;
    bool u_metrics_valid = false;
    bool thd_metrics_valid = false;
    bool global_maximum_hop_valid = false;
    std::size_t affected_net_count = 0;
    std::size_t old_connection_count = 0;
    std::size_t new_connection_count = 0;
    std::set<ConnectionKey> added_keys;
    std::set<ConnectionKey> deleted_keys;
    std::set<ConnectionKey> unchanged_keys;
    std::size_t changed_key_count = 0;
    std::optional<std::vector<std::int64_t>> delta_l;
    std::vector<std::int64_t> delta_u;
    std::optional<std::vector<std::int64_t>> l_prime;
    std::vector<std::int64_t> u_prime;
    std::optional<T3LoadMetrics> l_metrics;
    std::optional<T3LoadMetrics> u_metrics;
    std::optional<std::int64_t> delta_uthd;
    std::optional<std::int64_t> delta_wthd;
    std::optional<std::int64_t> uthd_prime;
    std::optional<std::int64_t> wthd_prime;
    int unaffected_maximum_hop = 0;
    int new_affected_maximum_hop = 0;
    std::optional<int> global_maximum_hop_prime;
    std::size_t unreachable_new_connection_count = 0;
    bool resource_legal = true;
    std::vector<T3ResourceVector> resource_usage_prime;
    bool aggregate_communication_legal = true;
    std::vector<std::int64_t> delta_communication;
    std::vector<std::int64_t> communication_usage_prime;
    T3RejectReason first_reject = T3RejectReason::None;
    std::set<T3RejectReason> all_violations;
    std::int64_t thd_excess = 0;
    std::int64_t lmax_excess = 0;
    std::map<int, int> action_projection_count_by_net;
};

T3Evaluation evaluateT3StructuralCandidate(
    const T3StateCache& state, const T2CandidateResult& t2,
    const T2ActionKey& action);

T3Evaluation evaluateMasterMove(const T3StateCache& state,
                                const MasterMoveAction& action,
                                std::size_t provenance_count);

struct T3FullRebuildResult {
    T3Evaluation evaluation;
    std::vector<std::vector<Connection>> all_net_connections;
};

T3FullRebuildResult rebuildT3CandidateFromScratch(
    const T3StateCache& baseline, const T2CandidateResult& t2,
    const T2ActionKey& action);

T3FullRebuildResult rebuildMasterMoveFromScratch(
    const T3StateCache& baseline, const MasterMoveAction& action,
    std::size_t provenance_count);

bool t3EvaluationEquivalent(const T3Evaluation& incremental,
                            const T3Evaluation& rebuilt,
                            std::string* mismatch);

bool operator==(const T3LoadMetrics& lhs, const T3LoadMetrics& rhs);
bool operator==(const T3Evaluation& lhs, const T3Evaluation& rhs);

enum class T3CheckedOperationForTest { Add, Subtract, Multiply, SquareSum };
std::int64_t t3CheckedArithmeticForTest(T3CheckedOperationForTest operation,
                                        std::int64_t a, std::int64_t b);

}  // namespace route_prediction_v1
