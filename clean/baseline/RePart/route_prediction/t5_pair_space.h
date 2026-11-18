#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "t4_source_action_evaluator.h"

namespace route_prediction_v1 {

enum class T5ActionRole { Congestion, Repair };

struct T5RoleAction {
    T5ActionRole role = T5ActionRole::Congestion;
    int node_id = -1;
    int source_fpga = -1;
    int target_fpga = -1;
    std::size_t original_result_index = 0;
    std::optional<std::int64_t> thd_excess;
    std::optional<std::int64_t> repair_supply;
    std::optional<T3Class> t3_class;
    std::optional<T4RepairClass> t4_repair_class;
    T3EvaluationStatus upstream_status = T3EvaluationStatus::Success;
};

struct T5ActionKey {
    int node_id = -1;
    int source_fpga = -1;
    int target_fpga = -1;
};

struct T5PairKey {
    int congestion_node_id = -1;
    int congestion_source_fpga = -1;
    int congestion_target_fpga = -1;
    int repair_node_id = -1;
    int repair_source_fpga = -1;
    int repair_target_fpga = -1;
};

bool operator==(const T5ActionKey& lhs, const T5ActionKey& rhs);
bool operator<(const T5ActionKey& lhs, const T5ActionKey& rhs);
bool operator==(const T5PairKey& lhs, const T5PairKey& rhs);
bool operator<(const T5PairKey& lhs, const T5PairKey& rhs);
bool operator==(const T5RoleAction& lhs, const T5RoleAction& rhs);

enum class T5StructuralConflictReason {
    None,
    SameNodeConflict,
    InvalidCurrentMaster,
    MasterReplicaConflict,
    DuplicateFinalMaster
};

enum class T5PairSpaceStopReason {
    None,
    InvalidRoleActionInput,
    ExactDuplicateActionInput,
    InconsistentDuplicateActionInput,
    NoRepairEligibleAction,
    NoNumericallyCoverableCongestionAction,
    ArithmeticOverflow,
    DuplicatePairKey
};

struct T5DuplicateInputDiagnostic {
    T5ActionRole role = T5ActionRole::Congestion;
    T5ActionKey action_key;
    std::vector<std::size_t> input_indices;
    bool exact = false;
};

struct T5PairCandidate {
    T5PairKey key;
    T5RoleAction congestion;
    T5RoleAction repair;
    bool same_node_conflict = false;
    bool invalid_current_master = false;
    bool master_replica_conflict = false;
    bool duplicate_final_master = false;
    T5StructuralConflictReason first_structural_conflict =
        T5StructuralConflictReason::None;
    bool can_enter_joint_evaluation = false;
};

struct T5PairSpaceDiagnostics {
    std::size_t raw_t3_input_count = 0;
    std::size_t raw_t4_input_count = 0;
    std::size_t unique_congestion_action_key_count = 0;
    std::size_t unique_repair_action_key_count = 0;
    std::size_t exact_duplicate_congestion_count = 0;
    std::size_t inconsistent_duplicate_congestion_count = 0;
    std::size_t exact_duplicate_repair_count = 0;
    std::size_t inconsistent_duplicate_repair_count = 0;
    std::vector<T5DuplicateInputDiagnostic> duplicate_inputs;
    std::optional<std::size_t> repair_count;
    std::optional<std::int64_t> maximum_repair_supply;
    std::optional<std::size_t> congestion_count;
    std::optional<std::size_t> expected_pair_count;
    std::optional<std::size_t> actual_pair_count;
    std::optional<std::size_t> unique_pair_key_count;
    std::optional<std::size_t> same_node_conflict_count;
    std::optional<std::size_t> invalid_current_master_count;
    std::optional<std::size_t> master_replica_conflict_count;
    std::optional<std::size_t> duplicate_final_master_count;
    std::optional<std::size_t> multiple_structural_conflict_count;
    std::map<T5StructuralConflictReason, std::size_t>
        first_structural_conflict_counts;
    std::optional<std::size_t> can_enter_joint_evaluation_count;
    std::optional<bool> pair_count_conserved;
};

struct T5PairSpaceResult {
    T5PairSpaceStopReason stop_reason = T5PairSpaceStopReason::None;
    T5PairSpaceDiagnostics diagnostics;
    std::optional<std::vector<T5RoleAction>> congestion_actions;
    std::optional<std::vector<T5RoleAction>> repair_actions;
    std::optional<std::vector<T5PairCandidate>> pairs;
};

enum class T5PairSpaceMode { SupplyCovered, AllPairEligible };

T5PairSpaceResult buildT5PairSpace(
    const std::vector<T3Evaluation>& t3_inputs,
    const std::vector<T4Evaluation>& t4_inputs,
    const std::vector<NodePlacement>& placement,
    int fpga_count,
    T5PairSpaceMode mode = T5PairSpaceMode::SupplyCovered);

std::size_t t5CheckedPairCount(std::size_t congestion_count,
                               std::size_t repair_count);

bool operator==(const T5DuplicateInputDiagnostic& lhs,
                const T5DuplicateInputDiagnostic& rhs);
bool operator==(const T5PairCandidate& lhs, const T5PairCandidate& rhs);
bool operator==(const T5PairSpaceDiagnostics& lhs,
                const T5PairSpaceDiagnostics& rhs);
bool operator==(const T5PairSpaceResult& lhs,
                const T5PairSpaceResult& rhs);

}  // namespace route_prediction_v1
