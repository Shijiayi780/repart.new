#include "t5_pair_space.h"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>

namespace route_prediction_v1 {
namespace {

T5ActionKey actionKey(const T5RoleAction& action) {
    return {action.node_id, action.source_fpga, action.target_fpga};
}

bool relevantFieldsEqual(const T5RoleAction& lhs,
                         const T5RoleAction& rhs) {
    return std::tie(lhs.role, lhs.node_id, lhs.source_fpga,
                    lhs.target_fpga, lhs.thd_excess, lhs.repair_supply,
                    lhs.t3_class, lhs.t4_repair_class,
                    lhs.upstream_status)
        == std::tie(rhs.role, rhs.node_id, rhs.source_fpga,
                    rhs.target_fpga, rhs.thd_excess, rhs.repair_supply,
                    rhs.t3_class, rhs.t4_repair_class,
                    rhs.upstream_status);
}

bool validRoleAction(const T5RoleAction& action) {
    if (action.upstream_status != T3EvaluationStatus::Success) return false;
    if (action.role == T5ActionRole::Congestion) {
        return action.t3_class.has_value()
            && !action.t4_repair_class.has_value()
            && action.thd_excess.has_value()
            && *action.thd_excess >= 0
            && !action.repair_supply.has_value();
    }
    if (action.role != T5ActionRole::Repair
        || !action.t4_repair_class.has_value()
        || action.t3_class.has_value()
        || action.thd_excess.has_value()) return false;
    switch (*action.t4_repair_class) {
    case T4RepairClass::HardIllegal:
    case T4RepairClass::NotRepairing:
        return true;
    case T4RepairClass::RepairEligible:
        return action.repair_supply.has_value() && *action.repair_supply > 0;
    }
    return false;
}

T5RoleAction fromT3(const T3Evaluation& input, std::size_t index) {
    T5RoleAction action;
    action.role = T5ActionRole::Congestion;
    action.node_id = input.action.node_id;
    action.source_fpga = input.source_master_fpga;
    action.target_fpga = input.action.target_fpga;
    action.original_result_index = index;
    action.thd_excess = input.thd_excess;
    action.t3_class = input.classification;
    action.upstream_status = input.status;
    return action;
}

T5RoleAction fromT4(const T4Evaluation& input, std::size_t index) {
    T5RoleAction action;
    action.role = T5ActionRole::Repair;
    action.node_id = input.source_action.action.node_id;
    action.source_fpga = input.master_move.source_master_fpga;
    action.target_fpga = input.source_action.action.target_fpga;
    action.original_result_index = index;
    action.repair_supply = input.repair_supply;
    action.t4_repair_class = input.repair_class;
    action.upstream_status = input.master_move.status;
    return action;
}

bool contains(const std::vector<int>& values, int value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

T5PairCandidate makePair(const T5RoleAction& congestion,
                         const T5RoleAction& repair,
                         const std::vector<NodePlacement>& placement,
                         int fpga_count) {
    T5PairCandidate pair;
    pair.congestion = congestion;
    pair.repair = repair;
    pair.key = {congestion.node_id, congestion.source_fpga,
                congestion.target_fpga, repair.node_id,
                repair.source_fpga, repair.target_fpga};
    pair.same_node_conflict = congestion.node_id == repair.node_id;

    const auto invalid = [&](const T5RoleAction& action) {
        return action.node_id < 0
            || action.node_id >= static_cast<int>(placement.size())
            || action.source_fpga < 0 || action.source_fpga >= fpga_count
            || action.target_fpga < 0 || action.target_fpga >= fpga_count
            || placement.at(action.node_id).master_fpga
                != action.source_fpga;
    };
    pair.invalid_current_master = invalid(congestion) || invalid(repair);

    const auto replicaConflict = [&](const T5RoleAction& action) {
        if (action.node_id < 0
            || action.node_id >= static_cast<int>(placement.size())
            || action.target_fpga < 0 || action.target_fpga >= fpga_count) {
            return false;
        }
        return contains(placement.at(action.node_id).replica_fpgas,
                        action.target_fpga);
    };
    pair.master_replica_conflict =
        replicaConflict(congestion) || replicaConflict(repair);
    pair.duplicate_final_master = pair.master_replica_conflict;

    if (pair.same_node_conflict) {
        pair.first_structural_conflict =
            T5StructuralConflictReason::SameNodeConflict;
    } else if (pair.invalid_current_master) {
        pair.first_structural_conflict =
            T5StructuralConflictReason::InvalidCurrentMaster;
    } else if (pair.master_replica_conflict) {
        pair.first_structural_conflict =
            T5StructuralConflictReason::MasterReplicaConflict;
    } else if (pair.duplicate_final_master) {
        pair.first_structural_conflict =
            T5StructuralConflictReason::DuplicateFinalMaster;
    }
    pair.can_enter_joint_evaluation =
        pair.first_structural_conflict == T5StructuralConflictReason::None;
    return pair;
}

template <typename Map>
bool inspectDuplicates(const Map& grouped,
                       T5ActionRole role,
                       T5PairSpaceDiagnostics& diagnostics,
                       T5PairSpaceStopReason& stop) {
    bool found = false;
    for (const auto& item : grouped) {
        if (item.second.size() < 2) continue;
        found = true;
        bool exact = true;
        for (std::size_t i = 1; i < item.second.size(); ++i) {
            exact = exact && relevantFieldsEqual(
                item.second.front(), item.second.at(i));
        }
        T5DuplicateInputDiagnostic duplicate;
        duplicate.role = role;
        duplicate.action_key = item.first;
        duplicate.exact = exact;
        for (const auto& action : item.second) {
            duplicate.input_indices.push_back(action.original_result_index);
        }
        std::sort(duplicate.input_indices.begin(),
                  duplicate.input_indices.end());
        diagnostics.duplicate_inputs.push_back(duplicate);
        auto& exact_count = role == T5ActionRole::Congestion
            ? diagnostics.exact_duplicate_congestion_count
            : diagnostics.exact_duplicate_repair_count;
        auto& inconsistent_count = role == T5ActionRole::Congestion
            ? diagnostics.inconsistent_duplicate_congestion_count
            : diagnostics.inconsistent_duplicate_repair_count;
        if (exact) ++exact_count;
        else ++inconsistent_count;
    }
    if (found) {
        const bool inconsistent = std::any_of(
            diagnostics.duplicate_inputs.begin(),
            diagnostics.duplicate_inputs.end(),
            [](const auto& item) { return !item.exact; });
        stop = inconsistent
            ? T5PairSpaceStopReason::InconsistentDuplicateActionInput
            : T5PairSpaceStopReason::ExactDuplicateActionInput;
    }
    return found;
}

}  // namespace

bool operator==(const T5ActionKey& a, const T5ActionKey& b) {
    return std::tie(a.node_id, a.source_fpga, a.target_fpga)
        == std::tie(b.node_id, b.source_fpga, b.target_fpga);
}
bool operator<(const T5ActionKey& a, const T5ActionKey& b) {
    return std::tie(a.node_id, a.source_fpga, a.target_fpga)
        < std::tie(b.node_id, b.source_fpga, b.target_fpga);
}
bool operator==(const T5PairKey& a, const T5PairKey& b) {
    return std::tie(a.congestion_node_id, a.congestion_source_fpga,
                    a.congestion_target_fpga, a.repair_node_id,
                    a.repair_source_fpga, a.repair_target_fpga)
        == std::tie(b.congestion_node_id, b.congestion_source_fpga,
                    b.congestion_target_fpga, b.repair_node_id,
                    b.repair_source_fpga, b.repair_target_fpga);
}
bool operator<(const T5PairKey& a, const T5PairKey& b) {
    return std::tie(a.congestion_node_id, a.congestion_source_fpga,
                    a.congestion_target_fpga, a.repair_node_id,
                    a.repair_source_fpga, a.repair_target_fpga)
        < std::tie(b.congestion_node_id, b.congestion_source_fpga,
                   b.congestion_target_fpga, b.repair_node_id,
                   b.repair_source_fpga, b.repair_target_fpga);
}
bool operator==(const T5RoleAction& a, const T5RoleAction& b) {
    return relevantFieldsEqual(a, b)
        && a.original_result_index == b.original_result_index;
}

std::size_t t5CheckedPairCount(std::size_t a, std::size_t b) {
    if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a) {
        throw std::overflow_error("T5 pair count overflow");
    }
    return a * b;
}

T5PairSpaceResult buildT5PairSpace(
    const std::vector<T3Evaluation>& t3_inputs,
    const std::vector<T4Evaluation>& t4_inputs,
    const std::vector<NodePlacement>& placement,
    int fpga_count, T5PairSpaceMode mode) {
    T5PairSpaceResult result;
    auto& d = result.diagnostics;
    d.raw_t3_input_count = t3_inputs.size();
    d.raw_t4_input_count = t4_inputs.size();

    std::vector<T5RoleAction> congestion_all;
    std::vector<T5RoleAction> repair_all;
    for (std::size_t i = 0; i < t3_inputs.size(); ++i) {
        congestion_all.push_back(fromT3(t3_inputs.at(i), i));
    }
    for (std::size_t i = 0; i < t4_inputs.size(); ++i) {
        repair_all.push_back(fromT4(t4_inputs.at(i), i));
    }
    if (!std::all_of(congestion_all.begin(), congestion_all.end(),
                     validRoleAction)
        || !std::all_of(repair_all.begin(), repair_all.end(),
                        validRoleAction)) {
        result.stop_reason = T5PairSpaceStopReason::InvalidRoleActionInput;
        return result;
    }

    std::map<T5ActionKey, std::vector<T5RoleAction>> congestion_by_key;
    std::map<T5ActionKey, std::vector<T5RoleAction>> repair_by_key;
    for (const auto& action : congestion_all)
        congestion_by_key[actionKey(action)].push_back(action);
    d.unique_congestion_action_key_count = congestion_by_key.size();
    T5PairSpaceStopReason duplicate_stop = T5PairSpaceStopReason::None;
    if (inspectDuplicates(congestion_by_key, T5ActionRole::Congestion,
                          d, duplicate_stop)) {
        result.stop_reason = duplicate_stop;
        return result;
    }
    for (const auto& action : repair_all)
        repair_by_key[actionKey(action)].push_back(action);
    d.unique_repair_action_key_count = repair_by_key.size();
    if (inspectDuplicates(repair_by_key, T5ActionRole::Repair,
                          d, duplicate_stop)) {
        result.stop_reason = duplicate_stop;
        return result;
    }

    std::vector<T5RoleAction> repairs;
    for (const auto& action : repair_all) {
        if (*action.t4_repair_class == T4RepairClass::RepairEligible)
            repairs.push_back(action);
    }
    std::sort(repairs.begin(), repairs.end(), [](const auto& a, const auto& b) {
        return actionKey(a) < actionKey(b);
    });
    d.repair_count = repairs.size();
    result.repair_actions = repairs;
    if (repairs.empty()) {
        result.stop_reason = T5PairSpaceStopReason::NoRepairEligibleAction;
        return result;
    }
    std::int64_t maximum_supply = 0;
    for (const auto& repair : repairs)
        maximum_supply = std::max(maximum_supply, *repair.repair_supply);
    d.maximum_repair_supply = maximum_supply;

    std::vector<T5RoleAction> congestion;
    for (const auto& action : congestion_all) {
        if (*action.t3_class == T3Class::PairEligible
            && (mode == T5PairSpaceMode::AllPairEligible
                || *action.thd_excess <= maximum_supply)) {
            congestion.push_back(action);
        }
    }
    std::sort(congestion.begin(), congestion.end(), [](const auto& a,
                                                        const auto& b) {
        return actionKey(a) < actionKey(b);
    });
    d.congestion_count = congestion.size();
    result.congestion_actions = congestion;
    if (congestion.empty()) {
        result.stop_reason =
            T5PairSpaceStopReason::NoNumericallyCoverableCongestionAction;
        return result;
    }

    std::size_t expected = 0;
    try {
        expected = t5CheckedPairCount(congestion.size(), repairs.size());
    } catch (const std::overflow_error&) {
        result.stop_reason = T5PairSpaceStopReason::ArithmeticOverflow;
        return result;
    }
    d.expected_pair_count = expected;
    std::vector<T5PairCandidate> pairs;
    pairs.reserve(expected);
    std::set<T5PairKey> keys;
    for (const auto& c : congestion) {
        for (const auto& r : repairs) {
            auto pair = makePair(c, r, placement, fpga_count);
            if (!keys.insert(pair.key).second) {
                result.stop_reason = T5PairSpaceStopReason::DuplicatePairKey;
                return result;
            }
            pairs.push_back(std::move(pair));
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) {
        return a.key < b.key;
    });
    d.actual_pair_count = pairs.size();
    d.unique_pair_key_count = keys.size();
    d.same_node_conflict_count = 0;
    d.invalid_current_master_count = 0;
    d.master_replica_conflict_count = 0;
    d.duplicate_final_master_count = 0;
    d.multiple_structural_conflict_count = 0;
    d.can_enter_joint_evaluation_count = 0;
    for (const auto& pair : pairs) {
        *d.same_node_conflict_count += pair.same_node_conflict;
        *d.invalid_current_master_count += pair.invalid_current_master;
        *d.master_replica_conflict_count += pair.master_replica_conflict;
        *d.duplicate_final_master_count += pair.duplicate_final_master;
        const int conditions = pair.same_node_conflict
            + pair.invalid_current_master + pair.master_replica_conflict
            + pair.duplicate_final_master;
        *d.multiple_structural_conflict_count += conditions > 1;
        ++d.first_structural_conflict_counts[pair.first_structural_conflict];
        *d.can_enter_joint_evaluation_count +=
            pair.can_enter_joint_evaluation;
    }
    d.pair_count_conserved = pairs.size() == expected
        && keys.size() == expected;
    result.pairs = std::move(pairs);
    return result;
}

bool operator==(const T5DuplicateInputDiagnostic& a,
                const T5DuplicateInputDiagnostic& b) {
    return a.role == b.role && a.action_key == b.action_key
        && a.input_indices == b.input_indices && a.exact == b.exact;
}
bool operator==(const T5PairCandidate& a, const T5PairCandidate& b) {
    return a.key == b.key && a.congestion == b.congestion
        && a.repair == b.repair
        && a.same_node_conflict == b.same_node_conflict
        && a.invalid_current_master == b.invalid_current_master
        && a.master_replica_conflict == b.master_replica_conflict
        && a.duplicate_final_master == b.duplicate_final_master
        && a.first_structural_conflict == b.first_structural_conflict
        && a.can_enter_joint_evaluation == b.can_enter_joint_evaluation;
}
bool operator==(const T5PairSpaceDiagnostics& a,
                const T5PairSpaceDiagnostics& b) {
    return std::tie(a.raw_t3_input_count, a.raw_t4_input_count,
        a.unique_congestion_action_key_count,
        a.unique_repair_action_key_count,
        a.exact_duplicate_congestion_count,
        a.inconsistent_duplicate_congestion_count,
        a.exact_duplicate_repair_count,
        a.inconsistent_duplicate_repair_count, a.duplicate_inputs,
        a.repair_count, a.maximum_repair_supply, a.congestion_count,
        a.expected_pair_count, a.actual_pair_count,
        a.unique_pair_key_count, a.same_node_conflict_count,
        a.invalid_current_master_count, a.master_replica_conflict_count,
        a.duplicate_final_master_count,
        a.multiple_structural_conflict_count,
        a.first_structural_conflict_counts,
        a.can_enter_joint_evaluation_count, a.pair_count_conserved)
        == std::tie(b.raw_t3_input_count, b.raw_t4_input_count,
        b.unique_congestion_action_key_count,
        b.unique_repair_action_key_count,
        b.exact_duplicate_congestion_count,
        b.inconsistent_duplicate_congestion_count,
        b.exact_duplicate_repair_count,
        b.inconsistent_duplicate_repair_count, b.duplicate_inputs,
        b.repair_count, b.maximum_repair_supply, b.congestion_count,
        b.expected_pair_count, b.actual_pair_count,
        b.unique_pair_key_count, b.same_node_conflict_count,
        b.invalid_current_master_count, b.master_replica_conflict_count,
        b.duplicate_final_master_count,
        b.multiple_structural_conflict_count,
        b.first_structural_conflict_counts,
        b.can_enter_joint_evaluation_count, b.pair_count_conserved);
}
bool operator==(const T5PairSpaceResult& a, const T5PairSpaceResult& b) {
    return a.stop_reason == b.stop_reason && a.diagnostics == b.diagnostics
        && a.congestion_actions == b.congestion_actions
        && a.repair_actions == b.repair_actions && a.pairs == b.pairs;
}

}  // namespace route_prediction_v1
