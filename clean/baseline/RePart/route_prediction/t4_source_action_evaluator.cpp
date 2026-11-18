#include "t4_source_action_evaluator.h"

#include <limits>
#include <stdexcept>

namespace route_prediction_v1 {
namespace {

std::int64_t checkedNegate(std::int64_t value) {
    if (value == std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error("T4 RepairSupply overflow");
    }
    return -value;
}

void classify(T4Evaluation& result) {
    const auto& move = result.master_move;
    if (move.status == T3EvaluationStatus::ArithmeticOverflow) return;
    if (!move.all_violations.empty()) {
        result.repair_class = T4RepairClass::HardIllegal;
        return;
    }
    if (move.metric_availability != MetricAvailability::Complete
        || !move.delta_wthd) {
        throw std::logic_error("T4 legal move lacks complete THD metrics");
    }
    if (*move.delta_wthd >= 0) {
        result.repair_class = T4RepairClass::NotRepairing;
        return;
    }
    result.repair_class = T4RepairClass::RepairEligible;
    result.repair_supply = checkedNegate(*move.delta_wthd);
    if (*result.repair_supply <= 0) {
        throw std::logic_error("T4 RepairSupply must be positive");
    }
}

T4Evaluation wrap(const T4SourceShorteningAction& action,
                  T3Evaluation move) {
    T4Evaluation result;
    result.source_action = action;
    result.master_move = std::move(move);
    try {
        classify(result);
    } catch (const std::overflow_error& error) {
        result.master_move.status = T3EvaluationStatus::ArithmeticOverflow;
        result.master_move.arithmetic_overflow_diagnostic = error.what();
        result.master_move.classification.reset();
        result.repair_class.reset();
        result.repair_supply.reset();
    }
    return result;
}

}  // namespace

T4Evaluation evaluateT4SourceAction(
    const T3StateCache& state,
    const T4SourceShorteningAction& action) {
    return wrap(action, evaluateMasterMove(state, action.action, 0));
}

T4Evaluation rebuildT4SourceActionFromScratch(
    const T3StateCache& state,
    const T4SourceShorteningAction& action) {
    return wrap(action,
                rebuildMasterMoveFromScratch(state, action.action, 0)
                    .evaluation);
}

}  // namespace route_prediction_v1
