#pragma once

#include <cstdint>
#include <optional>

#include "t3_candidate_evaluator.h"
#include "t4_source_shortening_candidates.h"

namespace route_prediction_v1 {

enum class T4RepairClass { HardIllegal, NotRepairing, RepairEligible };

struct T4Evaluation {
    T4SourceShorteningAction source_action;
    T3Evaluation master_move;
    std::optional<T4RepairClass> repair_class;
    std::optional<std::int64_t> repair_supply;
};

T4Evaluation evaluateT4SourceAction(
    const T3StateCache& state,
    const T4SourceShorteningAction& action);

T4Evaluation rebuildT4SourceActionFromScratch(
    const T3StateCache& state,
    const T4SourceShorteningAction& action);

}  // namespace route_prediction_v1
