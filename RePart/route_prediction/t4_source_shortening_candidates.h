#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "t3_candidate_evaluator.h"

namespace route_prediction_v1 {

struct T4SourceShorteningAction {
    T2ActionKey action;
    std::vector<ConnectionKey> shortened_connection_keys;
};

struct T4SourceShorteningDiagnostics {
    std::int64_t source_node_count = 0;
    std::int64_t source_connection_count = 0;
    std::int64_t raw_node_target_count = 0;
    std::int64_t no_op_target_count = 0;
    std::int64_t master_replica_conflict_count = 0;
    std::int64_t unique_action_count = 0;
    std::int64_t minimum_shortened_connection_count = 0;
    std::int64_t maximum_shortened_connection_count = 0;
    std::int64_t total_shortened_connection_count = 0;
};

struct T4SourceShorteningResult {
    std::vector<T4SourceShorteningAction> actions;
    T4SourceShorteningDiagnostics diagnostics;
};

std::vector<std::vector<Connection>> buildT4SourceConnectionsByNode(
    const T3StateCache& state);

T4SourceShorteningResult generateT4SourceShorteningActions(
    const T3StateCache& state);

}  // namespace route_prediction_v1
