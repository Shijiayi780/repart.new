#pragma once

#include <set>
#include <vector>

#include "connection_projector.h"
#include "hot_candidate_types.h"
#include "local_move_evaluator.h"

namespace route_prediction_b4 {

struct CandidateFunnel {
    std::int64_t mandatory_connections=0, unique_nets=0, source_nodes=0, effective_sink_nodes=0,
      structure_nodes=0, raw_operations=0, excluded_master=0, excluded_replica=0,
      resource_rejected=0, simulated=0, neutral=0, worsening=0, effective=0,
      lowers_max=0, max_same_squared_lower=0, larger_new_hotspot=0,
      weighted_thd_nonpositive=0, hard_legal=0, hop_rejected=0, communication_rejected=0;
};

struct CandidateGenerationResult {
    std::vector<int> mandatory_connection_ids;
    std::set<int> mandatory_net_ids, source_nodes, effective_sink_nodes, structure_nodes;
    std::vector<MoveEvaluation> operations;
    CandidateFunnel funnel;
    double structure_seconds = 0;
    double raw_enumeration_seconds = 0;
    double resource_prefilter_seconds = 0;
    double local_projection_seconds = 0;
    double legality_seconds = 0;
};

CandidateGenerationResult generateCandidates(const route_prediction_v1::CaseData& data,
    const route_prediction_v1::FpgaTopology& topology, const LocalMoveEvaluator& evaluator,
    int hot_link_id);

} // namespace route_prediction_b4
