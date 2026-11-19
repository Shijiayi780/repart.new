#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace route_prediction_b4 {

using ResourceVector = std::array<std::int64_t, 8>;

enum class CandidateClass { ResourceRejected, HopRejected, CommunicationRejected,
                            HotLinkNeutral, HotLinkWorsening, HotLinkEffective };

struct MoveEvaluation {
    int node = -1;
    int source_fpga = -1;
    int target_fpga = -1;
    CandidateClass classification = CandidateClass::ResourceRejected;
    std::int64_t hot_load_after = 0, delta_hot_load = 0;
    std::int64_t max_load_after = 0, delta_max_load = 0;
    long double squared_load_after = 0, delta_squared_load = 0;
    std::int64_t unweighted_thd_after = 0, delta_unweighted_thd = 0;
    std::int64_t weighted_thd_after = 0, delta_weighted_thd = 0;
    int maximum_hop_after = 0;
    int added_connections = 0, deleted_connections = 0, changed_connections = 0;
    int link9_rank_after = 0;
    std::vector<int> hot_links_after;
    std::vector<std::int64_t> link_loads_after;
    bool resource_legal = false, hop_legal = false, communication_legal = false;
};

struct EvaluationTiming {
    double resource_seconds = 0;
    double projection_and_routing_seconds = 0;
    double legality_and_metrics_seconds = 0;
};

const char* candidateClassName(CandidateClass value);

}  // namespace route_prediction_b4
