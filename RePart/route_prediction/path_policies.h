#pragma once

#include <cstdint>
#include <vector>

#include "fpga_topology.h"
#include "prediction_types.h"

namespace route_prediction_b3 {

enum class PathPolicy { Baseline, ReverseTie, LoadAwareEqualShortest };

struct PolicyRun {
    PathPolicy policy = PathPolicy::Baseline;
    std::vector<route_prediction_v1::RoutedConnection> routed;
    route_prediction_v1::PredictionMetrics metrics;
};

const char* policyName(PathPolicy policy);

class EqualShortestPathPolicies {
public:
    static PolicyRun route(const std::vector<route_prediction_v1::Connection>& connections,
                           const route_prediction_v1::FpgaTopology& topology,
                           PathPolicy policy);

    static std::vector<int> chooseLoadAwarePath(
        int source, int destination, std::int64_t demand,
        const std::vector<std::int64_t>& current_link_loads,
        const route_prediction_v1::FpgaTopology& topology);

    static void validateShortestPath(const route_prediction_v1::RoutedConnection& routed,
                                     const route_prediction_v1::FpgaTopology& topology);
};

}  // namespace route_prediction_b3
