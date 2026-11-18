#pragma once

#include <vector>

#include "fpga_topology.h"
#include "prediction_types.h"

namespace route_prediction_v1 {

class MetricsCalculator {
public:
    static PredictionMetrics calculate(const std::vector<RoutedConnection>& routed,
                                       const FpgaTopology& topology);
};

}  // namespace route_prediction_v1
