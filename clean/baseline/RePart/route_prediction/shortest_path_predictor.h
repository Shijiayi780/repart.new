#pragma once

#include <vector>

#include "fpga_topology.h"
#include "prediction_types.h"

namespace route_prediction_v1 {

class ShortestPathPredictor {
public:
    static std::vector<RoutedConnection> route(const std::vector<Connection>& connections,
                                               const FpgaTopology& topology);
};

}  // namespace route_prediction_v1
