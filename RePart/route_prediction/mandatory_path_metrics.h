#pragma once

#include <cstdint>
#include <vector>

#include "fpga_topology.h"
#include "prediction_types.h"

namespace route_prediction_v1 {

class MandatoryPathTable {
public:
    explicit MandatoryPathTable(const FpgaTopology& topology);

    static constexpr int kInfinity = -1;

    int distanceWithout(int link_id, int source_fpga, int destination_fpga) const;
    const std::vector<int>& mandatoryEdges(int source_fpga, int destination_fpga) const;

private:
    std::vector<std::vector<std::vector<int>>> distances_without_;
    std::vector<std::vector<std::vector<int>>> mandatory_edges_;
};

struct MandatoryLoadMetrics {
    std::vector<std::int64_t> mandatory_link_loads;
    std::int64_t maximum_mandatory_link_load = 0;
    std::int64_t total_mandatory_link_load = 0;
    std::int64_t squared_mandatory_link_load_sum = 0;
    long double normalized_mandatory_link_concentration = 0.0L;
    std::vector<int> hot_mandatory_link_ids;
};

class MandatoryLoadCalculator {
public:
    static MandatoryLoadMetrics calculate(const std::vector<Connection>& connections,
                                            const FpgaTopology& topology,
                                            const MandatoryPathTable& mandatory_paths);
};

}  // namespace route_prediction_v1
