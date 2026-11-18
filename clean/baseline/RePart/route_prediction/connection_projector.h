#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "fpga_topology.h"
#include "prediction_types.h"

namespace route_prediction_v1 {

struct CaseData {
    std::vector<std::string> node_names;
    std::unordered_map<std::string, int> node_ids;
    std::vector<Net> nets;
    std::vector<NodePlacement> placements;
};

struct PlacementStats {
    int master_nodes = 0;
    int replicated_nodes = 0;
    int replica_locations = 0;
};

class ConnectionProjector {
public:
    static CaseData load(const std::string& case_directory,
                         const std::string& partition_path,
                         const FpgaTopology& topology);
    static PlacementStats validatePlacements(const std::vector<NodePlacement>& placements,
                                             int fpga_count);
    static std::vector<Connection> projectV2(const std::vector<Net>& nets,
                                             const std::vector<NodePlacement>& placements);
};

}  // namespace route_prediction_v1
