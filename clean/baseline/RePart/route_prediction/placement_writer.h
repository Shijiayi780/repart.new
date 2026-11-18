#pragma once

#include <string>
#include <vector>

#include "connection_projector.h"

namespace route_prediction_b5 {

struct PlacementDiff {
    int node = -1;
    int old_master = -1;
    int new_master = -1;
};

void writePlacement(const std::string& path,
                    const route_prediction_v1::CaseData& data,
                    const route_prediction_v1::FpgaTopology& topology);

std::vector<route_prediction_v1::NodePlacement> readPlacement(
    const std::string& path, const std::vector<std::string>& node_names,
    const route_prediction_v1::FpgaTopology& topology);

std::vector<PlacementDiff> comparePlacements(
    const std::vector<route_prediction_v1::NodePlacement>& before,
    const std::vector<route_prediction_v1::NodePlacement>& after);

}  // namespace route_prediction_b5
