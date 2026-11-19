#include "mandatory_path_metrics.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>

namespace route_prediction_v1 {
namespace {

std::vector<int> distancesAvoidingLink(const FpgaTopology& topology,
                                       int source_fpga,
                                       int removed_link_id) {
    std::vector<int> distances(topology.fpgaCount(), MandatoryPathTable::kInfinity);
    std::queue<int> pending;
    distances.at(source_fpga) = 0;
    pending.push(source_fpga);
    while (!pending.empty()) {
        const int current = pending.front();
        pending.pop();
        for (int next : topology.adjacency(current)) {
            if (topology.linkId(current, next) == removed_link_id) continue;
            if (distances.at(next) != MandatoryPathTable::kInfinity) continue;
            distances.at(next) = distances.at(current) + 1;
            pending.push(next);
        }
    }
    return distances;
}

std::int64_t checkedAdd(std::int64_t current, std::int64_t increment, const char* label) {
    const __int128 next = static_cast<__int128>(current) + increment;
    if (next > std::numeric_limits<std::int64_t>::max()
        || next < std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error(std::string(label) + " exceeds int64_t");
    }
    return static_cast<std::int64_t>(next);
}

}  // namespace

MandatoryPathTable::MandatoryPathTable(const FpgaTopology& topology) {
    const int fpga_count = topology.fpgaCount();
    const int link_count = static_cast<int>(topology.links().size());
    distances_without_.assign(
        link_count,
        std::vector<std::vector<int>>(
            fpga_count, std::vector<int>(fpga_count, kInfinity)));
    mandatory_edges_.assign(
        fpga_count, std::vector<std::vector<int>>(fpga_count));

    for (int link_id = 0; link_id < link_count; ++link_id) {
        for (int source = 0; source < fpga_count; ++source) {
            distances_without_.at(link_id).at(source) =
                distancesAvoidingLink(topology, source, link_id);
        }
    }

    for (int source = 0; source < fpga_count; ++source) {
        for (int destination = 0; destination < fpga_count; ++destination) {
            const int original_distance = topology.hop(source, destination);
            if (original_distance == kInfinity) continue;
            for (int link_id = 0; link_id < link_count; ++link_id) {
                const int distance_without =
                    distances_without_.at(link_id).at(source).at(destination);
                if (distance_without == kInfinity || distance_without > original_distance) {
                    mandatory_edges_.at(source).at(destination).push_back(link_id);
                }
            }
        }
    }
}

int MandatoryPathTable::distanceWithout(int link_id,
                                        int source_fpga,
                                        int destination_fpga) const {
    return distances_without_.at(link_id).at(source_fpga).at(destination_fpga);
}

const std::vector<int>& MandatoryPathTable::mandatoryEdges(int source_fpga,
                                                           int destination_fpga) const {
    return mandatory_edges_.at(source_fpga).at(destination_fpga);
}

MandatoryLoadMetrics MandatoryLoadCalculator::calculate(
    const std::vector<Connection>& connections,
    const FpgaTopology& topology,
    const MandatoryPathTable& mandatory_paths) {
    MandatoryLoadMetrics metrics;
    metrics.mandatory_link_loads.assign(topology.links().size(), 0);

    for (const auto& connection : connections) {
        if (connection.source_fpga < 0 || connection.source_fpga >= topology.fpgaCount()
            || connection.destination_fpga < 0
            || connection.destination_fpga >= topology.fpgaCount()) {
            throw std::runtime_error("connection FPGA endpoint is out of range");
        }
        if (connection.demand < 0) throw std::runtime_error("connection demand must be nonnegative");
        for (int link_id : mandatory_paths.mandatoryEdges(
                 connection.source_fpga, connection.destination_fpga)) {
            auto& load = metrics.mandatory_link_loads.at(link_id);
            load = checkedAdd(load, connection.demand, "mandatory link load");
        }
    }

    for (std::int64_t load : metrics.mandatory_link_loads) {
        if (load < 0) throw std::runtime_error("mandatory link load became negative");
        metrics.maximum_mandatory_link_load =
            std::max(metrics.maximum_mandatory_link_load, load);
        metrics.total_mandatory_link_load = checkedAdd(
            metrics.total_mandatory_link_load, load, "total mandatory link load");
        const __int128 square = static_cast<__int128>(load) * load;
        const __int128 next =
            static_cast<__int128>(metrics.squared_mandatory_link_load_sum) + square;
        if (next > std::numeric_limits<std::int64_t>::max()) {
            throw std::overflow_error("squared mandatory link-load sum exceeds int64_t");
        }
        metrics.squared_mandatory_link_load_sum = static_cast<std::int64_t>(next);
    }

    if (metrics.total_mandatory_link_load > 0) {
        const long double total =
            static_cast<long double>(metrics.total_mandatory_link_load);
        metrics.normalized_mandatory_link_concentration =
            static_cast<long double>(metrics.squared_mandatory_link_load_sum)
            / (total * total);
    }
    if (metrics.maximum_mandatory_link_load > 0) {
        for (int link_id = 0;
             link_id < static_cast<int>(metrics.mandatory_link_loads.size());
             ++link_id) {
            if (metrics.mandatory_link_loads.at(link_id)
                == metrics.maximum_mandatory_link_load) {
                metrics.hot_mandatory_link_ids.push_back(link_id);
            }
        }
    }
    return metrics;
}

}  // namespace route_prediction_v1
