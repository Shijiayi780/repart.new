#include "t4_source_shortening_candidates.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace route_prediction_v1 {
namespace {

ConnectionKey keyOf(const Connection& connection) {
    return {connection.net_id, connection.source_fpga,
            connection.destination_fpga};
}

int extendedDistance(const FpgaTopology& topology, int source,
                     int destination) {
    const int distance = topology.hop(source, destination);
    return distance < 0 ? std::numeric_limits<int>::max() : distance;
}

bool actionLess(const T4SourceShorteningAction& lhs,
                const T4SourceShorteningAction& rhs) {
    return lhs.action < rhs.action;
}

}  // namespace

std::vector<std::vector<Connection>> buildT4SourceConnectionsByNode(
    const T3StateCache& state) {
    if (state.net_connections_current.size() != state.data.nets.size()) {
        throw std::invalid_argument(
            "T4 per-net connection cache dimension mismatch");
    }
    std::vector<std::vector<Connection>> result(
        state.data.placements.size());
    for (int net_id = 0; net_id < static_cast<int>(state.data.nets.size());
         ++net_id) {
        const auto& net = state.data.nets.at(net_id);
        if (net.id != net_id) {
            throw std::invalid_argument("T4 requires dense ordered net IDs");
        }
        if (net.source_node < 0
            || net.source_node >= static_cast<int>(result.size())) {
            throw std::invalid_argument("T4 net source node is out of range");
        }
        const auto& connections = state.net_connections_current.at(net_id);
        for (const auto& connection : connections) {
            if (connection.net_id != net.id
                || connection.source_node != net.source_node) {
                throw std::invalid_argument(
                    "T4 cached connection does not match its source net");
            }
            result.at(net.source_node).push_back(connection);
        }
    }
    for (auto& connections : result) {
        std::sort(connections.begin(), connections.end(),
                  [](const Connection& lhs, const Connection& rhs) {
                      return keyOf(lhs) < keyOf(rhs);
                  });
    }
    return result;
}

T4SourceShorteningResult generateT4SourceShorteningActions(
    const T3StateCache& state) {
    T4SourceShorteningResult result;
    const auto source_connections = buildT4SourceConnectionsByNode(state);
    const int fpga_count = state.topology.fpgaCount();
    if (source_connections.size() != state.data.placements.size()) {
        throw std::logic_error("T4 source index node dimension mismatch");
    }

    for (int node_id = 0;
         node_id < static_cast<int>(source_connections.size()); ++node_id) {
        const auto& connections = source_connections.at(node_id);
        if (connections.empty()) continue;
        ++result.diagnostics.source_node_count;
        result.diagnostics.source_connection_count += connections.size();
        const auto& placement = state.data.placements.at(node_id);
        const int current_master = placement.master_fpga;
        if (current_master < 0 || current_master >= fpga_count) {
            throw std::invalid_argument("T4 current master is out of range");
        }

        for (int target = 0; target < fpga_count; ++target) {
            ++result.diagnostics.raw_node_target_count;
            if (target == current_master) {
                ++result.diagnostics.no_op_target_count;
                continue;
            }
            if (target < 0 || target >= fpga_count) {
                continue;
            }
            if (std::binary_search(placement.replica_fpgas.begin(),
                                   placement.replica_fpgas.end(), target)) {
                ++result.diagnostics.master_replica_conflict_count;
                continue;
            }

            T4SourceShorteningAction candidate;
            candidate.action = {node_id, target};
            for (const auto& connection : connections) {
                const int old_distance = extendedDistance(
                    state.topology, current_master,
                    connection.destination_fpga);
                const int new_distance = extendedDistance(
                    state.topology, target, connection.destination_fpga);
                if (new_distance < old_distance) {
                    candidate.shortened_connection_keys.push_back(
                        keyOf(connection));
                }
            }
            if (!candidate.shortened_connection_keys.empty()) {
                std::sort(candidate.shortened_connection_keys.begin(),
                          candidate.shortened_connection_keys.end());
                candidate.shortened_connection_keys.erase(
                    std::unique(candidate.shortened_connection_keys.begin(),
                                candidate.shortened_connection_keys.end()),
                    candidate.shortened_connection_keys.end());
                result.actions.push_back(std::move(candidate));
            }
        }
    }

    std::sort(result.actions.begin(), result.actions.end(), actionLess);
    result.actions.erase(
        std::unique(result.actions.begin(), result.actions.end(),
                    [](const T4SourceShorteningAction& lhs,
                       const T4SourceShorteningAction& rhs) {
                        return lhs.action == rhs.action;
                    }),
        result.actions.end());
    result.diagnostics.unique_action_count = result.actions.size();
    for (const auto& candidate : result.actions) {
        const auto count = static_cast<std::int64_t>(
            candidate.shortened_connection_keys.size());
        result.diagnostics.total_shortened_connection_count += count;
        if (result.diagnostics.minimum_shortened_connection_count == 0
            || count < result.diagnostics.minimum_shortened_connection_count) {
            result.diagnostics.minimum_shortened_connection_count = count;
        }
        result.diagnostics.maximum_shortened_connection_count = std::max(
            result.diagnostics.maximum_shortened_connection_count, count);
    }
    return result;
}

}  // namespace route_prediction_v1
