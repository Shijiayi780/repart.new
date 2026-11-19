#include "hot_link_diagnosis.h"

#include <algorithm>
#include <queue>
#include <stdexcept>

namespace route_prediction_b2 {

ReverseIndexDiagnosis::ReverseIndexDiagnosis(const route_prediction_v1::FpgaTopology& topology,
                                             std::vector<IndexedConnection> connections)
    : topology_(topology), connections_(std::move(connections)),
      link_to_connections_(topology.links().size()), link_to_unique_nets_(topology.links().size()) {
    int maximum_id = -1;
    for (const auto& item : connections_) maximum_id = std::max(maximum_id, item.id);
    id_to_position_.assign(maximum_id + 1, -1);
    for (int position = 0; position < static_cast<int>(connections_.size()); ++position) {
        const auto& item = connections_[position];
        if (item.id < 0 || id_to_position_[item.id] != -1) throw std::runtime_error("duplicate or invalid connection id");
        id_to_position_[item.id] = position;
        for (int link : item.physical_link_ids) {
            if (link < 0 || link >= static_cast<int>(link_to_connections_.size())) throw std::runtime_error("connection path has invalid link id");
            link_to_connections_[link].push_back(item.id);
            link_to_unique_nets_[link].insert(item.net_id);
        }
    }
    for (auto& ids : link_to_connections_) std::sort(ids.begin(), ids.end());
}

const IndexedConnection& ReverseIndexDiagnosis::connection(int id) const {
    if (id < 0 || id >= static_cast<int>(id_to_position_.size()) || id_to_position_[id] < 0) {
        throw std::runtime_error("unknown connection id in reverse index");
    }
    return connections_[id_to_position_[id]];
}

void ReverseIndexDiagnosis::validateForwardReconstruction() const {
    std::int64_t forward_entries = 0;
    std::int64_t reverse_entries = 0;
    for (const auto& item : connections_) forward_entries += item.physical_link_ids.size();
    for (int link = 0; link < static_cast<int>(link_to_connections_.size()); ++link) {
        reverse_entries += link_to_connections_[link].size();
        for (int connection_id : link_to_connections_[link]) {
            const auto& path = connection(connection_id).physical_link_ids;
            if (std::find(path.begin(), path.end(), link) == path.end()) {
                throw std::runtime_error("reverse index contains a connection whose path omits the link");
            }
        }
    }
    if (forward_entries != reverse_entries) throw std::runtime_error("forward/reverse entry count mismatch");
}

int ReverseIndexDiagnosis::distanceAvoidingLink(int source, int destination, int removed_link_id) const {
    std::vector<int> distance(topology_.fpgaCount(), -1);
    std::queue<int> queue;
    distance[source] = 0;
    queue.push(source);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        for (int next : topology_.adjacency(current)) {
            if (topology_.linkId(current, next) == removed_link_id) continue;
            if (distance[next] != -1) continue;
            distance[next] = distance[current] + 1;
            queue.push(next);
        }
    }
    return distance[destination];
}

HotLinkDiagnosis ReverseIndexDiagnosis::diagnose(int link_id, std::int64_t link_load) const {
    if (link_id < 0 || link_id >= static_cast<int>(link_to_connections_.size())) throw std::runtime_error("invalid hot link id");
    HotLinkDiagnosis result;
    result.link_id = link_id;
    result.link_load = link_load;
    result.connection_ids = link_to_connections_[link_id];
    result.unique_net_ids = link_to_unique_nets_[link_id];
    std::int64_t indexed_load = 0;
    for (int connection_id : result.connection_ids) {
        const auto& item = connection(connection_id);
        ConnectionBypass bypass;
        bypass.connection_id = item.id;
        bypass.net_id = item.net_id;
        bypass.demand = item.demand;
        bypass.original_distance = topology_.hop(item.source_fpga, item.destination_fpga);
        bypass.distance_without_link = distanceAvoidingLink(item.source_fpga, item.destination_fpga, link_id);
        if (bypass.distance_without_link < 0) {
            bypass.category = ConnectionBypass::Category::Unavoidable;
            result.unavoidable_load += item.demand;
        } else {
            bypass.delta = bypass.distance_without_link - bypass.original_distance;
            if (bypass.delta < 0) throw std::runtime_error("edge removal shortened a shortest distance");
            if (bypass.delta == 0) {
                bypass.category = ConnectionBypass::Category::EqualShortest;
                result.equal_shortest_bypass_load += item.demand;
            } else {
                bypass.category = ConnectionBypass::Category::ExtraHop;
                result.extra_hop_bypass_load_by_delta[bypass.delta] += item.demand;
            }
        }
        indexed_load += item.demand;
        result.bypasses.push_back(bypass);
    }
    std::int64_t classified = result.equal_shortest_bypass_load + result.unavoidable_load;
    for (const auto& [delta, load] : result.extra_hop_bypass_load_by_delta) classified += load;
    if (indexed_load != link_load || classified != link_load) throw std::runtime_error("hot-link load classification does not conserve LinkLoad");
    return result;
}

const char* categoryName(ConnectionBypass::Category category) {
    switch (category) {
        case ConnectionBypass::Category::EqualShortest: return "EqualShortestBypass";
        case ConnectionBypass::Category::ExtraHop: return "ExtraHopBypass";
        case ConnectionBypass::Category::Unavoidable: return "Unavoidable";
    }
    return "Unknown";
}

}  // namespace route_prediction_b2
