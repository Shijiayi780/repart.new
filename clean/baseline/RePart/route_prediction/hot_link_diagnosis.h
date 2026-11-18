#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "fpga_topology.h"

namespace route_prediction_b2 {

struct IndexedConnection {
    int id = -1;
    int net_id = -1;
    int source_fpga = -1;
    int destination_fpga = -1;
    std::int64_t demand = 1;
    std::vector<int> physical_link_ids;
};

struct ConnectionBypass {
    int connection_id = -1;
    int net_id = -1;
    std::int64_t demand = 0;
    int original_distance = -1;
    int distance_without_link = -1;
    int delta = -1;
    enum class Category { EqualShortest, ExtraHop, Unavoidable } category = Category::Unavoidable;
};

struct HotLinkDiagnosis {
    int link_id = -1;
    std::int64_t link_load = 0;
    std::vector<int> connection_ids;
    std::set<int> unique_net_ids;
    std::vector<ConnectionBypass> bypasses;
    std::int64_t equal_shortest_bypass_load = 0;
    std::map<int, std::int64_t> extra_hop_bypass_load_by_delta;
    std::int64_t unavoidable_load = 0;
};

class ReverseIndexDiagnosis {
public:
    ReverseIndexDiagnosis(const route_prediction_v1::FpgaTopology& topology,
                          std::vector<IndexedConnection> connections);

    const std::vector<std::vector<int>>& linkToConnections() const { return link_to_connections_; }
    const std::vector<std::set<int>>& linkToUniqueNets() const { return link_to_unique_nets_; }
    const std::vector<IndexedConnection>& connections() const { return connections_; }
    void validateForwardReconstruction() const;
    HotLinkDiagnosis diagnose(int link_id, std::int64_t link_load) const;
    int distanceAvoidingLink(int source, int destination, int removed_link_id) const;

private:
    const IndexedConnection& connection(int id) const;

    const route_prediction_v1::FpgaTopology& topology_;
    std::vector<IndexedConnection> connections_;
    std::vector<int> id_to_position_;
    std::vector<std::vector<int>> link_to_connections_;
    std::vector<std::set<int>> link_to_unique_nets_;
};

const char* categoryName(ConnectionBypass::Category category);

}  // namespace route_prediction_b2
