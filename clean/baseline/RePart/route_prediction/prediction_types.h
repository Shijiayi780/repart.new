#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace route_prediction_v1 {

struct NodePlacement {
    int master_fpga = -1;
    std::vector<int> replica_fpgas;
};

struct Net {
    int id = -1;
    int source_node = -1;
    std::vector<int> sink_nodes;
    std::int64_t weight = 0;
};

struct Connection {
    int id = -1;
    int net_id = -1;
    int source_node = -1;
    int source_fpga = -1;
    int destination_fpga = -1;
    std::int64_t demand = 1;
    std::int64_t net_weight = 0;
};

struct PhysicalLink {
    int id = -1;
    int u = -1;
    int v = -1;
};

struct RoutedConnection {
    Connection connection;
    std::vector<int> fpga_path;
    std::vector<int> physical_link_ids;
    bool reachable = false;
};

struct PredictionMetrics {
    std::vector<std::int64_t> link_loads;
    std::int64_t connection_count = 0;
    std::int64_t unreachable_count = 0;
    std::int64_t unweighted_thd = 0;
    std::int64_t weighted_thd = 0;
    std::int64_t total_link_load = 0;
    std::int64_t total_routed_path_edges = 0;
    std::int64_t total_routed_path_edges_weighted_by_demand = 0;
    std::int64_t max_link_load = 0;
    std::int64_t squared_link_load_sum = 0;
    long double normalized_link_concentration = 0.0L;
    long double mean_link_load = 0.0L;
    long double link_load_variance = 0.0L;
    // Kept as a compatibility alias for existing reports.
    double average_link_load = 0.0;
    bool all_connection_demands_are_one = true;
    std::vector<int> hot_link_ids;
};

}  // namespace route_prediction_v1
