#include "prediction_metrics.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace route_prediction_v1 {
namespace {
std::int64_t checkedAddProduct(std::int64_t current, std::int64_t left, std::int64_t right,
                               const char* label) {
    const __int128 next = static_cast<__int128>(current)
                          + static_cast<__int128>(left) * static_cast<__int128>(right);
    if (next > std::numeric_limits<std::int64_t>::max()
        || next < std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error(std::string(label) + " exceeds int64_t");
    }
    return static_cast<std::int64_t>(next);
}
}  // namespace

PredictionMetrics MetricsCalculator::calculate(const std::vector<RoutedConnection>& routed,
                                               const FpgaTopology& topology) {
    PredictionMetrics metrics;
    metrics.connection_count = static_cast<std::int64_t>(routed.size());
    metrics.link_loads.assign(topology.links().size(), 0);
    for (const auto& item : routed) {
        if (!item.reachable) {
            ++metrics.unreachable_count;
            continue;
        }
        const auto hops = static_cast<std::int64_t>(item.physical_link_ids.size());
        metrics.all_connection_demands_are_one &= item.connection.demand == 1;
        metrics.unweighted_thd += hops;
        metrics.weighted_thd = checkedAddProduct(metrics.weighted_thd, hops,
                                                 item.connection.net_weight, "weighted THD");
        metrics.total_routed_path_edges += hops;
        metrics.total_routed_path_edges_weighted_by_demand = checkedAddProduct(
            metrics.total_routed_path_edges_weighted_by_demand, hops, item.connection.demand,
            "demand-weighted routed path edges");
        for (int link : item.physical_link_ids) metrics.link_loads.at(link) += item.connection.demand;
    }
    for (std::int64_t load : metrics.link_loads) {
        metrics.total_link_load += load;
        metrics.max_link_load = std::max(metrics.max_link_load, load);
        const __int128 square = static_cast<__int128>(load) * static_cast<__int128>(load);
        const __int128 next = static_cast<__int128>(metrics.squared_link_load_sum) + square;
        if (next > std::numeric_limits<std::int64_t>::max()) {
            throw std::overflow_error("squared link-load sum exceeds int64_t");
        }
        metrics.squared_link_load_sum = static_cast<std::int64_t>(next);
    }
    if (!metrics.link_loads.empty()) {
        const long double m = static_cast<long double>(metrics.link_loads.size());
        metrics.mean_link_load = static_cast<long double>(metrics.total_link_load) / m;
        metrics.average_link_load = static_cast<double>(metrics.mean_link_load);
        metrics.link_load_variance = static_cast<long double>(metrics.squared_link_load_sum) / m
                                     - metrics.mean_link_load * metrics.mean_link_load;
    }
    if (metrics.total_link_load > 0) {
        const long double total = static_cast<long double>(metrics.total_link_load);
        metrics.normalized_link_concentration =
            static_cast<long double>(metrics.squared_link_load_sum) / (total * total);
    }
    for (int link = 0; link < static_cast<int>(metrics.link_loads.size()); ++link) {
        if (metrics.link_loads[link] == metrics.max_link_load) metrics.hot_link_ids.push_back(link);
    }
    if (metrics.total_link_load != metrics.total_routed_path_edges_weighted_by_demand) {
        throw std::runtime_error(
            "link-load conservation failed: total_link_load="
            + std::to_string(metrics.total_link_load)
            + " total_routed_path_edges_weighted_by_demand="
            + std::to_string(metrics.total_routed_path_edges_weighted_by_demand));
    }
    if (metrics.all_connection_demands_are_one
        && metrics.total_link_load != metrics.unweighted_thd) {
        throw std::runtime_error(
            "unit-demand conservation failed: total_link_load="
            + std::to_string(metrics.total_link_load)
            + " unweighted_thd=" + std::to_string(metrics.unweighted_thd));
    }
    return metrics;
}

}  // namespace route_prediction_v1
