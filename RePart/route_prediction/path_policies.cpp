#include "path_policies.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

#include "prediction_metrics.h"

namespace route_prediction_b3 {
using namespace route_prediction_v1;
namespace {

std::vector<int> toLinks(const std::vector<int>& path, const FpgaTopology& topology) {
    std::vector<int> links;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const int link = topology.linkId(path[i - 1], path[i]);
        if (link < 0) throw std::runtime_error("path policy produced a non-physical edge");
        links.push_back(link);
    }
    return links;
}

std::vector<int> reverseTiePath(int source, int destination, const FpgaTopology& topology) {
    std::vector<int> distance(topology.fpgaCount(), -1), parent(topology.fpgaCount(), -1);
    std::queue<int> queue;
    distance[source] = 0;
    queue.push(source);
    while (!queue.empty()) {
        const int current = queue.front(); queue.pop();
        auto neighbors = topology.adjacency(current);
        std::sort(neighbors.rbegin(), neighbors.rend());
        for (int next : neighbors) {
            if (distance[next] >= 0) continue;
            distance[next] = distance[current] + 1;
            parent[next] = current;
            queue.push(next);
        }
    }
    if (distance[destination] < 0) return {};
    std::vector<int> reverse;
    for (int current = destination; current != -1; current = parent[current]) {
        reverse.push_back(current);
        if (current == source) break;
    }
    return {reverse.rbegin(), reverse.rend()};
}

struct CandidateScore {
    std::int64_t maximum = 0;
    long double squares = 0;
    std::vector<int> links;
};

bool better(const CandidateScore& a, const CandidateScore& b) {
    if (a.maximum != b.maximum) return a.maximum < b.maximum;
    if (a.squares != b.squares) return a.squares < b.squares;
    return a.links < b.links;
}

CandidateScore score(const std::vector<int>& links, std::int64_t demand,
                     const std::vector<std::int64_t>& loads) {
    CandidateScore result;
    result.links = links;
    std::vector<char> on_path(loads.size(), 0);
    for (int link : links) on_path.at(link) = 1;
    for (int link = 0; link < static_cast<int>(loads.size()); ++link) {
        const auto value = loads[link] + (on_path[link] ? demand : 0);
        result.maximum = std::max(result.maximum, value);
        result.squares += static_cast<long double>(value) * value;
    }
    return result;
}

RoutedConnection makeRouted(const Connection& connection, std::vector<int> fpga_path,
                            const FpgaTopology& topology) {
    RoutedConnection result;
    result.connection = connection;
    result.fpga_path = std::move(fpga_path);
    result.reachable = !result.fpga_path.empty();
    if (result.reachable) result.physical_link_ids = toLinks(result.fpga_path, topology);
    return result;
}
}  // namespace

const char* policyName(PathPolicy policy) {
    switch (policy) {
        case PathPolicy::Baseline: return "BaselinePolicy";
        case PathPolicy::ReverseTie: return "ReverseTiePolicy";
        case PathPolicy::LoadAwareEqualShortest: return "LoadAwareEqualShortestPolicy";
    }
    return "UnknownPolicy";
}

std::vector<int> EqualShortestPathPolicies::chooseLoadAwarePath(
    int source, int destination, std::int64_t demand,
    const std::vector<std::int64_t>& current_link_loads,
    const FpgaTopology& topology) {
    const int shortest = topology.hop(source, destination);
    if (shortest < 0) return {};
    std::vector<int> current{source}, best_path;
    CandidateScore best_score;
    bool found = false;
    std::function<void(int)> enumerate = [&](int vertex) {
        if (vertex == destination) {
            const auto links = toLinks(current, topology);
            const auto candidate = score(links, demand, current_link_loads);
            if (!found || better(candidate, best_score)) {
                found = true; best_score = candidate; best_path = current;
            }
            return;
        }
        const int used = static_cast<int>(current.size()) - 1;
        for (int next : topology.adjacency(vertex)) {
            if (topology.hop(next, destination) != shortest - used - 1) continue;
            current.push_back(next);
            enumerate(next);
            current.pop_back();
        }
    };
    enumerate(source);
    if (!found) throw std::runtime_error("shortest-path DAG enumeration found no path");
    return best_path;
}

void EqualShortestPathPolicies::validateShortestPath(const RoutedConnection& routed,
                                                     const FpgaTopology& topology) {
    if (!routed.reachable) return;
    topology.validatePath(routed.connection.source_fpga, routed.connection.destination_fpga, routed.fpga_path);
    if (routed.physical_link_ids.size() != routed.fpga_path.size() - 1) throw std::runtime_error("path/link continuity mismatch");
    if (static_cast<int>(routed.physical_link_ids.size()) != topology.hop(routed.connection.source_fpga, routed.connection.destination_fpga)) {
        throw std::runtime_error("B3 policy used a non-shortest path");
    }
    if (routed.physical_link_ids != toLinks(routed.fpga_path, topology)) throw std::runtime_error("path link sequence mismatch");
}

PolicyRun EqualShortestPathPolicies::route(const std::vector<Connection>& connections,
                                           const FpgaTopology& topology, PathPolicy policy) {
    PolicyRun result; result.policy = policy;
    result.routed.resize(connections.size());
    std::vector<int> order(connections.size());
    for (int i = 0; i < static_cast<int>(connections.size()); ++i) order[i] = i;
    if (policy == PathPolicy::LoadAwareEqualShortest) {
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            if (connections[a].demand != connections[b].demand) return connections[a].demand > connections[b].demand;
            return connections[a].id < connections[b].id;
        });
    }
    std::vector<std::int64_t> loads(topology.links().size(), 0);
    for (int position : order) {
        const auto& connection = connections[position];
        std::vector<int> path;
        if (policy == PathPolicy::Baseline) path = topology.path(connection.source_fpga, connection.destination_fpga);
        else if (policy == PathPolicy::ReverseTie) path = reverseTiePath(connection.source_fpga, connection.destination_fpga, topology);
        else path = chooseLoadAwarePath(connection.source_fpga, connection.destination_fpga, connection.demand, loads, topology);
        auto routed = makeRouted(connection, std::move(path), topology);
        validateShortestPath(routed, topology);
        if (routed.reachable && static_cast<int>(routed.physical_link_ids.size()) > topology.maximumHop()) routed.reachable = false;
        if (routed.reachable) for (int link : routed.physical_link_ids) loads.at(link) += connection.demand;
        result.routed[position] = std::move(routed);
    }
    result.metrics = MetricsCalculator::calculate(result.routed, topology);
    return result;
}

}  // namespace route_prediction_b3
