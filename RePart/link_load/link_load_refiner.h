#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../datastructure/hypergraph.h"

namespace link_load_v1 {

struct Options {
    bool enabled = false;
    double thd_degradation_limit = 0.02;
    double hot_link_threshold = 0.8;
    int max_iters = 20;
    double runtime_limit_seconds = 300.0;
    std::string report_path;
    std::string capacity_file;
};

struct PhysicalLink {
    int id = -1;
    int u = -1;
    int v = -1;
    double capacity = -1.0;
};

struct PredictedPath {
    int source_fpga = -1;
    int sink_fpga = -1;
    std::vector<int> physical_link_ids;
    int hop_count = -1;
};

struct PredictedDemand {
    int hyperedge_id = -1;
    int source_node = -1;
    int source_fpga = -1;
    int sink_fpga = -1;
    long long weight = 0;
    std::vector<int> sink_nodes;
    std::vector<int> physical_link_ids;
};

struct LinkStat {
    long long load = 0;
    int demand_count = 0;
};

struct EdgeContribution {
    long long thd = 0;
    long long cut_size = 0;
    int max_hop = 0;
    int unreachable_demands = 0;
    long long total_routed_demand_weight = 0;
    std::vector<long long> communication_usage;
    std::vector<LinkStat> links;
};

struct Metrics {
    long long thd = 0;
    long long cut_size = 0;
    int max_hop = 0;
    long long max_link_load = 0;
    long double squared_load_cost = 0.0L;
    double average_link_load = 0.0;
    double load_stddev = 0.0;
    int zero_load_links = 0;
    int unreachable_demands = 0;
    long long total_routed_demand_weight = 0;
    int resource_violations = 0;
    int communication_violations = 0;
    std::vector<std::vector<long long>> resource_usage;
    std::vector<long long> communication_usage;
    std::vector<LinkStat> links;
    std::vector<PredictedDemand> demands;
};

struct RejectCounters {
    long long same_fpga = 0;
    long long frozen = 0;
    long long resource = 0;
    long long maximum_hop = 0;
    long long communication = 0;
    long long thd = 0;
    long long unreachable = 0;
    long long not_improving = 0;
};

struct Result {
    Metrics before;
    Metrics after;
    int accepted_moves = 0;
    int completed_passes = 0;
    int generated_candidates = 0;
    int evaluated_moves = 0;
    RejectCounters rejected;
    double runtime_seconds = 0.0;
    std::string stop_reason;
};

class Model {
public:
    explicit Model(const FPGAManager& manager) : manager_(manager) {
        buildLinks();
        buildPaths();
    }

    const std::vector<PhysicalLink>& links() const { return links_; }
    const std::vector<std::vector<PredictedPath>>& paths() const { return paths_; }

    bool loadCapacityFile(const std::string& path, std::string& error) {
        if (path.empty()) return true;
        std::ifstream input(path);
        if (!input) {
            error = "cannot open link capacity file: " + path;
            return false;
        }
        std::map<std::pair<int, int>, double> parsed;
        std::string a, b;
        double capacity = 0.0;
        while (input >> a >> b >> capacity) {
            auto ia = manager_.name_to_id.find(a);
            auto ib = manager_.name_to_id.find(b);
            if (ia == manager_.name_to_id.end() || ib == manager_.name_to_id.end() || capacity < 0.0) {
                error = "invalid link capacity entry: " + a + " " + b;
                return false;
            }
            int u = std::min(ia->second, ib->second);
            int v = std::max(ia->second, ib->second);
            parsed[{u, v}] = capacity;
        }
        for (auto& link : links_) {
            auto it = parsed.find({link.u, link.v});
            if (it != parsed.end()) link.capacity = it->second;
        }
        return true;
    }

    Metrics evaluate(const Hypergraph& hg) const {
        Metrics m;
        const int n = manager_.fpga_num;
        m.resource_usage.assign(n, std::vector<long long>(8, 0));
        m.communication_usage.assign(n, 0);
        m.links.assign(links_.size(), LinkStat{});

        for (const auto& node : hg.hypernodes) {
            if (node.deleted || node.label < 0 || node.label >= n) continue;
            for (int r = 0; r < 8; ++r) m.resource_usage[node.label][r] += node.weights[r];
            for (int replica : node.replication_fpga_labels) {
                if (replica < 0 || replica >= n) continue;
                for (int r = 0; r < 8; ++r) m.resource_usage[replica][r] += node.weights[r];
            }
        }
        for (int f = 0; f < n; ++f) {
            for (int r = 0; r < 8; ++r) {
                if (m.resource_usage[f][r] > manager_.fpga_info[f][r + 1]) ++m.resource_violations;
            }
        }

        for (const auto& edge : hg.hyperedges) {
            if (edge.deleted || edge._size <= 1) continue;
            const int source_node = hg._incidence_array[edge._begin];
            const auto& source = hg.hypernodes[source_node];
            const int source_fpga = source.label;
            std::map<int, std::vector<int>> sinks_by_fpga;
            for (int p = edge._begin + 1; p < edge._begin + edge._size; ++p) {
                const int sink_id = hg._incidence_array[p];
                const auto& sink = hg.hypernodes[sink_id];
                sinks_by_fpga[sink.label].push_back(sink_id);
                for (int replica : sink.replication_fpga_labels) sinks_by_fpga[replica].push_back(sink_id);
            }
            for (int replica : source.replication_fpga_labels) sinks_by_fpga.erase(replica);
            sinks_by_fpga.erase(source_fpga);

            for (const auto& entry : sinks_by_fpga) {
                const int sink_fpga = entry.first;
                PredictedDemand demand;
                demand.hyperedge_id = edge.id;
                demand.source_node = source_node;
                demand.source_fpga = source_fpga;
                demand.sink_fpga = sink_fpga;
                demand.weight = edge.weight;
                demand.sink_nodes = entry.second;
                std::sort(demand.sink_nodes.begin(), demand.sink_nodes.end());
                demand.sink_nodes.erase(std::unique(demand.sink_nodes.begin(), demand.sink_nodes.end()), demand.sink_nodes.end());

                ++m.cut_size;
                if (source_fpga < 0 || source_fpga >= n || sink_fpga < 0 || sink_fpga >= n ||
                    paths_[source_fpga][sink_fpga].hop_count < 0) {
                    ++m.unreachable_demands;
                    m.demands.push_back(std::move(demand));
                    continue;
                }
                const auto& path = paths_[source_fpga][sink_fpga];
                demand.physical_link_ids = path.physical_link_ids;
                m.max_hop = std::max(m.max_hop, path.hop_count);
                m.thd += static_cast<long long>(edge.weight) * path.hop_count;
                m.total_routed_demand_weight += edge.weight;
                m.communication_usage[source_fpga] += edge.weight;
                m.communication_usage[sink_fpga] += edge.weight;
                for (int link_id : path.physical_link_ids) {
                    m.links[link_id].load += edge.weight;
                    ++m.links[link_id].demand_count;
                }
                m.demands.push_back(std::move(demand));
            }
        }

        for (int f = 0; f < n; ++f) {
            if (m.communication_usage[f] > manager_.fpga_info[f][0]) ++m.communication_violations;
        }
        long double sum = 0.0L;
        for (const auto& stat : m.links) {
            m.max_link_load = std::max(m.max_link_load, stat.load);
            m.squared_load_cost += static_cast<long double>(stat.load) * stat.load;
            sum += stat.load;
            if (stat.load == 0) ++m.zero_load_links;
        }
        if (!m.links.empty()) {
            m.average_link_load = static_cast<double>(sum / m.links.size());
            long double variance = 0.0L;
            for (const auto& stat : m.links) {
                long double d = stat.load - m.average_link_load;
                variance += d * d;
            }
            m.load_stddev = std::sqrt(static_cast<double>(variance / m.links.size()));
        }
        return m;
    }

    EdgeContribution evaluateEdge(const Hypergraph& hg, int edge_id,
                                  int override_node = -1, int override_fpga = -1) const {
        EdgeContribution c;
        const int n = manager_.fpga_num;
        c.communication_usage.assign(n, 0);
        c.links.assign(links_.size(), LinkStat{});
        const auto& edge = hg.hyperedges[edge_id];
        if (edge.deleted || edge._size <= 1) return c;
        auto label = [&](int node_id) {
            return node_id == override_node ? override_fpga : hg.hypernodes[node_id].label;
        };
        const int source_node = hg._incidence_array[edge._begin];
        const auto& source = hg.hypernodes[source_node];
        const int source_fpga = label(source_node);
        std::set<int> sink_fpgas;
        for (int p = edge._begin + 1; p < edge._begin + edge._size; ++p) {
            const int sink_id = hg._incidence_array[p];
            sink_fpgas.insert(label(sink_id));
            sink_fpgas.insert(hg.hypernodes[sink_id].replication_fpga_labels.begin(),
                              hg.hypernodes[sink_id].replication_fpga_labels.end());
        }
        for (int replica : source.replication_fpga_labels) sink_fpgas.erase(replica);
        sink_fpgas.erase(source_fpga);
        for (int sink_fpga : sink_fpgas) {
            ++c.cut_size;
            if (source_fpga < 0 || source_fpga >= n || sink_fpga < 0 || sink_fpga >= n ||
                paths_[source_fpga][sink_fpga].hop_count < 0) {
                ++c.unreachable_demands;
                continue;
            }
            const auto& path = paths_[source_fpga][sink_fpga];
            c.max_hop = std::max(c.max_hop, path.hop_count);
            c.thd += static_cast<long long>(edge.weight) * path.hop_count;
            c.total_routed_demand_weight += edge.weight;
            c.communication_usage[source_fpga] += edge.weight;
            c.communication_usage[sink_fpga] += edge.weight;
            for (int link_id : path.physical_link_ids) {
                c.links[link_id].load += edge.weight;
                ++c.links[link_id].demand_count;
            }
        }
        return c;
    }

private:
    const FPGAManager& manager_;
    std::vector<PhysicalLink> links_;
    std::vector<std::vector<int>> link_id_;
    std::vector<std::vector<PredictedPath>> paths_;

    void buildLinks() {
        const int n = manager_.fpga_num;
        link_id_.assign(n, std::vector<int>(n, -1));
        for (int u = 0; u < n; ++u) {
            for (int v = u + 1; v < n; ++v) {
                if (!manager_.adj_matrix[u][v]) continue;
                int id = static_cast<int>(links_.size());
                links_.push_back(PhysicalLink{id, u, v, -1.0});
                link_id_[u][v] = link_id_[v][u] = id;
            }
        }
    }

    void buildPaths() {
        const int n = manager_.fpga_num;
        paths_.assign(n, std::vector<PredictedPath>(n));
        for (int source = 0; source < n; ++source) {
            std::vector<int> distance(n, -1), parent(n, -1);
            std::queue<int> q;
            distance[source] = 0;
            q.push(source);
            while (!q.empty()) {
                int u = q.front(); q.pop();
                for (int v = 0; v < n; ++v) {
                    if (!manager_.adj_matrix[u][v] || distance[v] != -1) continue;
                    distance[v] = distance[u] + 1;
                    parent[v] = u;
                    q.push(v);
                }
            }
            for (int sink = 0; sink < n; ++sink) {
                PredictedPath path;
                path.source_fpga = source;
                path.sink_fpga = sink;
                path.hop_count = distance[sink];
                if (distance[sink] >= 0) {
                    std::vector<int> reversed;
                    for (int v = sink; v != source; v = parent[v]) reversed.push_back(link_id_[parent[v]][v]);
                    path.physical_link_ids.assign(reversed.rbegin(), reversed.rend());
                }
                paths_[source][sink] = std::move(path);
            }
        }
    }
};

inline bool lexicographicallyBetter(const Metrics& a, const Metrics& b) {
    constexpr double eps = 1e-9;
    if (a.max_link_load != b.max_link_load) return a.max_link_load < b.max_link_load;
    if (a.squared_load_cost != b.squared_load_cost) return a.squared_load_cost < b.squared_load_cost;
    if (std::fabs(a.load_stddev - b.load_stddev) > eps) return a.load_stddev < b.load_stddev;
    if (a.thd != b.thd) return a.thd < b.thd;
    return a.cut_size < b.cut_size;
}

class Refiner {
public:
    enum class FeasibilityFailure { None, Resource, Unreachable, MaximumHop, Communication, Thd };

    Refiner(Hypergraph& hg, const Options& options, long long repart_thd0 = -1)
        : hg_(hg), options_(options), model_(hg.fpga_manager), repart_thd0_(repart_thd0) {}

    static FeasibilityFailure feasibilityFailure(const Metrics& trial, const Metrics& baseline,
                                                  int maximum_hop, long long thd_limit) {
        if (trial.resource_violations) return FeasibilityFailure::Resource;
        if (trial.unreachable_demands > baseline.unreachable_demands) return FeasibilityFailure::Unreachable;
        if (trial.max_hop > maximum_hop) return FeasibilityFailure::MaximumHop;
        if (trial.communication_violations) return FeasibilityFailure::Communication;
        if (trial.thd > thd_limit) return FeasibilityFailure::Thd;
        return FeasibilityFailure::None;
    }

    Result run() {
        Result result;
        const auto started = std::chrono::steady_clock::now();
        std::string capacity_error;
        if (!model_.loadCapacityFile(options_.capacity_file, capacity_error)) {
            std::cerr << "Link capacity file warning: " << capacity_error << " (ignored by V1 objective)\n";
        }
        result.before = model_.evaluate(hg_);
        const long long thd_offset = repart_thd0_ >= 0 ? repart_thd0_ - result.before.thd : 0;
        result.before.thd += thd_offset;
        Metrics current = result.before;
        const long long thd_limit = static_cast<long long>(std::floor(result.before.thd * (1.0 + options_.thd_degradation_limit) + 1e-9));
        const int accepted_move_limit = 2 * static_cast<int>(hg_.hypernodes.size());
        int passes_without_max_load_improvement = 0;
        rebuildEdgeCache();
        if (!validateIncrementalEvaluator(current, thd_offset, thd_limit, 100)) {
            result.after = result.before;
            result.stop_reason = "incremental/full consistency validation failed";
            result.runtime_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            writeReport(result, thd_limit);
            return result;
        }

        for (int pass = 1; pass <= options_.max_iters; ++pass) {
            const auto pass_started = std::chrono::steady_clock::now();
            const RejectCounters reject_before = result.rejected;
            double filter_seconds = 0.0, path_seconds = 0.0, metric_seconds = 0.0;
            long long pass_total = 0, pass_incremental = 0, pass_feasible = 0, pass_improving = 0;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= options_.runtime_limit_seconds) {
                result.stop_reason = "runtime limit";
                break;
            }
            auto candidates = candidateNodes(current);
            result.generated_candidates += static_cast<int>(candidates.size());
            if (candidates.empty()) {
                result.stop_reason = "no movable nodes on hot-link demands";
                break;
            }

            int best_node = -1, best_target = -1, best_source = -1;
            Metrics best_metrics;
            bool pass_interrupted = false;
            for (int node_id : candidates) {
                if (!safeToMove(node_id)) { ++result.rejected.frozen; continue; }
                const int from = hg_.hypernodes[node_id].label;
                for (int target : targetFpgas(node_id, current)) {
                    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= options_.runtime_limit_seconds) {
                        result.stop_reason = "runtime limit during move evaluation";
                        pass_interrupted = true;
                        break;
                    }
                    ++pass_total;
                    ++result.evaluated_moves;
                    Metrics trial;
                    FeasibilityFailure failure = incrementalTrial(node_id, target, current, thd_limit, trial,
                                                                   filter_seconds, path_seconds, metric_seconds);
                    if (failure != FeasibilityFailure::Resource) ++pass_incremental;
                    switch (failure) {
                        case FeasibilityFailure::Resource: ++result.rejected.resource; continue;
                        case FeasibilityFailure::Unreachable: ++result.rejected.unreachable; continue;
                        case FeasibilityFailure::MaximumHop: ++result.rejected.maximum_hop; continue;
                        case FeasibilityFailure::Communication: ++result.rejected.communication; continue;
                        case FeasibilityFailure::Thd: ++result.rejected.thd; continue;
                        case FeasibilityFailure::None: break;
                    }
                    ++pass_feasible;
                    if (!lexicographicallyBetter(trial, current)) { ++result.rejected.not_improving; continue; }
                    ++pass_improving;
                    if (best_node < 0 || lexicographicallyBetter(trial, best_metrics) ||
                        (!lexicographicallyBetter(best_metrics, trial) && std::pair<int,int>{node_id,target} < std::pair<int,int>{best_node,best_target})) {
                        best_node = node_id; best_target = target; best_source = from; best_metrics = std::move(trial);
                    }
                }
                if (!result.stop_reason.empty() && result.stop_reason.find("runtime limit") == 0) break;
            }
            if (pass_interrupted) {
                const double pass_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - pass_started).count();
                std::cout << "LinkLoadV1 pass_summary pass=" << pass << " candidates=" << pass_total
                          << " resource_reject=" << result.rejected.resource-reject_before.resource
                          << " communication_reject=" << result.rejected.communication-reject_before.communication
                          << " unreachable_reject=" << result.rejected.unreachable-reject_before.unreachable
                          << " maxhop_reject=" << result.rejected.maximum_hop-reject_before.maximum_hop
                          << " thd_reject=" << result.rejected.thd-reject_before.thd
                          << " not_improving=" << result.rejected.not_improving-reject_before.not_improving
                          << " prefilter_survivors=" << pass_incremental << " feasible=" << pass_feasible
                          << " improving=" << pass_improving << " filter_s=" << filter_seconds
                          << " path_s=" << path_seconds << " metric_s=" << metric_seconds
                          << " total_s=" << pass_seconds << " completed_full_pass=false\n";
                break;
            }
            ++result.completed_passes;
            const double pass_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - pass_started).count();
            std::cout << "LinkLoadV1 pass_summary pass=" << pass
                      << " candidates=" << pass_total << " prefilter_survivors=" << pass_incremental
                      << " resource_reject=" << result.rejected.resource-reject_before.resource
                      << " communication_reject=" << result.rejected.communication-reject_before.communication
                      << " unreachable_reject=" << result.rejected.unreachable-reject_before.unreachable
                      << " maxhop_reject=" << result.rejected.maximum_hop-reject_before.maximum_hop
                      << " thd_reject=" << result.rejected.thd-reject_before.thd
                      << " not_improving=" << result.rejected.not_improving-reject_before.not_improving
                      << " feasible=" << pass_feasible << " improving=" << pass_improving
                      << " filter_s=" << filter_seconds << " path_s=" << path_seconds
                      << " metric_s=" << metric_seconds << " total_s=" << pass_seconds
                      << " completed_full_pass=true\n";
            if (best_node < 0) {
                result.stop_reason = "no improving feasible move in a complete pass";
                break;
            }
            const bool max_load_improved = best_metrics.max_link_load < current.max_link_load;
            std::cout << "LinkLoadV1 pass=" << pass << " node=" << best_node
                      << " from=" << hg_.fpga_manager.getName(best_source)
                      << " to=" << hg_.fpga_manager.getName(best_target)
                      << " MaxLinkLoad=" << current.max_link_load << "->" << best_metrics.max_link_load
                      << " SquaredLoadCost=" << static_cast<double>(current.squared_load_cost) << "->" << static_cast<double>(best_metrics.squared_load_cost)
                      << " THD=" << current.thd << "->" << best_metrics.thd << "\n";
            hg_.hypernodes[best_node].label = best_target;
            current = model_.evaluate(hg_);
            current.thd += thd_offset;
            rebuildEdgeCache();
            ++result.accepted_moves;
            if (max_load_improved) passes_without_max_load_improvement = 0;
            else ++passes_without_max_load_improvement;
            if (passes_without_max_load_improvement >= 3) {
                result.stop_reason = "MaxLinkLoad unchanged for 3 consecutive passes";
                break;
            }
            if (result.accepted_moves >= accepted_move_limit) {
                result.stop_reason = "accepted move limit (2 * number_of_nodes)";
                break;
            }
            if (pass == options_.max_iters) result.stop_reason = "maximum refinement passes";
        }
        result.after = model_.evaluate(hg_);
        result.after.thd += thd_offset;
        result.runtime_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        if (result.stop_reason.empty()) result.stop_reason = "completed";
        writeReport(result, thd_limit);
        return result;
    }

private:
    Hypergraph& hg_;
    Options options_;
    Model model_;
    long long repart_thd0_;
    std::vector<EdgeContribution> edge_cache_;

    void rebuildEdgeCache() {
        edge_cache_.resize(hg_.hyperedges.size());
        for (size_t i = 0; i < hg_.hyperedges.size(); ++i) edge_cache_[i] = model_.evaluateEdge(hg_, static_cast<int>(i));
    }

    std::vector<int> incidentEdges(int node_id) const {
        std::set<int> ids(hg_.hypernodes[node_id].incident_nets_as_source.begin(),
                          hg_.hypernodes[node_id].incident_nets_as_source.end());
        ids.insert(hg_.hypernodes[node_id].incident_nets_as_target.begin(),
                   hg_.hypernodes[node_id].incident_nets_as_target.end());
        return std::vector<int>(ids.begin(), ids.end());
    }

    FeasibilityFailure incrementalTrial(int node_id, int target, const Metrics& current,
                                        long long thd_limit, Metrics& trial,
                                        double& filter_seconds, double& path_seconds, double& metric_seconds,
                                        bool force_complete = false) const {
        const auto filter_start = std::chrono::steady_clock::now();
        trial = current;
        const auto& node = hg_.hypernodes[node_id];
        const int from = node.label;
        if (from == target) { filter_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-filter_start).count(); return FeasibilityFailure::Resource; }
        int resource_violation_count = 0;
        for (int r = 0; r < 8; ++r) {
            trial.resource_usage[from][r] -= node.weights[r];
            trial.resource_usage[target][r] += node.weights[r];
            if (trial.resource_usage[target][r] > hg_.fpga_manager.fpga_info[target][r+1]) ++resource_violation_count;
        }
        filter_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-filter_start).count();
        if (resource_violation_count && !force_complete) return FeasibilityFailure::Resource;

        const auto path_start = std::chrono::steady_clock::now();
        std::map<int, EdgeContribution> replacements;
        for (int edge_id : incidentEdges(node_id)) {
            const auto& old_c = edge_cache_[edge_id];
            EdgeContribution new_c = model_.evaluateEdge(hg_, edge_id, node_id, target);
            replacements.emplace(edge_id, new_c);
            trial.thd += new_c.thd - old_c.thd;
            trial.cut_size += new_c.cut_size - old_c.cut_size;
            trial.unreachable_demands += new_c.unreachable_demands - old_c.unreachable_demands;
            trial.total_routed_demand_weight += new_c.total_routed_demand_weight - old_c.total_routed_demand_weight;
            for (size_t f = 0; f < trial.communication_usage.size(); ++f)
                trial.communication_usage[f] += new_c.communication_usage[f] - old_c.communication_usage[f];
            for (size_t l = 0; l < trial.links.size(); ++l) {
                trial.links[l].load += new_c.links[l].load - old_c.links[l].load;
                trial.links[l].demand_count += new_c.links[l].demand_count - old_c.links[l].demand_count;
            }
        }
        trial.max_hop = 0;
        for (size_t edge_id = 0; edge_id < edge_cache_.size(); ++edge_id) {
            auto replacement = replacements.find(static_cast<int>(edge_id));
            trial.max_hop = std::max(trial.max_hop,
                replacement == replacements.end() ? edge_cache_[edge_id].max_hop : replacement->second.max_hop);
        }
        path_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-path_start).count();

        const auto metric_start = std::chrono::steady_clock::now();
        trial.resource_violations = resource_violation_count;
        trial.communication_violations = 0;
        for (int f = 0; f < hg_.fpga_manager.fpga_num; ++f)
            if (trial.communication_usage[f] > hg_.fpga_manager.fpga_info[f][0]) ++trial.communication_violations;
        trial.max_link_load = 0; trial.squared_load_cost = 0; trial.zero_load_links = 0;
        long double sum = 0;
        for (const auto& link : trial.links) {
            trial.max_link_load = std::max(trial.max_link_load, link.load);
            trial.squared_load_cost += static_cast<long double>(link.load) * link.load;
            sum += link.load; if (link.load == 0) ++trial.zero_load_links;
        }
        trial.average_link_load = trial.links.empty() ? 0.0 : static_cast<double>(sum / trial.links.size());
        long double variance = 0;
        for (const auto& link : trial.links) { long double d = link.load - trial.average_link_load; variance += d*d; }
        trial.load_stddev = trial.links.empty() ? 0.0 : std::sqrt(static_cast<double>(variance / trial.links.size()));
        metric_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-metric_start).count();
        return feasibilityFailure(trial, current, hg_.fpga_manager.max_hop_distance, thd_limit);
    }

    bool validateIncrementalEvaluator(const Metrics& current, long long thd_offset,
                                      long long thd_limit, int sample_count) {
        if (hg_.hypernodes.empty() || hg_.fpga_manager.fpga_num < 2) return true;
        std::mt19937 rng(0x5EED1234u);
        std::uniform_int_distribution<int> node_dist(0, static_cast<int>(hg_.hypernodes.size()) - 1);
        std::uniform_int_distribution<int> fpga_dist(0, hg_.fpga_manager.fpga_num - 1);
        int checked = 0, attempts = 0;
        while (checked < sample_count && attempts++ < sample_count * 20) {
            int node_id = node_dist(rng), target = fpga_dist(rng);
            if (hg_.hypernodes[node_id].deleted || target == hg_.hypernodes[node_id].label) continue;
            Metrics incremental;
            double a = 0, b = 0, c = 0;
            FeasibilityFailure incremental_failure = incrementalTrial(node_id, target, current, thd_limit,
                                                                       incremental, a, b, c, true);
            int from = hg_.hypernodes[node_id].label;
            hg_.hypernodes[node_id].label = target;
            Metrics reference = model_.evaluate(hg_);
            reference.thd += thd_offset;
            hg_.hypernodes[node_id].label = from;
            FeasibilityFailure reference_failure = feasibilityFailure(reference, current,
                                                                       hg_.fpga_manager.max_hop_distance, thd_limit);
            auto close = [](double x, double y) { return std::fabs(x-y) <= 1e-8 * std::max(1.0, std::max(std::fabs(x), std::fabs(y))); };
            bool same = incremental.thd == reference.thd && incremental.cut_size == reference.cut_size &&
                        incremental.max_link_load == reference.max_link_load &&
                        incremental.squared_load_cost == reference.squared_load_cost &&
                        close(incremental.load_stddev, reference.load_stddev) &&
                        incremental.resource_usage == reference.resource_usage &&
                        incremental.communication_usage == reference.communication_usage &&
                        incremental.resource_violations == reference.resource_violations &&
                        incremental.communication_violations == reference.communication_violations &&
                        incremental.unreachable_demands == reference.unreachable_demands &&
                        incremental_failure == reference_failure && incremental.links.size() == reference.links.size();
            if (same) {
                for (size_t l = 0; l < incremental.links.size(); ++l) {
                    if (incremental.links[l].load != reference.links[l].load ||
                        incremental.links[l].demand_count != reference.links[l].demand_count) { same = false; break; }
                }
            }
            if (!same) {
                std::cerr << "Incremental validation mismatch node=" << node_id
                          << " from=" << hg_.fpga_manager.getName(from)
                          << " to=" << hg_.fpga_manager.getName(target) << " incident_nets=";
                for (int edge : incidentEdges(node_id)) std::cerr << edge << ",";
                std::cerr << " incremental(THD=" << incremental.thd << ",cut=" << incremental.cut_size
                          << ",maxload=" << incremental.max_link_load << ") reference(THD=" << reference.thd
                          << ",cut=" << reference.cut_size << ",maxload=" << reference.max_link_load << ")\n";
                return false;
            }
            ++checked;
        }
        std::cout << "LinkLoadV1 incremental_reference_validation checked=" << checked << " result=passed\n";
        return checked == sample_count;
    }

    bool safeToMove(int node_id) const {
        const auto& node = hg_.hypernodes[node_id];
        if (node.deleted || !node.replication_fpga_labels.empty()) return false;
        // A source is conservatively frozen whenever any node on one of its nets has
        // a replica. Moving it could change which physical copy drives a sink.
        for (int edge_id : node.incident_nets_as_source) {
            const auto& edge = hg_.hyperedges[edge_id];
            for (int p = edge._begin; p < edge._begin + edge._size; ++p) {
                if (!hg_.hypernodes[hg_._incidence_array[p]].replication_fpga_labels.empty()) return false;
            }
        }
        return true;
    }

    std::vector<int> candidateNodes(const Metrics& metrics) const {
        std::set<int> result;
        if (metrics.max_link_load == 0) return {};
        const double cutoff = options_.hot_link_threshold * metrics.max_link_load;
        std::vector<char> hot(metrics.links.size(), 0);
        for (size_t i = 0; i < metrics.links.size(); ++i) if (metrics.links[i].load + 1e-9 >= cutoff) hot[i] = 1;
        for (const auto& demand : metrics.demands) {
            bool touches = false;
            for (int link : demand.physical_link_ids) if (hot[link]) { touches = true; break; }
            if (!touches) continue;
            result.insert(demand.source_node);
            result.insert(demand.sink_nodes.begin(), demand.sink_nodes.end());
        }
        return std::vector<int>(result.begin(), result.end());
    }

    std::vector<int> targetFpgas(int node_id, const Metrics& metrics) const {
        std::set<int> targets;
        const int current = hg_.hypernodes[node_id].label;
        auto addWithNeighbors = [&](int fpga) {
            if (fpga < 0 || fpga >= hg_.fpga_manager.fpga_num) return;
            targets.insert(fpga);
            for (int n = 0; n < hg_.fpga_manager.fpga_num; ++n) if (hg_.fpga_manager.adj_matrix[fpga][n]) targets.insert(n);
        };
        addWithNeighbors(current);
        const auto& node = hg_.hypernodes[node_id];
        std::set<int> edges(node.incident_nets_as_source.begin(), node.incident_nets_as_source.end());
        edges.insert(node.incident_nets_as_target.begin(), node.incident_nets_as_target.end());
        for (int edge_id : edges) {
            const auto& edge = hg_.hyperedges[edge_id];
            for (int p = edge._begin; p < edge._begin + edge._size; ++p) addWithNeighbors(hg_.hypernodes[hg_._incidence_array[p]].label);
            for (const auto& demand : metrics.demands) {
                if (demand.hyperedge_id == edge_id) { targets.insert(demand.source_fpga); targets.insert(demand.sink_fpga); }
            }
        }
        targets.erase(current);
        return std::vector<int>(targets.begin(), targets.end());
    }

    void writeReport(const Result& result, long long thd_limit) const {
        if (options_.report_path.empty()) return;
        std::ofstream out(options_.report_path);
        if (!out) { std::cerr << "Cannot write link-load report: " << options_.report_path << "\n"; return; }
        out << std::fixed << std::setprecision(6);
        out << "RePart Version 1 capacity-independent physical-link load report\n";
        out << "Model: one weighted demand per (net, distinct remote sink FPGA), routed on a deterministic shortest path.\n";
        out << "Shared edges of different sink-FPGA paths are counted independently; this is not a Steiner routing tree.\n";
        out << "Physical links are undirected. No TDM, utilization, or real congestion is claimed without hardware link capacities.\n";
        out << "capacity_file=" << (options_.capacity_file.empty() ? "none" : options_.capacity_file) << " (report-only; never used by move selection)\n";
        out << "thd_limit=" << thd_limit << " accepted_moves=" << result.accepted_moves
            << " completed_passes=" << result.completed_passes
            << " candidates=" << result.generated_candidates << " evaluated_moves=" << result.evaluated_moves
            << " runtime_seconds=" << result.runtime_seconds << " stop_reason=" << result.stop_reason << "\n";
        out << "reject_frozen=" << result.rejected.frozen << " reject_resource=" << result.rejected.resource
            << " reject_maximum_hop=" << result.rejected.maximum_hop << " reject_communication=" << result.rejected.communication
            << " reject_thd=" << result.rejected.thd << " reject_unreachable=" << result.rejected.unreachable
            << " reject_not_improving=" << result.rejected.not_improving << "\n\n";
        writeMetrics(out, "before", result.before);
        writeMetrics(out, "after", result.after);
    }

    void writeMetrics(std::ofstream& out, const std::string& name, const Metrics& m) const {
        out << "[" << name << "]\n";
        out << "THD=" << m.thd << "\ncut_size=" << m.cut_size << "\nmax_hop=" << m.max_hop
            << "\nMaxLinkLoad=" << m.max_link_load << "\nSquaredLoadCost=" << static_cast<double>(m.squared_load_cost)
            << "\naverage_link_load=" << m.average_link_load << "\nlink_load_stddev=" << m.load_stddev
            << "\nzero_load_links=" << m.zero_load_links << "\nunreachable_demands=" << m.unreachable_demands
            << "\ntotal_routed_demand_weight=" << m.total_routed_demand_weight
            << "\nresource_violations=" << m.resource_violations << "\ncommunication_violations=" << m.communication_violations << "\n";
        std::vector<int> order(m.links.size()); std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            if (m.links[a].load != m.links[b].load) return m.links[a].load > m.links[b].load;
            return a < b;
        });
        out << "top_links: id endpoints load demands percent_of_max\n";
        for (size_t i = 0; i < std::min<size_t>(10, order.size()); ++i) {
            int id = order[i]; const auto& link = model_.links()[id];
            double pct = m.max_link_load ? 100.0 * m.links[id].load / m.max_link_load : 0.0;
            out << id << " " << hg_.fpga_manager.getName(link.u) << "--" << hg_.fpga_manager.getName(link.v)
                << " " << m.links[id].load << " " << m.links[id].demand_count << " " << pct << "%\n";
        }
        out << "\n";
    }
};

} // namespace link_load_v1
