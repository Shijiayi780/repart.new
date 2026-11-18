#include "fpga_topology.h"

#include <algorithm>
#include <fstream>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>

namespace route_prediction_v1 {
namespace {

std::uint64_t edgeKey(int a, int b) {
    const auto lo = static_cast<std::uint32_t>(std::min(a, b));
    const auto hi = static_cast<std::uint32_t>(std::max(a, b));
    return (static_cast<std::uint64_t>(lo) << 32) | hi;
}

}  // namespace

void FpgaTopology::load(const std::string& info_path, const std::string& topo_path) {
    std::ifstream info(info_path);
    if (!info) throw std::runtime_error("cannot open design.info: " + info_path);
    std::vector<std::string> names;
    std::string line;
    while (std::getline(info, line)) {
        if (line.empty()) continue;
        std::istringstream fields(line);
        std::string fpga_name;
        if (!(fields >> fpga_name)) continue;
        names.push_back(fpga_name);
    }
    if (names.empty()) throw std::runtime_error("design.info contains no FPGAs");

    std::unordered_map<std::string, int> ids;
    for (int i = 0; i < static_cast<int>(names.size()); ++i) {
        if (!ids.emplace(names[i], i).second) throw std::runtime_error("duplicate FPGA in design.info: " + names[i]);
    }

    std::ifstream topo(topo_path);
    if (!topo) throw std::runtime_error("cannot open design.topo: " + topo_path);
    if (!std::getline(topo, line)) throw std::runtime_error("empty design.topo");
    const int maximum_hop = std::stoi(line);
    std::vector<std::pair<int, int>> edges;
    while (std::getline(topo, line)) {
        if (line.empty()) continue;
        std::istringstream fields(line);
        std::string a, b;
        if (!(fields >> a >> b)) throw std::runtime_error("invalid design.topo edge: " + line);
        if (!ids.count(a) || !ids.count(b)) throw std::runtime_error("unknown FPGA in design.topo: " + line);
        edges.emplace_back(ids.at(a), ids.at(b));
    }
    initialize(names, edges, maximum_hop);
}

void FpgaTopology::initialize(const std::vector<std::string>& fpga_names,
                              const std::vector<std::pair<int, int>>& undirected_edges,
                              int maximum_hop) {
    names_ = fpga_names;
    maximum_hop_ = maximum_hop;
    name_to_id_.clear();
    links_.clear();
    input_physical_links_.clear();
    link_ids_.clear();
    adjacency_.assign(names_.size(), {});
    for (int i = 0; i < static_cast<int>(names_.size()); ++i) {
        if (!name_to_id_.emplace(names_[i], i).second) throw std::runtime_error("duplicate FPGA name");
    }
    std::set<std::pair<int, int>> unique_edges;
    for (auto [a, b] : undirected_edges) {
        if (a < 0 || b < 0 || a >= fpgaCount() || b >= fpgaCount()) throw std::runtime_error("FPGA edge out of range");
        if (a == b) throw std::runtime_error("self-loop is not supported");
        input_physical_links_.push_back(
            {static_cast<int>(input_physical_links_.size()), a, b});
        if (a > b) std::swap(a, b);
        unique_edges.emplace(a, b);
    }
    for (const auto& [u, v] : unique_edges) {
        const int link_id = static_cast<int>(links_.size());
        links_.push_back({link_id, u, v});
        link_ids_[edgeKey(u, v)] = link_id;
        adjacency_[u].push_back(v);
        adjacency_[v].push_back(u);
    }
    for (auto& neighbors : adjacency_) std::sort(neighbors.begin(), neighbors.end());
    precomputePaths();
}

void FpgaTopology::precomputePaths() {
    const int n = fpgaCount();
    hops_.assign(n, std::vector<int>(n, -1));
    paths_.assign(n, std::vector<std::vector<int>>(n));
    for (int source = 0; source < n; ++source) {
        std::vector<int> parent(n, -1);
        std::queue<int> queue;
        hops_[source][source] = 0;
        queue.push(source);
        while (!queue.empty()) {
            const int current = queue.front();
            queue.pop();
            for (int next : adjacency_[current]) {
                if (hops_[source][next] != -1) continue;
                hops_[source][next] = hops_[source][current] + 1;
                parent[next] = current;
                queue.push(next);
            }
        }
        for (int destination = 0; destination < n; ++destination) {
            if (hops_[source][destination] < 0) continue;
            std::vector<int> reverse_path;
            for (int current = destination; current != -1; current = parent[current]) {
                reverse_path.push_back(current);
                if (current == source) break;
            }
            paths_[source][destination].assign(reverse_path.rbegin(), reverse_path.rend());
            validatePath(source, destination, paths_[source][destination]);
        }
    }
}

const std::string& FpgaTopology::name(int fpga) const {
    if (fpga < 0 || fpga >= fpgaCount()) throw std::runtime_error("FPGA id out of range");
    return names_[fpga];
}

int FpgaTopology::id(const std::string& fpga_name) const {
    const auto it = name_to_id_.find(fpga_name);
    if (it == name_to_id_.end()) throw std::runtime_error("unknown FPGA: " + fpga_name);
    return it->second;
}

const std::vector<int>& FpgaTopology::adjacency(int fpga) const {
    return adjacency_.at(fpga);
}

int FpgaTopology::hop(int source, int destination) const {
    return hops_.at(source).at(destination);
}

const std::vector<int>& FpgaTopology::path(int source, int destination) const {
    return paths_.at(source).at(destination);
}

int FpgaTopology::linkId(int u, int v) const {
    const auto it = link_ids_.find(edgeKey(u, v));
    if (it == link_ids_.end()) return -1;
    return it->second;
}

bool FpgaTopology::connected() const {
    for (int i = 0; i < fpgaCount(); ++i) {
        for (int j = 0; j < fpgaCount(); ++j) if (hops_[i][j] < 0) return false;
    }
    return true;
}

void FpgaTopology::validatePath(int source, int destination, const std::vector<int>& fpga_path) const {
    if (fpga_path.empty() || fpga_path.front() != source || fpga_path.back() != destination) {
        throw std::runtime_error("path endpoint validation failed");
    }
    for (std::size_t i = 1; i < fpga_path.size(); ++i) {
        if (linkId(fpga_path[i - 1], fpga_path[i]) < 0) throw std::runtime_error("path uses a non-physical edge");
    }
}

}  // namespace route_prediction_v1
