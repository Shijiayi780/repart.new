#include "connection_projector.h"

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace route_prediction_v1 {
namespace {

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

}  // namespace

CaseData ConnectionProjector::load(const std::string& case_directory,
                                   const std::string& partition_path,
                                   const FpgaTopology& topology) {
    CaseData data;
    std::ifstream area(case_directory + "/design.are");
    if (!area) throw std::runtime_error("cannot open design.are");
    std::string line;
    while (std::getline(area, line)) {
        if (line.empty()) continue;
        std::istringstream fields(line);
        std::string name;
        if (!(fields >> name)) continue;
        const int id = static_cast<int>(data.node_names.size());
        if (!data.node_ids.emplace(name, id).second) throw std::runtime_error("duplicate node in design.are: " + name);
        data.node_names.push_back(name);
    }
    data.placements.resize(data.node_names.size());

    std::ifstream net_file(case_directory + "/design.net");
    if (!net_file) throw std::runtime_error("cannot open design.net");
    while (std::getline(net_file, line)) {
        if (line.empty()) break;
        std::istringstream fields(line);
        std::string source_name;
        std::int64_t weight = 0;
        if (!(fields >> source_name >> weight)) throw std::runtime_error("invalid design.net line");
        if (!data.node_ids.count(source_name)) throw std::runtime_error("unknown source node: " + source_name);
        Net net;
        net.id = static_cast<int>(data.nets.size());
        net.source_node = data.node_ids.at(source_name);
        net.weight = weight;
        std::string sink_name;
        while (fields >> sink_name) {
            if (!data.node_ids.count(sink_name)) throw std::runtime_error("unknown sink node: " + sink_name);
            net.sink_nodes.push_back(data.node_ids.at(sink_name));
        }
        data.nets.push_back(std::move(net));
    }

    std::ifstream partition(partition_path);
    if (!partition) throw std::runtime_error("cannot open partition: " + partition_path);
    std::vector<bool> fpga_lines(topology.fpgaCount(), false);
    while (std::getline(partition, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) throw std::runtime_error("invalid partition line");
        const int fpga = topology.id(trim(line.substr(0, colon)));
        if (fpga_lines[fpga]) throw std::runtime_error("duplicate FPGA partition line");
        fpga_lines[fpga] = true;
        std::istringstream fields(line.substr(colon + 1));
        std::string token;
        while (fields >> token) {
            const bool replica = token.back() == '*';
            if (replica) token.pop_back();
            if (!data.node_ids.count(token)) throw std::runtime_error("unknown partition node: " + token);
            auto& placement = data.placements[data.node_ids.at(token)];
            if (replica) {
                if (std::find(placement.replica_fpgas.begin(), placement.replica_fpgas.end(), fpga) != placement.replica_fpgas.end()) {
                    throw std::runtime_error("duplicate replica placement: " + token);
                }
                placement.replica_fpgas.push_back(fpga);
            } else {
                if (placement.master_fpga != -1) throw std::runtime_error("multiple masters: " + token);
                placement.master_fpga = fpga;
            }
        }
    }
    for (auto& placement : data.placements) std::sort(placement.replica_fpgas.begin(), placement.replica_fpgas.end());
    validatePlacements(data.placements, topology.fpgaCount());
    return data;
}

PlacementStats ConnectionProjector::validatePlacements(const std::vector<NodePlacement>& placements,
                                                        int fpga_count) {
    PlacementStats stats;
    for (const auto& placement : placements) {
        if (placement.master_fpga < 0 || placement.master_fpga >= fpga_count) throw std::runtime_error("node lacks a legal master");
        ++stats.master_nodes;
        if (!placement.replica_fpgas.empty()) ++stats.replicated_nodes;
        int previous = -1;
        for (int replica : placement.replica_fpgas) {
            if (replica < 0 || replica >= fpga_count) throw std::runtime_error("illegal replica FPGA");
            if (replica == placement.master_fpga) throw std::runtime_error("master/replica overlap");
            if (replica == previous) throw std::runtime_error("duplicate replica FPGA");
            previous = replica;
            ++stats.replica_locations;
        }
    }
    return stats;
}

std::vector<Connection> ConnectionProjector::projectV2(const std::vector<Net>& nets,
                                                       const std::vector<NodePlacement>& placements) {
    std::vector<Connection> connections;
    for (const auto& net : nets) {
        const auto& source = placements.at(net.source_node);
        std::set<int> destinations;
        for (int sink_node : net.sink_nodes) {
            const auto& sink = placements.at(sink_node);
            destinations.insert(sink.master_fpga);
            destinations.insert(sink.replica_fpgas.begin(), sink.replica_fpgas.end());
        }
        destinations.erase(source.master_fpga);
        for (int replica : source.replica_fpgas) destinations.erase(replica);
        for (int destination : destinations) {
            connections.push_back({static_cast<int>(connections.size()), net.id, net.source_node,
                                   source.master_fpga, destination, 1, net.weight});
        }
    }
    return connections;
}

}  // namespace route_prediction_v1
