#include "placement_writer.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace route_prediction_b5 {
using namespace route_prediction_v1;

void writePlacement(const std::string& path, const CaseData& data,
                    const FpgaTopology& topology) {
    if (data.node_names.size() != data.placements.size())
        throw std::runtime_error("placement writer dimension mismatch");
    ConnectionProjector::validatePlacements(data.placements, topology.fpgaCount());
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create placement output: " + path);
    for (int fpga = 0; fpga < topology.fpgaCount(); ++fpga) {
        out << topology.name(fpga) << ':';
        for (int node = 0; node < static_cast<int>(data.placements.size()); ++node) {
            const auto& placement = data.placements[node];
            if (placement.master_fpga == fpga) out << ' ' << data.node_names[node];
            for (int replica : placement.replica_fpgas)
                if (replica == fpga) out << ' ' << data.node_names[node] << '*';
        }
        out << '\n';
    }
    out.close();
    if (!out) throw std::runtime_error("failed while writing placement output: " + path);
}

std::vector<NodePlacement> readPlacement(const std::string& path,
                                         const std::vector<std::string>& node_names,
                                         const FpgaTopology& topology) {
    std::unordered_map<std::string,int> ids;
    for (int i=0;i<static_cast<int>(node_names.size());++i)
        if(!ids.emplace(node_names[i],i).second) throw std::runtime_error("duplicate node name");
    std::vector<NodePlacement> placements(node_names.size());
    std::vector<bool> seen(topology.fpgaCount(),false);
    std::ifstream in(path); if(!in) throw std::runtime_error("cannot read placement: "+path);
    std::string line;
    while(std::getline(in,line)){
        const auto colon=line.find(':'); if(colon==std::string::npos) throw std::runtime_error("invalid placement line");
        const int fpga=topology.id(line.substr(0,colon)); if(seen[fpga]) throw std::runtime_error("duplicate FPGA line"); seen[fpga]=true;
        std::istringstream fields(line.substr(colon+1)); std::string token;
        while(fields>>token){const bool replica=token.back()=='*';if(replica)token.pop_back();
            const auto it=ids.find(token);if(it==ids.end())throw std::runtime_error("unknown placement node: "+token);
            auto&p=placements[it->second];if(replica)p.replica_fpgas.push_back(fpga);else{if(p.master_fpga!=-1)throw std::runtime_error("multiple masters");p.master_fpga=fpga;}}
    }
    for(bool value:seen)if(!value)throw std::runtime_error("missing FPGA line");
    for(auto&p:placements)std::sort(p.replica_fpgas.begin(),p.replica_fpgas.end());
    ConnectionProjector::validatePlacements(placements,topology.fpgaCount()); return placements;
}

std::vector<PlacementDiff> comparePlacements(const std::vector<NodePlacement>& before,
                                             const std::vector<NodePlacement>& after) {
    if (before.size() != after.size()) throw std::runtime_error("placement comparison size mismatch");
    std::vector<PlacementDiff> changes;
    for (int node = 0; node < static_cast<int>(before.size()); ++node) {
        if (before[node].replica_fpgas != after[node].replica_fpgas)
            throw std::runtime_error("replica set changed at node " + std::to_string(node));
        if (before[node].master_fpga != after[node].master_fpga)
            changes.push_back({node, before[node].master_fpga, after[node].master_fpga});
    }
    return changes;
}

}  // namespace route_prediction_b5
