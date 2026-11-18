#pragma once
#include "global_assignment_score.h"
#include <map>
#include <set>

namespace global_assignment_score {
struct ScheduledBoard {
    int fpga;
    double connection, heat, resource_penalty;
};
// All inputs must already satisfy resource, hard Hop and communication limits.
inline std::vector<Candidate> scheduled_board_order(
        int node, int placed, int total, const std::vector<ScheduledBoard>& boards) {
    const double t = double(placed)/std::max(1,total-1);
    if (placed<0 || total<1 || placed>=total) throw std::invalid_argument("placement depth");
    double max_connection=0, max_heat=0;
    for (const auto& b:boards) {
        max_connection=std::max(max_connection,b.connection);
        max_heat=std::max(max_heat,b.heat);
    }
    std::vector<Candidate> candidates;
    for (const auto& b:boards) {
        const double connection = max_connection>0 ? b.connection/max_connection : 0;
        const double heat = max_heat>0 ? b.heat/max_heat : 0;
        candidates.push_back({node,b.fpga,(.2+.6*t)*connection+(.8-.6*t)*heat-b.resource_penalty});
    }
    std::sort(candidates.begin(),candidates.end(),precedes);
    return candidates;
}

// Only used to form the legal normalization domain. Native execution checks
// remain in place. Per Net, each remote target board and the source count once.
template<class Nodes,class Edges>
bool communication_allows(const Nodes& nodes,const Edges& edges,const std::vector<int>& pins,
        const std::vector<std::vector<int>>& info,const std::vector<int>& usage,
        const std::map<int,int>& assignment,int node,int fpga) {
    std::set<int> affected(nodes[node].incident_nets_as_source.begin(),nodes[node].incident_nets_as_source.end());
    affected.insert(nodes[node].incident_nets_as_target.begin(),nodes[node].incident_nets_as_target.end());
    std::map<int,std::int64_t> delta;
    for (int e:affected) {
        const auto& edge=edges[e]; if(edge.deleted) continue;
        auto participants=[&](bool after) {
            auto location=[&](int v) {
                if(after && v==node) return fpga;
                auto it=assignment.find(v); return it==assignment.end() ? -1 : it->second;
            };
            const int source=location(pins[edge._begin]);
            std::set<int> result;
            if(source<0) return result;
            for(int j=1;j<edge._size;++j) {
                int target=location(pins[edge._begin+j]);
                if(target>=0 && target!=source) result.insert(target);
            }
            if(!result.empty()) result.insert(source);
            return result;
        };
        for(int f:participants(false)) delta[f]-=edge.weight;
        for(int f:participants(true)) delta[f]+=edge.weight;
    }
    for(auto [f,change]:delta)
        if(std::int64_t(usage[f])+change>info[f][0]) return false;
    return true;
}
} // namespace global_assignment_score
