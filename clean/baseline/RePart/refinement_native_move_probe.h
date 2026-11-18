#pragma once
#include <cstdint>
#include <fstream>
#include <set>
#include <stdexcept>

namespace refinement_native_probe {
// Physical net cost from node placements, not gain/target-count caches.
inline std::int64_t netCost(const Hypergraph& g,int eid,int moving=-1,int to=-1) {
    const auto& e=g.hyperedges[eid];
    int source=g._incidence_array[e._begin];
    int master=source==moving?to:g.hypernodes[source].label;
    std::set<int> destinations;
    for(int j=e._begin+1;j<e._begin+e._size;++j) {
        int u=g._incidence_array[j];
        destinations.insert(u==moving?to:g.hypernodes[u].label);
        for(int f:g.hypernodes[u].replication_fpga_labels) destinations.insert(f);
    }
    destinations.erase(master);
    for(int f:g.hypernodes[source].replication_fpga_labels) destinations.erase(f);
    std::int64_t cost=0;
    for(int f:destinations) {
        int h=g.fpga_manager.hop_distances[master][f];
        if(h<0) throw std::runtime_error("Independent Move projection has illegal hop");
        cost+=std::int64_t(e.weight)*h;
    }
    return cost;
}
inline std::int64_t cost(const Hypergraph& g) {
    std::int64_t value=0;
    for(const auto& e:g.hyperedges) if(!e.deleted) value+=netCost(g,e.id);
    return value;
}
inline void observe(const Hypergraph& frozen,int serial,int layer,long long reported,
                    const std::vector<std::vector<int>>& original_usage,
                    const std::vector<int>& original_comm,const std::string& directory) {
    Hypergraph g(frozen);
    auto usage=original_usage;
    auto comm=original_comm;
    KWayFMRefiner native(g,usage,comm);
    native.refiner_base.initActivateNodesEdges(native.activated_hyperedges,native.activated_nodes);
    native.computeAllEdgeHopDistanceCost();
    native.rateAllActivatedHypernodes();
    const auto baseline=cost(g);
    if(baseline!=reported) throw std::runtime_error("Native Move probe baseline cost mismatch");
    std::size_t rated=0,legal=0,mismatch=0;
    int best_node=-1,best_target=-1,best_key=0;
    std::int64_t best_gain=0;
    for(const auto& n:g.hypernodes) if(!n.deleted) {
        std::set<int> affected(n.incident_nets_as_source.begin(),n.incident_nets_as_source.end());
        affected.insert(n.incident_nets_as_target.begin(),n.incident_nets_as_target.end());
        for(int f=0;f<g.fpga_manager.fpga_num;++f) if(native.kpq[f].contains(n.id)) {
            ++rated;
            if(!native.refiner_base.moveIsFeasible(n.id,n.label,f)) continue;
            std::vector<int> delta(comm.size(),0);
            native.evaluateFPGACommChange(n.id,n.label,f,delta);
            bool feasible=true;
            for(int b=0;b<g.fpga_manager.fpga_num;++b)
                if(comm[b]+delta[b]>g.fpga_manager.fpga_info[b][0]) feasible=false;
            if(!feasible) continue;
            ++legal;
            std::int64_t gain=0;
            for(int eid:affected) if(!g.hyperedges[eid].deleted)
                gain+=netCost(g,eid)-netCost(g,eid,n.id,f);
            int key=native.kpq[f].getKey(n.id);
            if(gain!=key) ++mismatch;
            if(gain>best_gain) {best_gain=gain;best_node=n.id;best_target=f;best_key=key;}
        }
    }
    std::int64_t after=baseline;
    if(best_node>=0) {
        int from=g.hypernodes[best_node].label;
        std::vector<int> delta(comm.size(),0);
        if(!native.checkAndChangeComm(best_node,from,best_target,delta))
            throw std::runtime_error("Chosen Move failed original communication gate");
        native.moveHypernode(best_node,from,best_target);
        after=cost(g);
        if(baseline-after!=best_gain) throw std::runtime_error("Move delta/full-rebuild mismatch");
    }
    std::ofstream out(directory+"/move_"+std::to_string(serial)+"_layer_"+std::to_string(layer)+".tsv");
    out<<"baseline\trated_positive\tnative_legal\tgain_mismatches\tbest_node\tbest_target\tnative_key\tactual_gain\tafter\n"
       <<baseline<<'\t'<<rated<<'\t'<<legal<<'\t'<<mismatch<<'\t'<<best_node<<'\t'<<best_target<<'\t'<<best_key<<'\t'<<best_gain<<'\t'<<after<<'\n';
    if(!out) throw std::runtime_error("Native Move probe output failed");
}
}
