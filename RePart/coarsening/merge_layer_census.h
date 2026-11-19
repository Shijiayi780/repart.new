#pragma once

// Read-only census of every native Case05 coarsening-batch entry.
#include "merge_opportunity_probe.h"

namespace merge_census {
inline int batch=0;

inline void observe(const Hypergraph& graph,const FPGAManager& manager,
                    const PrioQueue& heap,const std::vector<int>& targets) {
    const char* path=std::getenv("MERGE_CENSUS_DIR");
    if(!path) throw std::runtime_error("MERGE_CENSUS_DIR required");
    const std::string base=std::string(path)+"/batch_"+std::to_string(batch);
    std::ofstream states(base+".tsv"), candidates(base+"_candidates.tsv"), metadata(base+"_metadata.tsv");
    if(!states||!candidates||!metadata) throw std::runtime_error("Cannot open layer census outputs");
    states<<std::setprecision(17);candidates<<std::setprecision(17);metadata<<std::setprecision(17);
    states<<"PARAM\t"<<penaltyPower<<'\t'<<edgesizeThresholdScale<<'\t'<<edgeSizeThreshold(graph)
          <<'\t'<<graph._num_vertices_current<<'\n';
    states<<"AVERAGES";
    for(auto value:manager.resource_averages) states<<'\t'<<value;
    states<<'\n';
    long long domain=0,legal=0,resource_rejected=0;
    for(const auto& node:graph.hypernodes) if(!node.deleted) {
        states<<"NODE\t"<<node.id;
        for(int w:node.weights) states<<'\t'<<w;
        states<<"\tS";
        for(int he:node.incident_nets_as_source) states<<','<<he;
        states<<"\tT";
        for(int he:node.incident_nets_as_target) states<<','<<he;
        states<<'\n';
        for(const auto& pair:merge_probe::contributions(graph,node.id)) {
            ++domain;
            if(!belowMinWeightFPGA(graph,node.id,pair.first,manager)) ++resource_rejected;
            else if(pair.second/heavyNodePenalty(graph,node.id,pair.first,manager)>0) ++legal;
        }
    }
    for(const auto& edge:graph.hyperedges) if(!edge.deleted) {
        states<<"NET\t"<<edge.id<<'\t'<<edge.weight;
        for(int j=edge._begin;j<edge._begin+edge._size;++j) states<<'\t'<<graph._incidence_array[j];
        states<<'\n';
    }
    PrioQueue shadow(graph.hypernodes.size());
    for(int slot=0;slot<4;++slot)
        for(int u=slot;u<static_cast<int>(graph.hypernodes.size());u+=4)
            if(heap.contains(u)) shadow.push(u,heap.getKey(u));
    if(shadow.empty()||shadow.top()!=heap.top()||shadow.topKey()!=heap.topKey())
        throw std::runtime_error("Census native heap replay mismatch");
    std::vector<merge_probe::Action> ordered;
    candidates<<"rank\tu\tv\tkey\n";
    while(!shadow.empty()) {
        int u=shadow.top();
        ordered.push_back({u,targets[u],static_cast<int>(ordered.size())+1,shadow.topKey()});
        candidates<<ordered.back().rank<<'\t'<<u<<'\t'<<targets[u]<<'\t'<<shadow.topKey()<<'\n';
        shadow.remove(u);
    }
    const auto g=ordered.front();
    merge_probe::Action c,i;
    for(const auto& a:ordered) {
        const bool same=(a.u==g.u&&a.v==g.v)||(a.u==g.v&&a.v==g.u);
        const bool shares=a.u==g.u||a.u==g.v||a.v==g.u||a.v==g.v;
        if(c.u<0&&!same&&shares) c=a;
        if(i.u<0&&!shares) i=a;
    }
    metadata<<"batch\t"<<batch<<"\nactive_nodes\t"<<graph._num_vertices_current
            <<"\npositive_candidate_domain\t"<<domain<<"\nlegal_candidates\t"<<legal
            <<"\nresource_rejected\t"<<resource_rejected<<'\n';
    for(const auto& item:std::vector<std::pair<char,merge_probe::Action>>{{'G',g},{'C',c},{'I',i}})
        metadata<<item.first<<'\t'<<item.second.u<<'\t'<<item.second.v<<'\t'<<item.second.rank<<'\t'<<item.second.key<<'\n';
    if(c.u>=0) {
        std::set<int> nodes{g.u,g.v,c.u,c.v};
        bool fits=true;
        metadata<<"G_C_union_resources";
        for(int d=0;d<8;++d) {
            long long demand=0;
            for(int u:nodes) demand+=graph.hypernodes[u].weights[d];
            metadata<<'\t'<<demand;
            if(demand>manager.resource_averages[d]) fits=false;
        }
        metadata<<"\nG_C_blocked_if_G_first\t"<<!fits<<'\n';
    }
    ++batch;
}
} // namespace merge_census
