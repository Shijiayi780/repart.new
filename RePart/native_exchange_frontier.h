#pragma once
#include "datastructure/hypergraph.h"
#include "refinement_snapshot.h"

namespace native_exchange_frontier {
template<class Heap>
void observe(const Hypergraph& g,const std::vector<std::vector<int>>& usage,
             const std::vector<int>& comm,const Heap& heap,const std::vector<int>& targets) {
    static bool saved=false;
    if(saved) return;
    saved=true;
    const char* dir=std::getenv("NATIVE_EXCHANGE_FRONTIER_DIR");
    if(!dir) throw std::runtime_error("NATIVE_EXCHANGE_FRONTIER_DIR required");
    long long cost=0;
    for(const auto& e:g.hyperedges) if(!e.deleted)
        for(int f=0;f<g.fpga_manager.fpga_num;++f)
            if(e.target_pins_in_part[f]>0)
                cost+=static_cast<long long>(e.weight)*g.fpga_manager.hop_distances[e.source_part_id][f];
    const auto before=refinement_snapshot::encode(g,cost,usage,comm);
    std::ofstream state(std::string(dir)+"/state.txt"),frontier(std::string(dir)+"/exchange.tsv");
    state<<before;
    frontier<<"u\tv\tnative_gain\n";
    for(const auto& n:g.hypernodes) if(!n.deleted&&heap.contains(n.id))
        frontier<<n.id<<'\t'<<targets[n.id]<<'\t'<<heap.getKey(n.id)<<'\n';
    if(!state||!frontier) throw std::runtime_error("Cannot save native exchange frontier");
    if(before!=refinement_snapshot::encode(g,cost,usage,comm))
        throw std::runtime_error("Exchange frontier observation modified state");
}
}
