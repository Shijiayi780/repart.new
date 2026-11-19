#pragma once
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Observation only: no algorithm state is modified, and no search is performed.
namespace refinement_snapshot {
inline std::string encode(const Hypergraph& g, long long cost,
                          const std::vector<std::vector<int>>& usage,
                          const std::vector<int>& comm) {
    std::ostringstream o;
    const auto& f=g.fpga_manager;
    o<<"META "<<cost<<' '<<f.fpga_num<<' '<<f.max_hop_distance<<'\n';
    for(int i=0;i<f.fpga_num;++i) {
        o<<"BOARD "<<i;
        for(auto x:f.fpga_info[i]) o<<' '<<x;
        o<<"\nHOP "<<i;
        for(auto x:f.hop_distances[i]) o<<' '<<x;
        o<<"\nUSAGE "<<i;
        for(auto x:usage[i]) o<<' '<<x;
        o<<"\nCOMM "<<i<<' '<<comm[i]<<'\n';
    }
    for(const auto& n:g.hypernodes) if(!n.deleted) {
        o<<"NODE "<<n.id<<' '<<n.label;
        for(auto x:n.weights) o<<' '<<x;
        o<<' '<<n.replication_fpga_labels.size();
        for(auto x:n.replication_fpga_labels) o<<' '<<x;
        o<<'\n';
    }
    for(const auto& e:g.hyperedges) if(!e.deleted) {
        o<<"NET "<<e.id<<' '<<e.weight<<' '<<e.source_part_id;
        for(int j=e._begin;j<e._begin+e._size;++j) o<<' '<<g._incidence_array[j];
        o<<'\n';
    }
    return o.str();
}
inline void observe(const Hypergraph& g,int layer,long long cost,
                    const std::vector<std::vector<int>>& usage,
                    const std::vector<int>& comm) {
    static int serial=0;
    const char* directory=std::getenv("REFINEMENT_SNAPSHOT_DIR");
    if(!directory) throw std::runtime_error("REFINEMENT_SNAPSHOT_DIR required");
    const auto before=encode(g,cost,usage,comm);
    const int current_serial=serial++;
    std::ofstream out(std::string(directory)+"/snapshot_"+std::to_string(current_serial)+
                      "_layer_"+std::to_string(layer)+".txt");
    out<<before;
    out.close();
#ifdef REPART_NATIVE_MOVE_PROBE
    refinement_native_probe::observe(g,current_serial,layer,cost,usage,comm,directory);
#endif
    if(!out || before!=encode(g,cost,usage,comm))
        throw std::runtime_error("Snapshot write or observed-state equality failed");
}
}
