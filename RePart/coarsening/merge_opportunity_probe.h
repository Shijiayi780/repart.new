#pragma once

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <set>
#include <stdexcept>

// Diagnostic/one-action intervention only, behind REPART_MERGE_OPPORTUNITY.
namespace merge_probe {
struct Action { int u=-1,v=-1,rank=-1; float key=-1; };
inline bool initialized=false, pending=false;
inline int observed=0;
inline int seen_batches=0;
inline std::string mode, directory;
inline std::vector<Action> actions;
inline std::vector<int> representative;
inline std::ofstream events, pairs, backend;
inline double diagnostic_seconds=0;

inline std::unordered_map<int,float> contributions(const Hypergraph& graph,int u) {
    std::unordered_map<int,float> scores;
    auto add=[&](const auto& incident) {
        for(int he:incident) {
            const auto& edge=graph.hyperedges[he];
            if(edge._size>edgeSizeThreshold(graph)) continue;
            const float value=heavyEdgeScore(graph,he);
            for(int j=edge._begin;j<edge._begin+edge._size;++j) {
                const int v=graph._incidence_array[j];
                if(v!=u) scores[v]+=value;
            }
        }
    };
    add(graph.hypernodes[u].incident_nets_as_source);
    add(graph.hypernodes[u].incident_nets_as_target);
    return scores;
}

inline float pair_score(const Hypergraph& graph,int u,int v,const FPGAManager& manager) {
    auto values=contributions(graph,u);
    const auto found=values.find(v);
    if(found==values.end()) return -1;
    if(!belowMinWeightFPGA(graph,u,v,manager)) return -1;
    return found->second/heavyNodePenalty(graph,u,v,manager);
}

inline void snapshot(const Hypergraph& graph,const FPGAManager& manager,int step) {
    const auto start=std::chrono::steady_clock::now();
    std::ofstream dump(directory+"/state_"+std::to_string(step)+".tsv");
    if(!dump) throw std::runtime_error("Cannot open graph diagnostic snapshot");
    dump<<std::setprecision(17)<<"PARAM\t"<<penaltyPower<<'\t'<<edgesizeThresholdScale
        <<'\t'<<edgeSizeThreshold(graph)<<'\t'<<graph._num_vertices_current<<'\n';
    dump<<"AVERAGES";
    for(auto value:manager.resource_averages) dump<<'\t'<<value;
    dump<<'\n';
    long long count=0;
    for(const auto& node:graph.hypernodes) if(!node.deleted) {
        dump<<"NODE\t"<<node.id;
        for(int value:node.weights) dump<<'\t'<<value;
        dump<<"\tS";
        for(int e:node.incident_nets_as_source) dump<<','<<e;
        dump<<"\tT";
        for(int e:node.incident_nets_as_target) dump<<','<<e;
        dump<<'\n';
        for(const auto& candidate:contributions(graph,node.id))
            if(belowMinWeightFPGA(graph,node.id,candidate.first,manager)
               && candidate.second/heavyNodePenalty(graph,node.id,candidate.first,manager)>0) ++count;
    }
    for(const auto& edge:graph.hyperedges) if(!edge.deleted) {
        dump<<"NET\t"<<edge.id<<'\t'<<edge.weight;
        for(int j=edge._begin;j<edge._begin+edge._size;++j) dump<<'\t'<<graph._incidence_array[j];
        dump<<'\n';
    }
    events<<"STATE\t"<<step<<'\t'<<graph._num_vertices_current<<'\t'<<count<<'\n';
    for(std::size_t i=0;i<actions.size();++i) {
        const int u=representative.at(actions[i].u),v=representative.at(actions[i].v);
        std::string status="legal";
        auto scores=contributions(graph,u);
        float rating=0;
        if(u==v) status="internalized";
        else if(scores.find(v)==scores.end()) status="not_in_current_candidate_domain";
        else if(!belowMinWeightFPGA(graph,u,v,manager)) status="resource_infeasible";
        else rating=scores[v]/heavyNodePenalty(graph,u,v,manager);
        pairs<<step<<'\t'<<"GCI"[i]<<'\t'<<u<<'\t'<<v<<'\t'<<status<<'\t'<<rating;
        for(int d=0;d<8;++d) pairs<<'\t'<<(u==v?graph.hypernodes[u].weights[d]:graph.hypernodes[u].weights[d]+graph.hypernodes[v].weights[d]);
        pairs<<'\t';
        bool first=true;
        std::set<int> common;
        if(u!=v) {
            auto visit=[&](const auto& incident) {
                for(int he:incident) {
                    const auto& edge=graph.hyperedges[he];
                    if(edge._size>edgeSizeThreshold(graph)) continue;
                    for(int j=edge._begin;j<edge._begin+edge._size;++j)
                        if(graph._incidence_array[j]==v) common.insert(he);
                }
            };
            visit(graph.hypernodes[u].incident_nets_as_source);
            visit(graph.hypernodes[u].incident_nets_as_target);
        }
        for(int he:common) {
            if(!first) pairs<<',';
            first=false;
            pairs<<he<<':'<<graph.hyperedges[he]._size<<':'<<graph.hyperedges[he].weight;
        }
        pairs<<'\n';
    }
    events.flush();pairs.flush();
    diagnostic_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
}

inline void begin(const Hypergraph& graph,const FPGAManager& manager,
                  const PrioQueue& heap,const std::vector<int>& targets) {
    if(initialized) return;
    const char* requested=std::getenv("MERGE_PROBE_BATCH");
    const int target_batch=requested?std::stoi(requested):0;
    if(target_batch<0) throw std::runtime_error("Negative merge checkpoint batch");
    const int current_batch=seen_batches++;
    if(current_batch<target_batch) return;
    if(current_batch!=target_batch) throw std::runtime_error("Missed merge checkpoint batch");
    initialized=true;
    const char* m=std::getenv("MERGE_PROBE_MODE");
    const char* path=std::getenv("MERGE_PROBE_DIR");
    if(!m||!path) throw std::runtime_error("Merge probe needs MODE and DIR");
    mode=m;directory=path;
    if(mode!="BASE"&&mode!="G"&&mode!="C"&&mode!="I") throw std::runtime_error("Invalid merge mode");
    events.open(directory+"/events.tsv");pairs.open(directory+"/pairs.tsv");backend.open(directory+"/backend.tsv");
    if(!events||!pairs||!backend) throw std::runtime_error("Cannot open merge probe outputs");
    events<<std::setprecision(17);pairs<<std::setprecision(17);backend<<std::setprecision(17);
    events<<"CHECKPOINT_BATCH\t"<<current_batch<<'\n';
    PrioQueue shadow(graph.hypernodes.size());
    for(int slot=0;slot<4;++slot)
        for(int u=slot;u<static_cast<int>(graph.hypernodes.size());u+=4)
            if(heap.contains(u)) shadow.push(u,heap.getKey(u));
    if(shadow.empty()||shadow.top()!=heap.top()||shadow.topKey()!=heap.topKey())
        throw std::runtime_error("Initial heap replay mismatch");
    std::vector<Action> ordered;
    std::ofstream candidates(directory+"/candidates.tsv");
    candidates<<std::setprecision(17)<<"rank\tu\tv\tkey\n";
    while(!shadow.empty()) {
        const int u=shadow.top();
        ordered.push_back({u,targets[u],static_cast<int>(ordered.size())+1,shadow.topKey()});
        candidates<<ordered.back().rank<<'\t'<<u<<'\t'<<targets[u]<<'\t'<<shadow.topKey()<<'\n';
        shadow.remove(u);
    }
    actions.push_back(ordered.front());
    const auto g=actions[0];
    Action c,i;
    for(const auto& a:ordered) {
        const bool same=(a.u==g.u&&a.v==g.v)||(a.u==g.v&&a.v==g.u);
        const bool shares=a.u==g.u||a.u==g.v||a.v==g.u||a.v==g.v;
        if(c.u<0&&!same&&shares) c=a;
        if(i.u<0&&!shares) i=a;
    }
    if(c.u<0||i.u<0) throw std::runtime_error("Missing C/I frozen candidates");
    actions.push_back(c);actions.push_back(i);
    for(std::size_t j=0;j<actions.size();++j)
        events<<"FROZEN\t"<<"GCI"[j]<<'\t'<<actions[j].u<<'\t'<<actions[j].v<<'\t'<<actions[j].rank<<'\t'<<actions[j].key<<'\n';
    representative.resize(graph.hypernodes.size());
    std::iota(representative.begin(),representative.end(),0);
    pending=mode!="BASE";
    snapshot(graph,manager,0);
}

inline int choose(int native) {
    if(!pending) return native;
    return actions.at(mode=="G"?0:mode=="C"?1:2).u;
}

inline void before_contract(const Hypergraph& graph,const FPGAManager& manager,int u,int v,float key) {
    if(!initialized) return;
    if(pending) {
        const auto a=actions.at(mode=="G"?0:mode=="C"?1:2);
        if(u!=a.u||v!=a.v||key!=a.key) throw std::runtime_error("Forced action does not match frozen candidate");
        pending=false;
    }
    if(observed>=5) return;
    events<<"ACTION\t"<<(observed+1)<<'\t'<<u<<'\t'<<v<<'\t'<<key<<'\t'<<pair_score(graph,u,v,manager)<<'\n';
}

inline void after_contract(const Hypergraph& graph,const FPGAManager& manager,int u,int v) {
    if(!initialized) return;
    if(observed>=5) return;
    ++observed;
    for(auto& rep:representative) if(rep==v) rep=u;
    snapshot(graph,manager,observed);
    if(observed==5) events<<"DIAGNOSTIC_SECONDS\t"<<diagnostic_seconds<<'\n';
}

inline void assignment(int iteration,bool success,int layer,long long cost) {
    backend<<"INITIAL\t"<<iteration<<'\t'<<success<<'\t'<<layer<<'\t'<<cost<<'\n';backend.flush();
}
inline void final_cost(int iteration,long long cost) {
    backend<<"FINAL\t"<<iteration<<'\t'<<cost<<'\n';backend.flush();
}
} // namespace merge_probe
