#pragma once
#define REPART_UNIFIED_ADAPTER_NO_MAIN
#include "../tools/unified_native_heap_adapter.cpp"
#include <functional>

namespace unified_native_sequence {
using Used=std::array<std::set<int>,4>;
struct Search {
    int visits=0,segments=0; long long start_cost=0,best_cost=0;
    State best;Used best_used;std::vector<Action> path,best_path,retained;
    bool allowed(const Action& a,const Used& used) {
        return !used[a.kind].count(a.u) && (a.kind!=1 || !used[1].count(a.target));
    }
    void dfs(State& s,Used used,int depth) {
        if(visits>=10000)return;
        ++visits;
        auto cost=refinement_native_probe::cost(s.g);
        if(cost<best_cost) {best_cost=cost;best=s;best_used=used;best_path=path;}
        if(depth==3)return;
        NativeHeaps heaps(s);auto candidates=heaps.build();
        std::sort(candidates.begin(),candidates.end(),[](const Action&a,const Action&b){
            return std::make_tuple(-a.gain,a.kind,a.u,a.target)<std::make_tuple(-b.gain,b.kind,b.u,b.target);
        });
        const auto frozen=placementState(s);
        for(auto a:candidates) {
            if(visits>=10000)break;
            if(!allowed(a,used))continue;
            State child=s;NativeHeaps executor(child);
            if(!executor.apply(a))continue;
            auto after=refinement_native_probe::cost(child.g);
            if(cost-after!=a.gain)throw std::runtime_error("Sequence native gain differs from full WTHD");
            Used next=used;next[a.kind].insert(a.u);if(a.kind==1)next[1].insert(a.target);
            path.push_back(a);dfs(child,next,depth+1);path.pop_back();
            if(placementState(s)!=frozen)throw std::runtime_error("DFS branch leaked state");
        }
    }
    void run(State& s) {
        start_cost=refinement_native_probe::cost(s.g);Used used;
        while(visits<10000) {
            ++segments;auto anchor=refinement_native_probe::cost(s.g);
            best_cost=anchor;best=s;best_used=used;best_path.clear();path.clear();
            dfs(s,used,0);
            if(best_cost>=anchor)break;
            s=best;used=best_used;retained.insert(retained.end(),best_path.begin(),best_path.end());
        }
    }
};
inline void refine(Hypergraph& g,std::vector<std::vector<int>>& usage,std::vector<int>& comm,long long& cost) {
    struct Scope {Scope(){unified_sequence_active=true;}~Scope(){unified_sequence_active=false;}} scope;
    State s{g,usage,comm};Search search;auto start=std::chrono::steady_clock::now();search.run(s);
    g=std::move(s.g);usage=std::move(s.usage);comm=std::move(s.comm);cost=refinement_native_probe::cost(g);
    std::cout<<"UNIFIED_SEQUENCE before="<<search.start_cost<<" after="<<cost<<" gain="<<search.start_cost-cost
        <<" visits="<<search.visits<<" steps="<<search.retained.size()<<" segments="<<search.segments
        <<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<std::endl;
    for(auto a:search.retained)std::cout<<"UNIFIED_ACTION "<<a.kind<<' '<<a.u<<' '<<a.target<<' '<<a.gain<<'\n';
}
}
