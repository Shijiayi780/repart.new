#pragma once
// Experimental, opt-in replacement of the first (replica-free) Exchange stage.
#include "exchanger.h"
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <stdexcept>

namespace exchange_action_pairs {
using Cost=std::int64_t;
struct Action {int u=-1,v=-1;};
struct Candidate {
    Action a,b;int length=1;Cost gain=0,g1=0,g2=0;std::uint64_t serial=0;
};
struct LowerPriority {
    bool operator()(const Candidate&a,const Candidate&b)const {
        return a.gain!=b.gain?a.gain<b.gain:a.serial>b.serial;
    }
};
struct Snapshot {
    std::vector<int> labels,sources;
    std::vector<std::vector<int>> pins,usage;
    std::vector<int> comm;
    bool operator==(const Snapshot&o)const{return labels==o.labels&&sources==o.sources&&pins==o.pins&&usage==o.usage&&comm==o.comm;}
};
class Search {
    Hypergraph& g;
    std::vector<std::vector<int>>& usage;
    std::vector<int>& comm;
    Exchanger executor;
    std::vector<int> nodes,edges;
    std::string prefix;
public:
    explicit Search(Hypergraph& graph,std::vector<std::vector<int>>&r,std::vector<int>&c,std::string p="")
        :g(graph),usage(r),comm(c),executor(g,usage,comm),prefix(std::move(p)) {
        for(const auto&n:g.hypernodes)if(!n.deleted){
            if(!n.replication_fpga_labels.empty())throw std::logic_error("EXPAIR_REQUIRES_NO_REPLICAS");
            nodes.push_back(n.id);
        }
        for(const auto&e:g.hyperedges)if(!e.deleted)edges.push_back(e.id);
    }
    Snapshot save()const {
        Snapshot s;s.usage=usage;s.comm=comm;
        for(int n:nodes)s.labels.push_back(g.hypernodes[n].label);
        for(int e:edges){s.sources.push_back(g.hyperedges[e].source_part_id);s.pins.push_back(g.hyperedges[e].target_pins_in_part);}
        return s;
    }
    void restore(const Snapshot&s){
        usage=s.usage;comm=s.comm;
        for(std::size_t i=0;i<nodes.size();++i)g.hypernodes[nodes[i]].label=s.labels[i];
        for(std::size_t i=0;i<edges.size();++i){g.hyperedges[edges[i]].source_part_id=s.sources[i];g.hyperedges[edges[i]].target_pins_in_part=s.pins[i];}
    }
    Cost cost()const {
        Cost c=0;
        for(int id:edges){const auto&e=g.hyperedges[id];
            for(int f=0;f<g.fpga_manager.fpga_num;++f)if(e.target_pins_in_part[f]>0){
                int h=g.fpga_manager.getHopDistance(e.source_part_id,f);
                if(h<0)throw std::logic_error("EXPAIR_INVALID_HOP_IN_COST");
                c+=static_cast<Cost>(e.weight)*h;
            }
        }
        return c;
    }
    // On rejection this may have changed temporary state; caller always restores
    // its checkpoint before the next candidate. No partial candidate is committed.
    bool apply(Action a){
        if(a.u==a.v||g.hypernodes[a.u].label==g.hypernodes[a.v].label)return false;
        if(!executor.exchangeIsFeasible(a.u,a.v))return false;
        std::vector<int> delta(g.fpga_manager.fpga_num,0);
        if(!executor.checkAndChangeComm(a.u,a.v,delta))return false;
        executor.exchangeHypernode(a.u,a.v);
        for(int id:edges){const auto&e=g.hyperedges[id];
            for(int f=0;f<g.fpga_manager.fpga_num;++f)if(e.target_pins_in_part[f]>0)
                if(g.fpga_manager.getHopDistance(e.source_part_id,f)<0)return false;
        }
        return true;
    }
    std::vector<Action> domain()const {
        std::vector<Action> a;
        for(std::size_t i=0;i<nodes.size();++i)for(std::size_t j=i+1;j<nodes.size();++j)
            if(g.hypernodes[nodes[i]].label!=g.hypernodes[nodes[j]].label)a.push_back({nodes[i],nodes[j]});
        return a;
    }
    void dump(const std::string& path)const {
        std::ofstream f(path);if(!f)throw std::runtime_error("EXPAIR_SNAPSHOT_OPEN");
        f<<"{\"nodes\":[";
        bool first=true;
        for(int id:nodes){if(!first)f<<',';first=false;const auto&n=g.hypernodes[id];
            f<<"{\"id\":"<<id<<",\"board\":"<<n.label<<",\"resources\":[";
            for(int d=0;d<8;++d){if(d)f<<',';f<<n.weights[d];}f<<"]}";
        }
        f<<"],\"nets\":[";first=true;
        for(int id:edges){if(!first)f<<',';first=false;const auto&e=g.hyperedges[id];
            f<<"{\"id\":"<<id<<",\"weight\":"<<e.weight<<",\"pins\":[";
            for(int p=e._begin;p<e._begin+e._size;++p){if(p!=e._begin)f<<',';f<<g._incidence_array[p];}f<<"]}";
        }
        f<<"],\"fpga_info\":[";
        for(int b=0;b<g.fpga_manager.fpga_num;++b){if(b)f<<',';f<<'[';
            for(int d=0;d<9;++d){if(d)f<<',';f<<g.fpga_manager.fpga_info[b][d];}f<<']';}
        f<<"],\"distances\":[";
        for(int a=0;a<g.fpga_manager.fpga_num;++a){if(a)f<<',';f<<'[';
            for(int b=0;b<g.fpga_manager.fpga_num;++b){if(b)f<<',';f<<g.fpga_manager.getHopDistance(a,b);}f<<']';}
        f<<"],\"usage\":[";
        for(std::size_t b=0;b<usage.size();++b){if(b)f<<',';f<<'[';for(int d=0;d<8;++d){if(d)f<<',';f<<usage[b][d];}f<<']';}
        f<<"],\"comm\":[";for(std::size_t b=0;b<comm.size();++b){if(b)f<<',';f<<comm[b];}f<<"],\"cost\":"<<cost()<<'}';
    }
    Cost run(){
        const Cost entry=cost();int round=0;
        // Compare the native stage from exactly this initial state, then restore.
        const auto original=save();const int native_gain=executor.exchange();const Cost native_after=cost();
        if(entry-native_after!=native_gain)throw std::logic_error("EXPAIR_NATIVE_GAIN_MISMATCH");
        restore(original);if(!(save()==original))throw std::logic_error("EXPAIR_NATIVE_RESTORE");
        std::cout<<"EXPAIR_BASELINE before="<<entry<<" gain="<<native_gain<<" after="<<native_after<<std::endl;
        while(true){
            const auto base=save();const Cost before=cost();std::uint64_t serial=0;
            std::priority_queue<Candidate,std::vector<Candidate>,LowerPriority> heap;
            if(!prefix.empty())dump(prefix+"_round"+std::to_string(round)+".json");
            std::ofstream records;
            if(!prefix.empty()){
                records.open(prefix+"_round"+std::to_string(round)+"_candidates.tsv");
                if(!records)throw std::runtime_error("EXPAIR_CANDIDATE_OPEN");
                records<<"serial\tlength\tu\tv\tx\ty\tg1\tg2\tgain\n";
            }
            const auto push=[&](Candidate c){
                if(c.gain<=0)return;
                c.serial=serial++;heap.push(c);
                if(records)records<<c.serial<<'\t'<<c.length<<'\t'<<c.a.u<<'\t'<<c.a.v<<'\t'<<c.b.u<<'\t'<<c.b.v<<'\t'<<c.g1<<'\t'<<c.g2<<'\t'<<c.gain<<'\n';
            };
            std::size_t legal_first=0,legal_pairs=0,negative_first_positive=0,shared_positive=0;
            Cost best_single=0,best_pair=0;
            for(Action a:domain()){
                restore(base);
                const int expected_first=executor.ExchangeGain(a.u,a.v);
                if(!apply(a))continue;
                ++legal_first;const Cost c1=cost(),g1=before-c1;
                if(g1!=expected_first)throw std::logic_error("EXPAIR_FIRST_GAIN_MISMATCH");
                best_single=std::max(best_single,g1);push({a,{},1,g1,g1,0,0});
                const auto first_state=save();const auto seconds=domain();
                for(Action b:seconds){
                    restore(first_state);
                    const int expected_second=executor.ExchangeGain(b.u,b.v);
                    if(!apply(b))continue;
                    ++legal_pairs;const Cost g2=c1-cost(),gain=g1+g2;
                    if(g2!=expected_second||gain!=before-cost())throw std::logic_error("EXPAIR_SECOND_GAIN_MISMATCH");
                    best_pair=std::max(best_pair,gain);
                    if(gain>0){negative_first_positive+=g1<0;shared_positive+=a.u==b.u||a.u==b.v||a.v==b.u||a.v==b.v;}
                    push({a,b,2,gain,g1,g2,0});
                }
            }
            restore(base);if(!(save()==base)||cost()!=before)throw std::logic_error("EXPAIR_BRANCH_LEAK");
            std::cout<<"EXPAIR_ROUND round="<<round<<" before="<<before<<" legal_first="<<legal_first<<" legal_pairs="<<legal_pairs
                <<" positive_candidates="<<heap.size()<<" best_single="<<best_single<<" best_pair="<<best_pair
                <<" negative_first_positive="<<negative_first_positive<<" shared_positive="<<shared_positive<<std::endl;
            if(heap.empty())break;
            const Candidate selected=heap.top();
            if(!apply(selected.a)||(selected.length==2&&!apply(selected.b)))throw std::logic_error("EXPAIR_COMMIT_ILLEGAL");
            if(before-cost()!=selected.gain)throw std::logic_error("EXPAIR_COMMIT_GAIN");
            std::cout<<"EXPAIR_COMMIT round="<<round<<" serial="<<selected.serial<<" length="<<selected.length<<" u="<<selected.a.u<<" v="<<selected.a.v
                <<" x="<<selected.b.u<<" y="<<selected.b.v<<" g1="<<selected.g1<<" g2="<<selected.g2<<" gain="<<selected.gain<<" after="<<cost()<<std::endl;
            ++round;
        }
        std::cout<<"EXPAIR_DONE before="<<entry<<" after="<<cost()<<" gain="<<entry-cost()<<" native_after="<<native_after<<" rounds="<<round<<std::endl;
        return entry-cost();
    }
};
inline Cost refine(Hypergraph&g,std::vector<std::vector<int>>&usage,std::vector<int>&comm){
    static int calls=0;++calls;std::string prefix;
    if(const char*dir=std::getenv("REPART_EXPAIR_DIR"))prefix=std::string(dir)+"/call"+std::to_string(calls);
    Search search(g,usage,comm,prefix);return search.run();
}
}
