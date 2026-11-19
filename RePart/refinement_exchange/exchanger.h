#pragma once
#ifdef REPART_UNIFIED_SEQUENCE
#include "../unified_sequence_switch.h"
#endif

#include <algorithm>
#include <bitset>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>
#include <limits>
#include <queue>

#include "../io/fpga_manager.h"
#include "../datastructure/hypergraph.h"
#ifdef REPART_COMM_PROBE
#include "../refinement_comm_probe.h"
#endif
#include "../datastructure/fast_reset_flag_array.h"
#include "../datastructure/binary_heap.h"
#ifdef REPART_EXCHANGE_FRONTIER
#include "../native_exchange_frontier.h"
#endif

using PrioQueueExchanger = BinaryMaxHeap<int, int>;

struct VertexExchangeRating {
    int target;
    int value;
    bool valid;
    VertexExchangeRating(int t, int v):target(t), value(v), valid(1) {}
    VertexExchangeRating():target(-1), value(-1), valid(0) {}
};

class Exchanger{
    public:
    Hypergraph &hypergraph;
    PrioQueueExchanger pq;
    std::vector<int> target;

    std::vector<std::vector<int>> &fpga_resource_usage;
    std::vector<int> &fpga_comm_weight_sum;

    Exchanger(Hypergraph& hg, 
              std::vector<std::vector<int>>& fpga_resource_usage, 
              std::vector<int>& fpga_comm_weight_sum) : 
        hypergraph(hg),
        pq((hg.hypernodes).size()), 
        target((hg.hypernodes).size()),
        fpga_resource_usage(fpga_resource_usage),
        fpga_comm_weight_sum(fpga_comm_weight_sum) {}
    
    int exchange() {
        int total_gain = 0;
        while(1) {
            int this_gain = 0;
            rateAllHypernodes();
#ifdef REPART_EXCHANGE_FRONTIER
            native_exchange_frontier::observe(hypergraph,fpga_resource_usage,
                fpga_comm_weight_sum,pq,target);
#endif

            while(!pq.empty()) {
                const int max_gain_node = pq.top();
                int from_part = hypergraph.hypernodes[max_gain_node].label;
                int exchanger = target[max_gain_node];
                int to_part = hypergraph.hypernodes[exchanger].label;

                if (exchangeIsFeasible(max_gain_node, exchanger)) {
                    std::vector<int> comm_change(hypergraph.fpga_manager.fpga_num, 0);
#ifdef REPART_COMM_PROBE
                    refinement_comm_probe::ActionProbe comm_probe(
                        hypergraph, fpga_comm_weight_sum, max_gain_node, exchanger);
                    const int comm_probe_gain = pq.topKey();
#endif
                    if (!checkAndChangeComm(max_gain_node, exchanger, comm_change)) {
                        pq.pop();
                        continue;
                    }

#ifdef REPART_COMM_PROBE
                    this_gain += comm_probe_gain;
#else
                    this_gain += pq.topKey();
#endif
                    exchangeHypernode(max_gain_node, exchanger);
#ifdef REPART_COMM_PROBE
                    comm_probe.finish("exchange", from_part, to_part,
                                      comm_probe_gain);
#endif

                    pq.remove(max_gain_node);
                    if (pq.contains(exchanger)) {
                        pq.remove(exchanger);
                    }
                    invalidateRating(max_gain_node, exchanger);
                }
                else {
                    pq.pop();
                }
            }
            total_gain += this_gain;
            if (this_gain == 0) {
                break;
            }
        }
        return total_gain;
    }

    bool checkAndChangeComm(const int u, const int v,
                            std::vector<int>& comm_change) {
        for (int he = 0; he < hypergraph.hyperedges.size(); ++he) {
            if (hypergraph.hyperedges[he].deleted)
                continue;
            Hyperedge& edge = hypergraph.hyperedges[he];
            bool in_edge = 1;
            int source_part = edge.source_part_id;
            for (int p = 0; p < hypergraph.fpga_manager.fpga_num; ++p) {
                if (edge.target_pins_in_part[p] > 0 and p != source_part) {
                    in_edge = 0;
                    comm_change[p] -= edge.weight;
                }
            }
            if (!in_edge) {
                comm_change[source_part] -= edge.weight;
            }
        }

        exchangeHypernode(u, v);

        for (int he = 0; he < hypergraph.hyperedges.size(); ++he) {
            if (hypergraph.hyperedges[he].deleted)
                continue;
            Hyperedge& edge = hypergraph.hyperedges[he];
            bool in_edge = 1;
            int source_part = edge.source_part_id;
            for (int p = 0; p < hypergraph.fpga_manager.fpga_num; ++p) {
                if (edge.target_pins_in_part[p] > 0 and p != source_part) {
                    in_edge = 0;
                    comm_change[p] += edge.weight;
                }
            }
            if (!in_edge) {
                comm_change[source_part] += edge.weight;
            }
        }

        exchangeHypernode(u, v);
        bool valid = 1;
        for (int i = 0; i < hypergraph.fpga_manager.fpga_num; ++i) {
            if (fpga_comm_weight_sum[i] + comm_change[i] > hypergraph.fpga_manager.fpga_info[i][0]) {
                valid = 0;
                break;
            }
        }
        if (valid) {
            for (int i = 0; i < hypergraph.fpga_manager.fpga_num; ++i) {
                fpga_comm_weight_sum[i] += comm_change[i];
            }
        }
        return valid;
    }

    void invalidateRating(const int u, const int v) {
        std::unordered_set<int> affected_hyperedges;
        std::unordered_set<int> affected_hypernodes;
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_source)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_target)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_source)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_target)
            affected_hyperedges.insert(he);

        for (const int he : affected_hyperedges) {
            Hyperedge& edge = hypergraph.hyperedges[he];
            for (int i = edge._begin; i < edge._begin + edge._size; ++i) {
                const int affected_hypernode = hypergraph._incidence_array[i];
                affected_hypernodes.insert(affected_hypernode);
                if (affected_hypernode == u || affected_hypernode == v)
                    continue;
                if (pq.contains(affected_hypernode))
                {
                    pq.remove(affected_hypernode);
                }
            }
        }

        for (int i = 0; i < hypergraph.hypernodes.size(); ++i) {
            if (pq.contains(i) &&
                affected_hypernodes.find(target[i]) != affected_hypernodes.end()) {
                pq.remove(i);
            }
        }

        
    }

    void exchangeHypernode(const int u, const int v) {
#ifdef REPART_UNIFIED_SEQUENCE
        if(unified_sequence_active) {
        int a=hypergraph.hypernodes[u].label,b=hypergraph.hypernodes[v].label;
        hypergraph.hypernodes[u].label=b;hypergraph.hypernodes[v].label=a;
        for(int d=0;d<8;++d) {
            fpga_resource_usage[b][d]+=hypergraph.hypernodes[u].weights[d]-hypergraph.hypernodes[v].weights[d];
            fpga_resource_usage[a][d]+=hypergraph.hypernodes[v].weights[d]-hypergraph.hypernodes[u].weights[d];
        }
        // Reconstruct incident caches with the original replica coverage rule.
        std::unordered_set<int> nets;
        for(int n:{u,v}) {
            nets.insert(hypergraph.hypernodes[n].incident_nets_as_source.begin(),hypergraph.hypernodes[n].incident_nets_as_source.end());
            nets.insert(hypergraph.hypernodes[n].incident_nets_as_target.begin(),hypergraph.hypernodes[n].incident_nets_as_target.end());
        }
        for(int id:nets) {
            auto& e=hypergraph.hyperedges[id];
            const auto& src=hypergraph.hypernodes[hypergraph._incidence_array[e._begin]];
            e.source_part_id=src.label;std::fill(e.target_pins_in_part.begin(),e.target_pins_in_part.end(),0);
            for(int j=e._begin+1;j<e._begin+e._size;++j) {
                const auto& n=hypergraph.hypernodes[hypergraph._incidence_array[j]];
                if(!src.replication_fpga_labels.count(n.label))++e.target_pins_in_part[n.label];
                for(int f:n.replication_fpga_labels)if(!src.replication_fpga_labels.count(f))++e.target_pins_in_part[f];
            }
        }
        return;
        }
#endif
        int from_part = hypergraph.hypernodes[u].label;
        int to_part = hypergraph.hypernodes[v].label;
        hypergraph.hypernodes[u].label = to_part;
        hypergraph.hypernodes[v].label = from_part;

        for (const int he : hypergraph.hypernodes[u].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = to_part;
        }
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[to_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[from_part]--;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = from_part;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[from_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[to_part]--;
        }

        for (int i = 0; i < 8; ++i) {
            fpga_resource_usage[to_part][i] += hypergraph.hypernodes[u].weights[i] - hypergraph.hypernodes[v].weights[i];
            fpga_resource_usage[from_part][i] += hypergraph.hypernodes[v].weights[i] - hypergraph.hypernodes[u].weights[i];
        }
    }

    void rateAllHypernodes() {
        for (int i = 0; i < hypergraph.hypernodes.size(); ++i) {
            if (hypergraph.hypernodes[i].deleted)
                continue;
            const VertexExchangeRating rating = rateOne(i);
            if (rating.valid) {
                pq.push(i, rating.value);
                target[i] = rating.target;
            }
        }
    }

    bool exchangeIsFeasible(const int u, const int v) {
#ifdef REPART_UNIFIED_SEQUENCE
        // A master location must be distinct from its existing replicas.
        if(unified_sequence_active && (hypergraph.hypernodes[u].replication_fpga_labels.count(hypergraph.hypernodes[v].label) ||
           hypergraph.hypernodes[v].replication_fpga_labels.count(hypergraph.hypernodes[u].label)))return false;
#endif
        bool valid = 1;
        Hypernode& u_node = hypergraph.hypernodes[u];
        Hypernode& v_node = hypergraph.hypernodes[v];
        int from_part = hypergraph.hypernodes[u].label;
        int to_part = hypergraph.hypernodes[v].label;
        for (int i = 0; i < 8; ++i) {
            bool from_part_valid = fpga_resource_usage[from_part][i] - u_node.weights[i] + v_node.weights[i] <= hypergraph.fpga_manager.fpga_info[from_part][i+1];
            bool to_part_valid = fpga_resource_usage[to_part][i] - v_node.weights[i] + u_node.weights[i] <= hypergraph.fpga_manager.fpga_info[to_part][i+1];
            if (!from_part_valid || !to_part_valid) {
                valid = 0;
                break;
            }
        }
        return valid;
    }

    VertexExchangeRating rateOne(const int u) {
        std::vector<std::pair<int, int>> tmp_ratings;
        int from_part = hypergraph.hypernodes[u].label;

        for (int v = 0; v < hypergraph.hypernodes.size(); ++v) {
            if (hypergraph.hypernodes[v].deleted)
                continue;
            if (hypergraph.hypernodes[v].label == from_part)
                continue;
            if (!exchangeIsFeasible(u, v))
                continue;
            int gain = ExchangeGain(u, v);
            tmp_ratings.push_back({v, gain});
        }

        int max_rating = -1;
        int max_target = -1;
        for (const auto& rating : tmp_ratings) {
            if (rating.second > max_rating) {
                max_rating = rating.second;
                max_target = rating.first;
            }
        }

        VertexExchangeRating result;
        if (max_rating > 0) {
            result.target = max_target;
            result.value = max_rating;
            result.valid = 1;
        }
        return result;
    }

    int ExchangeGain(const int u, const int v)
    {
#ifdef REPART_UNIFIED_SEQUENCE
        if(unified_sequence_active) {
        auto cost=[&]() {
            int value=0;
            for(const auto& e:hypergraph.hyperedges)if(!e.deleted)
                for(int f=0;f<hypergraph.fpga_manager.fpga_num;++f)if(e.target_pins_in_part[f]>0) {
                    int h=hypergraph.fpga_manager.getHopDistance(e.source_part_id,f);
                    if(h<0)return std::numeric_limits<int>::max()/4;
                    value+=e.weight*h;
                }
            return value;
        };
        int before=cost();exchangeHypernode(u,v);int after=cost();exchangeHypernode(u,v);
        return before-after;
        }
#endif
        int from_part = hypergraph.hypernodes[u].label;
        int to_part = hypergraph.hypernodes[v].label;

        std::unordered_set<int> affected_hyperedges;
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_source)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_target)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_source)
            affected_hyperedges.insert(he);
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_target)
            affected_hyperedges.insert(he);

        int init_cost = 0;
        for (const int he : affected_hyperedges) {
            Hyperedge& edge = hypergraph.hyperedges[he];
            int hop_distance = 0;
            for (int p = 0; p < hypergraph.fpga_manager.fpga_num; ++p) {
                if (edge.target_pins_in_part[p] > 0) {
                    hop_distance += hypergraph.fpga_manager.getHopDistance(edge.source_part_id, p);
                }
            }
            init_cost += hop_distance * edge.weight;
        }

        for (const int he : hypergraph.hypernodes[u].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = to_part;
        }
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[to_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[from_part]--;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = from_part;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[from_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[to_part]--;
        }
        int new_cost = 0;
        for (const int he : affected_hyperedges) {
            Hyperedge& edge = hypergraph.hyperedges[he];
            int hop_distance = 0;
            for (int p = 0; p < hypergraph.fpga_manager.fpga_num; ++p) {
                if (edge.target_pins_in_part[p] > 0) {
                    int a = hypergraph.fpga_manager.getHopDistance(edge.source_part_id, p);
                    if (a < 0) {
                        new_cost += 100000;
                    }
                    hop_distance += a;
                }
            }
            new_cost += hop_distance * edge.weight;
        }

        for (const int he : hypergraph.hypernodes[u].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = from_part;
        }
        for (const int he : hypergraph.hypernodes[u].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[from_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[to_part]--;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_source) {
            hypergraph.hyperedges[he].source_part_id = to_part;
        }
        for (const int he : hypergraph.hypernodes[v].incident_nets_as_target) {
            hypergraph.hyperedges[he].target_pins_in_part[to_part]++;
            hypergraph.hyperedges[he].target_pins_in_part[from_part]--;
        }

        return init_cost - new_cost;
    }


};
