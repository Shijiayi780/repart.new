#pragma once

#include <algorithm>
#include <bitset>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <assert.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>
#include <queue>

#include "../datastructure/hypergraph.h"
#include "metric.h"
#include "../datastructure/fast_reset_flag_array.h"
#include "../datastructure/binary_heap.h"

#include "../../boost_1_86_0/include/boost/thread/thread.hpp"
#include "../../boost_1_86_0/include/boost/thread/mutex.hpp"
#include "../../boost_1_86_0/include/boost/thread/shared_mutex.hpp"


using PrioQueue = BinaryMaxHeap<int, float>;
int thread_num = 4;
#ifdef REPART_MERGE_OPPORTUNITY
#include "merge_opportunity_probe.h"
#endif
#ifdef REPART_MERGE_CENSUS
#include "merge_layer_census.h"
#endif

class LazyVertexPairCoarsener {
    public:
    Hypergraph& hypergraph;
    PrioQueue pq;
    std::vector<int> target;
    FastResetFlagArray<> outdated_rating;
    const FPGAManager& fpga_manager;
#ifdef REPART_STATIC_COMMUNITY
    // Owned immutable copy: caller mutation cannot change labels during a batch.
    const std::vector<int> community_labels;
    const bool community_enabled;
    const bool community_trace;
    std::vector<std::vector<CommunityRatingCall>> community_calls;
    std::vector<CommunityRatingCounts> community_counts;
    std::size_t community_contractions = 0;
#endif
    
    LazyVertexPairCoarsener(Hypergraph& hg, const FPGAManager& fpga_manager
#ifdef REPART_STATIC_COMMUNITY
        , const std::vector<int>* labels = nullptr, bool trace = false
#endif
    ) :
        hypergraph(hg),
        pq((hg.hypernodes).size()), 
        target((hg.hypernodes).size()),
        outdated_rating((hg.hypernodes).size()),
        fpga_manager(fpga_manager)
#ifdef REPART_STATIC_COMMUNITY
        , community_labels(labels ? *labels : std::vector<int>{}),
        community_enabled(labels != nullptr), community_trace(trace),
        community_calls(labels && trace ? thread_num + 1 : 0),
        community_counts(labels ? thread_num + 1 : 0)
#endif
    {
#ifdef REPART_STATIC_COMMUNITY
        if (community_enabled && community_labels.size() != hg.hypernodes.size())
            throw std::invalid_argument("community label count must match node storage");
#endif
    }

#ifdef REPART_STATIC_COMMUNITY
    VertexPairRating rateCommunityNode(int node, int slot, const char* phase) {
        if (!community_enabled) return rateOne(hypergraph, node, fpga_manager);
        ++community_counts.at(slot).calls;
        std::vector<CommunityRejection>* rejected = nullptr;
        if (community_trace) {
            community_calls.at(slot).push_back(
                {hypergraph._num_vertices_current, node, phase, {}});
            rejected = &community_calls.at(slot).back().rejected;
        }
        return rateOne(hypergraph, node, fpga_manager, &community_labels, rejected,
                       &community_counts.at(slot));
    }
#endif


    void rateAllHypernodes() {
        using namespace std;
        vector<vector<pair<int, VertexPairRating>>> ratings(thread_num);
#ifdef REPART_CF_OBSERVE
        std::vector<std::thread::id> observation_thread_ids(thread_num);
#endif
        auto rateAllHypernodesThread = [&](int thread_id)
        {
#ifdef REPART_CF_OBSERVE
            observation_thread_ids[thread_id] = std::this_thread::get_id();
#endif
            for (int i = thread_id; i < hypergraph.hypernodes.size(); i += 4)
            {
                if (hypergraph.hypernodes[i].deleted)
                    continue;
#ifdef REPART_STATIC_COMMUNITY
                const VertexPairRating rating = rateCommunityNode(i, thread_id, "initial");
#else
                const VertexPairRating rating = rateOne(hypergraph, i, fpga_manager);
#endif
                ratings[thread_id].push_back({i, rating});
            }
        };

        std::vector<boost::thread> threads;
        for (int i = 0; i < thread_num; i++) {
            threads.push_back(boost::thread(rateAllHypernodesThread, i));
        }    

        for (int i = 0; i < thread_num; i++) {
            threads[i].join();
        }
#ifdef REPART_CF_OBSERVE
        for (int i = 0; i < thread_num; ++i) {
            cf_observe::record("coarsen_thread", i, observation_thread_ids[i],
                               thread_num, 4, "N/A", ratings[i].size());
        }
#endif
        for (int i = 0; i < thread_num; i++) {
            for (int j = 0; j < ratings[i].size(); j++) {
                const int hn = ratings[i][j].first;
                const VertexPairRating& rating = ratings[i][j].second;
                if (rating.valid) {
                    pq.push(hn, rating.value);
                    target[hn] = rating.target;
                }
            }
        }
    }

    

    void coarsen(int contract_num) {
#ifdef REPART_COMMUNITY_SEARCH
        community_search::Batch search_batch(hypergraph, fpga_manager);
#endif
        pq.clear();
        int contracted_nodes = 0;
        rateAllHypernodes();
#ifdef REPART_MERGE_CENSUS
        merge_census::observe(hypergraph, fpga_manager, pq, target);
#endif
#ifdef REPART_MERGE_OPPORTUNITY
        merge_probe::begin(hypergraph, fpga_manager, pq, target);
#endif

        while (!pq.empty()) {
            if (contracted_nodes == contract_num)
                break;

            const int rep_node =
#ifdef REPART_MERGE_OPPORTUNITY
                merge_probe::choose(pq.top());
#else
                pq.top();
#endif

            if (outdated_rating[rep_node]) {
#ifdef REPART_STATIC_COMMUNITY
                updatePQandContractionTarget(rep_node, rateCommunityNode(rep_node, thread_num, "stale"));
#else
                updatePQandContractionTarget(rep_node, rateOne(hypergraph, rep_node, fpga_manager));
#endif
            } else {
                const int contracted_node = target[rep_node];
#ifdef REPART_MERGE_OPPORTUNITY
                merge_probe::before_contract(hypergraph, fpga_manager,
                    rep_node, contracted_node, pq.getKey(rep_node));
#endif
#ifdef REPART_CF_OBSERVE
                cf_observe::record("contract", rep_node, contracted_node,
#ifdef REPART_MERGE_OPPORTUNITY
                                   pq.getKey(rep_node), hypergraph._num_vertices_current);
#else
                                   pq.topKey(), hypergraph._num_vertices_current);
#endif
#endif
#ifdef REPART_STATIC_COMMUNITY
                if (community_enabled && community_labels.at(rep_node) != community_labels.at(contracted_node))
                    throw std::logic_error("cross-community contraction invariant violated");
#endif
#ifdef REPART_COMMUNITY_SEARCH
                search_batch.beforeContract(rep_node, contracted_node);
#endif
                hypergraph.contract(rep_node, contracted_node);
#ifdef REPART_COMMUNITY_SEARCH
                search_batch.afterContract(rep_node, contracted_node);
#endif
#ifdef REPART_STATIC_COMMUNITY
                if (community_enabled) ++community_contractions;
#endif
                contracted_nodes++;
                if (hypergraph._num_vertices_current % 1000 == 0) {
                }

                if (pq.contains(contracted_node)) {
                    pq.remove(contracted_node);
                }
                invalidateAffectedHypernodes(rep_node);
#ifdef REPART_STATIC_COMMUNITY
                updatePQandContractionTarget(rep_node, rateCommunityNode(rep_node, thread_num, "after_contract"));
#else
                updatePQandContractionTarget(rep_node, rateOne(hypergraph, rep_node, fpga_manager));
#endif
#ifdef REPART_MERGE_OPPORTUNITY
                merge_probe::after_contract(hypergraph, fpga_manager,
                    rep_node, contracted_node);
#endif
            }
        }
    }


    void invalidateAffectedHypernodes(const int rep_node){
        for (const int he_idx : hypergraph.hypernodes[rep_node].incident_nets_as_source) {
            Hyperedge& he = hypergraph.hyperedges[he_idx];
            if (he._size > edgeSizeThreshold(hypergraph)) {
                continue;
            }
            for (int pin_idx = he._begin; pin_idx < he._begin + he._size; pin_idx++) {
                const int pin = hypergraph._incidence_array[pin_idx];
                outdated_rating.set(pin, true);
            }
        }
        for (const int he_idx : hypergraph.hypernodes[rep_node].incident_nets_as_target) {
            Hyperedge& he = hypergraph.hyperedges[he_idx];
            if (he._size > edgeSizeThreshold(hypergraph)) {
                continue;
            }
            for (int pin_idx = he._begin; pin_idx < he._begin + he._size; pin_idx++) {
                const int pin = hypergraph._incidence_array[pin_idx];
                outdated_rating.set(pin, true);
            }
        }
    }
    
    void updatePQandContractionTarget(const int hn, const VertexPairRating& rating) {
        outdated_rating.set(hn, false);
        if (rating.valid) {
            pq.updateKey(hn, rating.value);
            target[hn] = rating.target;
        } else {
            pq.remove(hn);
        }
    }

};
