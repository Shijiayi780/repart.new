#pragma once

#include <algorithm>
#include <bitset>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>
#include <queue>

#include "../datastructure/hypergraph.h"
#include "lazy_vertex_pair_coarsener_4th.h"
#include "metric.h"
#ifdef REPART_STATIC_COMMUNITY
#include "../community/static_runtime.h"
#endif

extern int edgesizeThresholdScale;
extern double penaltyPower;

using Coarsener1 = LazyVertexPairCoarsener;
class FullCoarsener{
    public:
    std::vector<Hypergraph> hypergraphs;
    const FPGAManager& fpga_manager;
#ifdef REPART_STATIC_COMMUNITY
    cf_bridge::StaticCommunityRun* community;
#endif

    FullCoarsener(Hypergraph& hg, const FPGAManager& fpga_manager
#ifdef REPART_STATIC_COMMUNITY
        , cf_bridge::StaticCommunityRun* community = nullptr
#endif
    ) : fpga_manager(fpga_manager)
#ifdef REPART_STATIC_COMMUNITY
        , community(community && community->enabled ? community : nullptr)
#endif
    {
        Hypergraph new_hypergraph = hg;
        hypergraphs.push_back(std::move(new_hypergraph));
    }

    bool checkCoarsenSame(const std::vector<Hypergraph>& hgs) {
        if (hgs.size() < 2) {
            return false;
        }
        else {
            if (hgs.back()._num_vertices_current == hgs[hgs.size()-2]._num_vertices_current) {
                return true;
            }            
        }
        return false;
    }

    double penaltyPowerGain(int num_final_nodes) {
        return 0.5*log(2)/log((hypergraphs[0]._num_vertices_current+1)/num_final_nodes);
    }

    std::vector<Hypergraph>& fullCoarsener(int num_final_nodes) {
#ifdef REPART_COMMUNITY_SEARCH
        community_search::initialize(hypergraphs.front());
#endif
        double penalty_power_gain = penaltyPowerGain(num_final_nodes);
#ifdef REPART_CF_OBSERVE
        const auto observation_start = std::chrono::steady_clock::now();
        const double observation_alpha_entry = penaltyPower;
        const double observation_paper_gain = 0.5*log(2)/log(
            (hypergraphs[0]._num_vertices_current+1.0)/num_final_nodes);
        int observation_attempt = 0, observation_retries = 0;
        cf_observe::record("coarsen_begin", hypergraphs[0]._num_vertices_current,
                           num_final_nodes, thread_num, observation_alpha_entry,
                           penalty_power_gain, observation_paper_gain);
#endif
#ifdef REPART_STATIC_COMMUNITY
        bool after_recovery_retry = false;
#endif
        while (1)
        {
            if (hypergraphs.back()._num_vertices_current <= num_final_nodes) {
#ifdef REPART_STATIC_COMMUNITY
                if (community) {
                    if (community->detector_calls == 0)
                        community->record("detector_skip", "TARGET_ALREADY_REACHED");
                    if (community->flowEnabled())
                        community->handleReachedTarget(
                            static_cast<std::size_t>(
                                hypergraphs.back()._num_vertices_current),
                            static_cast<std::size_t>(num_final_nodes));
                }
#endif
#ifdef REPART_CF_OBSERVE
                cf_observe::record("coarsen_stop", "target_reached");
#endif
                break;
            }
#ifdef REPART_STATIC_COMMUNITY
            if (community) {
                if (community->detector_calls == 0) {
                    community->initialize(hypergraphs.back());
                    if (community->flowEnabled())
                        community->runInitialScheduledFlow(hypergraphs.back());
                    else if (community->externalFlowCutCeilingEnabled())
                        community->runInitialExternalCutCeilingFlow(
                            hypergraphs.back());
                } else if (community->flowEnabled() &&
                           !after_recovery_retry) {
                    community->runPendingScheduledFlow(
                        hypergraphs.back(),
                        static_cast<std::size_t>(num_final_nodes));
                }
            }
#endif
            const int attempt_input_nodes =
                hypergraphs.back()._num_vertices_current;
            hypergraphs.emplace_back(hypergraphs.back());

            int coarsen_num = 0;
            if (hypergraphs.back()._num_vertices_current < 4 * num_final_nodes)
            {
                coarsen_num = hypergraphs.back()._num_vertices_current - num_final_nodes;
            }
            else
                coarsen_num = hypergraphs.back()._num_vertices_current/2;
            
            Coarsener1 coarsener(hypergraphs.back(), fpga_manager
#ifdef REPART_STATIC_COMMUNITY
                , community ? &community->labels : nullptr
#endif
            );
#ifdef REPART_STATIC_COMMUNITY
            if (community)
                community->record("attempt", community->attempts++, hypergraphs.size()-2,
                    hypergraphs.back()._num_vertices_current, num_final_nodes,
                    edgesizeThresholdScale, dynamicEdgeSizeThreshold(hypergraphs.back()), penaltyPower);
#endif
#ifdef REPART_CF_OBSERVE
            cf_observe::record("attempt", observation_attempt++, hypergraphs.size()-2,
                               observation_retries, penaltyPower,
                               observation_alpha_entry + (hypergraphs.size()-2)*observation_paper_gain,
                               edgesizeThresholdScale, dynamicEdgeSizeThreshold(hypergraphs.back()),
                               edgeSizeThreshold(hypergraphs.back()), coarsen_num);
            cf_observe::graph("graph_before", hypergraphs.back());
#endif
#ifdef REPART_COMMUNITY_SEARCH
            community_search::current_level=hypergraphs.size()-2;
#endif
            coarsener.coarsen(coarsen_num);
#ifdef REPART_STATIC_COMMUNITY
            if (community) {
                if (community->flowEnabled() &&
                    hypergraphs.back()._num_vertices_current >
                        attempt_input_nodes)
                    throw std::logic_error(
                        "COARSENING_INCREASED_ACTIVE_NODE_COUNT");
                const std::size_t actual_contractions =
                    hypergraphs.back()._num_vertices_current <=
                            attempt_input_nodes
                        ? static_cast<std::size_t>(
                        attempt_input_nodes -
                        hypergraphs.back()._num_vertices_current)
                        : 0;
                if (community->flowEnabled()) {
                    if (actual_contractions !=
                        coarsener.community_contractions)
                        throw std::logic_error(
                            "COMMUNITY_CONTRACTION_COUNT_MISMATCH");
                }
                community->contractions += coarsener.community_contractions;
                for (std::size_t slot = 0; slot < coarsener.community_counts.size(); ++slot) {
                    const auto& counts = coarsener.community_counts[slot];
                    community->cross_candidates += counts.cross_candidates;
                    community->positive_cross_candidates += counts.positive_cross_candidates;
                    community->record("rating_counts", community->attempts-1, slot,
                        counts.calls, counts.cross_candidates, counts.positive_cross_candidates,
                        "CROSS_COMMUNITY");
                }
                community->record("batch", community->attempts-1, coarsener.community_contractions,
                    hypergraphs.back()._num_vertices_current);
                if (community->flowEnabled())
                    community->noteCoarseningAttempt(
                        static_cast<std::size_t>(
                            hypergraphs.back()._num_vertices_current),
                        actual_contractions);
            }
#endif
#ifdef REPART_CF_OBSERVE
            cf_observe::graph("graph_after", hypergraphs.back());
#endif
            

            if (checkCoarsenSame(hypergraphs)) {
                hypergraphs.pop_back();
                if (dynamicEdgeSizeThreshold(hypergraphs.back()) > hypergraphs.back()._num_vertices_current) {
#ifdef REPART_STATIC_COMMUNITY
                    if (community && community->flowEnabled()) {
                        if (after_recovery_retry) {
                            after_recovery_retry = false;
                            community->stalled = true;
                            community->record(
                                "recovery_stop",
                                "RECOVERY_RETRY_FAILED");
                        } else {
                            const cf_bridge::RecoveryFlowAction action =
                                community->runRecoveryAfterRetryExhausted(
                                    hypergraphs.back());
                            if (action ==
                                cf_bridge::RecoveryFlowAction::
                                    RunOneOriginalAttempt) {
                                after_recovery_retry = true;
                                continue;
                            }
                            community->stalled = true;
                        }
                    } else if (community) {
                        community->stalled = true;
                    }
#endif
#ifdef REPART_CF_OBSERVE
                    cf_observe::record("coarsen_stop", "original_retry_exhausted");
#endif
                    break;
                }
                else
                {
                    edgesizeThresholdScale = 2*edgesizeThresholdScale;
#ifdef REPART_STATIC_COMMUNITY
                    if (community) community->record("retry", ++community->retries,
                        edgesizeThresholdScale, dynamicEdgeSizeThreshold(hypergraphs.back()));
#endif
#ifdef REPART_CF_OBSERVE
                    cf_observe::record("retry", ++observation_retries, edgesizeThresholdScale);
#endif
                }
            }
            else
            {
#ifdef REPART_STATIC_COMMUNITY
                if (community && community->flowEnabled() &&
                    after_recovery_retry)
                    after_recovery_retry = false;
#endif
                penaltyPower = penaltyPower + penalty_power_gain;
#ifdef REPART_CF_OBSERVE
                cf_observe::record("layer_accepted", hypergraphs.size()-1, penaltyPower,
                    observation_alpha_entry + (hypergraphs.size()-1)*observation_paper_gain);
#endif
            }
        }

#ifdef REPART_CF_OBSERVE
        cf_observe::record("coarsen_end", hypergraphs.size(), observation_attempt,
            observation_retries, std::chrono::duration<double>(
                std::chrono::steady_clock::now()-observation_start).count());
        cf_observe::flush();
#endif
#ifdef REPART_STATIC_COMMUNITY
        if (community) community->finish(hypergraphs.back()._num_vertices_current,
            num_final_nodes, dynamicEdgeSizeThreshold(hypergraphs.back()));
#endif
        return hypergraphs;
    }

    void printCoarsenSize() {
        for (int i = 0; i < hypergraphs.size(); i++) {
        }
    }
        
};
