#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <assert.h>
#include <chrono>
#include <random>
#ifdef REPART_GLOBAL_ASSIGNMENT
#include "global_assignment_layer.h"
#include "scheduled_board_score.h"
#include <cstdlib>
#include <sstream>
#include <atomic>
#endif

#include "../datastructure/hypergraph.h"


#include "../../boost_1_86_0/include/boost/thread/thread.hpp"
#include "../../boost_1_86_0/include/boost/thread/mutex.hpp"
#include "../../boost_1_86_0/include/boost/thread/shared_mutex.hpp"



class AssignmentSolver {
public:
    const std::vector<Hypernode>& hypernodes;        
    const std::vector<Hyperedge>& hyperedges;        
    const std::vector<int>& _incidence_array;         
    const std::vector<std::vector<int>>& fpga_info;   
    const std::vector<std::vector<int>>& comm_cost;   
    const int max_hop_distance;                       

    const int nodes_num;                              

    const int thread_num = std::min(nodes_num, 4); 

    long long best_cost;                                 
    std::map<int, int> best_assignment;               
    std::vector<std::vector<int>> fpgas_resource_usage; 
    std::vector<int> fpgas_comm_weight_sum; 

    const int fpga_num;                 

    std::vector<std::vector<int>> sorted_hypernode_ids = std::vector<std::vector<int>>(thread_num, std::vector<int>());    
    std::vector<int> sorted_fpga_ids;    

    const int cannot_communicate_penalty = 2*max_hop_distance; 



    std::vector<int> flag_finished = std::vector<int>(thread_num, 0);   
    const int when_finished = 256 / fpga_num;     
    const bool only_one_feasible_solution = false;         
    const double finished_threshold = 0.98; 

    std::chrono::steady_clock::time_point start_time; 
    const int time_limit_second; 
    const std::chrono::seconds time_limit = std::chrono::seconds(time_limit_second); 



    std::vector<std::vector<int>> node_sorted_index_backtrack_num = std::vector<std::vector<int>>(thread_num, std::vector<int>(nodes_num, 0));   
    const int depth_exit_threshold = 128; 
    std::vector<int> depth_exit_count = std::vector<int>(thread_num, 0); 


    std::vector<std::mt19937> rngs; 
    std::vector<std::uniform_int_distribution<int>> dists; 
#ifdef REPART_GLOBAL_ASSIGNMENT
    bool global_score_enabled = false;
    bool global_seed_a = false;
    bool global_board_order = false;
    bool scheduled_board_score = false;
    bool first_complete_only = false;
    std::atomic<bool> first_complete_stop{false};
    int first_complete_thread = -1;
    struct ScheduleDiagnostic {
        std::uint64_t states=0, changed_native=0, changed_fixed=0;
        std::vector<std::string> first_depth;
    };
    std::vector<ScheduleDiagnostic> schedule_diagnostics;
    std::vector<double> global_fpga_heat;
    global_assignment_score::LayerData global_layer;
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
    struct ConstraintCounts {
        std::uint64_t states=0, leaves=0, retreats=0;
        std::uint64_t scored=0, score_resource=0, score_hop=0, score_comm=0, score_ok=0;
        std::uint64_t tried=0, resource=0, hop=0, comm=0, pruned=0, placed=0;
        std::array<std::uint64_t,8> score_dims{}, execution_dims{};
    };
    std::vector<ConstraintCounts> constraint_counts=std::vector<ConstraintCounts>(thread_num);
#endif
#endif

    AssignmentSolver(const Hypergraph& hg, const FPGAManager& fpga_manager, const int time_limit_second)
        : hypernodes(hg.hypernodes), hyperedges(hg.hyperedges), _incidence_array(hg._incidence_array), nodes_num(hg._num_vertices_current),
          fpga_info(fpga_manager.fpga_info),comm_cost(fpga_manager.hop_distances), max_hop_distance(fpga_manager.max_hop_distance), fpga_num(fpga_info.size()),
          time_limit_second(time_limit_second) {
        best_cost = std::numeric_limits<long long>::max();

        best_assignment = {};

        std::mt19937 master_rng(42); 
        std::uniform_int_distribution<int> master_dist(0, std::numeric_limits<int>::max());

        rngs.reserve(thread_num);
        dists.reserve(thread_num);
        for(int thread_id = 0; thread_id < thread_num; ++thread_id) {
#ifdef REPART_CF_OBSERVE
            rngs.emplace_back(cf_observe::seed(thread_id, master_rng()));
#else
            rngs.emplace_back(master_rng());
#endif
            dists.emplace_back(0, std::numeric_limits<int>::max());
        }

        sort_hypernodes();
        sort_fpgas();
#ifdef REPART_GLOBAL_ASSIGNMENT
        const char* first_mode=std::getenv("REPART_ASSIGNMENT_FIRST_ONLY");
        first_complete_only=first_mode && std::string(first_mode)=="1";
        const char* mode = std::getenv("REPART_GLOBAL_ASSIGNMENT");
        global_score_enabled = mode && std::string(mode) == "1";
        const char* seed_mode = std::getenv("REPART_GLOBAL_SEED_A");
        global_seed_a = global_score_enabled && seed_mode && std::string(seed_mode) == "1";
        const char* board_mode = std::getenv("REPART_GLOBAL_BOARD_ORDER");
        global_board_order = global_score_enabled && board_mode && std::string(board_mode) == "1";
        const char* schedule_mode=std::getenv("REPART_BOARD_SCHEDULE");
        scheduled_board_score=schedule_mode && std::string(schedule_mode)=="1";
        if(scheduled_board_score && !global_board_order)
            throw std::invalid_argument("scheduled score requires board-order mode");
        if (global_board_order && global_seed_a)
            throw std::invalid_argument("board-order and seed-only modes are exclusive");
        if (global_score_enabled) {
            if(scheduled_board_score) {
                schedule_diagnostics.resize(thread_num);
                for(auto& trace:schedule_diagnostics) trace.first_depth.resize(nodes_num);
            }
            global_layer = global_assignment_score::build_layer(hg, fpga_manager);
            if (global_board_order)
                global_fpga_heat = global_assignment_score::a_fpga_heat(comm_cost,max_hop_distance);
            auto override_parameter = [](const char* name, double& value) {
                if (const char* text = std::getenv(name)) {
                    std::size_t end = 0;
                    value = std::stod(text, &end);
                    if (end != std::string(text).size() || !std::isfinite(value) || value < 0)
                        throw std::invalid_argument(name);
                }
            };
            auto& p = global_layer.parameters;
            override_parameter("REPART_GLOBAL_THETA", p.theta);
            override_parameter("REPART_GLOBAL_ETA", p.eta);
            override_parameter("REPART_GLOBAL_LAMBDA", p.lambda);
            override_parameter("REPART_GLOBAL_MU", p.mu);
            override_parameter("REPART_GLOBAL_EPSILON", p.epsilon);
            if (p.epsilon <= 0 || p.epsilon > 1) throw std::invalid_argument("epsilon");
            std::cout << "global_assignment nodes=" << nodes_num
                      << " threads=" << thread_num << " layer_budget=" << time_limit_second
                      << " seed_a=" << global_seed_a << " board_order=" << global_board_order
                      << " schedule=" << scheduled_board_score << ' ';
            global_assignment_score::log_parameters(std::cout, p);
            if(scheduled_board_score)
                std::cout << "board_schedule heat=.8-.6*t communication=.2+.6*t t=depth/max(1,n-1) normalize=legal_candidate_max resource=theta/c eta_unused=1\n";
            if (global_seed_a) {
                for (int t=0;t<thread_num;++t)
                    std::cout << "global_seed thread=" << t
                              << " node=" << sorted_hypernode_ids[t].front()
                              << " preferred_fpga=" << sorted_fpga_ids.front() << '\n';
            }
            if (global_board_order) {
                for (int f=0;f<fpga_num;++f)
                    std::cout << "board_heat fpga=" << f << " value=" << global_fpga_heat[f] << '\n';
                for (int t=0;t<thread_num;++t) {
                    std::cout << "board_node_order thread=" << t << " ids=";
                    for (int u:sorted_hypernode_ids[t]) std::cout << u << ',';
                    std::cout << '\n';
                }
            }
        }
#endif
    }

    std::pair<std::map<int, int>, long long> solve() {
#ifdef REPART_GLOBAL_ASSIGNMENT
        first_complete_stop.store(false);
        first_complete_thread=-1;
#endif
        best_cost = std::numeric_limits<long long>::max();
        best_assignment.clear();
        fpgas_resource_usage.assign(fpga_num, std::vector<int>(8, 0));
        fpgas_comm_weight_sum.assign(fpga_num, 0);

        start_time = std::chrono::steady_clock::now();

        int num_threads = std::min(thread_num, nodes_num);
        std::vector<boost::thread> threads;
        boost::mutex mutex_queue;
#ifdef REPART_CF_OBSERVE
        std::vector<std::thread::id> observation_thread_ids(num_threads);
#endif


        auto thread_work = [&](int thread_id) {
#ifdef REPART_CF_OBSERVE
            observation_thread_ids[thread_id] = std::this_thread::get_id();
#endif
            std::map<int, int> local_assignment;
            std::vector<std::vector<int>> local_resource_usage(fpga_num, std::vector<int>(8, 0));
            std::vector<int> local_comm_weight_sum(fpga_num, 0);
            std::vector<std::set<int>> fpga_contained_hyperedges(fpga_num, std::set<int>());
            long long local_total_cost = 0;

            backtrack(0, local_assignment, local_resource_usage, local_comm_weight_sum, fpga_contained_hyperedges, local_total_cost, thread_id);
        };

        for(int thread_id = 0; thread_id < num_threads; ++thread_id) {
            threads.emplace_back(boost::thread(thread_work, thread_id));
        }

        for(auto& th : threads) {
            th.join();
        }
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
        const double search_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start_time).count();
        std::cout << "constraint_layer nodes=" << nodes_num << " success=" << !best_assignment.empty()
                  << " search_seconds=" << search_seconds << '\n';
        for(int t=0;t<thread_num;++t) {
            const auto& c=constraint_counts[t];
            std::cout << "constraint_count thread=" << t << " states=" << c.states << " leaves=" << c.leaves
                      << " retreats=" << c.retreats << " scored=" << c.scored
                      << " score_resource=" << c.score_resource << " score_hop=" << c.score_hop
                      << " score_comm=" << c.score_comm << " score_ok=" << c.score_ok
                      << " tried=" << c.tried << " resource=" << c.resource << " hop=" << c.hop
                      << " comm=" << c.comm << " pruned=" << c.pruned << " placed=" << c.placed << '\n';
            std::cout << "constraint_dims thread=" << t << " score=";
            for(auto v:c.score_dims) std::cout << v << ',';
            std::cout << " execution=";
            for(auto v:c.execution_dims) std::cout << v << ',';
            std::cout << '\n';
        }
#endif
#ifdef REPART_GLOBAL_ASSIGNMENT
        if(first_complete_only)
            std::cout << "assignment_stop first_only=1 accepted_complete=" << !best_assignment.empty()
                      << " thread=" << first_complete_thread << " cost=" << best_cost << '\n';
        if(scheduled_board_score) {
            for(int t=0;t<thread_num;++t) {
                const auto& trace=schedule_diagnostics[t];
                std::cout << "schedule_diagnostic thread=" << t << " states=" << trace.states
                          << " changed_native=" << trace.changed_native
                          << " changed_fixed=" << trace.changed_fixed << '\n';
                for(const auto& line:trace.first_depth)
                    if(!line.empty()) std::cout << "schedule_first_state thread=" << t << ' ' << line << '\n';
            }
        }
#endif
#ifdef REPART_CF_OBSERVE
        for (int i = 0; i < num_threads; ++i)
            cf_observe::record("assignment_thread", i, observation_thread_ids[i], num_threads);
#endif

        return {best_assignment, best_cost};
    }


private:

    boost::mutex mutex_best_cost; 

    int rate_connection(int u, int v) {
        int score = 0;
        for (const int he : hypernodes[u].incident_nets_as_target) {
            const Hyperedge& edge = hyperedges[he];
            if (edge.deleted) continue;
            if(_incidence_array[edge._begin] == v) {
                score += edge.weight;
            }
        }
        for (const int he : hypernodes[v].incident_nets_as_target) {
            const Hyperedge& edge = hyperedges[he];
            if (edge.deleted) continue;
            if(_incidence_array[edge._begin] == u) {
                score += edge.weight;
            }
        }
        return score;
    }


    void resort_hypernodes(const std::vector<std::pair<int, int>>& connection_counts, const int thread_id) {
        std::unordered_set<int> hypernode_used;
        for(int i = 0; i < nodes_num; ++i) {
            if(i == 0){
                sorted_hypernode_ids[thread_id].push_back(connection_counts[thread_id].first);
                hypernode_used.insert(connection_counts[thread_id].first);
            }
            else{
                int max_connection = -1;
                int max_connection_id = -1;
                for(int j = 0; j < nodes_num; ++j) {
                    if(hypernode_used.find(connection_counts[j].first) != hypernode_used.end()) {
                        continue;
                    }
                    int connection = rate_connection(sorted_hypernode_ids[thread_id].back(), connection_counts[j].first);
                    if(connection > max_connection) {
                        max_connection = connection;
                        max_connection_id = j;
                    }
                }

                if(max_connection_id != -1) {
                    hypernode_used.insert(connection_counts[max_connection_id].first);
                    sorted_hypernode_ids[thread_id].push_back(connection_counts[max_connection_id].first);
                }
                else{
                    for(int j = 0; j < nodes_num; ++j) {
                        if(hypernode_used.find(connection_counts[j].first) != hypernode_used.end()) {
                            continue;
                        }
                        sorted_hypernode_ids[thread_id].push_back(connection_counts[j].first);
                        hypernode_used.insert(connection_counts[j].first);
                    }
                }
            }
        }
        

    }


    void sort_hypernodes() {
        std::vector<std::pair<int, int>> connection_counts; 
        for (const auto& hn : hypernodes) {
            if (hn.deleted) continue;
            int count = hn.incident_nets_as_target.size();
            for(auto& he : hn.incident_nets_as_source) {
                count += (hyperedges[he]._size - 1);
            }

            connection_counts.emplace_back(hn.id, count);
        }

        std::sort(connection_counts.begin(), connection_counts.end(),
                  [](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool {
                      return a.second > b.second;
                  });



        for(int thread_id = 0; thread_id < thread_num; ++thread_id) {
            resort_hypernodes(connection_counts, thread_id);
        }
    }

    void resort_fpgas(const std::vector<std::pair<int, int>>& connection_counts) {
        std::vector<bool> fpga_used(fpga_num, false);
        for(int i = 0; i < fpga_num; ++i) {
            if(i == 0){
                sorted_fpga_ids.push_back(connection_counts[0].first);
                fpga_used[connection_counts[i].first] = true;
            }
            else{
                int min_distance = std::numeric_limits<int>::max();
                int min_distance_id = -1;
                for(int j = 0; j < fpga_num; ++j) {
                    if(fpga_used[connection_counts[j].first]) {
                        continue;
                    }
                    if(comm_cost[sorted_fpga_ids.back()][connection_counts[j].first] < min_distance && comm_cost[sorted_fpga_ids.back()][connection_counts[j].first] != -1) {
                        min_distance = comm_cost[sorted_fpga_ids.back()][connection_counts[j].first];
                        min_distance_id = j;
                    }
                }
                if(min_distance_id != -1) {
                    fpga_used[connection_counts[min_distance_id].first] = true;
                    sorted_fpga_ids.push_back(connection_counts[min_distance_id].first);
                }
                else{
                    for(int j = 0; j < fpga_num; ++j) {
                        if(fpga_used[connection_counts[j].first]) {
                            continue;
                        }
                        sorted_fpga_ids.push_back(connection_counts[j].first);
                        fpga_used[connection_counts[j].first] = true;
                    }
                }
            }
        }

    }

    void sort_fpgas() {
        std::vector<std::pair<int, int>> connection_counts; 
        for (int i = 0; i < fpga_num; ++i) {
            int count = 0;
            for(int j = 0; j < fpga_num; j++) {
                if (i == j) continue;
                if (comm_cost[i][j] != -1) {
                    count += comm_cost[i][j];
                }
                else{
                    count += cannot_communicate_penalty;
                }
            }
            connection_counts.emplace_back(i, count);
        }

        std::sort(connection_counts.begin(), connection_counts.end(),
                  [](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool {
                      return a.second < b.second;
                  });


        resort_fpgas(connection_counts);
    }

    const Hypernode& get_hypernode_by_id(int id) const {
        return hypernodes[id];
    }

    void backtrack(int node_sorted_index,
                   std::map<int, int>& assignment,
                   std::vector<std::vector<int>>& resource_usage,
                   std::vector<int>& comm_weight_sum,
                   std::vector<std::set<int>>& fpga_contained_hyperedges,
                   long long total_cost,
                   const int thread_id) {
#ifdef REPART_GLOBAL_ASSIGNMENT
        if(first_complete_only && first_complete_stop.load(std::memory_order_acquire)) return;
#endif
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
        ++constraint_counts[thread_id].states;
#endif

        if(flag_finished[thread_id] == when_finished) {
            return;
        }

        if (node_sorted_index == nodes_num) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            ++constraint_counts[thread_id].leaves;
#endif
#ifdef REPART_GLOBAL_ASSIGNMENT
            if(first_complete_only) {
                boost::mutex::scoped_lock lock(mutex_best_cost);
                if(!first_complete_stop.load(std::memory_order_relaxed)) {
                    best_cost=total_cost;
                    best_assignment=assignment;
                    fpgas_resource_usage=resource_usage;
                    fpgas_comm_weight_sum=comm_weight_sum;
                    first_complete_thread=thread_id;
                    first_complete_stop.store(true,std::memory_order_release);
                }
                return;
            }
#endif
            if((double)total_cost > (double)best_cost * finished_threshold) {
                ++flag_finished[thread_id];
            }
            else{
                flag_finished[thread_id] = 0;
            }

            boost::mutex::scoped_lock lock(mutex_best_cost); 
            if (total_cost < best_cost) {
                best_cost = total_cost;
                best_assignment = assignment;
                fpgas_resource_usage = resource_usage;
                fpgas_comm_weight_sum = comm_weight_sum;

                if(only_one_feasible_solution) {
                    flag_finished[thread_id] = when_finished;
                }
            }
            lock.unlock();


            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                flag_finished[thread_id] = when_finished;
            }
            return;
        }

        if(node_sorted_index_backtrack_num[thread_id][node_sorted_index] > depth_exit_threshold || flag_finished[thread_id] > 0) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            ++constraint_counts[thread_id].retreats;
#endif
            node_sorted_index_backtrack_num[thread_id][node_sorted_index] = 0;
            if(flag_finished[thread_id] > 0) {
                flag_finished[thread_id] = 0;
            }
            int random_value = dists[thread_id](rngs[thread_id]);
            depth_exit_count[thread_id] = random_value % ((node_sorted_index+2) / 2);


            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                flag_finished[thread_id] = when_finished;
            }
            return;
        }

        if(node_sorted_index < nodes_num) {
            ++node_sorted_index_backtrack_num[thread_id][node_sorted_index];
        }


        // Native A keeps its exact node/FPGA order. Global mode branches over
        // every unassigned node/FPGA pair, ordered by the current frozen-layer score.
        std::vector<std::pair<int,int>> choices;
#ifdef REPART_GLOBAL_ASSIGNMENT
        if (global_seed_a && node_sorted_index == 0) {
            // Root only: reuse A's per-thread hot node and FPGA attempt order.
            const int seed = sorted_hypernode_ids[thread_id].front();
            for (int f : sorted_fpga_ids) choices.emplace_back(seed,f);
        } else if (global_score_enabled) {
            std::vector<global_assignment_score::Candidate> candidates;
            const std::vector<int> candidate_nodes = global_board_order
                ? std::vector<int>{sorted_hypernode_ids[thread_id][node_sorted_index]}
                : global_layer.nodes;
            for (int u : candidate_nodes) {
                if (assignment.count(u)) continue;
                std::vector<global_assignment_score::ScheduledBoard> scheduled_boards;
                global_assignment_score::Resources demand{};
                for (int k=0;k<8;++k) demand[k]=hypernodes[u].weights[k];
                for (int f=0;f<fpga_num;++f) {
                    global_assignment_score::Resources capacity{}, used{};
                    for (int k=0;k<8;++k) {
                        capacity[k]=fpga_info[f][k+1]; used[k]=resource_usage[f][k];
                    }
                    const auto neighbors = global_assignment_score::assigned_neighbors(
                        global_layer,u,f,assignment,comm_cost);
                    auto score = global_assignment_score::evaluate(
                        demand,capacity,used,f,global_layer.degrees[f],max_hop_distance,
                        neighbors,true,global_layer.parameters);
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                    auto& counters=constraint_counts[thread_id];
                    ++counters.scored;
                    if(score.rejection==global_assignment_score::Result::Rejection::Resource) {
                        ++counters.score_resource; ++counters.score_dims[score.resource_dimension];
                    } else if(score.rejection==global_assignment_score::Result::Rejection::Hop) ++counters.score_hop;
#endif
                    if (score.feasible && global_board_order) {
                        score.topology_reward = global_layer.parameters.eta * global_fpga_heat[f];
                        score.score = score.connection - score.resource_penalty + score.topology_reward;
                    }
                    if(score.feasible && scheduled_board_score) {
                        const bool allowed=global_assignment_score::communication_allows(hypernodes,hyperedges,_incidence_array,
                                fpga_info,comm_weight_sum,assignment,u,f);
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                        if(allowed) ++counters.score_ok; else ++counters.score_comm;
#endif
                        if(allowed)
                            scheduled_boards.push_back({f,score.connection,global_fpga_heat[f],score.resource_penalty});
                        continue;
                    }
                    if (score.feasible) candidates.push_back({u,f,score.score});
                }
                if(scheduled_board_score) {
                    candidates=global_assignment_score::scheduled_board_order(u,node_sorted_index,nodes_num,scheduled_boards);
                    auto& trace=schedule_diagnostics[thread_id];
                    ++trace.states;
                    std::vector<int> actual,native;
                    std::vector<global_assignment_score::Candidate> fixed;
                    for(auto entry:candidates) actual.push_back(entry.fpga);
                    for(int f:sorted_fpga_ids)
                        for(auto b:scheduled_boards) if(b.fpga==f) native.push_back(f);
                    for(auto b:scheduled_boards)
                        fixed.push_back({u,b.fpga,b.connection-b.resource_penalty+global_layer.parameters.eta*b.heat});
                    std::sort(fixed.begin(),fixed.end(),global_assignment_score::precedes);
                    std::vector<int> fixed_order;
                    for(auto entry:fixed) fixed_order.push_back(entry.fpga);
                    trace.changed_native += actual!=native;
                    trace.changed_fixed += actual!=fixed_order;
                    if(trace.first_depth[node_sorted_index].empty()) {
                        double max_b=0,max_h=0;
                        for(auto b:scheduled_boards) {max_b=std::max(max_b,b.connection);max_h=std::max(max_h,b.heat);}
                        double progress=double(node_sorted_index)/std::max(1,nodes_num-1);
                        std::ostringstream line;
                        line << std::setprecision(17) << "depth=" << node_sorted_index << " node=" << u
                             << " t=" << progress << " wB=" << .2+.6*progress << " wH=" << .8-.6*progress;
                        for(auto entry:candidates)
                            for(auto b:scheduled_boards) if(b.fpga==entry.fpga)
                                line << " board=" << b.fpga << ",B=" << b.connection
                                     << ",Bhat=" << (max_b>0?b.connection/max_b:0)
                                     << ",Hhat=" << (max_h>0?b.heat/max_h:0)
                                     << ",R=" << b.resource_penalty << ",S=" << entry.score;
                        trace.first_depth[node_sorted_index]=line.str();
                    }
                }
            }
            std::sort(candidates.begin(),candidates.end(),global_assignment_score::precedes);
            for (const auto& candidate:candidates)
                choices.emplace_back(candidate.node,candidate.fpga);
        } else
#endif
        {
            const int u=sorted_hypernode_ids[thread_id][node_sorted_index];
            for (int f:sorted_fpga_ids) choices.emplace_back(u,f);
        }
        for (const auto& choice : choices) {
#ifdef REPART_GLOBAL_ASSIGNMENT
            if(first_complete_only && first_complete_stop.load(std::memory_order_acquire)) return;
#endif
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            ++constraint_counts[thread_id].tried;
#endif
            const int hypernode_id=choice.first, f_id=choice.second;
            const Hypernode& hn = get_hypernode_by_id(hypernode_id);


            bool feasible = true;
            for (int i = 0; i < 8; ++i) {
                if (resource_usage[f_id][i] + hn.weights[i] > fpga_info[f_id][i + 1]) { 
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                    ++constraint_counts[thread_id].resource;
                    ++constraint_counts[thread_id].execution_dims[i];
#endif
                    feasible = false;
                    break;
                }
            }
            if (!feasible) {
                continue; 
            }

            long long incremental_cost = 0;
            bool communication_feasible = true;
            std::map<int, int> added_comm_weights; 

            std::vector<std::set<int>> fpga_contained_hyperedges_new = fpga_contained_hyperedges;

            for (const auto& he_id : hn.incident_nets_as_source) {
                const Hyperedge& he = hyperedges[he_id];
                if (he.deleted) continue;

                for (int i = 1; i < he._size; ++i) {
                    int target_node = _incidence_array[he._begin + i];
                    auto it = assignment.find(target_node);
                    if (it != assignment.end()) {
                        int target_fpga_id = it->second;
                        if(fpga_contained_hyperedges_new[target_fpga_id].find(he_id) != fpga_contained_hyperedges_new[target_fpga_id].end()) {
                            continue;
                        }

                        if(target_fpga_id == f_id) {
                            continue;
                        }

                        int C = comm_cost[f_id][target_fpga_id];
                        if (C == -1) {
                            communication_feasible = false;
                            break; 
                        }

                        fpga_contained_hyperedges_new[target_fpga_id].insert(he_id);

                        if(fpga_contained_hyperedges_new[f_id].find(he_id) == fpga_contained_hyperedges_new[f_id].end()) {
                            fpga_contained_hyperedges_new[f_id].insert(he_id);
                            added_comm_weights[f_id] += he.weight;
                        }
                        
                        incremental_cost += static_cast<int>(he.weight) * C;

                        fpga_contained_hyperedges_new[target_fpga_id].insert(he_id);
                        added_comm_weights[target_fpga_id] += he.weight;
                    }
                }
                if (!communication_feasible) {
                    break;
                }
            }

            if (!communication_feasible) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                ++constraint_counts[thread_id].hop;
#endif
                continue; 
            }

            for (const auto& he_id : hn.incident_nets_as_target) {
                if(fpga_contained_hyperedges_new[f_id].find(he_id) != fpga_contained_hyperedges_new[f_id].end()) {
                    continue;
                }
                const Hyperedge& he = hyperedges[he_id];
                if (he.deleted) continue;

                int source_node = _incidence_array[he._begin];
                auto it = assignment.find(source_node);
                if (it != assignment.end()) {
                    int source_fpga_id = it->second;

                    if(source_fpga_id == f_id) {
                        continue;
                    }

                    int C = comm_cost[source_fpga_id][f_id];
                    if (C == -1) {
                        communication_feasible = false;
                        break; 
                    }
                    incremental_cost += static_cast<long long>(he.weight) * C;

                    if(fpga_contained_hyperedges_new[source_fpga_id].find(he_id) == fpga_contained_hyperedges_new[source_fpga_id].end()) {
                        fpga_contained_hyperedges_new[source_fpga_id].insert(he_id);
                        added_comm_weights[source_fpga_id] += he.weight;
                    }
                    fpga_contained_hyperedges_new[f_id].insert(he_id);
                    added_comm_weights[f_id] += he.weight;
                }
            }

            if (!communication_feasible) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                ++constraint_counts[thread_id].hop;
#endif
                continue; 
            }

            bool exceeded = false;
            for (const auto& [f, w] : added_comm_weights) {
                if (comm_weight_sum[f] + w > fpga_info[f][0]) { 
                    exceeded = true;
                    break;
                }
            }
            if (exceeded) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                ++constraint_counts[thread_id].comm;
#endif
                continue; 
            }

            long long new_total_cost = total_cost + incremental_cost;

            if (new_total_cost >= best_cost) {
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
                ++constraint_counts[thread_id].pruned;
#endif
                continue;
            }

            std::vector<std::vector<int>> new_resource_usage = resource_usage;
            for (int i = 0; i < 8; ++i) {
                new_resource_usage[f_id][i] += hn.weights[i];
            }

            std::vector<int> new_comm_weight_sum = comm_weight_sum;
            for (const auto& [f, w] : added_comm_weights) {
                new_comm_weight_sum[f] += w;
            }

            assignment[hypernode_id] = f_id;
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            ++constraint_counts[thread_id].placed;
#endif

            backtrack(node_sorted_index + 1, assignment, new_resource_usage, new_comm_weight_sum, fpga_contained_hyperedges_new, new_total_cost, thread_id);
            
            assignment.erase(hypernode_id);

            if(flag_finished[thread_id] == when_finished) {
                return;
            }

            if(depth_exit_count[thread_id] > 0){
                --depth_exit_count[thread_id];
                return;
            }
        }
        
    }
};



void AssignmentInitialPartition(std::vector<Hypergraph>& Hypergraphs, const FPGAManager& fpga_manager, const int every_layer_time_limit_second,
                                long long& best_cost, std::map<int, int>& best_assignment,  
                                std::vector<std::vector<int>>& fpgas_resource_usage, std::vector<int>& fpgas_comm_weight_sum, int& which_layer) {
    for(int i = Hypergraphs.size()-1; i >= 0; --i) {
        const Hypergraph &hg = Hypergraphs[i];

        int node_nums = hg._num_vertices_current;
        int fpga_nums = fpga_manager.fpga_num;


#ifdef REPART_CF_OBSERVE
        cf_observe::record("assignment_begin", i, node_nums, every_layer_time_limit_second);
#endif
        AssignmentSolver ass_solver(hg, fpga_manager, every_layer_time_limit_second);
        std::pair<std::map<int, int>, long long> result = ass_solver.solve();
#ifdef REPART_CF_OBSERVE
        cf_observe::record("assignment_end", i, result.first.size(), result.second);
        cf_observe::flush();
#endif

        best_assignment = result.first;
        best_cost = result.second;

        if (!best_assignment.empty()) {
#ifdef REPART_GLOBAL_ASSIGNMENT
            // Diagnostic after all search threads join; no search-order changes.
            long long checked_cost = 0;
            for (const auto& edge : hg.hyperedges) {
                if (edge.deleted || edge._size == 0) continue;
                const int source = best_assignment.at(hg._incidence_array[edge._begin]);
                std::set<int> targets;
                for (int p = 1; p < edge._size; ++p)
                    targets.insert(best_assignment.at(hg._incidence_array[edge._begin+p]));
                for (int target : targets) {
                    const int hop = fpga_manager.hop_distances[source][target];
                    if (hop < 0) throw std::runtime_error("invalid initial Hop");
                    checked_cost += static_cast<long long>(edge.weight) * hop;
                }
            }
            if (checked_cost != best_cost) throw std::runtime_error("initial cost mismatch");
            std::cout << "Initial Assignment Cost: " << best_cost
                      << " checked=" << checked_cost << " layer=" << i
                      << " nodes=" << node_nums << std::endl;
#endif
            fpgas_resource_usage = ass_solver.fpgas_resource_usage;
            fpgas_comm_weight_sum = ass_solver.fpgas_comm_weight_sum;
            which_layer = i;
            break;
        }
    }
    return;
}
