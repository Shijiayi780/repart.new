#include <iostream>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <time.h>
#include <chrono>
#include <unistd.h>
#include <getopt.h>
#include <cmath>
#include <cstdlib>

#include "./datastructure/hypergraph.h"
#include "./io/hypergraph_readin.h"
#include "./io/fpga_manager.h"

#include "./coarsening/full_coarsener_4thread.h"

#include "./assignment/assignment_4thread_nodes.h"

#include "./refinement_replicate/replication_fast_4thread.h"
#include "./refinement_delete/delete_fast_4thread.h"
#include "./refinement_move/move_fast.h"
#include "./refinement_exchange/exchanger.h"
#ifdef REPART_EXCHANGE_ACTION_PAIRS
#include "./refinement_exchange/action_pair_heap.h"
#endif
#include "./link_load/link_load_refiner.h"
#ifdef REPART_UNIFIED_SEQUENCE
#include "unified_native_sequence.h"
#endif
#ifdef REPART_REFINEMENT_SNAPSHOT
#ifdef REPART_NATIVE_MOVE_PROBE
#include "refinement_native_move_probe.h"
#endif
#include "refinement_snapshot.h"
#endif



int main(int argc, char* argv[]) {

    auto start = std::chrono::steady_clock::now();

    std::unordered_map<std::string,int> vertex_name_to_id;
    std::unordered_map<int,std::string> vertex_id_to_name;

    Hypergraph hypergraph_init;

    std::string inputDirectory;
    std::string outputFile;
    link_load_v1::Options link_load_options;
#ifdef REPART_STATIC_COMMUNITY
    cf_bridge::StaticCommunityRun community_run;
    std::string community_log_path;
    std::ofstream community_log;
    bool static_community_requested = false;
    bool cut_only_flow_requested = false;
    bool external_flow_requested = false;
    bool external_flow_cut_ceiling_requested = false;
#endif

    int opt;
    enum {
#ifdef REPART_STATIC_COMMUNITY
        OPT_STATIC_COMMUNITY = 2000,
        OPT_CUT_ONLY_FLOW,
        OPT_EXTERNAL_FLOW,
        OPT_EXTERNAL_FLOW_CUT_CEILING,
        OPT_COMMUNITY_LOG,
#endif
        OPT_LINK_LOAD_REFINE = 1000,
        OPT_THD_DEGRADATION_LIMIT,
        OPT_HOT_LINK_THRESHOLD,
        OPT_LINK_LOAD_MAX_ITERS,
        OPT_LINK_LOAD_REPORT,
        OPT_LINK_CAPACITY_FILE
    };
    static option long_options[] = {
#ifdef REPART_STATIC_COMMUNITY
        {"static-community", no_argument, nullptr, OPT_STATIC_COMMUNITY},
        {"cut-only-flow", no_argument, nullptr, OPT_CUT_ONLY_FLOW},
        {"external-flow", no_argument, nullptr, OPT_EXTERNAL_FLOW},
        {"external-flow-cut-ceiling", no_argument, nullptr,
         OPT_EXTERNAL_FLOW_CUT_CEILING},
        {"community-log", required_argument, nullptr, OPT_COMMUNITY_LOG},
#endif
        {"link-load-refine", no_argument, nullptr, OPT_LINK_LOAD_REFINE},
        {"thd-degradation-limit", required_argument, nullptr, OPT_THD_DEGRADATION_LIMIT},
        {"hot-link-threshold", required_argument, nullptr, OPT_HOT_LINK_THRESHOLD},
        {"link-load-max-iters", required_argument, nullptr, OPT_LINK_LOAD_MAX_ITERS},
        {"link-load-report", required_argument, nullptr, OPT_LINK_LOAD_REPORT},
        {"link-capacity-file", required_argument, nullptr, OPT_LINK_CAPACITY_FILE},
        {nullptr, 0, nullptr, 0}
    };
    while ((opt = getopt_long(argc, argv, "t:s:", long_options, nullptr)) != -1) {
        switch (opt) {
#ifdef REPART_STATIC_COMMUNITY
            case OPT_STATIC_COMMUNITY:
                static_community_requested = true;
                break;
            case OPT_CUT_ONLY_FLOW:
                cut_only_flow_requested = true;
                break;
            case OPT_EXTERNAL_FLOW:
                external_flow_requested = true;
                break;
            case OPT_EXTERNAL_FLOW_CUT_CEILING:
                external_flow_cut_ceiling_requested = true;
                break;
            case OPT_COMMUNITY_LOG:
                community_log_path = optarg;
                break;
#endif
            case 't':
                inputDirectory = optarg;
                break;
            case 's':
                outputFile = optarg;
                break;
            case OPT_LINK_LOAD_REFINE:
                link_load_options.enabled = true;
                break;
            case OPT_THD_DEGRADATION_LIMIT:
                link_load_options.thd_degradation_limit = std::stod(optarg);
                break;
            case OPT_HOT_LINK_THRESHOLD:
                link_load_options.hot_link_threshold = std::stod(optarg);
                break;
            case OPT_LINK_LOAD_MAX_ITERS:
                link_load_options.max_iters = std::stoi(optarg);
                break;
            case OPT_LINK_LOAD_REPORT:
                link_load_options.report_path = optarg;
                break;
            case OPT_LINK_CAPACITY_FILE:
                link_load_options.capacity_file = optarg;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " -t <input_directory> -s <output_file>" << std::endl;
                return 1;
        }
    }

    if (inputDirectory.empty() || outputFile.empty()) {
        std::cerr << "Both -t and -s options are required." << std::endl;
        std::cerr << "Usage: " << argv[0] << " -t <input_directory> -s <output_file>" << std::endl;
        return 1;
    }
    if (link_load_options.thd_degradation_limit < 0.0 ||
        link_load_options.hot_link_threshold < 0.0 || link_load_options.hot_link_threshold > 1.0 ||
        link_load_options.max_iters < 0) {
        std::cerr << "Invalid link-load refinement options." << std::endl;
        return 1;
    }

#ifdef REPART_STATIC_COMMUNITY
    const int community_modes_requested =
        static_cast<int>(static_community_requested) +
        static_cast<int>(cut_only_flow_requested) +
        static_cast<int>(external_flow_requested) +
        static_cast<int>(external_flow_cut_ceiling_requested);
    if (community_modes_requested > 1) {
        std::cerr << "--static-community, --cut-only-flow, --external-flow, and "
                     "--external-flow-cut-ceiling are mutually exclusive."
                  << std::endl;
        return 1;
    }
    if (static_community_requested) community_run.enableStaticOnly();
    if (cut_only_flow_requested) community_run.enableCutOnlyFlow();
    if (external_flow_requested) community_run.enableExternalFlow();
    if (external_flow_cut_ceiling_requested)
        community_run.enableExternalFlowCutCeiling();
    if (community_run.enabled) {
        if (community_log_path.empty()) community_log_path = outputFile + ".community.tsv";
        community_log.open(community_log_path);
        if (!community_log) {
            std::cerr << "Cannot open community log: " << community_log_path << std::endl;
            return 1;
        }
        community_log << std::setprecision(17);
        community_run.log = &community_log;
        if (community_run.externalFlowCutCeilingEnabled())
            community_run.record("configuration", "E", "INITIAL_ONCE", "threads", thread_num,
                "initial_flow", 1, "scheduled_flow", 0, "recovery_flow", 0,
                "lambda_ext", 1, "distance", "1-cosine",
                "capacity_scale", 1000000, "cut_ceiling", "ENTRY_RAW_CUT",
                "commit", "ATOMIC_SWEEP");
        else if (community_run.externalFlowEnabled())
            community_run.record("configuration", "D", "INITIAL_ONCE", "threads", thread_num,
                "scheduled_flow", 1, "rho", "3/4", "Bmax", 3,
                "recovery_flow", 1, "lambda_ext", 1,
                "distance", "1-cosine", "capacity_scale", 1000000);
        else if (community_run.cutOnlyFlowEnabled())
            community_run.record("configuration", "C", "INITIAL_ONCE", "threads", thread_num,
                "scheduled_flow", 1, "rho", "3/4", "Bmax", 3,
                "recovery_flow", 0);
        else
            community_run.record("configuration", "B", "INITIAL_ONCE", "threads", thread_num,
                "timed_flow", 0, "recovery_flow", 0);
    }
#endif
    std::cout << "Input Directory: " << inputDirectory << std::endl;
    std::cout << "Output File: " << outputFile << std::endl;

    std::string are_file = inputDirectory + "/design.are";
    std::string net_file = inputDirectory + "/design.net";
    std::string info_filepath = inputDirectory + "/design.info";
    std::string topo_filepath = inputDirectory + "/design.topo";

    FPGAManager fpga_manager;
    fpga_manager.init(info_filepath, topo_filepath);


    read_in(are_file, net_file, vertex_name_to_id, vertex_id_to_name, hypergraph_init, fpga_manager);
#ifdef REPART_EFFECTIVE_RESISTANCE
    if (const char* er_file = std::getenv("REPART_ER_FILE")) {
        repart_er::initialize(er_file, hypergraph_init.hyperedges.size());
        std::cout << "ER scoring enabled: floating weight/(ER-1), original resource penalty\n";
    }
#endif
    std::cout << "Hypergraph initialized\n";
    long long best_cost = std::numeric_limits<long long>::max();

    if(fpga_manager.fpga_num <= 32 && hypergraph_init.hypernodes.size() <= 20000){
        FullCoarsener coarsener(hypergraph_init, fpga_manager
#ifdef REPART_STATIC_COMMUNITY
            , &community_run
#endif
        );
        std::vector<Hypergraph>& Hypergraphs_initial = coarsener.fullCoarsener(2*fpga_manager.fpga_num);
        coarsener.printCoarsenSize();
        std::cout << "Coarsening finished\n";

        int explore_num = 5;
        std::cout << "Explore " << explore_num << " times\n";

        Hypergraph hypergraph_best_overall;


        while(explore_num--){
            long long best_cost_in_this_exploration = std::numeric_limits<long long>::max();

            std::vector<Hypergraph> Hypergraphs = Hypergraphs_initial;

            int every_layer_time_limit_second = 8+explore_num;  

            int which_layer;

            std::map<int, int> best_assignment;   
            std::vector<std::vector<int>> fpgas_resource_usage; 
            std::vector<int> fpgas_comm_weight_sum; 

#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            const auto assignment_phase_start=std::chrono::steady_clock::now();
#endif
            AssignmentInitialPartition(Hypergraphs, fpga_manager, every_layer_time_limit_second, best_cost_in_this_exploration, best_assignment, fpgas_resource_usage, fpgas_comm_weight_sum, which_layer);
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            const auto assignment_phase_end=std::chrono::steady_clock::now();
            std::cout << "phase_time exploration=" << 4-explore_num << " assignment_seconds="
                      << std::chrono::duration<double>(assignment_phase_end-assignment_phase_start).count() << '\n';
#endif
#ifdef REPART_STATIC_COMMUNITY
            if (community_run.enabled) community_run.record("assignment", 4-explore_num,
                !best_assignment.empty(), best_assignment.empty() ? -1 : which_layer,
                Hypergraphs.back()._num_vertices_current, every_layer_time_limit_second);
#endif

            std::cout << "Assignment Initial Partition Finished\n";
#ifdef REPART_MERGE_OPPORTUNITY
            merge_probe::assignment(4-explore_num, !best_assignment.empty(),
                best_assignment.empty() ? -1 : which_layer,
                best_cost_in_this_exploration);
#endif

            if(best_assignment.empty()){
                continue;
            }


            updateLastLayer(Hypergraphs, which_layer, best_assignment);


            bool replication_flag = false;
            int delete1_threshold = 1;
            int delete2_threshold = 0;

            for(int i = which_layer; i >= 0; --i) {
                Hypergraphs[i].initPinsInPart();

#ifdef REPART_UNIFIED_SEQUENCE
                if(i==which_layer && std::getenv("REPART_RUN_UNIFIED_SEQUENCE")) {
                    unified_native_sequence::refine(Hypergraphs[i],fpgas_resource_usage,fpgas_comm_weight_sum,best_cost_in_this_exploration);
                    for(const auto& n:Hypergraphs[i].hypernodes)if(!n.deleted&&!n.replication_fpga_labels.empty())replication_flag=true;
                } else {
#endif

                if(!replication_flag && (which_layer - i) < 3){
#ifdef REPART_EXCHANGE_ACTION_PAIRS
                    if((i==which_layer || std::getenv("REPART_EXPAIR_FINER")) && std::getenv("REPART_RUN_EXCHANGE_PAIRS")) {
                        std::cout<<"EXPAIR_LAYER layer="<<i<<" nodes="<<Hypergraphs[i]._num_vertices_current<<std::endl;
                        best_cost_in_this_exploration-=exchange_action_pairs::refine(Hypergraphs[i],fpgas_resource_usage,fpgas_comm_weight_sum);
                    } else {
#endif
                    Exchanger exchanger(Hypergraphs[i], fpgas_resource_usage, fpgas_comm_weight_sum);
                    int exchange_gain = exchanger.exchange();
                    best_cost_in_this_exploration -= exchange_gain;
#ifdef REPART_EXCHANGE_ACTION_PAIRS
                    }
#endif
                }

                if(replication_flag){
                    Delete deleter1(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost_in_this_exploration);
                    deleter1.Solve(delete1_threshold);
                }

                {
                    Refiner move(Hypergraphs[i], fpgas_resource_usage, fpgas_comm_weight_sum);
                    int move_gain = move.refine();
                    best_cost_in_this_exploration -= move_gain;
                }

                if(replication_flag){
                    Delete deleter2(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost_in_this_exploration);
                    deleter2.Solve(delete2_threshold);
                }

                {
                    int gain_threshold = (best_cost_in_this_exploration / Hypergraphs[i]._num_vertices_current) > 1 ? 1 : (best_cost_in_this_exploration / Hypergraphs[i]._num_vertices_current);
                    if(i == 0){
                        gain_threshold = 1;
                    }

                    Replication replication(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost_in_this_exploration);
                    replication.Solve(gain_threshold, replication_flag);
                }


#ifdef REPART_UNIFIED_SEQUENCE
                }
#endif
#ifdef REPART_REFINEMENT_SNAPSHOT
                refinement_snapshot::observe(Hypergraphs[i], i, best_cost_in_this_exploration,
                    fpgas_resource_usage, fpgas_comm_weight_sum);
#endif
                if(i > 0){
                    updataFormerLayer(Hypergraphs, i);
                }
            }


            
            int replication_threshold = 1;

            while(1){
                auto now_time = std::chrono::steady_clock::now();
                if(now_time - start > std::chrono::minutes(50)){
                    break;
                }

                long long new_best_cost = best_cost_in_this_exploration;

                {
                    Refiner move(Hypergraphs[0], fpgas_resource_usage, fpgas_comm_weight_sum);
                    int move_gain = move.refine();
                    new_best_cost -= move_gain;
                }

                {
                    Delete deleter(Hypergraphs[0], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, new_best_cost);
                    deleter.Solve(0);
                }

                {
                    Replication replication(Hypergraphs[0], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, new_best_cost);
                    replication.Solve(replication_threshold, replication_flag);
                }

                if(new_best_cost == best_cost_in_this_exploration){
                    best_cost_in_this_exploration = new_best_cost;
                    break;
                }
                best_cost_in_this_exploration = new_best_cost;
            }
            std::cout << "Refinement Finished\n";
#ifdef REPART_CONSTRAINT_DIAGNOSTICS
            std::cout << "phase_time exploration=" << 4-explore_num << " refinement_seconds="
                      << std::chrono::duration<double>(std::chrono::steady_clock::now()-assignment_phase_end).count() << '\n';
#endif

            std::cout << "Best Cost Temp: " << best_cost_in_this_exploration << std::endl;
#ifdef REPART_MERGE_OPPORTUNITY
            merge_probe::final_cost(4-explore_num, best_cost_in_this_exploration);
#endif

            if(best_cost_in_this_exploration < best_cost) {
                best_cost = best_cost_in_this_exploration;
                hypergraph_best_overall = Hypergraphs[0];
            }

            auto now_time = std::chrono::steady_clock::now();
            if(now_time - start > std::chrono::minutes(5)){
                break;
            }
        }


        if(link_load_options.enabled) {
            link_load_v1::Refiner link_load_refiner(hypergraph_best_overall, link_load_options, best_cost);
            const auto link_load_result = link_load_refiner.run();
            best_cost = link_load_result.after.thd;
        }

        std::vector<std::queue<int>> fpga_include_nodes(fpga_manager.fpga_num);

        for(auto& node : hypergraph_best_overall.hypernodes) {
            fpga_include_nodes[node.label].push(node.id);
            for(const auto& f: node.replication_fpga_labels) {
                fpga_include_nodes[f].push(-node.id-1);
            }
        }

        std::ofstream file(outputFile);
        for(int i=0;i<fpga_manager.fpga_num;++i) {
            file << fpga_manager.getName(i) << ": ";
            while(!fpga_include_nodes[i].empty()) {
                if(fpga_include_nodes[i].front() < 0) {
                    file << vertex_id_to_name[-fpga_include_nodes[i].front()-1] << "* ";
                } else {
                    file << vertex_id_to_name[fpga_include_nodes[i].front()] << " ";
                }
                fpga_include_nodes[i].pop();
            }
            file << std::endl;
        }

    }
    else {

        FullCoarsener coarsener(hypergraph_init, fpga_manager
#ifdef REPART_STATIC_COMMUNITY
            , &community_run
#endif
        );
        std::vector<Hypergraph>& Hypergraphs = coarsener.fullCoarsener(2*fpga_manager.fpga_num);
        coarsener.printCoarsenSize();
        std::cout << "Coarsening finished\n";

        

        std::map<int, int> best_assignment;   
        std::vector<std::vector<int>> fpgas_resource_usage; 
        std::vector<int> fpgas_comm_weight_sum; 

        int every_layer_time_limit_second = 60;  

        int which_layer;

        AssignmentInitialPartition(Hypergraphs, fpga_manager, every_layer_time_limit_second, best_cost, best_assignment, fpgas_resource_usage, fpgas_comm_weight_sum, which_layer);
#ifdef REPART_STATIC_COMMUNITY
        if (community_run.enabled) community_run.record("assignment", 0,
            !best_assignment.empty(), best_assignment.empty() ? -1 : which_layer,
            Hypergraphs.back()._num_vertices_current, every_layer_time_limit_second);
#endif

        std::cout << "Assignment Initial Partition Finished\n";

        updateLastLayer(Hypergraphs, which_layer, best_assignment);

        bool replication_flag = false;
        int delete1_threshold = 1;
        int delete2_threshold = 0;

        for(int i = which_layer; i >= 0; --i) {
            Hypergraphs[i].initPinsInPart();

            if(!replication_flag && (which_layer - i) < 3){
                Exchanger exchanger(Hypergraphs[i], fpgas_resource_usage, fpgas_comm_weight_sum);
                int exchange_gain = exchanger.exchange();
                best_cost -= exchange_gain;
            }

            if(replication_flag){
                Delete deleter1(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost);
                deleter1.Solve(delete1_threshold);
            }

            {
                Refiner move(Hypergraphs[i], fpgas_resource_usage, fpgas_comm_weight_sum);
                int move_gain = move.refine();
                best_cost -= move_gain;
            }

            if(replication_flag){
                Delete deleter2(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost);
                deleter2.Solve(delete2_threshold);
            }

            {
                int gain_threshold = (best_cost / Hypergraphs[i]._num_vertices_current) > 1 ? 1 : (best_cost / Hypergraphs[i]._num_vertices_current);
                if(i == 0){
                    gain_threshold = 1;
                }

                Replication replication(Hypergraphs[i], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, best_cost);
                replication.Solve(gain_threshold, replication_flag);
            }


            if(i > 0){
                updataFormerLayer(Hypergraphs, i);
            }
        }

        
        int replication_threshold = 1;

        while(1){
            auto now_time = std::chrono::steady_clock::now();
            if(now_time - start > std::chrono::minutes(50)){
                break;
            }

            long long new_best_cost = best_cost;

            {
                Refiner move(Hypergraphs[0], fpgas_resource_usage, fpgas_comm_weight_sum);
                int move_gain = move.refine();
                new_best_cost -= move_gain;
            }

            {
                Delete deleter(Hypergraphs[0], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, new_best_cost);
                deleter.Solve(0);
            }

            {
                Replication replication(Hypergraphs[0], fpga_manager, fpgas_resource_usage, fpgas_comm_weight_sum, new_best_cost);
                replication.Solve(replication_threshold, replication_flag);
            }

            if(new_best_cost == best_cost){
                best_cost = new_best_cost;
                break;
            }
            best_cost = new_best_cost;
        }
        std::cout << "Refinement Finished\n";

        if(link_load_options.enabled) {
            link_load_v1::Refiner link_load_refiner(Hypergraphs[0], link_load_options, best_cost);
            const auto link_load_result = link_load_refiner.run();
            best_cost = link_load_result.after.thd;
        }

        std::vector<std::queue<int>> fpga_include_nodes(fpga_manager.fpga_num);

        for(auto& node : Hypergraphs[0].hypernodes) {
            fpga_include_nodes[node.label].push(node.id);
            for(const auto& f: node.replication_fpga_labels) {
                fpga_include_nodes[f].push(-node.id-1);
            }
        }

        std::ofstream file(outputFile);
        for(int i=0;i<fpga_manager.fpga_num;++i) {
            file << fpga_manager.getName(i) << ": ";
            while(!fpga_include_nodes[i].empty()) {
                if(fpga_include_nodes[i].front() < 0) {
                    file << vertex_id_to_name[-fpga_include_nodes[i].front()-1] << "* ";
                } else {
                    file << vertex_id_to_name[fpga_include_nodes[i].front()] << " ";
                }
                fpga_include_nodes[i].pop();
            }
            file << std::endl;
        }
    }

    auto end_time = std::chrono::steady_clock::now();

    if(link_load_options.enabled && !link_load_options.report_path.empty()) {
        std::ofstream report_append(link_load_options.report_path, std::ios::app);
        report_append << "total_runtime_seconds="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start).count() / 1000.0
                      << std::endl;
    }

    std::cout << "Total Time: " << std::chrono::duration_cast<std::chrono::minutes>(end_time - start).count() << "min = " <<
    std::chrono::duration_cast<std::chrono::seconds>(end_time - start).count() << "s = " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start).count() << "ms" << std::endl;

    std::cout << "Best Cost (Total Hop Distance Cost): " << best_cost << std::endl;

    return 0;
}
