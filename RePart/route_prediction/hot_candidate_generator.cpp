#include "hot_candidate_generator.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>

namespace route_prediction_b4 {
using namespace route_prediction_v1;
namespace {
bool has(const std::vector<int>&x,int v){return std::find(x.begin(),x.end(),v)!=x.end();}
int avoiding(int s,int t,int removed,const FpgaTopology&g){std::vector<int>d(g.fpgaCount(),-1);std::queue<int>q;d[s]=0;q.push(s);while(!q.empty()){int u=q.front();q.pop();for(int v:g.adjacency(u)){if(g.linkId(u,v)==removed||d[v]>=0)continue;d[v]=d[u]+1;q.push(v);}}return d[t];}
}
CandidateGenerationResult generateCandidates(const CaseData& data,const FpgaTopology& topology,const LocalMoveEvaluator&evaluator,int hot){
    CandidateGenerationResult r;auto started=std::chrono::steady_clock::now();const auto& routed=evaluator.baselineRouted();
    for(const auto&c:routed){if(!has(c.physical_link_ids,hot))continue;int alt=avoiding(c.connection.source_fpga,c.connection.destination_fpga,hot,topology);int original=c.physical_link_ids.size();if(alt<0||alt>original){r.mandatory_connection_ids.push_back(c.connection.id);r.mandatory_net_ids.insert(c.connection.net_id);const auto&n=data.nets[c.connection.net_id];r.source_nodes.insert(n.source_node);r.structure_nodes.insert(n.source_node);for(int sink:n.sink_nodes){const auto&p=data.placements[sink];if(p.master_fpga==c.connection.destination_fpga||has(p.replica_fpgas,c.connection.destination_fpga)){r.effective_sink_nodes.insert(sink);r.structure_nodes.insert(sink);}}}}
    r.structure_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();auto&f=r.funnel;f.mandatory_connections=r.mandatory_connection_ids.size();f.unique_nets=r.mandatory_net_ids.size();f.source_nodes=r.source_nodes.size();f.effective_sink_nodes=r.effective_sink_nodes.size();f.structure_nodes=r.structure_nodes.size();f.raw_operations=static_cast<std::int64_t>(f.structure_nodes)*topology.fpgaCount();started=std::chrono::steady_clock::now();EvaluationTiming timing;
    for(int node:r.structure_nodes){const auto&p=data.placements[node];for(int target=0;target<topology.fpgaCount();++target){if(target==p.master_fpga){++f.excluded_master;continue;}if(has(p.replica_fpgas,target)){++f.excluded_replica;continue;}auto e=evaluator.evaluate(node,target,&timing);r.operations.push_back(e);if(e.classification==CandidateClass::ResourceRejected){++f.resource_rejected;continue;}++f.simulated;if(e.classification==CandidateClass::HopRejected){++f.hop_rejected;continue;}if(e.classification==CandidateClass::CommunicationRejected){++f.communication_rejected;continue;}++f.hard_legal;if(e.classification==CandidateClass::HotLinkNeutral)++f.neutral;else if(e.classification==CandidateClass::HotLinkWorsening)++f.worsening;else{++f.effective;if(e.max_load_after<evaluator.baselineMetrics().max_link_load)++f.lowers_max;if(e.max_load_after==evaluator.baselineMetrics().max_link_load&&e.delta_squared_load<0)++f.max_same_squared_lower;if(e.max_load_after>evaluator.baselineMetrics().max_link_load)++f.larger_new_hotspot;if(e.delta_weighted_thd<=0)++f.weighted_thd_nonpositive;}}}
    r.raw_enumeration_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();r.resource_prefilter_seconds=timing.resource_seconds;r.local_projection_seconds=timing.projection_and_routing_seconds;r.legality_seconds=timing.legality_and_metrics_seconds;
    std::cout<<"b4_phase_times structure="<<r.structure_seconds<<" raw_enumeration="<<r.raw_enumeration_seconds<<" resource_prefilter="<<r.resource_prefilter_seconds<<" local_projection_routing="<<r.local_projection_seconds<<" legality_metrics="<<r.legality_seconds<<"\n";
    return r;
}
} // namespace route_prediction_b4
