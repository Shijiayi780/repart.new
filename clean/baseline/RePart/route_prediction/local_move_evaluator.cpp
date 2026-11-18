#include "local_move_evaluator.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

#include "shortest_path_predictor.h"

namespace route_prediction_b4 {
using namespace route_prediction_v1;
namespace {
using Key = std::tuple<int,int,int>;
Key key(const RoutedConnection& r) { return {r.connection.net_id,r.connection.source_fpga,r.connection.destination_fpga}; }
bool contains(const std::vector<int>& values,int x){return std::find(values.begin(),values.end(),x)!=values.end();}
}

const char* candidateClassName(CandidateClass value) {
    switch(value){case CandidateClass::ResourceRejected:return "ResourceRejected";case CandidateClass::HopRejected:return "MaximumHopRejected";case CandidateClass::CommunicationRejected:return "CommunicationRejected";case CandidateClass::HotLinkNeutral:return "StructurallyRelatedButHotLinkNeutral";case CandidateClass::HotLinkWorsening:return "HotLinkWorsening";case CandidateClass::HotLinkEffective:return "HotLinkEffective";} return "Unknown";
}

LocalMoveEvaluator::LocalMoveEvaluator(const CaseData& data,const FpgaTopology& topology,
                                       ConstraintData constraints,int hot_link_id)
    :data_(data),topology_(topology),constraints_(std::move(constraints)),hot_link_id_(hot_link_id) {
    if(constraints_.node_resources.size()!=data_.placements.size()||constraints_.fpga_capacities.size()!=static_cast<size_t>(topology_.fpgaCount()))throw std::runtime_error("constraint dimensions mismatch");
    incident_nets_.resize(data_.placements.size());
    for(const auto& n:data_.nets){incident_nets_[n.source_node].push_back(n.id);for(int s:n.sink_nodes)incident_nets_[s].push_back(n.id);}
    for(auto& x:incident_nets_){std::sort(x.begin(),x.end());x.erase(std::unique(x.begin(),x.end()),x.end());}
    resource_usage_.assign(topology_.fpgaCount(),{});
    for(int node=0;node<static_cast<int>(data_.placements.size());++node){const auto&p=data_.placements[node];for(int f:p.replica_fpgas)for(int d=0;d<8;++d)resource_usage_[f][d]+=constraints_.node_resources[node][d];for(int d=0;d<8;++d)resource_usage_[p.master_fpga][d]+=constraints_.node_resources[node][d];}
    communication_usage_=communicationUsage(data_.placements);
    const auto all=ConnectionProjector::projectV2(data_.nets,data_.placements);
    baseline_routed_=ShortestPathPredictor::route(all,topology_);baseline_metrics_=MetricsCalculator::calculate(baseline_routed_,topology_);
}

std::vector<RoutedConnection> LocalMoveEvaluator::routeNets(const std::vector<int>& ids,const std::vector<NodePlacement>& placements)const{
    std::vector<Net> nets;for(int id:ids)nets.push_back(data_.nets.at(id));auto c=ConnectionProjector::projectV2(nets,placements);return ShortestPathPredictor::route(c,topology_);
}

std::vector<std::int64_t> LocalMoveEvaluator::communicationUsage(const std::vector<NodePlacement>& placements)const{
    std::vector<std::int64_t> usage(topology_.fpgaCount(),0);
    for(const auto& n:data_.nets){std::set<int> participants;const auto&s=placements[n.source_node];participants.insert(s.master_fpga);participants.insert(s.replica_fpgas.begin(),s.replica_fpgas.end());for(int node:n.sink_nodes){const auto&p=placements[node];participants.insert(p.master_fpga);participants.insert(p.replica_fpgas.begin(),p.replica_fpgas.end());}if(participants.size()>1)for(int f:participants)usage[f]+=n.weight;}
    return usage;
}

MoveEvaluation LocalMoveEvaluator::summarize(int node,int target,const std::vector<RoutedConnection>& before,const std::vector<RoutedConnection>& after,const std::vector<std::int64_t>& comm_after,bool full)const{
    MoveEvaluation e;e.node=node;e.source_fpga=data_.placements[node].master_fpga;e.target_fpga=target;e.resource_legal=true;e.hop_legal=true;e.communication_legal=true;
    for(int d=0;d<8;++d)if(resource_usage_[target][d]+constraints_.node_resources[node][d]>constraints_.fpga_capacities[target][d])e.resource_legal=false;
    if(!e.resource_legal){e.classification=CandidateClass::ResourceRejected;return e;}
    for(const auto&r:after)if(!r.reachable)e.hop_legal=false;
    if(!e.hop_legal){e.classification=CandidateClass::HopRejected;return e;}
    for(int f=0;f<topology_.fpgaCount();++f)if(comm_after[f]>constraints_.fpga_comm_capacities[f])e.communication_legal=false;
    if(!e.communication_legal){e.classification=CandidateClass::CommunicationRejected;return e;}
    std::vector<std::int64_t> loads=baseline_metrics_.link_loads;
    std::int64_t du=0,dw=0;for(const auto&r:before){int h=r.physical_link_ids.size();du-=h;dw-=h*r.connection.net_weight;for(int l:r.physical_link_ids)--loads[l];}for(const auto&r:after){int h=r.physical_link_ids.size();du+=h;dw+=h*r.connection.net_weight;for(int l:r.physical_link_ids)++loads[l];}
    e.link_loads_after=loads;e.delta_hot_load=loads[hot_link_id_]-baseline_metrics_.link_loads[hot_link_id_];e.hot_load_after=loads[hot_link_id_];e.unweighted_thd_after=baseline_metrics_.unweighted_thd+du;e.delta_unweighted_thd=du;e.weighted_thd_after=baseline_metrics_.weighted_thd+dw;e.delta_weighted_thd=dw;
    for(auto l:loads){e.max_load_after=std::max(e.max_load_after,l);e.squared_load_after+=static_cast<long double>(l)*l;}e.delta_max_load=e.max_load_after-baseline_metrics_.max_link_load;e.delta_squared_load=e.squared_load_after-baseline_metrics_.squared_link_load_sum;for(int l=0;l<static_cast<int>(loads.size());++l){if(loads[l]==e.max_load_after)e.hot_links_after.push_back(l);if(loads[l]>loads[hot_link_id_])++e.link9_rank_after;}++e.link9_rank_after;
    std::map<Key,const RoutedConnection*> bm,am;for(const auto&r:before)bm[key(r)]=&r;for(const auto&r:after)am[key(r)]=&r;for(const auto&[k,r]:bm)if(!am.count(k))++e.deleted_connections;for(const auto&[k,r]:am)if(!bm.count(k))++e.added_connections;std::set<Key> keys;for(auto&x:bm)keys.insert(x.first);for(auto&x:am)keys.insert(x.first);for(const auto&k:keys)if(!bm.count(k)||!am.count(k)||bm[k]->physical_link_ids!=am[k]->physical_link_ids)++e.changed_connections;
    e.maximum_hop_after=0;
    if(full){for(const auto&r:after)e.maximum_hop_after=std::max(e.maximum_hop_after,static_cast<int>(r.physical_link_ids.size()));}
    else{std::set<int> affected;for(const auto&r:before)affected.insert(r.connection.net_id);for(const auto&r:baseline_routed_)if(!affected.count(r.connection.net_id))e.maximum_hop_after=std::max(e.maximum_hop_after,static_cast<int>(r.physical_link_ids.size()));for(const auto&r:after)e.maximum_hop_after=std::max(e.maximum_hop_after,static_cast<int>(r.physical_link_ids.size()));}
    e.classification=e.delta_hot_load<0?CandidateClass::HotLinkEffective:(e.delta_hot_load==0?CandidateClass::HotLinkNeutral:CandidateClass::HotLinkWorsening);return e;
}

bool LocalMoveEvaluator::resourceLegal(int node,int target)const{for(int d=0;d<8;++d)if(resource_usage_[target][d]+constraints_.node_resources[node][d]>constraints_.fpga_capacities[target][d])return false;return true;}

MoveEvaluation LocalMoveEvaluator::evaluate(int node,int target,EvaluationTiming* timing)const{
    auto tick=std::chrono::steady_clock::now();
    if(!resourceLegal(node,target)){MoveEvaluation e;e.node=node;e.source_fpga=data_.placements[node].master_fpga;e.target_fpga=target;e.classification=CandidateClass::ResourceRejected;if(timing)timing->resource_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();return e;}
    if(timing)timing->resource_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();tick=std::chrono::steady_clock::now();
    auto placements=data_.placements;placements[node].master_fpga=target;const auto before=routeNets(incident_nets_[node],data_.placements);const auto after=routeNets(incident_nets_[node],placements);
    auto comm=communication_usage_; // incident-net exact delta
    auto netComm=[&](int id,const std::vector<NodePlacement>&p){std::vector<std::int64_t>u(topology_.fpgaCount(),0);const auto&n=data_.nets[id];std::set<int>x;const auto&s=p[n.source_node];x.insert(s.master_fpga);x.insert(s.replica_fpgas.begin(),s.replica_fpgas.end());for(int v:n.sink_nodes){const auto&q=p[v];x.insert(q.master_fpga);x.insert(q.replica_fpgas.begin(),q.replica_fpgas.end());}if(x.size()>1)for(int f:x)u[f]=n.weight;return u;};
    for(int id:incident_nets_[node]){auto b=netComm(id,data_.placements),a=netComm(id,placements);for(int f=0;f<topology_.fpgaCount();++f)comm[f]+=a[f]-b[f];}
    if(timing)timing->projection_and_routing_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();tick=std::chrono::steady_clock::now();auto result=summarize(node,target,before,after,comm,false);if(timing)timing->legality_and_metrics_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();return result;
}

MoveEvaluation LocalMoveEvaluator::evaluateFull(int node,int target)const{
    auto placements=data_.placements;placements[node].master_fpga=target;auto before=baseline_routed_;auto all=ConnectionProjector::projectV2(data_.nets,placements);auto after=ShortestPathPredictor::route(all,topology_);return summarize(node,target,before,after,communicationUsage(placements),true);
}

bool LocalMoveEvaluator::equivalentToFull(const MoveEvaluation&a,const MoveEvaluation&b)const{
    return a.classification==b.classification&&a.link_loads_after==b.link_loads_after&&a.hot_load_after==b.hot_load_after&&a.max_load_after==b.max_load_after&&a.squared_load_after==b.squared_load_after&&a.unweighted_thd_after==b.unweighted_thd_after&&a.weighted_thd_after==b.weighted_thd_after&&a.maximum_hop_after==b.maximum_hop_after&&a.delta_hot_load==b.delta_hot_load;
}
} // namespace route_prediction_b4
