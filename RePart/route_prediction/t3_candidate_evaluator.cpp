#include "t3_candidate_evaluator.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace route_prediction_v1 {
namespace {

ConnectionKey keyOf(const Connection& c) {
    return {c.net_id, c.source_fpga, c.destination_fpga};
}

std::int64_t addChecked(std::int64_t a, std::int64_t b,
                        const char* label) {
    const __int128 value = static_cast<__int128>(a) + b;
    if (value > std::numeric_limits<std::int64_t>::max()
        || value < std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error(label);
    }
    return static_cast<std::int64_t>(value);
}

std::int64_t multiplyChecked(std::int64_t a, std::int64_t b,
                             const char* label) {
    const __int128 value = static_cast<__int128>(a) * b;
    if (value > std::numeric_limits<std::int64_t>::max()
        || value < std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error(label);
    }
    return static_cast<std::int64_t>(value);
}

std::int64_t subtractChecked(std::int64_t a, std::int64_t b,
                             const char* label) {
    const __int128 value = static_cast<__int128>(a) - b;
    if (value > std::numeric_limits<std::int64_t>::max()
        || value < std::numeric_limits<std::int64_t>::min()) {
        throw std::overflow_error(label);
    }
    return static_cast<std::int64_t>(value);
}

T3LoadMetrics scanMetrics(const std::vector<std::int64_t>& loads) {
    T3LoadMetrics result;
    for (std::int64_t load : loads) {
        if (load < 0) throw std::logic_error("negative T3 load component");
        result.maximum = std::max(result.maximum, load);
        result.sum = addChecked(result.sum, load, "T3 load sum overflow");
        result.squared_sum = addChecked(
            result.squared_sum, multiplyChecked(load, load,
                                                "T3 load square overflow"),
            "T3 squared load sum overflow");
    }
    if (result.sum > 0) {
        const long double sum = static_cast<long double>(result.sum);
        result.normalized_concentration =
            static_cast<long double>(result.squared_sum) / (sum * sum);
    }
    return result;
}

std::set<int> participants(const Net& net,
                           const std::vector<NodePlacement>& placements) {
    std::set<int> result;
    const auto insert = [&](int node) {
        const auto& p = placements.at(node);
        result.insert(p.master_fpga);
        result.insert(p.replica_fpgas.begin(), p.replica_fpgas.end());
    };
    insert(net.source_node);
    for (int sink : net.sink_nodes) insert(sink);
    return result;
}

std::vector<std::int64_t> netCommunication(
    const Net& net, const std::vector<NodePlacement>& placements,
    int fpga_count) {
    std::vector<std::int64_t> result(fpga_count, 0);
    const auto p = participants(net, placements);
    if (p.size() > 1) {
        for (int fpga : p) result.at(fpga) = net.weight;
    }
    return result;
}

std::vector<std::int64_t> allCommunication(
    const std::vector<Net>& nets,
    const std::vector<NodePlacement>& placements, int fpga_count) {
    std::vector<std::int64_t> result(fpga_count, 0);
    for (const auto& net : nets) {
        const auto contribution = netCommunication(net, placements, fpga_count);
        for (int f = 0; f < fpga_count; ++f) {
            result.at(f) = addChecked(result.at(f), contribution.at(f),
                                      "T3 communication overflow");
        }
    }
    return result;
}

std::vector<T3ResourceVector> allResources(
    const std::vector<NodePlacement>& placements,
    const std::vector<T3ResourceVector>& node_resources, int fpga_count) {
    std::vector<T3ResourceVector> result(fpga_count, T3ResourceVector{});
    for (int node = 0; node < static_cast<int>(placements.size()); ++node) {
        const auto add_at = [&](int fpga) {
            for (int d = 0; d < 8; ++d) {
                result.at(fpga).at(d) = addChecked(
                    result.at(fpga).at(d), node_resources.at(node).at(d),
                    "T3 resource overflow");
            }
        };
        add_at(placements.at(node).master_fpga);
        for (int replica : placements.at(node).replica_fpgas) add_at(replica);
    }
    return result;
}

bool resourcesLegal(const std::vector<T3ResourceVector>& usage,
                    const std::vector<T3ResourceVector>& capacities) {
    for (std::size_t f = 0; f < usage.size(); ++f) {
        for (int d = 0; d < 8; ++d) {
            if (usage.at(f).at(d) > capacities.at(f).at(d)) return false;
        }
    }
    return true;
}

bool communicationLegal(const std::vector<std::int64_t>& usage,
                        const std::vector<std::int64_t>& capacities) {
    for (std::size_t f = 0; f < usage.size(); ++f) {
        if (usage.at(f) > capacities.at(f)) return false;
    }
    return true;
}

std::set<ConnectionKey> keysOf(const std::vector<Connection>& connections) {
    std::set<ConnectionKey> result;
    for (const auto& c : connections) result.insert(keyOf(c));
    return result;
}

void setKeyDifferences(const std::set<ConnectionKey>& old_keys,
                       const std::set<ConnectionKey>& new_keys,
                       T3Evaluation& result) {
    std::set_difference(old_keys.begin(), old_keys.end(), new_keys.begin(),
                        new_keys.end(),
                        std::inserter(result.deleted_keys,
                                      result.deleted_keys.end()));
    std::set_difference(new_keys.begin(), new_keys.end(), old_keys.begin(),
                        old_keys.end(),
                        std::inserter(result.added_keys,
                                      result.added_keys.end()));
    std::set_intersection(old_keys.begin(), old_keys.end(), new_keys.begin(),
                          new_keys.end(),
                          std::inserter(result.unchanged_keys,
                                        result.unchanged_keys.end()));
    result.changed_key_count =
        result.deleted_keys.size() + result.added_keys.size();
}

T3RejectReason firstReject(const std::set<T3RejectReason>& violations) {
    for (T3RejectReason reason : {
             T3RejectReason::PositionViolation,
             T3RejectReason::ResourceViolation,
             T3RejectReason::TopologyUnreachable,
             T3RejectReason::MaximumHopViolation,
             T3RejectReason::AggregateCommunicationViolation}) {
        if (violations.count(reason)) return reason;
    }
    return T3RejectReason::None;
}

void finishClassification(const T3StateCache& state, T3Evaluation& result) {
    if (!result.all_violations.empty()) {
        result.classification = T3Class::HardIllegal;
        result.first_reject = firstReject(result.all_violations);
        return;
    }
    const auto& u = *result.u_metrics;
    const auto& base = state.baseline_u_metrics;
    if (!(std::tie(u.maximum, u.squared_sum)
          < std::tie(base.maximum, base.squared_sum))) {
        result.classification = T3Class::NotMandatoryImproving;
        return;
    }
    result.thd_excess = std::max<std::int64_t>(
        0, subtractChecked(*result.wthd_prime,
                           state.constraints.thd_budget,
                           "T3 THD excess overflow"));
    result.lmax_excess = std::max<std::int64_t>(
        0, subtractChecked(result.l_metrics->maximum,
                           state.baseline_l_metrics.maximum,
                           "T3 Lmax excess overflow"));
    result.classification =
        *result.wthd_prime <= state.constraints.thd_budget
                && result.l_metrics->maximum
                       <= state.baseline_l_metrics.maximum
            ? T3Class::StandaloneAcceptable
            : T3Class::PairEligible;
}

void addUContribution(std::vector<std::int64_t>& delta,
                      const Connection& c,
                      const MandatoryPathTable& mandatory, int sign) {
    const std::int64_t amount = sign > 0 ? c.demand : -c.demand;
    for (int link : mandatory.mandatoryEdges(c.source_fpga,
                                             c.destination_fpga)) {
        delta.at(link) = addChecked(delta.at(link), amount,
                                    "T3 DeltaU overflow");
    }
}

void addRouteContribution(std::vector<std::int64_t>& delta_l,
                          std::int64_t& delta_uthd,
                          std::int64_t& delta_wthd,
                          const RawDeterministicRoute& route, int sign) {
    const std::int64_t demand = sign > 0 ? route.connection.demand
                                         : -route.connection.demand;
    for (int link : route.physical_link_ids) {
        delta_l.at(link) = addChecked(delta_l.at(link), demand,
                                      "T3 DeltaL overflow");
    }
    const auto hops = static_cast<std::int64_t>(route.hop_count);
    const auto uthd = multiplyChecked(route.connection.demand, hops,
                                      "T3 demand-hop overflow");
    const auto wthd = multiplyChecked(route.connection.net_weight, hops,
                                      "T3 weight-hop overflow");
    delta_uthd = addChecked(delta_uthd, sign > 0 ? uthd : -uthd,
                            "T3 DeltaUTHD overflow");
    delta_wthd = addChecked(delta_wthd, sign > 0 ? wthd : -wthd,
                            "T3 DeltaWTHD overflow");
}

bool placementsEqual(const std::vector<NodePlacement>& a,
                     const std::vector<NodePlacement>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].master_fpga != b[i].master_fpga
            || a[i].replica_fpgas != b[i].replica_fpgas) return false;
    }
    return true;
}

const std::vector<T2Provenance>& provenanceFor(
    const T2CandidateResult& t2, const T2ActionKey& action) {
    const auto it = t2.provenance_by_action.find(action);
    if (std::find(t2.candidates.begin(), t2.candidates.end(), action)
            == t2.candidates.end()
        || it == t2.provenance_by_action.end()) {
        throw std::invalid_argument("T3 action is not in T2 candidate set");
    }
    return it->second;
}

void initializeResult(const T3StateCache& state,
                      const MasterMoveAction& action,
                      std::size_t provenance_count,
                      T3Evaluation& result) {
    result.action = action;
    result.target_fpga = action.target_fpga;
    result.t2_provenance_count = provenance_count;
    result.source_master_fpga =
        state.data.placements.at(action.node_id).master_fpga;
}

void markOverflow(T3Evaluation& result, const std::overflow_error& error) {
    result.status = T3EvaluationStatus::ArithmeticOverflow;
    result.arithmetic_overflow_diagnostic = error.what();
    result.classification.reset();
}

}  // namespace

RawDeterministicRoute routeDeterministicWithoutHopGate(
    const Connection& connection, const FpgaTopology& topology,
    int maximum_hop) {
    RawDeterministicRoute result;
    result.connection = connection;
    const int hop = topology.hop(connection.source_fpga,
                                 connection.destination_fpga);
    result.topology_reachable = hop >= 0;
    if (!result.topology_reachable) return result;
    result.hop_count = hop;
    result.exceeds_maximum_hop = hop > maximum_hop;
    result.fpga_path = topology.path(connection.source_fpga,
                                     connection.destination_fpga);
    topology.validatePath(connection.source_fpga, connection.destination_fpga,
                          result.fpga_path);
    for (std::size_t i = 1; i < result.fpga_path.size(); ++i) {
        const int link = topology.linkId(result.fpga_path[i - 1],
                                         result.fpga_path[i]);
        if (link < 0) throw std::logic_error("raw route uses missing link");
        result.physical_link_ids.push_back(link);
    }
    return result;
}

T3StateCache::T3StateCache(const CaseData& input_data,
                           const FpgaTopology& input_topology,
                           const T3Constraints& input_constraints)
    : data(input_data), topology(input_topology),
      mandatory_paths(topology), constraints(input_constraints) {
    const auto node_count = data.placements.size();
    const int fpga_count = topology.fpgaCount();
    if (constraints.node_resources.size() != node_count
        || constraints.fpga_resource_capacities.size()
               != static_cast<std::size_t>(fpga_count)
        || constraints.fpga_communication_capacities.size()
               != static_cast<std::size_t>(fpga_count)) {
        throw std::invalid_argument("T3 constraint dimensions mismatch");
    }
    ConnectionProjector::validatePlacements(data.placements, fpga_count);
    incident_nets.resize(node_count);
    for (const auto& net : data.nets) {
        incident_nets.at(net.source_node).push_back(net.id);
        for (int sink : net.sink_nodes) incident_nets.at(sink).push_back(net.id);
    }
    for (auto& ids : incident_nets) {
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    }
    net_connections_current.resize(data.nets.size());
    per_net_maximum_hop.assign(data.nets.size(), 0);
    baseline_l.assign(topology.links().size(), 0);
    baseline_u.assign(topology.links().size(), 0);
    for (const auto& net : data.nets) {
        if (net.id < 0 || net.id >= static_cast<int>(data.nets.size())) {
            throw std::invalid_argument("T3 requires dense net IDs");
        }
        auto connections = ConnectionProjector::projectV2({net}, data.placements);
        ++current_projection_count_by_net[net.id];
        std::sort(connections.begin(), connections.end(),
                  [](const Connection& a, const Connection& b) {
                      return keyOf(a) < keyOf(b);
                  });
        net_connections_current.at(net.id) = connections;
        for (const auto& connection : connections) {
            auto route = routeDeterministicWithoutHopGate(
                connection, topology, constraints.maximum_hop);
            if (!route.topology_reachable) {
                throw std::invalid_argument(
                    "T3 baseline contains topology-unreachable connection");
            }
            per_net_maximum_hop.at(net.id) = std::max(
                per_net_maximum_hop.at(net.id), route.hop_count);
            addUContribution(baseline_u, connection, mandatory_paths, +1);
            addRouteContribution(baseline_l, baseline_uthd, baseline_wthd,
                                 route, +1);
            if (!current_routes.emplace(keyOf(connection), route).second) {
                throw std::logic_error("duplicate stable baseline connection key");
            }
        }
    }
    baseline_l_metrics = scanMetrics(baseline_l);
    baseline_u_metrics = scanMetrics(baseline_u);
    baseline_resource_usage = allResources(
        data.placements, constraints.node_resources, fpga_count);
    baseline_communication_usage = allCommunication(
        data.nets, data.placements, fpga_count);
}

T3Evaluation evaluateMasterMove(const T3StateCache& state,
                                const MasterMoveAction& action,
                                std::size_t provenance_count) {
    T3Evaluation result;
    try {
        initializeResult(state, action, provenance_count, result);
        const auto& original_placement = state.data.placements.at(action.node_id);
        if (action.target_fpga == original_placement.master_fpga
            || std::find(original_placement.replica_fpgas.begin(),
                         original_placement.replica_fpgas.end(),
                         action.target_fpga)
                   != original_placement.replica_fpgas.end()) {
            result.classification = T3Class::HardIllegal;
            result.first_reject = T3RejectReason::PositionViolation;
            result.all_violations.insert(T3RejectReason::PositionViolation);
            result.metric_availability =
                MetricAvailability::NotEvaluatedInvalidAction;
            return result;
        }
        const auto frozen = state.data.placements;
        auto action_placements = frozen;
        action_placements.at(action.node_id).master_fpga = action.target_fpga;
        const auto& affected = state.incident_nets.at(action.node_id);
        result.affected_net_count = affected.size();
        std::vector<std::int64_t> delta_l(state.topology.links().size(), 0);
        result.delta_u.assign(state.topology.links().size(), 0);
        std::int64_t delta_uthd = 0, delta_wthd = 0;
        std::set<ConnectionKey> old_keys, new_keys;
        for (int net_id : affected) {
            const auto& old_connections =
                state.net_connections_current.at(net_id);
            auto new_connections = ConnectionProjector::projectV2(
                {state.data.nets.at(net_id)}, action_placements);
            ++result.action_projection_count_by_net[net_id];
            result.old_connection_count += old_connections.size();
            result.new_connection_count += new_connections.size();
            const auto old_net_keys = keysOf(old_connections);
            const auto new_net_keys = keysOf(new_connections);
            old_keys.insert(old_net_keys.begin(), old_net_keys.end());
            new_keys.insert(new_net_keys.begin(), new_net_keys.end());
            for (const auto& connection : old_connections) {
                addUContribution(result.delta_u, connection,
                                 state.mandatory_paths, -1);
                addRouteContribution(delta_l, delta_uthd, delta_wthd,
                                     state.current_routes.at(keyOf(connection)),
                                     -1);
            }
            for (const auto& connection : new_connections) {
                addUContribution(result.delta_u, connection,
                                 state.mandatory_paths, +1);
                const auto route = routeDeterministicWithoutHopGate(
                    connection, state.topology,
                    state.constraints.maximum_hop);
                if (!route.topology_reachable) {
                    ++result.unreachable_new_connection_count;
                    result.all_violations.insert(
                        T3RejectReason::TopologyUnreachable);
                } else {
                    addRouteContribution(delta_l, delta_uthd, delta_wthd,
                                         route, +1);
                    result.new_affected_maximum_hop = std::max(
                        result.new_affected_maximum_hop, route.hop_count);
                }
            }
        }
        setKeyDifferences(old_keys, new_keys, result);

        result.resource_usage_prime = state.baseline_resource_usage;
        for (int d = 0; d < 8; ++d) {
            const auto amount =
                state.constraints.node_resources.at(action.node_id).at(d);
            auto& old_value = result.resource_usage_prime
                                  .at(original_placement.master_fpga).at(d);
            auto& new_value = result.resource_usage_prime
                                  .at(action.target_fpga).at(d);
            old_value = subtractChecked(old_value, amount,
                                        "T3 resource subtraction overflow");
            new_value = addChecked(new_value, amount,
                                   "T3 resource addition overflow");
        }
        result.resource_legal = resourcesLegal(
            result.resource_usage_prime,
            state.constraints.fpga_resource_capacities);
        if (!result.resource_legal) {
            result.all_violations.insert(T3RejectReason::ResourceViolation);
        }

        result.delta_communication.assign(state.topology.fpgaCount(), 0);
        for (int net_id : affected) {
            const auto old_c = netCommunication(
                state.data.nets.at(net_id), frozen,
                state.topology.fpgaCount());
            const auto new_c = netCommunication(
                state.data.nets.at(net_id), action_placements,
                state.topology.fpgaCount());
            for (int f = 0; f < state.topology.fpgaCount(); ++f) {
                result.delta_communication.at(f) = addChecked(
                    result.delta_communication.at(f),
                    subtractChecked(new_c.at(f), old_c.at(f),
                                    "T3 communication delta overflow"),
                    "T3 communication delta sum overflow");
            }
        }
        result.communication_usage_prime =
            state.baseline_communication_usage;
        for (int f = 0; f < state.topology.fpgaCount(); ++f) {
            result.communication_usage_prime.at(f) = addChecked(
                result.communication_usage_prime.at(f),
                result.delta_communication.at(f),
                "T3 communication result overflow");
        }
        result.aggregate_communication_legal = communicationLegal(
            result.communication_usage_prime,
            state.constraints.fpga_communication_capacities);
        if (!result.aggregate_communication_legal) {
            result.all_violations.insert(
                T3RejectReason::AggregateCommunicationViolation);
        }

        result.u_prime = state.baseline_u;
        for (std::size_t i = 0; i < result.u_prime.size(); ++i) {
            result.u_prime.at(i) = addChecked(
                result.u_prime.at(i), result.delta_u.at(i),
                "T3 U prime overflow");
        }
        result.u_metrics = scanMetrics(result.u_prime);
        result.u_metrics_valid = true;
        std::set<int> affected_set(affected.begin(), affected.end());
        for (int net = 0;
             net < static_cast<int>(state.per_net_maximum_hop.size()); ++net) {
            if (!affected_set.count(net)) {
                result.unaffected_maximum_hop = std::max(
                    result.unaffected_maximum_hop,
                    state.per_net_maximum_hop.at(net));
            }
        }
        if (result.unreachable_new_connection_count > 0) {
            result.metric_availability =
                MetricAvailability::UndefinedDueToUnreachable;
            result.classification = T3Class::HardIllegal;
            result.first_reject = firstReject(result.all_violations);
            if (!placementsEqual(frozen, state.data.placements)) {
                throw std::logic_error("T3 mutated frozen placement");
            }
            return result;
        }
        result.global_maximum_hop_prime = std::max(
            result.unaffected_maximum_hop,
            result.new_affected_maximum_hop);
        result.global_maximum_hop_valid = true;
        if (*result.global_maximum_hop_prime
            > state.constraints.maximum_hop) {
            result.all_violations.insert(
                T3RejectReason::MaximumHopViolation);
        }
        result.delta_l = delta_l;
        result.l_prime = state.baseline_l;
        for (std::size_t i = 0; i < result.l_prime->size(); ++i) {
            result.l_prime->at(i) = addChecked(
                result.l_prime->at(i), delta_l.at(i),
                "T3 L prime overflow");
        }
        result.l_metrics = scanMetrics(*result.l_prime);
        result.delta_uthd = delta_uthd;
        result.delta_wthd = delta_wthd;
        result.uthd_prime = addChecked(state.baseline_uthd, delta_uthd,
                                       "T3 UTHD prime overflow");
        result.wthd_prime = addChecked(state.baseline_wthd, delta_wthd,
                                       "T3 WTHD prime overflow");
        result.l_metrics_valid = true;
        result.thd_metrics_valid = true;
        result.metric_availability = MetricAvailability::Complete;
        finishClassification(state, result);
        if (!placementsEqual(frozen, state.data.placements)) {
            throw std::logic_error("T3 mutated frozen placement");
        }
    } catch (const std::overflow_error& error) {
        markOverflow(result, error);
    }
    return result;
}

T3Evaluation evaluateT3StructuralCandidate(
    const T3StateCache& state, const T2CandidateResult& t2,
    const T2ActionKey& action) {
    const auto provenance_count = provenanceFor(t2, action).size();
    auto result = evaluateMasterMove(state, action, provenance_count);
    if (result.metric_availability
        == MetricAvailability::NotEvaluatedInvalidAction) {
        result.t2_invariant_violation = true;
    }
    return result;
}

T3FullRebuildResult rebuildMasterMoveFromScratch(
    const T3StateCache& baseline, const MasterMoveAction& action,
    std::size_t provenance_count) {
    T3FullRebuildResult output;
    auto& result = output.evaluation;
    try {
        initializeResult(baseline, action, provenance_count, result);
        const auto& original = baseline.data.placements.at(action.node_id);
        if (action.target_fpga == original.master_fpga
            || std::find(original.replica_fpgas.begin(),
                         original.replica_fpgas.end(), action.target_fpga)
                   != original.replica_fpgas.end()) {
            result.classification = T3Class::HardIllegal;
            result.first_reject = T3RejectReason::PositionViolation;
            result.all_violations.insert(T3RejectReason::PositionViolation);
            result.metric_availability =
                MetricAvailability::NotEvaluatedInvalidAction;
            return output;
        }
        auto moved = baseline.data.placements;
        moved.at(action.node_id).master_fpga = action.target_fpga;
        const auto& affected = baseline.incident_nets.at(action.node_id);
        result.affected_net_count = affected.size();
        std::set<ConnectionKey> old_keys, new_affected_keys;
        for (int net : affected) {
            const auto& old = baseline.net_connections_current.at(net);
            result.old_connection_count += old.size();
            const auto keys = keysOf(old);
            old_keys.insert(keys.begin(), keys.end());
        }
        output.all_net_connections.resize(baseline.data.nets.size());
        std::vector<std::int64_t> full_l(baseline.topology.links().size(), 0);
        std::vector<std::int64_t> full_u(baseline.topology.links().size(), 0);
        std::int64_t full_uthd = 0, full_wthd = 0;
        int global_hop = 0;
        for (const auto& net : baseline.data.nets) {
            auto connections = ConnectionProjector::projectV2({net}, moved);
            output.all_net_connections.at(net.id) = connections;
            if (std::find(affected.begin(), affected.end(), net.id)
                != affected.end()) {
                result.new_connection_count += connections.size();
                const auto keys = keysOf(connections);
                new_affected_keys.insert(keys.begin(), keys.end());
            }
            for (const auto& connection : connections) {
                addUContribution(full_u, connection,
                                 baseline.mandatory_paths, +1);
                const auto route = routeDeterministicWithoutHopGate(
                    connection, baseline.topology,
                    baseline.constraints.maximum_hop);
                if (!route.topology_reachable) {
                    ++result.unreachable_new_connection_count;
                } else {
                    addRouteContribution(full_l, full_uthd, full_wthd,
                                         route, +1);
                    global_hop = std::max(global_hop, route.hop_count);
                    if (std::find(affected.begin(), affected.end(), net.id)
                        != affected.end()) {
                        result.new_affected_maximum_hop = std::max(
                            result.new_affected_maximum_hop,
                            route.hop_count);
                    }
                }
            }
        }
        setKeyDifferences(old_keys, new_affected_keys, result);
        result.resource_usage_prime = allResources(
            moved, baseline.constraints.node_resources,
            baseline.topology.fpgaCount());
        result.resource_legal = resourcesLegal(
            result.resource_usage_prime,
            baseline.constraints.fpga_resource_capacities);
        if (!result.resource_legal) result.all_violations.insert(
            T3RejectReason::ResourceViolation);
        result.communication_usage_prime = allCommunication(
            baseline.data.nets, moved, baseline.topology.fpgaCount());
        result.delta_communication.resize(baseline.topology.fpgaCount());
        for (int f = 0; f < baseline.topology.fpgaCount(); ++f) {
            result.delta_communication.at(f) = subtractChecked(
                result.communication_usage_prime.at(f),
                baseline.baseline_communication_usage.at(f),
                "T3 rebuild communication delta overflow");
        }
        result.aggregate_communication_legal = communicationLegal(
            result.communication_usage_prime,
            baseline.constraints.fpga_communication_capacities);
        if (!result.aggregate_communication_legal) {
            result.all_violations.insert(
                T3RejectReason::AggregateCommunicationViolation);
        }
        result.delta_u.resize(full_u.size());
        for (std::size_t i = 0; i < full_u.size(); ++i) {
            result.delta_u.at(i) = subtractChecked(
                full_u.at(i), baseline.baseline_u.at(i),
                "T3 rebuild DeltaU overflow");
        }
        result.u_prime = full_u;
        result.u_metrics = scanMetrics(full_u);
        result.u_metrics_valid = true;
        std::set<int> affected_set(affected.begin(), affected.end());
        for (int net = 0;
             net < static_cast<int>(baseline.per_net_maximum_hop.size());
             ++net) {
            if (!affected_set.count(net)) {
                result.unaffected_maximum_hop = std::max(
                    result.unaffected_maximum_hop,
                    baseline.per_net_maximum_hop.at(net));
            }
        }
        if (result.unreachable_new_connection_count > 0) {
            result.all_violations.insert(T3RejectReason::TopologyUnreachable);
            result.metric_availability =
                MetricAvailability::UndefinedDueToUnreachable;
            result.classification = T3Class::HardIllegal;
            result.first_reject = firstReject(result.all_violations);
            return output;
        }
        result.global_maximum_hop_prime = global_hop;
        result.global_maximum_hop_valid = true;
        if (global_hop > baseline.constraints.maximum_hop) {
            result.all_violations.insert(
                T3RejectReason::MaximumHopViolation);
        }
        result.delta_l = std::vector<std::int64_t>(full_l.size());
        for (std::size_t i = 0; i < full_l.size(); ++i) {
            result.delta_l->at(i) = subtractChecked(
                full_l.at(i), baseline.baseline_l.at(i),
                "T3 rebuild DeltaL overflow");
        }
        result.l_prime = full_l;
        result.l_metrics = scanMetrics(full_l);
        result.delta_uthd = subtractChecked(
            full_uthd, baseline.baseline_uthd,
            "T3 rebuild DeltaUTHD overflow");
        result.delta_wthd = subtractChecked(
            full_wthd, baseline.baseline_wthd,
            "T3 rebuild DeltaWTHD overflow");
        result.uthd_prime = full_uthd;
        result.wthd_prime = full_wthd;
        result.l_metrics_valid = true;
        result.thd_metrics_valid = true;
        result.metric_availability = MetricAvailability::Complete;
        finishClassification(baseline, result);
    } catch (const std::overflow_error& error) {
        markOverflow(result, error);
    }
    return output;
}

T3FullRebuildResult rebuildT3CandidateFromScratch(
    const T3StateCache& baseline, const T2CandidateResult& t2,
    const T2ActionKey& action) {
    const auto provenance_count = provenanceFor(t2, action).size();
    auto output = rebuildMasterMoveFromScratch(
        baseline, action, provenance_count);
    if (output.evaluation.metric_availability
        == MetricAvailability::NotEvaluatedInvalidAction) {
        output.evaluation.t2_invariant_violation = true;
    }
    return output;
}

bool operator==(const T3LoadMetrics& lhs, const T3LoadMetrics& rhs) {
    return lhs.maximum == rhs.maximum
        && lhs.squared_sum == rhs.squared_sum && lhs.sum == rhs.sum
        && lhs.normalized_concentration == rhs.normalized_concentration;
}

bool operator==(const T3Evaluation& a, const T3Evaluation& b) {
    return std::tie(a.action.node_id, a.action.target_fpga,
                    a.source_master_fpga, a.target_fpga,
                    a.t2_provenance_count, a.status,
                    a.arithmetic_overflow_diagnostic, a.classification,
                    a.metric_availability, a.t2_invariant_violation,
                    a.l_metrics_valid, a.u_metrics_valid,
                    a.thd_metrics_valid, a.global_maximum_hop_valid,
                    a.affected_net_count, a.old_connection_count,
                    a.new_connection_count, a.added_keys, a.deleted_keys,
                    a.unchanged_keys, a.changed_key_count, a.delta_l,
                    a.delta_u, a.l_prime, a.u_prime, a.l_metrics,
                    a.u_metrics, a.delta_uthd, a.delta_wthd, a.uthd_prime,
                    a.wthd_prime, a.unaffected_maximum_hop,
                    a.new_affected_maximum_hop,
                    a.global_maximum_hop_prime,
                    a.unreachable_new_connection_count, a.resource_legal,
                    a.resource_usage_prime,
                    a.aggregate_communication_legal,
                    a.delta_communication, a.communication_usage_prime,
                    a.first_reject, a.all_violations, a.thd_excess,
                    a.lmax_excess, a.action_projection_count_by_net)
        == std::tie(b.action.node_id, b.action.target_fpga,
                    b.source_master_fpga, b.target_fpga,
                    b.t2_provenance_count, b.status,
                    b.arithmetic_overflow_diagnostic, b.classification,
                    b.metric_availability, b.t2_invariant_violation,
                    b.l_metrics_valid, b.u_metrics_valid,
                    b.thd_metrics_valid, b.global_maximum_hop_valid,
                    b.affected_net_count, b.old_connection_count,
                    b.new_connection_count, b.added_keys, b.deleted_keys,
                    b.unchanged_keys, b.changed_key_count, b.delta_l,
                    b.delta_u, b.l_prime, b.u_prime, b.l_metrics,
                    b.u_metrics, b.delta_uthd, b.delta_wthd, b.uthd_prime,
                    b.wthd_prime, b.unaffected_maximum_hop,
                    b.new_affected_maximum_hop,
                    b.global_maximum_hop_prime,
                    b.unreachable_new_connection_count, b.resource_legal,
                    b.resource_usage_prime,
                    b.aggregate_communication_legal,
                    b.delta_communication, b.communication_usage_prime,
                    b.first_reject, b.all_violations, b.thd_excess,
                    b.lmax_excess, b.action_projection_count_by_net);
}

bool t3EvaluationEquivalent(const T3Evaluation& incremental,
                            const T3Evaluation& rebuilt,
                            std::string* mismatch) {
    T3Evaluation normalized = rebuilt;
    normalized.action_projection_count_by_net =
        incremental.action_projection_count_by_net;
    if (incremental == normalized) return true;
    if (mismatch) *mismatch = "incremental and full-rebuild T3 results differ";
    return false;
}

std::int64_t t3CheckedArithmeticForTest(T3CheckedOperationForTest operation,
                                        std::int64_t a, std::int64_t b) {
    switch (operation) {
        case T3CheckedOperationForTest::Add:
            return addChecked(a, b, "T3 test add overflow");
        case T3CheckedOperationForTest::Subtract:
            return subtractChecked(a, b, "T3 test subtract overflow");
        case T3CheckedOperationForTest::Multiply:
            return multiplyChecked(a, b, "T3 test multiply overflow");
        case T3CheckedOperationForTest::SquareSum:
            return addChecked(a, multiplyChecked(b, b,
                                                 "T3 test square overflow"),
                              "T3 test square sum overflow");
    }
    throw std::logic_error("unknown T3 checked test operation");
}

}  // namespace route_prediction_v1
