#include "t2_structural_candidates.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <set>
#include <stdexcept>
#include <tuple>

namespace route_prediction_v1 {

bool operator==(const T2ActionKey& lhs, const T2ActionKey& rhs) {
    return std::tie(lhs.node_id, lhs.target_fpga)
        == std::tie(rhs.node_id, rhs.target_fpga);
}

bool operator<(const T2ActionKey& lhs, const T2ActionKey& rhs) {
    return std::tie(lhs.node_id, lhs.target_fpga)
        < std::tie(rhs.node_id, rhs.target_fpga);
}

bool operator==(const ConnectionKey& lhs, const ConnectionKey& rhs) {
    return std::tie(lhs.net_id, lhs.source_fpga, lhs.destination_fpga)
        == std::tie(rhs.net_id, rhs.source_fpga, rhs.destination_fpga);
}

bool operator<(const ConnectionKey& lhs, const ConnectionKey& rhs) {
    return std::tie(lhs.net_id, lhs.source_fpga, lhs.destination_fpga)
        < std::tie(rhs.net_id, rhs.source_fpga, rhs.destination_fpga);
}

int canonicalRole(T2NodeRole role) {
    switch (role) {
        case T2NodeRole::Source: return 0;
        case T2NodeRole::Sink: return 1;
    }
    throw std::logic_error("unknown T2 node role");
}

int reasonOrder(T2Reason reason) {
    switch (reason) {
        case T2Reason::SourceEndpointChanged: return 0;
        case T2Reason::DestinationRemoved: return 1;
        case T2Reason::SourceCoverageChanged: return 2;
    }
    throw std::logic_error("unknown T2 provenance reason");
}

const char* t2StopReasonName(T2StopReason reason) {
    switch (reason) {
        case T2StopReason::Success: return "Success";
        case T2StopReason::UnsupportedParallelPhysicalLinks:
            return "UnsupportedParallelPhysicalLinks";
        case T2StopReason::NoPhysicalLink: return "NoPhysicalLink";
        case T2StopReason::NoPositiveShortestPathMandatoryLoad:
            return "NoPositiveShortestPathMandatoryLoad";
    }
    throw std::logic_error("unknown T2 stop reason");
}

bool operator==(const CanonicalProvenanceKey& lhs,
                const CanonicalProvenanceKey& rhs) {
    return std::tie(
               lhs.hot_link_id, lhs.hot_link_endpoints, lhs.net_id,
               lhs.old_source_fpga, lhs.old_destination_fpga, lhs.node_id,
               lhs.target_fpga, lhs.canonical_role, lhs.sink_master_at_Fd,
               lhs.sink_replicas_at_Fd, lhs.old_connection_key,
               lhs.old_connection_key_disappeared, lhs.new_connection_keys,
               lhs.canonical_reasons)
        == std::tie(
               rhs.hot_link_id, rhs.hot_link_endpoints, rhs.net_id,
               rhs.old_source_fpga, rhs.old_destination_fpga, rhs.node_id,
               rhs.target_fpga, rhs.canonical_role, rhs.sink_master_at_Fd,
               rhs.sink_replicas_at_Fd, rhs.old_connection_key,
               rhs.old_connection_key_disappeared, rhs.new_connection_keys,
               rhs.canonical_reasons);
}

bool operator<(const CanonicalProvenanceKey& lhs,
               const CanonicalProvenanceKey& rhs) {
    return std::tie(
               lhs.hot_link_id, lhs.hot_link_endpoints, lhs.net_id,
               lhs.old_source_fpga, lhs.old_destination_fpga, lhs.node_id,
               lhs.target_fpga, lhs.canonical_role, lhs.sink_master_at_Fd,
               lhs.sink_replicas_at_Fd, lhs.old_connection_key,
               lhs.old_connection_key_disappeared, lhs.new_connection_keys,
               lhs.canonical_reasons)
        < std::tie(
               rhs.hot_link_id, rhs.hot_link_endpoints, rhs.net_id,
               rhs.old_source_fpga, rhs.old_destination_fpga, rhs.node_id,
               rhs.target_fpga, rhs.canonical_role, rhs.sink_master_at_Fd,
               rhs.sink_replicas_at_Fd, rhs.old_connection_key,
               rhs.old_connection_key_disappeared, rhs.new_connection_keys,
               rhs.canonical_reasons);
}

std::pair<int, int> canonicalEndpoints(int endpoint_1, int endpoint_2) {
    return {std::min(endpoint_1, endpoint_2),
            std::max(endpoint_1, endpoint_2)};
}

CanonicalProvenanceKey canonicalProvenanceKey(
    const T2Provenance& provenance) {
    CanonicalProvenanceKey key;
    key.hot_link_id = provenance.hot_link_id;
    key.hot_link_endpoints = canonicalEndpoints(
        provenance.hot_link_endpoints.first,
        provenance.hot_link_endpoints.second);
    key.net_id = provenance.net_id;
    key.old_source_fpga = provenance.old_source_fpga;
    key.old_destination_fpga = provenance.old_destination_fpga;
    key.node_id = provenance.node_id;
    key.target_fpga = provenance.target_fpga;
    key.canonical_role = canonicalRole(provenance.node_role);
    key.sink_master_at_Fd = provenance.sink_master_at_Fd;
    key.sink_replicas_at_Fd = provenance.sink_replicas_at_Fd;
    std::sort(key.sink_replicas_at_Fd.begin(),
              key.sink_replicas_at_Fd.end());
    key.old_connection_key = provenance.old_connection_key;
    key.old_connection_key_disappeared =
        provenance.old_connection_key_disappeared;
    key.new_connection_keys = provenance.new_connection_keys;
    std::sort(key.new_connection_keys.begin(), key.new_connection_keys.end());
    key.canonical_reasons.reserve(provenance.reasons.size());
    for (T2Reason reason : provenance.reasons) {
        key.canonical_reasons.push_back(reasonOrder(reason));
    }
    std::sort(key.canonical_reasons.begin(), key.canonical_reasons.end());
    return key;
}

void canonicalizeProvenance(std::vector<T2Provenance>& provenance) {
    std::sort(provenance.begin(), provenance.end(),
              [](const T2Provenance& lhs, const T2Provenance& rhs) {
                  return canonicalProvenanceKey(lhs)
                      < canonicalProvenanceKey(rhs);
              });
    provenance.erase(
        std::unique(provenance.begin(), provenance.end(),
                    [](const T2Provenance& lhs, const T2Provenance& rhs) {
                        return canonicalProvenanceKey(lhs)
                            == canonicalProvenanceKey(rhs);
                    }),
        provenance.end());
}

namespace {

ConnectionKey connectionKey(const Connection& connection) {
    return {connection.net_id, connection.source_fpga,
            connection.destination_fpga};
}

std::vector<Connection> projectOneNet(
    const Net& net,
    const std::vector<NodePlacement>& placements) {
    return ConnectionProjector::projectV2({net}, placements);
}

std::set<int> rawDestinations(
    const Net& net,
    const std::vector<NodePlacement>& placements) {
    std::set<int> destinations;
    for (int sink : net.sink_nodes) {
        const auto& placement = placements.at(sink);
        destinations.insert(placement.master_fpga);
        destinations.insert(placement.replica_fpgas.begin(),
                            placement.replica_fpgas.end());
    }
    return destinations;
}

std::set<int> sourceCoverage(
    const Net& net,
    const std::vector<NodePlacement>& placements) {
    const auto& source = placements.at(net.source_node);
    std::set<int> coverage{source.master_fpga};
    coverage.insert(source.replica_fpgas.begin(),
                    source.replica_fpgas.end());
    return coverage;
}

std::set<int> setDifference(const std::set<int>& lhs,
                            const std::set<int>& rhs) {
    std::set<int> result;
    std::set_difference(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
                        std::inserter(result, result.end()));
    return result;
}

bool contains(const std::vector<int>& values, int value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

bool hasParallelPhysicalLinks(const FpgaTopology& topology) {
    std::set<std::pair<int, int>> keys;
    for (const auto& link : topology.inputPhysicalLinks()) {
        if (!keys.insert(canonicalEndpoints(link.u, link.v)).second) {
            return true;
        }
    }
    return false;
}

}  // namespace

T2Provenance buildT2Provenance(
    const PhysicalLink& hot_link,
    const Connection& original_connection,
    int node_id,
    int target_fpga,
    const Net& net,
    const std::vector<NodePlacement>& placements,
    const std::vector<NodePlacement>& action_placements,
    const std::vector<Connection>& old_connections,
    const std::vector<Connection>& new_connections) {
    (void)old_connections;
    T2Provenance provenance;
    provenance.hot_link_id = hot_link.id;
    provenance.hot_link_endpoints = {hot_link.u, hot_link.v};
    provenance.net_id = net.id;
    provenance.old_source_fpga = original_connection.source_fpga;
    provenance.old_destination_fpga =
        original_connection.destination_fpga;
    provenance.old_connection_key = connectionKey(original_connection);
    provenance.node_id = node_id;
    provenance.target_fpga = target_fpga;
    provenance.old_connection_key_disappeared = true;
    for (const auto& connection : new_connections) {
        provenance.new_connection_keys.push_back(connectionKey(connection));
    }
    std::sort(provenance.new_connection_keys.begin(),
              provenance.new_connection_keys.end());
    provenance.new_connection_keys.erase(
        std::unique(provenance.new_connection_keys.begin(),
                    provenance.new_connection_keys.end()),
        provenance.new_connection_keys.end());

    if (node_id == net.source_node) {
        provenance.node_role = T2NodeRole::Source;
    } else {
        provenance.node_role = T2NodeRole::Sink;
    }

    if (provenance.node_role == T2NodeRole::Sink) {
        const auto& node_placement = placements.at(node_id);
        provenance.sink_master_at_Fd =
            node_placement.master_fpga
            == original_connection.destination_fpga;
        for (int replica : node_placement.replica_fpgas) {
            if (replica == original_connection.destination_fpga) {
                provenance.sink_replicas_at_Fd.push_back(replica);
            }
        }
        std::sort(provenance.sink_replicas_at_Fd.begin(),
                  provenance.sink_replicas_at_Fd.end());
    }

    if (node_id == net.source_node
        && action_placements.at(node_id).master_fpga
            != placements.at(node_id).master_fpga) {
        provenance.reasons.push_back(T2Reason::SourceEndpointChanged);
    }

    const auto d_raw_old = rawDestinations(net, placements);
    const auto c_src_old = sourceCoverage(net, placements);
    const auto d_old = setDifference(d_raw_old, c_src_old);
    const auto d_raw_new = rawDestinations(net, action_placements);
    const auto c_src_new = sourceCoverage(net, action_placements);
    const auto d_new = setDifference(d_raw_new, c_src_new);
    const int old_destination = original_connection.destination_fpga;

    if (d_old.count(old_destination) != 0
        && d_raw_new.count(old_destination) == 0) {
        provenance.reasons.push_back(T2Reason::DestinationRemoved);
    }
    if (d_raw_new.count(old_destination) != 0
        && c_src_new.count(old_destination) != 0
        && d_new.count(old_destination) == 0) {
        provenance.reasons.push_back(T2Reason::SourceCoverageChanged);
    }

    assert(!provenance.reasons.empty());
    if (provenance.reasons.empty()) {
        throw std::logic_error("T2 provenance reason_set is empty");
    }
    return provenance;
}

T2CandidateResult generateT2StructuralCongestionCandidates(
    const CaseData& data,
    const FpgaTopology& topology,
    const MandatoryPathTable& mandatory_paths,
    const MandatoryLoadMetrics& mandatory_metrics) {
    T2CandidateResult result;

    if (hasParallelPhysicalLinks(topology)) {
        result.stop_reason =
            T2StopReason::UnsupportedParallelPhysicalLinks;
        return result;
    }

    std::vector<Connection> current_connections;
    std::map<int, std::vector<Connection>> net_connections_current;
    std::vector<const Net*> ordered_nets;
    ordered_nets.reserve(data.nets.size());
    for (const auto& net : data.nets) ordered_nets.push_back(&net);
    std::sort(ordered_nets.begin(), ordered_nets.end(),
              [](const Net* lhs, const Net* rhs) {
                  return lhs->id < rhs->id;
              });
    for (const Net* net : ordered_nets) {
        auto connections = projectOneNet(*net, data.placements);
        ++result.current_projection_count_by_net[net->id];
        std::sort(connections.begin(), connections.end(),
                  [](const Connection& lhs, const Connection& rhs) {
                      return connectionKey(lhs) < connectionKey(rhs);
                  });
        auto inserted = net_connections_current.emplace(
            net->id, std::move(connections));
        if (!inserted.second) {
            throw std::logic_error("duplicate net ID in T2 input");
        }
        current_connections.insert(
            current_connections.end(), inserted.first->second.begin(),
            inserted.first->second.end());
    }

    if (topology.links().empty()) {
        result.stop_reason = T2StopReason::NoPhysicalLink;
        return result;
    }
    if (mandatory_metrics.mandatory_link_loads.size()
        != topology.links().size()) {
        throw std::invalid_argument(
            "mandatory U vector does not match physical links");
    }

    const std::int64_t maximum = *std::max_element(
        mandatory_metrics.mandatory_link_loads.begin(),
        mandatory_metrics.mandatory_link_loads.end());
    if (maximum == 0) {
        result.stop_reason =
            T2StopReason::NoPositiveShortestPathMandatoryLoad;
        return result;
    }
    for (const auto& link : topology.links()) {
        if (mandatory_metrics.mandatory_link_loads.at(link.id) == maximum) {
            result.hot_link_ids.push_back(link.id);
        }
    }
    std::sort(result.hot_link_ids.begin(), result.hot_link_ids.end());

    std::set<T2ActionKey> candidate_set;
    for (int hot_link_id : result.hot_link_ids) {
        const auto& hot_link = topology.links().at(hot_link_id);
        std::vector<Connection> causes;
        for (const auto& connection : current_connections) {
            if (contains(mandatory_paths.mandatoryEdges(
                             connection.source_fpga,
                             connection.destination_fpga),
                         hot_link_id)) {
                causes.push_back(connection);
            }
        }
        std::sort(causes.begin(), causes.end(),
                  [](const Connection& lhs, const Connection& rhs) {
                      return connectionKey(lhs) < connectionKey(rhs);
                  });
        result.mandatory_cause_count +=
            static_cast<std::int64_t>(causes.size());

        for (const auto& cause : causes) {
            const auto net_it = std::find_if(
                data.nets.begin(), data.nets.end(),
                [&](const Net& net) { return net.id == cause.net_id; });
            if (net_it == data.nets.end()) {
                throw std::logic_error("cause connection has unknown net ID");
            }
            const Net& net = *net_it;
            const auto current_it =
                net_connections_current.find(cause.net_id);
            if (current_it == net_connections_current.end()) {
                throw std::logic_error("missing current per-net projection");
            }
            const auto& current_net_connections = current_it->second;

            std::set<int> structure_nodes{net.source_node};
            for (int sink : net.sink_nodes) {
                const auto& placement = data.placements.at(sink);
                if (placement.master_fpga == cause.destination_fpga
                    || contains(placement.replica_fpgas,
                                cause.destination_fpga)) {
                    structure_nodes.insert(sink);
                }
            }

            for (int node : structure_nodes) {
                const auto& placement = data.placements.at(node);
                for (int target = 0; target < topology.fpgaCount(); ++target) {
                    if (target == placement.master_fpga
                        || contains(placement.replica_fpgas, target)) {
                        continue;
                    }
                    ++result.raw_source_check_count;
                    const T2ActionKey action{node, target};
                    auto action_placements = data.placements;
                    action_placements.at(node).master_fpga = target;
                    const auto action_connections =
                        projectOneNet(net, action_placements);
                    std::set<ConnectionKey> new_keys;
                    for (const auto& connection : action_connections) {
                        new_keys.insert(connectionKey(connection));
                    }
                    const ConnectionKey old_key = connectionKey(cause);
                    if (new_keys.count(old_key) != 0) continue;

                    ++result.structurally_effective_source_count;
                    auto provenance = buildT2Provenance(
                        hot_link, cause, node, target, net,
                        data.placements, action_placements,
                        current_net_connections, action_connections);
                    candidate_set.insert(action);
                    result.provenance_by_action[action].push_back(
                        std::move(provenance));
                    ++result.valid_operation_count_before_dedup;
                }
            }
        }
    }

    result.candidates.assign(candidate_set.begin(), candidate_set.end());
    for (const auto& action : result.candidates) {
        canonicalizeProvenance(result.provenance_by_action.at(action));
    }
    result.candidate_count_after_dedup =
        static_cast<std::int64_t>(result.candidates.size());
    result.stop_reason = T2StopReason::Success;
    return result;
}

}  // namespace route_prediction_v1
