#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "connection_projector.h"
#include "mandatory_path_metrics.h"

namespace route_prediction_v1 {

struct T2ActionKey {
    int node_id = -1;
    int target_fpga = -1;
};

bool operator==(const T2ActionKey& lhs, const T2ActionKey& rhs);
bool operator<(const T2ActionKey& lhs, const T2ActionKey& rhs);

struct ConnectionKey {
    int net_id = -1;
    int source_fpga = -1;
    int destination_fpga = -1;
};

bool operator==(const ConnectionKey& lhs, const ConnectionKey& rhs);
bool operator<(const ConnectionKey& lhs, const ConnectionKey& rhs);

enum class T2NodeRole { Source, Sink };

enum class T2Reason {
    SourceEndpointChanged,
    DestinationRemoved,
    SourceCoverageChanged
};

int canonicalRole(T2NodeRole role);
int reasonOrder(T2Reason reason);

enum class T2StopReason {
    Success,
    UnsupportedParallelPhysicalLinks,
    NoPhysicalLink,
    NoPositiveShortestPathMandatoryLoad
};

const char* t2StopReasonName(T2StopReason reason);

struct T2Provenance {
    int hot_link_id = -1;
    std::pair<int, int> hot_link_endpoints{-1, -1};
    int net_id = -1;
    int old_source_fpga = -1;
    int old_destination_fpga = -1;
    int node_id = -1;
    int target_fpga = -1;
    T2NodeRole node_role = T2NodeRole::Source;
    bool sink_master_at_Fd = false;
    std::vector<int> sink_replicas_at_Fd;
    ConnectionKey old_connection_key;
    bool old_connection_key_disappeared = false;
    std::vector<ConnectionKey> new_connection_keys;
    std::vector<T2Reason> reasons;
};

struct CanonicalProvenanceKey {
    int hot_link_id = -1;
    std::pair<int, int> hot_link_endpoints{-1, -1};
    int net_id = -1;
    int old_source_fpga = -1;
    int old_destination_fpga = -1;
    int node_id = -1;
    int target_fpga = -1;
    int canonical_role = -1;
    bool sink_master_at_Fd = false;
    std::vector<int> sink_replicas_at_Fd;
    ConnectionKey old_connection_key;
    bool old_connection_key_disappeared = false;
    std::vector<ConnectionKey> new_connection_keys;
    std::vector<int> canonical_reasons;
};

bool operator==(const CanonicalProvenanceKey& lhs,
                const CanonicalProvenanceKey& rhs);
bool operator<(const CanonicalProvenanceKey& lhs,
               const CanonicalProvenanceKey& rhs);

std::pair<int, int> canonicalEndpoints(int endpoint_1, int endpoint_2);
CanonicalProvenanceKey canonicalProvenanceKey(const T2Provenance& provenance);
void canonicalizeProvenance(std::vector<T2Provenance>& provenance);

struct T2CandidateResult {
    std::vector<T2ActionKey> candidates;
    std::map<T2ActionKey, std::vector<T2Provenance>> provenance_by_action;
    std::vector<int> hot_link_ids;
    std::int64_t mandatory_cause_count = 0;
    std::int64_t raw_source_check_count = 0;
    std::int64_t structurally_effective_source_count = 0;
    std::int64_t valid_operation_count_before_dedup = 0;
    std::int64_t candidate_count_after_dedup = 0;
    T2StopReason stop_reason = T2StopReason::Success;
    std::map<int, int> current_projection_count_by_net;
};

T2Provenance buildT2Provenance(
    const PhysicalLink& hot_link,
    const Connection& original_connection,
    int node_id,
    int target_fpga,
    const Net& net,
    const std::vector<NodePlacement>& placements,
    const std::vector<NodePlacement>& action_placements,
    const std::vector<Connection>& old_connections,
    const std::vector<Connection>& new_connections);

T2CandidateResult generateT2StructuralCongestionCandidates(
    const CaseData& data,
    const FpgaTopology& topology,
    const MandatoryPathTable& mandatory_paths,
    const MandatoryLoadMetrics& mandatory_metrics);

}  // namespace route_prediction_v1
