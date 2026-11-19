#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "flow_preparation.h"

namespace cf {

struct ScheduledFlowPolicy {
    std::size_t rho_numerator;
    std::size_t rho_denominator;
    std::size_t max_successful_layers;
};

inline bool operator==(const ScheduledFlowPolicy& left,
                       const ScheduledFlowPolicy& right) {
    return left.rho_numerator == right.rho_numerator &&
           left.rho_denominator == right.rho_denominator &&
           left.max_successful_layers == right.max_successful_layers;
}

inline ScheduledFlowPolicy cutOnlyV0ScheduledPolicy() {
    return {3, 4, 3};
}

inline void validateScheduledFlowPolicy(const ScheduledFlowPolicy& policy) {
    if (policy.rho_denominator == 0)
        throw std::invalid_argument("FLOW_POLICY_ZERO_DENOMINATOR");
    if (policy.rho_numerator == 0 ||
        policy.rho_numerator > policy.rho_denominator)
        throw std::invalid_argument("FLOW_POLICY_INVALID_RHO");
    if (policy.max_successful_layers == 0)
        throw std::invalid_argument("FLOW_POLICY_ZERO_BMAX");
}

// Compute floor(value * numerator / denominator) without constructing the
// potentially overflowing product. The recurrence performs binary long
// division and is exact for every valid nonzero ratio not greater than one.
inline std::size_t scheduledNodeThreshold(
    std::size_t value, const ScheduledFlowPolicy& policy) {
    validateScheduledFlowPolicy(policy);
    std::size_t quotient = 0;
    std::size_t remainder = 0;
    constexpr std::size_t bits = std::numeric_limits<std::size_t>::digits;
    for (std::size_t offset = bits; offset > 0; --offset) {
        std::size_t carry = 0;

        // Double the current remainder modulo the denominator.
        if (remainder >= policy.rho_denominator - remainder) {
            remainder -= policy.rho_denominator - remainder;
            ++carry;
        } else {
            remainder += remainder;
        }

        // Append the next input bit multiplied by the numerator.
        const bool bit = ((value >> (offset - 1)) & std::size_t{1}) != 0;
        if (bit) {
            if (remainder >=
                policy.rho_denominator - policy.rho_numerator) {
                remainder -=
                    policy.rho_denominator - policy.rho_numerator;
                ++carry;
            } else {
                remainder += policy.rho_numerator;
            }
        }

        if (quotient >
            (std::numeric_limits<std::size_t>::max() - carry) / 2)
            throw std::overflow_error("FLOW_POLICY_THRESHOLD_OVERFLOW");
        quotient = quotient * 2 + carry;
    }
    return quotient;
}

inline bool shouldSchedulePeriodic(
    std::size_t active_nodes, std::size_t nodes_at_last_scheduled,
    std::size_t successful_layers_since_scheduled,
    const ScheduledFlowPolicy& policy) {
    const std::size_t threshold =
        scheduledNodeThreshold(nodes_at_last_scheduled, policy);
    return active_nodes <= threshold ||
           successful_layers_since_scheduled >= policy.max_successful_layers;
}

inline void checkedIncrementSuccessfulLayers(std::size_t& layers) {
    if (layers == std::numeric_limits<std::size_t>::max())
        throw std::overflow_error("FLOW_POLICY_LAYER_COUNT_OVERFLOW");
    ++layers;
}

enum class ScheduledBoundaryDecision {
    ContinueWithoutFlow,
    RunPending,
    SkipTargetReached
};

inline const char* scheduledBoundaryDecisionName(
    ScheduledBoundaryDecision decision) {
    switch (decision) {
        case ScheduledBoundaryDecision::ContinueWithoutFlow:
            return "CONTINUE_WITHOUT_FLOW";
        case ScheduledBoundaryDecision::RunPending:
            return "RUN_PENDING";
        case ScheduledBoundaryDecision::SkipTargetReached:
            return "SCHEDULED_SKIP_TARGET_REACHED";
    }
    return "UNKNOWN";
}

// Preserve the original target check before considering a pending scheduled
// invocation. A reached target therefore records a skip and creates no call.
inline ScheduledBoundaryDecision decideScheduledBoundary(
    std::size_t active_nodes, std::size_t target_nodes,
    bool scheduled_pending) {
    if (!scheduled_pending)
        return ScheduledBoundaryDecision::ContinueWithoutFlow;
    if (active_nodes <= target_nodes)
        return ScheduledBoundaryDecision::SkipTargetReached;
    return ScheduledBoundaryDecision::RunPending;
}

enum class ScheduledFlowAction {
    None,
    RunInitial,
    RunPeriodic,
    SkipTargetReached
};

inline const char* scheduledFlowActionName(ScheduledFlowAction action) {
    switch (action) {
        case ScheduledFlowAction::None:
            return "NONE";
        case ScheduledFlowAction::RunInitial:
            return "SCHEDULED_INITIAL";
        case ScheduledFlowAction::RunPeriodic:
            return "SCHEDULED_PERIODIC";
        case ScheduledFlowAction::SkipTargetReached:
            return "SCHEDULED_SKIP_TARGET_REACHED";
    }
    return "UNKNOWN";
}

struct ScheduledFlowState {
    bool initialized = false;
    std::size_t nodes_at_last_scheduled = 0;
    std::size_t successful_layers_since_scheduled = 0;
    bool scheduled_pending = false;
    ScheduledFlowPolicy policy = cutOnlyV0ScheduledPolicy();
};

struct ScheduledTriggerResult {
    bool counted_successful_layer;
    bool node_reduction;
    bool layer_limit;
    bool pending;
    std::size_t node_threshold;
    std::size_t successful_layers;
};

// The caller reaches this transition only after the original target check and
// community detection. RunInitial denotes the one synchronous initial sweep;
// its reset is independent of whether that sweep accepts any label movement.
inline ScheduledFlowAction beginScheduledFlowAfterDetection(
    ScheduledFlowState& state, std::size_t active_nodes) {
    if (state.initialized)
        throw std::logic_error("SCHEDULED_ALREADY_INITIALIZED");
    validateScheduledFlowPolicy(state.policy);
    state.initialized = true;
    state.nodes_at_last_scheduled = active_nodes;
    state.successful_layers_since_scheduled = 0;
    state.scheduled_pending = false;
    return ScheduledFlowAction::RunInitial;
}

// Only an attempt with at least one actual contraction is a successful layer.
// A zero-contraction attempt leaves B and the pending bit unchanged.
inline ScheduledTriggerResult noteCoarseningAttempt(
    ScheduledFlowState& state, std::size_t active_nodes,
    std::size_t contractions) {
    if (!state.initialized)
        throw std::logic_error("SCHEDULED_NOT_INITIALIZED");
    validateScheduledFlowPolicy(state.policy);
    const std::size_t threshold =
        scheduledNodeThreshold(state.nodes_at_last_scheduled, state.policy);
    if (contractions == 0)
        return {false, false, false, state.scheduled_pending, threshold,
                state.successful_layers_since_scheduled};

    checkedIncrementSuccessfulLayers(
        state.successful_layers_since_scheduled);
    const bool node_reduction = active_nodes <= threshold;
    const bool layer_limit =
        state.successful_layers_since_scheduled >=
        state.policy.max_successful_layers;
    state.scheduled_pending =
        state.scheduled_pending || node_reduction || layer_limit;
    return {true,
            node_reduction,
            layer_limit,
            state.scheduled_pending,
            threshold,
            state.successful_layers_since_scheduled};
}

inline ScheduledTriggerResult noteSuccessfulCoarseningLayer(
    ScheduledFlowState& state, std::size_t active_nodes) {
    return noteCoarseningAttempt(state, active_nodes, 1);
}

// This transition is taken at the synchronous scheduled call site. Returning
// RunPeriodic consumes exactly one pending call and resets Nlast/B regardless
// of the sweep's accepted-move count. TargetReached consumes only an existing
// pending call and does not pretend that a sweep ran.
inline ScheduledFlowAction takeScheduledActionBeforeCandidateBatch(
    ScheduledFlowState& state, std::size_t active_nodes,
    std::size_t target_nodes) {
    if (!state.initialized)
        throw std::logic_error("SCHEDULED_NOT_INITIALIZED");
    const ScheduledBoundaryDecision decision = decideScheduledBoundary(
        active_nodes, target_nodes, state.scheduled_pending);
    if (decision == ScheduledBoundaryDecision::ContinueWithoutFlow)
        return ScheduledFlowAction::None;
    if (decision == ScheduledBoundaryDecision::SkipTargetReached) {
        state.scheduled_pending = false;
        return ScheduledFlowAction::SkipTargetReached;
    }

    state.nodes_at_last_scheduled = active_nodes;
    state.successful_layers_since_scheduled = 0;
    state.scheduled_pending = false;
    return ScheduledFlowAction::RunPeriodic;
}

struct FlowRuleIdentity {
    std::string objective;
    std::string c1_cut;
    std::string c2_preparation;
    std::string c3_network;
    std::string c4_solver;
    std::string c5_transaction;
    std::string c6_sweep;
};

inline bool operator==(const FlowRuleIdentity& left,
                       const FlowRuleIdentity& right) {
    return left.objective == right.objective &&
           left.c1_cut == right.c1_cut &&
           left.c2_preparation == right.c2_preparation &&
           left.c3_network == right.c3_network &&
           left.c4_solver == right.c4_solver &&
           left.c5_transaction == right.c5_transaction &&
           left.c6_sweep == right.c6_sweep;
}

inline FlowRuleIdentity cutOnlyV0RuleIdentity() {
    return {"CUT_ONLY_V0",
            "C1_WEIGHTED_CUT_RECOMPUTE_V0",
            "C2_PAIR_CORRIDOR_V0",
            "C3_LAWLER_UINT64_V0",
            "C4_DETERMINISTIC_DINIC_V0",
            "C5_STRICT_DESCENT_TRANSACTION_V0",
            "C6_FROZEN_PAIR_SWEEP_V0"};
}

inline FlowRuleIdentity externalFlowV0RuleIdentity() {
    return {"NORMALIZED_AFFECTED_CUT_PLUS_LAMBDA1_COSINE_PHI_V0",
            "D1_EXTERNAL_PROFILE_CURRENT_NODE_IDS_W_OVER_OUTSIDE_COUNT_V0",
            "C2_PAIR_CORRIDOR_NO_ONE_HOP_EXPANSION_V0",
            "D3_LAWLER_ALL_CAPACITIES_FIXED_POINT_1E6_V0",
            "C4_DETERMINISTIC_DINIC_V0",
            "D4_STRICT_J_DESCENT_C_FALLBACK_V0",
            "D5_FROZEN_PAIR_SWEEP_V0"};
}

struct FlowStateNet {
    std::size_t net;
    CutWeight weight;
    std::vector<std::size_t> endpoints;
};

inline bool operator==(const FlowStateNet& left, const FlowStateNet& right) {
    return left.net == right.net && left.weight == right.weight &&
           left.endpoints == right.endpoints;
}

struct FlowStateLabel {
    std::size_t node;
    int community;
};

inline bool operator==(const FlowStateLabel& left,
                       const FlowStateLabel& right) {
    return left.node == right.node && left.community == right.community;
}

struct FlowStatePair {
    int community_i;
    int community_j;
    CutWeight boundary_weight;
    std::size_t anchor_i;
    std::size_t anchor_j;
};

inline bool operator==(const FlowStatePair& left,
                       const FlowStatePair& right) {
    return left.community_i == right.community_i &&
           left.community_j == right.community_j &&
           left.boundary_weight == right.boundary_weight &&
           left.anchor_i == right.anchor_i &&
           left.anchor_j == right.anchor_j;
}

struct FlowStateCorridorEndpoint {
    std::size_t node;
    int community;
    bool movable;
};

inline bool operator==(const FlowStateCorridorEndpoint& left,
                       const FlowStateCorridorEndpoint& right) {
    return left.node == right.node && left.community == right.community &&
           left.movable == right.movable;
}

struct FlowStateCorridorNet {
    std::size_t net;
    CutWeight weight;
    std::vector<FlowStateCorridorEndpoint> endpoints;
};

inline bool operator==(const FlowStateCorridorNet& left,
                       const FlowStateCorridorNet& right) {
    return left.net == right.net && left.weight == right.weight &&
           left.endpoints == right.endpoints;
}

struct FlowStateCorridor {
    int community_i;
    int community_j;
    CutWeight boundary_weight;
    std::size_t anchor_i;
    std::size_t anchor_j;
    std::vector<std::size_t> boundary_nodes;
    std::vector<std::size_t> movable_nodes;
    std::vector<std::size_t> fixed_i;
    std::vector<std::size_t> fixed_j;
    std::vector<FlowStateCorridorNet> related_nets;
};

inline bool operator==(const FlowStateCorridor& left,
                       const FlowStateCorridor& right) {
    return left.community_i == right.community_i &&
           left.community_j == right.community_j &&
           left.boundary_weight == right.boundary_weight &&
           left.anchor_i == right.anchor_i &&
           left.anchor_j == right.anchor_j &&
           left.boundary_nodes == right.boundary_nodes &&
           left.movable_nodes == right.movable_nodes &&
           left.fixed_i == right.fixed_i &&
           left.fixed_j == right.fixed_j &&
           left.related_nets == right.related_nets;
}

struct FlowEntryPairState {
    FlowStatePair frozen_pair;
    FlowStateCorridor entry_corridor;
};

inline bool operator==(const FlowEntryPairState& left,
                       const FlowEntryPairState& right) {
    return left.frozen_pair == right.frozen_pair &&
           left.entry_corridor == right.entry_corridor;
}

struct FlowInputState {
    std::vector<std::size_t> active_nodes;
    std::vector<FlowStateNet> active_nets;
    std::vector<FlowStateLabel> labels;
    std::vector<FlowEntryPairState> entry_pairs;
    FlowRuleIdentity rules;
    ScheduledFlowPolicy scheduled_policy;
};

inline bool operator==(const FlowInputState& left,
                       const FlowInputState& right) {
    return left.active_nodes == right.active_nodes &&
           left.active_nets == right.active_nets &&
           left.labels == right.labels &&
           left.entry_pairs == right.entry_pairs &&
           left.rules == right.rules &&
           left.scheduled_policy == right.scheduled_policy;
}

namespace flow_lifecycle_state_detail {

inline FlowStatePair copyPair(const CommunityPair& pair) {
    return {pair.community_i, pair.community_j, pair.boundary_weight,
            pair.anchor_i, pair.anchor_j};
}

inline FlowStateCorridor copyCorridor(const PairCorridor& corridor) {
    FlowStateCorridor result{corridor.community_i,
                             corridor.community_j,
                             corridor.boundary_weight,
                             corridor.anchor_i,
                             corridor.anchor_j,
                             corridor.boundary_nodes,
                             corridor.movable_nodes,
                             corridor.fixed_i,
                             corridor.fixed_j,
                             {}};
    result.related_nets.reserve(corridor.related_nets.size());
    for (const CorridorNet& net : corridor.related_nets) {
        FlowStateCorridorNet copied{net.net, net.weight, {}};
        copied.endpoints.reserve(net.endpoints.size());
        for (const CorridorEndpoint& endpoint : net.endpoints) {
            copied.endpoints.push_back(
                {endpoint.node, endpoint.community, endpoint.movable});
        }
        result.related_nets.push_back(std::move(copied));
    }
    return result;
}

inline void hashByte(std::uint64_t& hash, unsigned char byte) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= UINT64_C(1099511628211);
}

inline void hashUnsigned(std::uint64_t& hash, std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64; shift += 8)
        hashByte(hash, static_cast<unsigned char>((value >> shift) & 0xffU));
}

inline void hashSigned(std::uint64_t& hash, int value) {
    hashUnsigned(hash,
                 static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
}

inline void hashSize(std::uint64_t& hash, std::size_t value) {
    hashUnsigned(hash, static_cast<std::uint64_t>(value));
}

inline void hashString(std::uint64_t& hash, const std::string& value) {
    hashSize(hash, value.size());
    for (unsigned char byte : value) hashByte(hash, byte);
}

template <typename T, typename HashElement>
inline void hashVector(std::uint64_t& hash, const std::vector<T>& values,
                       HashElement hash_element) {
    hashSize(hash, values.size());
    for (const T& value : values) hash_element(hash, value);
}

inline void hashPair(std::uint64_t& hash, const FlowStatePair& pair) {
    hashSigned(hash, pair.community_i);
    hashSigned(hash, pair.community_j);
    hashUnsigned(hash, pair.boundary_weight);
    hashSize(hash, pair.anchor_i);
    hashSize(hash, pair.anchor_j);
}

inline void hashCorridor(std::uint64_t& hash,
                         const FlowStateCorridor& corridor) {
    hashSigned(hash, corridor.community_i);
    hashSigned(hash, corridor.community_j);
    hashUnsigned(hash, corridor.boundary_weight);
    hashSize(hash, corridor.anchor_i);
    hashSize(hash, corridor.anchor_j);
    const auto hash_size_value = [](std::uint64_t& target,
                                    std::size_t value) {
        hashSize(target, value);
    };
    hashVector(hash, corridor.boundary_nodes, hash_size_value);
    hashVector(hash, corridor.movable_nodes, hash_size_value);
    hashVector(hash, corridor.fixed_i, hash_size_value);
    hashVector(hash, corridor.fixed_j, hash_size_value);
    hashVector(hash, corridor.related_nets,
               [](std::uint64_t& target, const FlowStateCorridorNet& net) {
                   hashSize(target, net.net);
                   hashUnsigned(target, net.weight);
                   hashVector(
                       target, net.endpoints,
                       [](std::uint64_t& endpoint_hash,
                          const FlowStateCorridorEndpoint& endpoint) {
                           hashSize(endpoint_hash, endpoint.node);
                           hashSigned(endpoint_hash, endpoint.community);
                           hashUnsigned(endpoint_hash,
                                        endpoint.movable ? 1U : 0U);
                       });
               });
}

}  // namespace flow_lifecycle_state_detail

// Build the complete pre-call K. Both the frozen pair list and every entry
// corridor are derived from the same unmodified flow-start labels.
inline FlowInputState buildFlowInputState(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels,
    const FlowRuleIdentity& rules,
    const ScheduledFlowPolicy& scheduled_policy) {
    validateScheduledFlowPolicy(scheduled_policy);
    const std::vector<CommunityPair> pairs =
        buildCommunityPairs(hypergraph, community_labels);

    FlowInputState state{{}, {}, {}, {}, rules, scheduled_policy};
    for (std::size_t node = 0; node < hypergraph.hypernodes.size(); ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        state.active_nodes.push_back(node);
        state.labels.push_back({node, community_labels[node]});
    }

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        const Hyperedge& net = hypergraph.hyperedges[edge];
        if (net.deleted) continue;
        const std::vector<std::size_t> endpoints =
            flow_preparation_detail::activeNetEndpoints(hypergraph, edge);
        state.active_nets.push_back(
            {edge, static_cast<CutWeight>(net.weight), endpoints});
    }

    state.entry_pairs.reserve(pairs.size());
    for (const CommunityPair& pair : pairs) {
        const PairCorridor corridor = buildPairCorridor(
            hypergraph, community_labels, pair.community_i, pair.community_j);
        state.entry_pairs.push_back(
            {flow_lifecycle_state_detail::copyPair(pair),
             flow_lifecycle_state_detail::copyCorridor(corridor)});
    }
    return state;
}

inline FlowInputState buildCutOnlyV0FlowInputState(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels) {
    return buildFlowInputState(hypergraph, community_labels,
                               cutOnlyV0RuleIdentity(),
                               cutOnlyV0ScheduledPolicy());
}

inline FlowInputState buildExternalFlowV0InputState(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels) {
    return buildFlowInputState(hypergraph, community_labels,
                               externalFlowV0RuleIdentity(),
                               cutOnlyV0ScheduledPolicy());
}

inline std::uint64_t flowInputStateHash(const FlowInputState& state) {
    using namespace flow_lifecycle_state_detail;
    std::uint64_t hash = UINT64_C(1469598103934665603);
    hashVector(hash, state.active_nodes,
               [](std::uint64_t& target, std::size_t value) {
                   hashSize(target, value);
               });
    hashVector(hash, state.active_nets,
               [](std::uint64_t& target, const FlowStateNet& net) {
                   hashSize(target, net.net);
                   hashUnsigned(target, net.weight);
                   hashVector(target, net.endpoints,
                              [](std::uint64_t& endpoint_hash,
                                 std::size_t endpoint) {
                                  hashSize(endpoint_hash, endpoint);
                              });
               });
    hashVector(hash, state.labels,
               [](std::uint64_t& target, const FlowStateLabel& label) {
                   hashSize(target, label.node);
                   hashSigned(target, label.community);
               });
    hashVector(hash, state.entry_pairs,
               [](std::uint64_t& target, const FlowEntryPairState& entry) {
                   hashPair(target, entry.frozen_pair);
                   hashCorridor(target, entry.entry_corridor);
               });
    hashString(hash, state.rules.objective);
    hashString(hash, state.rules.c1_cut);
    hashString(hash, state.rules.c2_preparation);
    hashString(hash, state.rules.c3_network);
    hashString(hash, state.rules.c4_solver);
    hashString(hash, state.rules.c5_transaction);
    hashString(hash, state.rules.c6_sweep);
    hashSize(hash, state.scheduled_policy.rho_numerator);
    hashSize(hash, state.scheduled_policy.rho_denominator);
    hashSize(hash, state.scheduled_policy.max_successful_layers);
    return hash;
}

enum class FlowInvocationKind {
    ScheduledInitial,
    ScheduledPeriodic,
    Recovery
};

inline const char* flowInvocationKindName(FlowInvocationKind kind) {
    switch (kind) {
        case FlowInvocationKind::ScheduledInitial:
            return "SCHEDULED_INITIAL";
        case FlowInvocationKind::ScheduledPeriodic:
            return "SCHEDULED_PERIODIC";
        case FlowInvocationKind::Recovery:
            return "RECOVERY";
    }
    return "UNKNOWN";
}

struct FlowInvocationRecord {
    std::size_t call_id;
    FlowInvocationKind kind;
    std::uint64_t state_hash;
    std::uint64_t bucket_hash;
    std::size_t state_bucket;
    std::size_t attempted_state_index;
    bool state_was_already_attempted;
};

// Hashes select candidate buckets only. Every lookup compares the complete K;
// bucket_count=1 is therefore a supported collision-stress configuration.
class FlowAttemptedStates {
public:
    explicit FlowAttemptedStates(
        std::size_t bucket_count = 64,
        std::uint64_t bucket_hash_mask =
            std::numeric_limits<std::uint64_t>::max())
        : buckets_(bucket_count), bucket_hash_mask_(bucket_hash_mask) {
        if (bucket_count == 0)
            throw std::invalid_argument("FLOW_ATTEMPT_ZERO_BUCKETS");
    }

    bool contains(const FlowInputState& state) const {
        const std::uint64_t bucket_hash = bucketHash(state);
        return findState(state, bucket_hash) != noIndex();
    }

    FlowInvocationRecord registerBeforeCall(FlowInvocationKind kind,
                                            const FlowInputState& state) {
        if (invocations_.size() == std::numeric_limits<std::size_t>::max())
            throw std::overflow_error("FLOW_CALL_ID_OVERFLOW");
        const std::uint64_t hash = flowInputStateHash(state);
        const std::uint64_t bucket_hash = hash & bucket_hash_mask_;
        const std::size_t bucket = bucketFor(bucket_hash);
        std::size_t state_index = findState(state, bucket_hash);
        const bool already_attempted = state_index != noIndex();
        if (!already_attempted) {
            state_index = states_.size();
            states_.push_back({bucket_hash, state});
            buckets_[bucket].push_back(state_index);
        }

        const FlowInvocationRecord record{invocations_.size() + 1,
                                          kind,
                                          hash,
                                          bucket_hash,
                                          bucket,
                                          state_index,
                                          already_attempted};
        invocations_.push_back(record);
        return record;
    }

    std::size_t attemptedStateCount() const { return states_.size(); }
    std::size_t invocationCount() const { return invocations_.size(); }
    std::size_t bucketCount() const { return buckets_.size(); }
    std::uint64_t bucketHashMask() const { return bucket_hash_mask_; }

    const FlowInputState& attemptedState(std::size_t index) const {
        if (index >= states_.size())
            throw std::out_of_range("FLOW_ATTEMPT_STATE_INDEX");
        return states_[index].state;
    }

    const std::vector<FlowInvocationRecord>& invocations() const {
        return invocations_;
    }

private:
    struct StoredState {
        std::uint64_t bucket_hash;
        FlowInputState state;
    };

    static std::size_t noIndex() {
        return std::numeric_limits<std::size_t>::max();
    }

    std::size_t bucketFor(std::uint64_t hash) const {
        return static_cast<std::size_t>(
            hash % static_cast<std::uint64_t>(buckets_.size()));
    }

    std::uint64_t bucketHash(const FlowInputState& state) const {
        return flowInputStateHash(state) & bucket_hash_mask_;
    }

    std::size_t findState(const FlowInputState& state,
                          std::uint64_t bucket_hash) const {
        const std::size_t bucket = bucketFor(bucket_hash);
        for (const std::size_t index : buckets_[bucket]) {
            const StoredState& stored = states_[index];
            if (stored.bucket_hash == bucket_hash && stored.state == state)
                return index;
        }
        return noIndex();
    }

    std::vector<std::vector<std::size_t>> buckets_;
    std::uint64_t bucket_hash_mask_;
    std::vector<StoredState> states_;
    std::vector<FlowInvocationRecord> invocations_;
};

}  // namespace cf
