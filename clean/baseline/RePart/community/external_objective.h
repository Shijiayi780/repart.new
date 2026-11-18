#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "external_profile.h"

namespace cf {

constexpr long double kExternalLambdaV0 = 1.0L;
constexpr CutWeight kExternalCapacityScaleV0 = UINT64_C(1000000);

struct ExternalDistance {
    bool informative = false;
    long double distance = 0.0L;
    long double strength = 0.0L;
};

struct ExternalPairObjective {
    CutWeight raw_cut = 0;
    CutWeight affected_cut_weight = 0;
    CutWeight affected_total_weight = 0;
    long double normalized_cut = 0.0L;
    std::optional<long double> phi;
    std::optional<long double> objective;
    std::size_t comparable_members = 0;
};

namespace external_objective_detail {

struct CommunitySnapshot {
    int community = -1;
    std::vector<std::size_t> members;
    std::vector<SparseExternalVector> profiles;
    std::map<std::size_t, long double> normalized_sum;
};

inline std::vector<std::size_t> communityMembers(
    const Hypergraph& hypergraph, const std::vector<int>& labels,
    int community) {
    if (labels.size() != hypergraph.hypernodes.size())
        throw std::invalid_argument("EXTERNAL_LABEL_SHAPE");
    std::vector<std::size_t> result;
    for (std::size_t node = 0; node < labels.size(); ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        if (labels[node] < 0)
            throw std::invalid_argument("EXTERNAL_ACTIVE_LABEL");
        if (labels[node] == community) result.push_back(node);
    }
    return result;
}

inline CommunitySnapshot buildCommunitySnapshot(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    const std::vector<int>& labels,
    int community) {
    CommunitySnapshot result;
    result.community = community;
    result.members = communityMembers(hypergraph, labels, community);
    const std::vector<unsigned char> mask =
        makeExternalGroupMask(workspace, result.members);
    result.profiles.reserve(result.members.size());
    for (const std::size_t node : result.members) {
        SparseExternalVector profile =
            buildExternalProfile(hypergraph, workspace, node, mask);
        if (profile.informative()) {
            for (const auto& entry : profile.entries)
                result.normalized_sum[entry.first] += entry.second;
        }
        result.profiles.push_back(std::move(profile));
    }
    return result;
}

inline const SparseExternalVector* memberProfile(
    const CommunitySnapshot& snapshot, std::size_t node) {
    const auto position =
        std::lower_bound(snapshot.members.begin(), snapshot.members.end(), node);
    if (position == snapshot.members.end() || *position != node) return nullptr;
    return &snapshot.profiles[static_cast<std::size_t>(
        position - snapshot.members.begin())];
}

inline void subtractProfile(
    std::map<std::size_t, long double>& values,
    const SparseExternalVector& profile) {
    if (!profile.informative()) return;
    for (const auto& entry : profile.entries) {
        auto found = values.find(entry.first);
        if (found == values.end())
            throw std::logic_error("EXTERNAL_PROTOTYPE_SUBTRACT_MISSING");
        found->second -= entry.second;
        if (found->second == 0.0L) values.erase(found);
    }
}

inline bool netIsCut(const std::vector<std::size_t>& endpoints,
                     const std::vector<int>& labels) {
    const int first = labels[endpoints.front()];
    for (std::size_t index = 1; index < endpoints.size(); ++index)
        if (labels[endpoints[index]] != first) return true;
    return false;
}

inline void checkedWeightAdd(CutWeight& target, CutWeight value,
                             const char* message) {
    if (value > std::numeric_limits<CutWeight>::max() - target)
        throw std::overflow_error(message);
    target += value;
}

}  // namespace external_objective_detail

// Frozen terminal distance for assigning node to target.  The target center
// is built from the flow-entry labels, excludes node's own qhat contribution
// when it is already a member, and always masks dimension node because that
// object becomes internal under the candidate assignment.
inline ExternalDistance frozenExternalDistance(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    const std::vector<int>& frozen_labels,
    std::size_t node,
    const external_objective_detail::CommunitySnapshot& target) {
    std::vector<std::size_t> candidate_members = target.members;
    if (!std::binary_search(candidate_members.begin(), candidate_members.end(),
                            node)) {
        candidate_members.insert(
            std::lower_bound(candidate_members.begin(), candidate_members.end(),
                             node),
            node);
    }
    const std::vector<unsigned char> candidate_mask =
        makeExternalGroupMask(workspace, candidate_members);
    const SparseExternalVector candidate =
        buildExternalProfile(hypergraph, workspace, node, candidate_mask);

    std::map<std::size_t, long double> center = target.normalized_sum;
    if (const SparseExternalVector* own =
            external_objective_detail::memberProfile(target, node))
        external_objective_detail::subtractProfile(center, *own);
    const auto prototype = l2NormalizeSparse(center, node);
    if (!candidate.informative() || prototype.empty())
        return {false, 0.0L, candidate.strength};
    return {true, cosineDistance(candidate.entries, prototype),
            candidate.strength};
}

inline ExternalPairObjective recomputeExternalPairObjective(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    const std::vector<int>& labels,
    int community_i,
    int community_j) {
    if (community_i == community_j || community_i < 0 || community_j < 0)
        throw std::invalid_argument("EXTERNAL_OBJECTIVE_INVALID_PAIR");
    ExternalPairObjective result;
    result.raw_cut = recomputeCut(hypergraph, labels);

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        if (hypergraph.hyperedges[edge].deleted) continue;
        const auto& endpoints = workspace.net_endpoints.at(edge);
        bool affected = false;
        for (const std::size_t node : endpoints) {
            affected = affected || labels[node] == community_i ||
                       labels[node] == community_j;
        }
        if (!affected) continue;
        const CutWeight weight =
            static_cast<CutWeight>(hypergraph.hyperedges[edge].weight);
        external_objective_detail::checkedWeightAdd(
            result.affected_total_weight, weight,
            "EXTERNAL_AFFECTED_TOTAL_OVERFLOW");
        if (external_objective_detail::netIsCut(endpoints, labels))
            external_objective_detail::checkedWeightAdd(
                result.affected_cut_weight, weight,
                "EXTERNAL_AFFECTED_CUT_OVERFLOW");
    }
    if (result.affected_total_weight == 0)
        throw std::logic_error("EXTERNAL_AFFECTED_ZERO_DENOMINATOR");
    result.normalized_cut =
        static_cast<long double>(result.affected_cut_weight) /
        static_cast<long double>(result.affected_total_weight);

    long double phi_sum = 0.0L;
    for (const int community : {community_i, community_j}) {
        const auto snapshot = external_objective_detail::buildCommunitySnapshot(
            hypergraph, workspace, labels, community);
        for (std::size_t index = 0; index < snapshot.members.size(); ++index) {
            const SparseExternalVector& profile = snapshot.profiles[index];
            if (!profile.informative()) continue;
            std::map<std::size_t, long double> leave_one_out =
                snapshot.normalized_sum;
            external_objective_detail::subtractProfile(leave_one_out, profile);
            const auto prototype = l2NormalizeSparse(leave_one_out);
            if (prototype.empty()) continue;
            phi_sum += cosineDistance(profile.entries, prototype);
            ++result.comparable_members;
        }
    }
    if (result.comparable_members == 0) return result;
    result.phi = phi_sum /
                 static_cast<long double>(result.comparable_members);
    result.objective = result.normalized_cut +
                       kExternalLambdaV0 * *result.phi;
    return result;
}

}  // namespace cf
