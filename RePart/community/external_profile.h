#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "flow_preparation.h"

namespace cf {

struct SparseExternalVector {
    std::vector<std::pair<std::size_t, long double>> entries;
    long double strength = 0.0L;

    bool informative() const { return strength > 0.0L && !entries.empty(); }
};

// The active endpoint and incidence snapshot is independent of temporary
// community labels.  A D sweep builds it once and reuses it for every pair.
struct ExternalProfileWorkspace {
    std::vector<std::vector<std::size_t>> net_endpoints;
    std::vector<std::vector<std::size_t>> incident_nets;
    std::vector<unsigned char> active_nodes;
};

inline ExternalProfileWorkspace buildExternalProfileWorkspace(
    const Hypergraph& hypergraph) {
    ExternalProfileWorkspace result;
    result.net_endpoints.resize(hypergraph.hyperedges.size());
    result.incident_nets.resize(hypergraph.hypernodes.size());
    result.active_nodes.assign(hypergraph.hypernodes.size(), 0);
    for (std::size_t node = 0; node < hypergraph.hypernodes.size(); ++node)
        result.active_nodes[node] = hypergraph.hypernodes[node].deleted ? 0 : 1;

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        if (hypergraph.hyperedges[edge].deleted) continue;
        result.net_endpoints[edge] =
            flow_preparation_detail::activeNetEndpoints(hypergraph, edge);
        for (const std::size_t node : result.net_endpoints[edge])
            result.incident_nets[node].push_back(edge);
    }
    return result;
}

inline std::vector<unsigned char> makeExternalGroupMask(
    const ExternalProfileWorkspace& workspace,
    const std::vector<std::size_t>& members) {
    std::vector<unsigned char> mask(workspace.active_nodes.size(), 0);
    for (const std::size_t node : members) {
        if (node >= mask.size() || workspace.active_nodes[node] == 0)
            throw std::invalid_argument("EXTERNAL_PROFILE_INVALID_GROUP_NODE node=" +
                                        std::to_string(node));
        mask[node] = 1;
    }
    return mask;
}

inline SparseExternalVector buildExternalProfile(
    const Hypergraph& hypergraph,
    const ExternalProfileWorkspace& workspace,
    std::size_t node,
    const std::vector<unsigned char>& group_mask) {
    if (workspace.active_nodes.size() != hypergraph.hypernodes.size() ||
        workspace.incident_nets.size() != hypergraph.hypernodes.size() ||
        workspace.net_endpoints.size() != hypergraph.hyperedges.size())
        throw std::invalid_argument("EXTERNAL_PROFILE_WORKSPACE_SHAPE");
    if (group_mask.size() != workspace.active_nodes.size())
        throw std::invalid_argument("EXTERNAL_PROFILE_GROUP_SHAPE");
    if (node >= workspace.active_nodes.size() || workspace.active_nodes[node] == 0)
        throw std::invalid_argument("EXTERNAL_PROFILE_INACTIVE_NODE node=" +
                                    std::to_string(node));
    if (group_mask[node] == 0)
        throw std::invalid_argument("EXTERNAL_PROFILE_NODE_OUTSIDE_GROUP node=" +
                                    std::to_string(node));

    std::map<std::size_t, long double> accumulated;
    long double strength = 0.0L;
    for (const std::size_t edge : workspace.incident_nets[node]) {
        if (edge >= hypergraph.hyperedges.size() ||
            hypergraph.hyperedges[edge].deleted)
            throw std::logic_error("EXTERNAL_PROFILE_STALE_WORKSPACE");
        const std::vector<std::size_t>& endpoints =
            workspace.net_endpoints[edge];
        std::size_t outside_count = 0;
        for (const std::size_t endpoint : endpoints)
            if (group_mask[endpoint] == 0) ++outside_count;
        if (outside_count == 0) continue;

        const long double share =
            static_cast<long double>(hypergraph.hyperedges[edge].weight) /
            static_cast<long double>(outside_count);
        for (const std::size_t endpoint : endpoints) {
            if (group_mask[endpoint] != 0) continue;
            accumulated[endpoint] += share;
            strength += share;
        }
    }

    SparseExternalVector result;
    result.strength = strength;
    if (!(strength > 0.0L)) return result;
    result.entries.reserve(accumulated.size());
    for (const auto& entry : accumulated)
        result.entries.push_back({entry.first, entry.second / strength});
    return result;
}

inline long double sparseL2Norm(
    const std::vector<std::pair<std::size_t, long double>>& values) {
    long double sum = 0.0L;
    for (const auto& entry : values) sum += entry.second * entry.second;
    return std::sqrt(sum);
}

inline std::vector<std::pair<std::size_t, long double>> l2NormalizeSparse(
    const std::map<std::size_t, long double>& values,
    std::size_t masked_dimension = static_cast<std::size_t>(-1)) {
    long double norm_squared = 0.0L;
    for (const auto& entry : values) {
        if (entry.first == masked_dimension || entry.second == 0.0L) continue;
        norm_squared += entry.second * entry.second;
    }
    if (!(norm_squared > 0.0L)) return {};
    const long double norm = std::sqrt(norm_squared);
    std::vector<std::pair<std::size_t, long double>> result;
    result.reserve(values.size());
    for (const auto& entry : values) {
        if (entry.first == masked_dimension || entry.second == 0.0L) continue;
        result.push_back({entry.first, entry.second / norm});
    }
    return result;
}

inline long double cosineDistance(
    const std::vector<std::pair<std::size_t, long double>>& left,
    const std::vector<std::pair<std::size_t, long double>>& right) {
    if (left.empty() || right.empty())
        throw std::invalid_argument("EXTERNAL_COSINE_EMPTY_VECTOR");
    std::size_t i = 0, j = 0;
    long double dot = 0.0L;
    while (i < left.size() && j < right.size()) {
        if (left[i].first < right[j].first) {
            ++i;
        } else if (right[j].first < left[i].first) {
            ++j;
        } else {
            dot += left[i].second * right[j].second;
            ++i;
            ++j;
        }
    }
    const long double denominator = sparseL2Norm(left) * sparseL2Norm(right);
    if (!(denominator > 0.0L))
        throw std::invalid_argument("EXTERNAL_COSINE_ZERO_NORM");
    long double cosine = dot / denominator;
    if (cosine > 1.0L) cosine = 1.0L;
    if (cosine < -1.0L) cosine = -1.0L;
    return 1.0L - cosine;
}

}  // namespace cf
