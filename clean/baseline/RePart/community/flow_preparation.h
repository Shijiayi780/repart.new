#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cut_recompute.h"

namespace cf {

// A sweep freezes these unordered community identities in this deterministic
// order. community_i/community_j are oriented by the anchors observed when the
// list is built; callers must use the community IDs, not the anchors, as the
// persistent pair identity.
struct CommunityPair {
    int community_i;
    int community_j;
    CutWeight boundary_weight;
    std::size_t anchor_i;
    std::size_t anchor_j;
};

struct CorridorEndpoint {
    std::size_t node;
    int community;
    bool movable;
};

// Retain the complete deduplicated endpoint set and the current endpoint
// communities. This is deliberately richer than a list of corridor pins: a
// later flow-network builder needs internal and third-community fixed pins too.
struct CorridorNet {
    std::size_t net;
    CutWeight weight;
    std::vector<CorridorEndpoint> endpoints;
};

struct PairCorridor {
    int community_i;
    int community_j;
    CutWeight boundary_weight;
    std::size_t anchor_i;
    std::size_t anchor_j;
    std::vector<std::size_t> boundary_nodes;
    std::vector<std::size_t> movable_nodes;
    std::vector<std::size_t> fixed_i;
    std::vector<std::size_t> fixed_j;
    std::vector<CorridorNet> related_nets;
};

namespace flow_preparation_detail {

using CommunityAnchors = std::map<int, std::size_t>;

inline CommunityAnchors validateLabelsAndFindAnchors(
    const Hypergraph& hypergraph, const std::vector<int>& community_labels) {
    const std::size_t node_count = hypergraph.hypernodes.size();
    if (community_labels.size() != node_count)
        throw std::invalid_argument("FLOW_PREP_LABEL_SHAPE");

    CommunityAnchors anchors;
    for (std::size_t node = 0; node < node_count; ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        const int community = community_labels[node];
        if (community < 0)
            throw std::invalid_argument("FLOW_PREP_ACTIVE_LABEL node=" +
                                        std::to_string(node));
        const auto inserted = anchors.emplace(community, node);
        if (!inserted.second && node < inserted.first->second)
            inserted.first->second = node;
    }
    return anchors;
}

inline std::vector<std::size_t> activeNetEndpoints(
    const Hypergraph& hypergraph, std::size_t edge) {
    const Hyperedge& net = hypergraph.hyperedges[edge];
    const std::string context = " net=" + std::to_string(edge);
    if (net._begin < 0 || net._size <= 0)
        throw std::invalid_argument("FLOW_PREP_INVALID_INCIDENCE_RANGE" + context);
    if (net.weight < 0)
        throw std::invalid_argument("FLOW_PREP_NEGATIVE_NET_WEIGHT" + context);

    const std::size_t begin = static_cast<std::size_t>(net._begin);
    const std::size_t count = static_cast<std::size_t>(net._size);
    if (begin > hypergraph._incidence_array.size() ||
        count > hypergraph._incidence_array.size() - begin)
        throw std::invalid_argument("FLOW_PREP_INVALID_INCIDENCE_RANGE" + context);

    std::vector<std::size_t> endpoints;
    endpoints.reserve(count);
    for (std::size_t pin = 0; pin < count; ++pin) {
        const int node = hypergraph._incidence_array[begin + pin];
        if (node < 0 ||
            static_cast<std::size_t>(node) >= hypergraph.hypernodes.size() ||
            hypergraph.hypernodes[static_cast<std::size_t>(node)].deleted)
            throw std::invalid_argument("FLOW_PREP_INVALID_ACTIVE_ENDPOINT" + context);
        endpoints.push_back(static_cast<std::size_t>(node));
    }
    std::sort(endpoints.begin(), endpoints.end());
    endpoints.erase(std::unique(endpoints.begin(), endpoints.end()), endpoints.end());
    return endpoints;
}

inline void checkedAdd(CutWeight& total, CutWeight weight,
                       const std::string& context) {
    if (weight > std::numeric_limits<CutWeight>::max() - total)
        throw std::overflow_error("FLOW_PREP_WEIGHT_SUM_OVERFLOW" + context);
    total += weight;
}

}  // namespace flow_preparation_detail

// Build and order the complete positive-weight pair list for one sweep.
// A multi-community net contributes its full weight to every co-occurring
// unordered pair. This scheduling weight is not the Cut_H objective.
inline std::vector<CommunityPair> buildCommunityPairs(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels) {
    const auto anchors = flow_preparation_detail::validateLabelsAndFindAnchors(
        hypergraph, community_labels);
    std::map<std::pair<int, int>, CutWeight> pair_weights;

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        const Hyperedge& net = hypergraph.hyperedges[edge];
        if (net.deleted) continue;
        const auto endpoints = flow_preparation_detail::activeNetEndpoints(
            hypergraph, edge);

        std::vector<int> communities;
        communities.reserve(endpoints.size());
        for (const std::size_t node : endpoints)
            communities.push_back(community_labels[node]);
        std::sort(communities.begin(), communities.end());
        communities.erase(std::unique(communities.begin(), communities.end()),
                          communities.end());

        const CutWeight weight = static_cast<CutWeight>(net.weight);
        for (std::size_t first = 0; first < communities.size(); ++first) {
            for (std::size_t second = first + 1; second < communities.size(); ++second) {
                const auto key = std::make_pair(communities[first], communities[second]);
                auto& total = pair_weights[key];
                flow_preparation_detail::checkedAdd(
                    total, weight, " net=" + std::to_string(edge) +
                                       " pair=" + std::to_string(key.first) + "," +
                                       std::to_string(key.second));
            }
        }
    }

    std::vector<CommunityPair> pairs;
    pairs.reserve(pair_weights.size());
    for (const auto& entry : pair_weights) {
        if (entry.second == 0) continue;
        int community_i = entry.first.first;
        int community_j = entry.first.second;
        std::size_t anchor_i = anchors.at(community_i);
        std::size_t anchor_j = anchors.at(community_j);
        if (anchor_j < anchor_i) {
            std::swap(community_i, community_j);
            std::swap(anchor_i, anchor_j);
        }
        pairs.push_back({community_i, community_j, entry.second,
                         anchor_i, anchor_j});
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const CommunityPair& left, const CommunityPair& right) {
                  if (left.boundary_weight != right.boundary_weight)
                      return left.boundary_weight > right.boundary_weight;
                  if (left.anchor_i != right.anchor_i)
                      return left.anchor_i < right.anchor_i;
                  if (left.anchor_j != right.anchor_j)
                      return left.anchor_j < right.anchor_j;
                  if (left.community_i != right.community_i)
                      return left.community_i < right.community_i;
                  return left.community_j < right.community_j;
              });
    return pairs;
}

// Rebuild one frozen community-ID pair against the latest committed labels.
// The returned boundary weight, anchors, corridor, fixed sets, and related nets
// are all current; neither the hypergraph nor the labels are modified.
inline PairCorridor buildPairCorridor(
    const Hypergraph& hypergraph,
    const std::vector<int>& community_labels,
    int community_a, int community_b) {
    const auto anchors = flow_preparation_detail::validateLabelsAndFindAnchors(
        hypergraph, community_labels);
    if (community_a < 0 || community_b < 0 || community_a == community_b)
        throw std::invalid_argument("FLOW_PREP_INVALID_PAIR");
    if (anchors.find(community_a) == anchors.end() ||
        anchors.find(community_b) == anchors.end())
        throw std::invalid_argument("FLOW_PREP_PAIR_COMMUNITY_MISSING");

    int community_i = community_a;
    int community_j = community_b;
    std::size_t anchor_i = anchors.at(community_i);
    std::size_t anchor_j = anchors.at(community_j);
    if (anchor_j < anchor_i) {
        std::swap(community_i, community_j);
        std::swap(anchor_i, anchor_j);
    }

    const std::size_t node_count = hypergraph.hypernodes.size();
    std::vector<unsigned char> boundary_mask(node_count, 0);
    CutWeight boundary_weight = 0;

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        const Hyperedge& net = hypergraph.hyperedges[edge];
        if (net.deleted) continue;
        const auto endpoints = flow_preparation_detail::activeNetEndpoints(
            hypergraph, edge);
        bool covers_i = false;
        bool covers_j = false;
        for (const std::size_t node : endpoints) {
            covers_i = covers_i || community_labels[node] == community_i;
            covers_j = covers_j || community_labels[node] == community_j;
        }
        if (!covers_i || !covers_j) continue;

        flow_preparation_detail::checkedAdd(
            boundary_weight, static_cast<CutWeight>(net.weight),
            " net=" + std::to_string(edge) + " pair=" +
                std::to_string(community_i) + "," +
                std::to_string(community_j));
        for (const std::size_t node : endpoints) {
            const int community = community_labels[node];
            if (community == community_i || community == community_j)
                boundary_mask[node] = 1;
        }
    }

    PairCorridor result;
    result.community_i = community_i;
    result.community_j = community_j;
    result.boundary_weight = boundary_weight;
    result.anchor_i = anchor_i;
    result.anchor_j = anchor_j;

    std::vector<unsigned char> movable_mask(node_count, 0);
    for (std::size_t node = 0; node < node_count; ++node) {
        if (hypergraph.hypernodes[node].deleted) continue;
        const int community = community_labels[node];
        if (boundary_mask[node]) {
            result.boundary_nodes.push_back(node);
            if (node != anchor_i && node != anchor_j) {
                result.movable_nodes.push_back(node);
                movable_mask[node] = 1;
            }
        }
        if (community == community_i && !movable_mask[node])
            result.fixed_i.push_back(node);
        else if (community == community_j && !movable_mask[node])
            result.fixed_j.push_back(node);
    }

    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        const Hyperedge& net = hypergraph.hyperedges[edge];
        if (net.deleted) continue;
        const auto endpoints = flow_preparation_detail::activeNetEndpoints(
            hypergraph, edge);
        bool touches_movable = false;
        for (const std::size_t node : endpoints)
            touches_movable = touches_movable || movable_mask[node] != 0;
        if (!touches_movable) continue;

        CorridorNet related;
        related.net = edge;
        related.weight = static_cast<CutWeight>(net.weight);
        related.endpoints.reserve(endpoints.size());
        for (const std::size_t node : endpoints) {
            related.endpoints.push_back(
                {node, community_labels[node], movable_mask[node] != 0});
        }
        result.related_nets.push_back(std::move(related));
    }

    return result;
}

}  // namespace cf
