#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "../datastructure/hypergraph.h"

namespace cf {

using CutWeight = std::uint64_t;

// Independently recompute the weighted cut-net objective on the current layer.
// Every active net contributes its current weight exactly once iff its distinct
// active endpoints carry at least two different temporary community labels.
inline CutWeight recomputeCut(const Hypergraph& hypergraph,
                              const std::vector<int>& community_labels) {
    const std::size_t node_count = hypergraph.hypernodes.size();
    if (community_labels.size() != node_count)
        throw std::invalid_argument("CUT_LABEL_SHAPE");

    for (std::size_t node = 0; node < node_count; ++node) {
        if (!hypergraph.hypernodes[node].deleted && community_labels[node] < 0)
            throw std::invalid_argument("CUT_ACTIVE_LABEL node=" +
                                        std::to_string(node));
    }

    CutWeight total = 0;
    for (std::size_t edge = 0; edge < hypergraph.hyperedges.size(); ++edge) {
        const Hyperedge& net = hypergraph.hyperedges[edge];
        if (net.deleted) continue;

        const std::string context = " net=" + std::to_string(edge);
        if (net._begin < 0 || net._size <= 0)
            throw std::invalid_argument("CUT_INVALID_INCIDENCE_RANGE" + context);
        if (net.weight < 0)
            throw std::invalid_argument("CUT_NEGATIVE_NET_WEIGHT" + context);

        const std::size_t begin = static_cast<std::size_t>(net._begin);
        const std::size_t count = static_cast<std::size_t>(net._size);
        if (begin > hypergraph._incidence_array.size() ||
            count > hypergraph._incidence_array.size() - begin)
            throw std::invalid_argument("CUT_INVALID_INCIDENCE_RANGE" + context);

        std::vector<std::size_t> endpoints;
        endpoints.reserve(count);
        for (std::size_t pin = 0; pin < count; ++pin) {
            const int node = hypergraph._incidence_array[begin + pin];
            if (node < 0 || static_cast<std::size_t>(node) >= node_count ||
                hypergraph.hypernodes[static_cast<std::size_t>(node)].deleted)
                throw std::invalid_argument("CUT_INVALID_ACTIVE_ENDPOINT" + context);
            endpoints.push_back(static_cast<std::size_t>(node));
        }
        std::sort(endpoints.begin(), endpoints.end());
        endpoints.erase(std::unique(endpoints.begin(), endpoints.end()), endpoints.end());

        const int first_label = community_labels[endpoints.front()];
        bool is_cut = false;
        for (std::size_t index = 1; index < endpoints.size(); ++index) {
            if (community_labels[endpoints[index]] != first_label) {
                is_cut = true;
                break;
            }
        }
        if (!is_cut) continue;

        const CutWeight weight = static_cast<CutWeight>(net.weight);
        if (weight > std::numeric_limits<CutWeight>::max() - total)
            throw std::overflow_error("CUT_WEIGHT_SUM_OVERFLOW" + context);
        total += weight;
    }
    return total;
}

}  // namespace cf
