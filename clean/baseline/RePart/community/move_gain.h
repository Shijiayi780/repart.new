#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>
#include "incidence_bipartite.h"

namespace cf {
// I: initialize once per detector graph; never source totals from the Q oracle.
inline std::vector<double> initializeCommunityDegree(
    const IncidenceBipartite& graph, const std::vector<int>& labels,
    std::size_t community_slots) {
    if (labels.size() != graph.active.size() || graph.degree.size() != graph.active.size())
        throw std::invalid_argument("MOVE_INIT_SHAPE");
    std::vector<double> community_degree(community_slots, 0.0); // I1
    for (std::size_t v = 0; v < graph.active.size(); ++v) {
        if (!graph.active[v]) continue;
        if (labels[v] < 0 || static_cast<std::size_t>(labels[v]) >= community_slots)
            throw std::invalid_argument("MOVE_INIT_LABEL");
        if (!std::isfinite(graph.degree[v]) || graph.degree[v] < 0.0)
            throw std::invalid_argument("MOVE_INIT_DEGREE");
        community_degree[labels[v]] += graph.degree[v]; // I2
        if (!std::isfinite(community_degree[labels[v]]))
            throw std::overflow_error("MOVE_INIT_OVERFLOW");
    }
    return community_degree;
}

// S: scratch belongs to this vertex evaluation and this exact label state.
inline std::vector<double> collectMoveWeights(
    const IncidenceBipartite& graph, const std::vector<int>& labels,
    std::size_t i, std::size_t community_slots) {
    if (labels.size() != graph.active.size() || graph.adjacency.size() != graph.active.size())
        throw std::invalid_argument("MOVE_SCAN_SHAPE");
    if (i >= graph.active.size() || !graph.active[i])
        throw std::invalid_argument("MOVE_SCAN_VERTEX");
    if (labels[i] < 0 || static_cast<std::size_t>(labels[i]) >= community_slots)
        throw std::invalid_argument("MOVE_SCAN_LABEL");
    std::vector<double> weight_to(community_slots, 0.0); // S1
    for (const auto& arc : graph.adjacency[i]) {
        const auto j = arc.target;
        if (j >= graph.active.size() || !graph.active[j])
            throw std::invalid_argument("MOVE_SCAN_ENDPOINT");
        if (!std::isfinite(arc.weight) || arc.weight < 0.0)
            throw std::invalid_argument("MOVE_SCAN_WEIGHT");
        if (j == i) continue; // S2: self-loop degree stays in graph.degree.
        if (labels[j] < 0 || static_cast<std::size_t>(labels[j]) >= community_slots)
            throw std::invalid_argument("MOVE_SCAN_LABEL");
        weight_to[labels[j]] += arc.weight; // S3: parallel edges accumulate separately.
        if (!std::isfinite(weight_to[labels[j]]))
            throw std::overflow_error("MOVE_SCAN_OVERFLOW");
    }
    return weight_to;
}

// G: all inputs describe the state BEFORE moving i. This function changes none.
inline double moveGain(
    const IncidenceBipartite& graph, const std::vector<int>& labels,
    const std::vector<double>& community_degree, const std::vector<double>& weight_to,
    std::size_t i, int target) {
    if (labels.size() != graph.active.size() || graph.degree.size() != graph.active.size() ||
        community_degree.size() != weight_to.size())
        throw std::invalid_argument("MOVE_GAIN_SHAPE");
    if (i >= graph.active.size() || !graph.active[i])
        throw std::invalid_argument("MOVE_GAIN_VERTEX");
    const int A = labels[i]; // G1
    const int B = target;    // G2
    if (A < 0 || B < 0 || static_cast<std::size_t>(A) >= community_degree.size() ||
        static_cast<std::size_t>(B) >= community_degree.size())
        throw std::invalid_argument("MOVE_GAIN_LABEL");
    if (A == B) return 0.0; // G3
    const double k = graph.degree[i]; // G4: includes twice the physical loop weight.
    const double KA = community_degree[A]; // G5: still includes k.
    const double KB = community_degree[B];
    const double W = graph.total_edge_weight; // G6
    const double a = weight_to[A]; // G7: excludes loops.
    const double b = weight_to[B];
    if (!std::isfinite(W) || W <= 0.0)
        throw std::invalid_argument("MOVE_GAIN_TOTAL_WEIGHT");
    if (!std::isfinite(k) || !std::isfinite(KA) || !std::isfinite(KB) ||
        !std::isfinite(a) || !std::isfinite(b) ||
        k < 0.0 || KA < 0.0 || KB < 0.0 || a < 0.0 || b < 0.0)
        throw std::invalid_argument("MOVE_GAIN_STATISTIC");
    const double internal_change = (b - a) / W; // G8
    const double degree_change = KB - KA + k;
    const double numerator = k * degree_change;
    const double denominator = 2.0 * W * W;
    if (!std::isfinite(internal_change) || !std::isfinite(degree_change) ||
        !std::isfinite(numerator) || !std::isfinite(denominator) || denominator == 0.0)
        throw std::overflow_error("MOVE_GAIN_ARITHMETIC");
    const double null_change = numerator / denominator; // G9
    const double gain = internal_change - null_change;  // G10
    if (!std::isfinite(null_change) || !std::isfinite(gain))
        throw std::overflow_error("MOVE_GAIN_ARITHMETIC");
    return gain;
}
} // namespace cf
