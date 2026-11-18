#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include "move_gain.h"

namespace cf {
inline constexpr double local_move_threshold = 1e-12;

// Acceptance and ranking are distinct: epsilon never defines a tie.
inline bool betterLocalMove(double gain, std::size_t key,
                            double best_gain, std::size_t best_key) {
    if (!std::isfinite(gain)) throw std::invalid_argument("LOCAL_GAIN_NONFINITE");
    return gain > local_move_threshold &&
           (gain > best_gain || (gain == best_gain && key < best_key));
}

// Each touched slot is reset once; accumulation follows the original arc order.
struct LocalMoveWorkspace {
    std::vector<double> weight_to;
    std::vector<unsigned char> present;
    std::vector<int> touched;
    std::size_t reset_slots = 0;

    explicit LocalMoveWorkspace(std::size_t slots) : weight_to(slots, 0.0), present(slots, 0) {}

    void collect(const IncidenceBipartite& graph, const std::vector<int>& labels, std::size_t i) {
        if (labels.size() != graph.active.size() || graph.adjacency.size() != graph.active.size() ||
            i >= graph.active.size() || !graph.active[i])
            throw std::invalid_argument("LOCAL_SCAN_SHAPE_OR_VERTEX");
        for (const int c : touched) { weight_to[c] = 0.0; present[c] = 0; ++reset_slots; }
        touched.clear();
        const auto touch = [&](int c) {
            if (c < 0 || static_cast<std::size_t>(c) >= weight_to.size())
                throw std::invalid_argument("LOCAL_SCAN_LABEL");
            if (!present[c]) { present[c] = 1; touched.push_back(c); }
        };
        touch(labels[i]); // Original community is always a candidate, even at zero degree.
        for (const auto& arc : graph.adjacency[i]) {
            const auto j = arc.target;
            if (j >= graph.active.size() || !graph.active[j] ||
                !std::isfinite(arc.weight) || arc.weight < 0.0)
                throw std::invalid_argument("LOCAL_SCAN_ARC");
            if (j == i) continue;
            const int c = labels[j];
            touch(c); // Include zero-weight adjacency; do not infer presence from weight.
            weight_to[c] += arc.weight;
            if (!std::isfinite(weight_to[c])) throw std::overflow_error("LOCAL_SCAN_OVERFLOW");
        }
    }
};

struct LocalMoveResult {
    std::vector<int> labels;
    std::vector<double> community_degree;
    std::vector<std::size_t> member_count, community_key;
    std::size_t rounds = 0, accepted_moves = 0, evaluations = 0, reset_slots = 0;
    bool zero_total_weight = false;
};

struct LocalMoveEvent {
    std::size_t round, vertex;
    int source, target;
    double gain;
    // Populated only when an observer is supplied; statistics below are post-update.
    std::vector<int> candidates;
    std::vector<double> gains;
};
using LocalMoveObserver = std::function<void(const LocalMoveEvent&, const LocalMoveResult&,
                                            const std::vector<double>&)>;

// One detector graph, singleton initialization, no aggregation or RePart mutation.
inline LocalMoveResult localMove(const IncidenceBipartite& graph,
                                const std::vector<std::size_t>& anchors,
                                const LocalMoveObserver& observer = {}) {
    const auto n = graph.active.size();
    if (graph.degree.size() != n || graph.adjacency.size() != n || anchors.size() != n ||
        !std::isfinite(graph.total_edge_weight) || graph.total_edge_weight < 0.0)
        throw std::invalid_argument("LOCAL_GRAPH_SHAPE_OR_WEIGHT");
    if (n > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::overflow_error("LOCAL_LABEL_RANGE");
    LocalMoveResult result;
    result.labels.assign(n, -1);
    result.member_count.assign(n, 0);
    result.community_key.assign(n, std::numeric_limits<std::size_t>::max());
    std::vector<std::set<std::pair<std::size_t, std::size_t>>> members(n);
    std::set<std::size_t> unique_anchors;
    for (std::size_t i = 0; i < n; ++i) {
        if (!graph.active[i]) continue;
        if (!unique_anchors.insert(anchors[i]).second)
            throw std::invalid_argument("LOCAL_DUPLICATE_ANCHOR");
        result.labels[i] = static_cast<int>(i);
        result.member_count[i] = 1;
        result.community_key[i] = anchors[i];
        members[i].insert({anchors[i], i});
    }
    result.community_degree = initializeCommunityDegree(graph, result.labels, n);
    if (graph.total_edge_weight == 0.0) { result.zero_total_weight = true; return result; }
    LocalMoveWorkspace workspace(n);
    std::vector<int> candidates;
    for (;;) {
        ++result.rounds;
        std::size_t round_moves = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (!graph.active[i]) continue;
            const int A = result.labels[i];
            workspace.collect(graph, result.labels, i);
            candidates.clear();
            for (int c : workspace.touched)
                if (result.member_count[c] != 0) candidates.push_back(c);
            std::sort(candidates.begin(), candidates.end(), [&](int x, int y) {
                return result.community_key[x] < result.community_key[y];
            });
            int best = A;
            double best_gain = 0.0;
            LocalMoveEvent event{result.rounds, i, A, A, 0.0, {}, {}};
            if (observer) event.candidates = candidates;
            for (int B : candidates) {
                const double gain = moveGain(graph, result.labels, result.community_degree,
                                             workspace.weight_to, i, B);
                if (observer) event.gains.push_back(gain);
                if (betterLocalMove(gain, result.community_key[B], best_gain, result.community_key[best])) {
                    best = B; best_gain = gain;
                }
            }
            if (best != A) {
                result.community_degree[A] -= graph.degree[i];
                result.community_degree[best] += graph.degree[i];
                if (!std::isfinite(result.community_degree[A]) || !std::isfinite(result.community_degree[best]))
                    throw std::overflow_error("LOCAL_DEGREE_OVERFLOW");
                if (members[A].erase({anchors[i], i}) != 1)
                    throw std::logic_error("LOCAL_MEMBER_MISSING");
                members[best].insert({anchors[i], i});
                --result.member_count[A]; ++result.member_count[best];
                result.labels[i] = best;
                result.community_key[A] = members[A].empty() ? std::numeric_limits<std::size_t>::max()
                                                            : members[A].begin()->first;
                result.community_key[best] = members[best].begin()->first;
                ++round_moves; ++result.accepted_moves;
            }
            ++result.evaluations;
            result.reset_slots = workspace.reset_slots;
            if (observer) {
                event.target = best; event.gain = best_gain;
                observer(event, result, workspace.weight_to);
            }
        }
        if (round_moves == 0) break;
    }
    return result;
}
} // namespace cf
