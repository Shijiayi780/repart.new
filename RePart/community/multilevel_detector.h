#pragma once

#include <numeric>
#include <string>
#include "aggregate_graph.h"
#include "local_move.h"
#include "modularity_recompute.h"

namespace cf {
struct DetectorLevel {
    std::size_t level = 0, vertices = 0, edges = 0, communities = 0;
    std::size_t accepted_moves = 0, rounds = 0;
    // A completed nonzero-W local stage ends with a zero-move round.
    // nullopt means there was no scan (W=0), not a measured zero round.
    std::optional<std::size_t> last_round_moves;
    std::optional<double> q_before, q_after;
    std::string action;
};

struct MultilevelResult {
    std::vector<int> initial_vertex_labels; // Both sides; inactive slots are -1.
    std::vector<int> node_labels; // Original node side only; normalized by stable key.
    std::vector<int> initial_to_final, final_level_labels;
    std::vector<std::vector<int>> aggregation_maps;
    std::vector<DetectorLevel> levels;
};

using DetectorObserver = std::function<void(
    const DetectorLevel&, const IncidenceBipartite&, const std::vector<std::size_t>&,
    const LocalMoveResult&, const std::vector<int>&)>;

// The initial graph uses the frozen node IDs [0,node_slots) and net IDs thereafter.
// Generic undirected test graphs are also supported; node_slots=0 extracts no nodes.
inline MultilevelResult detectCommunities(const IncidenceBipartite& initial,
                                          const DetectorObserver& observer = {}) {
    const auto slots = initial.active.size();
    if (initial.node_slots > slots)
        throw std::invalid_argument("DETECTOR_NODE_SLOTS");
    if (slots > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::overflow_error("DETECTOR_LABEL_RANGE");
    MultilevelResult result;
    result.initial_to_final.assign(slots, -1);
    for (std::size_t v = 0; v < slots; ++v)
        if (initial.active[v]) result.initial_to_final[v] = static_cast<int>(v);
    IncidenceBipartite current = initial;
    std::vector<std::size_t> anchors(slots);
    std::iota(anchors.begin(), anchors.end(), std::size_t{0});

    for (;;) {
        DetectorLevel level;
        level.level = result.levels.size();
        level.edges = current.edges.size();
        std::vector<int> singletons(current.active.size(), -1);
        for (std::size_t x = 0; x < current.active.size(); ++x) {
            if (!current.active[x]) continue;
            ++level.vertices;
            singletons[x] = static_cast<int>(x);
        }
        level.q_before = recomputeModularity(current.edges, current.active, singletons).q;
        // Fresh singleton labels/statistics/members/keys at EVERY level.
        const auto moved = localMove(current, anchors);
        for (const auto count : moved.member_count) level.communities += count != 0;
        level.accepted_moves = moved.accepted_moves;
        level.rounds = moved.rounds;
        if (moved.rounds != 0) level.last_round_moves = 0;
        level.q_after = recomputeModularity(current.edges, current.active, moved.labels).q;
        if (level.vertices == 0) level.action = "EMPTY_GRAPH";
        else if (moved.zero_total_weight) level.action = "ZERO_TOTAL_WEIGHT";
        else if (level.communities == level.vertices) level.action = "NO_VERTEX_REDUCTION";
        else level.action = "AGGREGATE";
        result.levels.push_back(level);
        if (observer) observer(level, current, anchors, moved, result.initial_to_final);

        // Neither a zero final scan nor equality of Q across aggregation stops this loop.
        if (level.action != "AGGREGATE") {
            result.final_level_labels = moved.labels;
            result.initial_vertex_labels.assign(slots, -1);
            for (std::size_t v = 0; v < slots; ++v)
                if (initial.active[v])
                    result.initial_vertex_labels[v] = moved.labels.at(result.initial_to_final[v]);
            break;
        }
        auto coarse = aggregateCommunityGraph(current, moved.labels, anchors);
        for (std::size_t v = 0; v < slots; ++v)
            if (initial.active[v])
                result.initial_to_final[v] = coarse.old_to_new.at(result.initial_to_final[v]);
        result.aggregation_maps.push_back(std::move(coarse.old_to_new));
        current = std::move(coarse.graph);
        anchors = std::move(coarse.anchors);
    }

    // Keep the full two-sided labels above. Normalize only communities containing nodes.
    std::map<int, std::size_t> final_key;
    for (std::size_t v = 0; v < slots; ++v) {
        if (!initial.active[v]) continue;
        const int c = result.initial_vertex_labels[v];
        const auto entry = final_key.emplace(c, v);
        if (!entry.second) entry.first->second = std::min(entry.first->second, v);
    }
    std::set<std::pair<std::size_t, int>> node_communities;
    for (std::size_t v = 0; v < initial.node_slots; ++v)
        if (initial.active[v]) {
            const int c = result.initial_vertex_labels[v];
            node_communities.insert({final_key.at(c), c});
        }
    std::map<int, int> node_id;
    for (const auto& entry : node_communities)
        node_id.emplace(entry.second, static_cast<int>(node_id.size()));
    result.node_labels.assign(initial.node_slots, -1);
    for (std::size_t v = 0; v < initial.node_slots; ++v)
        if (initial.active[v]) result.node_labels[v] = node_id.at(result.initial_vertex_labels[v]);
    return result;
}

inline MultilevelResult detectStaticCommunities(const Hypergraph& hypergraph,
                                                const DetectorObserver& observer = {}) {
    return detectCommunities(buildIncidenceBipartite(hypergraph), observer);
}
} // namespace cf
