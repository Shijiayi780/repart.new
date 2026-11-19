#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

#include "lawler_network.h"

namespace cf {

struct DeterministicMinCutResult {
    CutWeight max_flow;
    CutWeight cut_capacity;
    std::vector<unsigned char> source_reachable;
    std::size_t successful_bfs_rounds;
    std::size_t augmentations;
    std::size_t reverse_arc_augmentations;
};

namespace deterministic_dinic_detail {

struct ResidualArc {
    std::size_t to;
    std::size_t reverse_index;
    CutWeight residual_capacity;
    bool original_forward;
};

class Solver {
public:
    Solver(std::size_t vertex_count, std::size_t source, std::size_t sink)
        : adjacency_(vertex_count),
          level_(vertex_count, noLevel()),
          next_arc_(vertex_count, 0),
          source_(source),
          sink_(sink),
          successful_bfs_rounds_(0),
          augmentations_(0),
          reverse_arc_augmentations_(0) {}

    void addArc(std::size_t from, std::size_t to, CutWeight capacity) {
        // A self-loop never crosses an s-t cut or carries useful s-t flow.
        // Keeping it out of the paired residual representation also avoids a
        // special reverse-index case while preserving the exact objective.
        if (from == to) return;
        const std::size_t forward_index = adjacency_[from].size();
        const std::size_t reverse_index = adjacency_[to].size();
        adjacency_[from].push_back(
            {to, reverse_index, capacity, true});
        adjacency_[to].push_back(
            {from, forward_index, 0, false});
    }

    CutWeight run() {
        CutWeight total = 0;
        while (buildLevels()) {
            ++successful_bfs_rounds_;
            std::fill(next_arc_.begin(), next_arc_.end(), 0);
            while (true) {
                const CutWeight pushed = send(
                    source_, std::numeric_limits<CutWeight>::max());
                if (pushed == 0) break;
                if (pushed > std::numeric_limits<CutWeight>::max() - total)
                    throw std::overflow_error("DINIC_FLOW_SUM_OVERFLOW");
                total += pushed;
                ++augmentations_;
            }
        }
        return total;
    }

    std::vector<unsigned char> sourceReachable() const {
        std::vector<unsigned char> reachable(adjacency_.size(), 0);
        std::queue<std::size_t> pending;
        reachable[source_] = 1;
        pending.push(source_);
        while (!pending.empty()) {
            const std::size_t vertex = pending.front();
            pending.pop();
            for (const ResidualArc& arc : adjacency_[vertex]) {
                if (arc.residual_capacity == 0 || reachable[arc.to] != 0)
                    continue;
                reachable[arc.to] = 1;
                pending.push(arc.to);
            }
        }
        return reachable;
    }

    std::size_t successfulBfsRounds() const {
        return successful_bfs_rounds_;
    }

    std::size_t augmentations() const { return augmentations_; }

    std::size_t reverseArcAugmentations() const {
        return reverse_arc_augmentations_;
    }

private:
    static constexpr std::size_t noLevel() {
        return std::numeric_limits<std::size_t>::max();
    }

    bool buildLevels() {
        std::fill(level_.begin(), level_.end(), noLevel());
        std::queue<std::size_t> pending;
        level_[source_] = 0;
        pending.push(source_);
        while (!pending.empty()) {
            const std::size_t vertex = pending.front();
            pending.pop();
            for (const ResidualArc& arc : adjacency_[vertex]) {
                if (arc.residual_capacity == 0 || level_[arc.to] != noLevel())
                    continue;
                level_[arc.to] = level_[vertex] + 1;
                pending.push(arc.to);
            }
        }
        return level_[sink_] != noLevel();
    }

    CutWeight send(std::size_t vertex, CutWeight limit) {
        if (vertex == sink_) return limit;
        for (std::size_t& index = next_arc_[vertex];
             index < adjacency_[vertex].size(); ++index) {
            ResidualArc& arc = adjacency_[vertex][index];
            if (arc.residual_capacity == 0 ||
                level_[arc.to] != level_[vertex] + 1)
                continue;
            const CutWeight pushed = send(
                arc.to, std::min(limit, arc.residual_capacity));
            if (pushed == 0) continue;
            arc.residual_capacity -= pushed;
            ResidualArc& reverse =
                adjacency_[arc.to][arc.reverse_index];
            if (pushed > std::numeric_limits<CutWeight>::max() -
                             reverse.residual_capacity)
                throw std::overflow_error("DINIC_RESIDUAL_SUM_OVERFLOW");
            reverse.residual_capacity += pushed;
            if (!arc.original_forward) ++reverse_arc_augmentations_;
            return pushed;
        }
        return 0;
    }

    std::vector<std::vector<ResidualArc>> adjacency_;
    std::vector<std::size_t> level_;
    std::vector<std::size_t> next_arc_;
    std::size_t source_;
    std::size_t sink_;
    std::size_t successful_bfs_rounds_;
    std::size_t augmentations_;
    std::size_t reverse_arc_augmentations_;
};

inline CutWeight cutCapacity(
    const std::vector<LawlerArc>& arcs,
    const std::vector<unsigned char>& source_reachable) {
    CutWeight total = 0;
    for (std::size_t index = 0; index < arcs.size(); ++index) {
        const LawlerArc& arc = arcs[index];
        if (source_reachable[arc.from] == 0 ||
            source_reachable[arc.to] != 0)
            continue;
        if (arc.capacity > std::numeric_limits<CutWeight>::max() - total)
            throw std::overflow_error(
                "DINIC_CUT_SUM_OVERFLOW arc=" + std::to_string(index));
        total += arc.capacity;
    }
    return total;
}

}  // namespace deterministic_dinic_detail

// Deterministic Dinic: original arcs are inserted in input order; BFS and DFS
// scan residual adjacency in that stored order. The returned source side is
// exactly the set reachable from source through positive residual capacity
// after the maximum flow terminates.
inline DeterministicMinCutResult solveDeterministicMinCut(
    std::size_t vertex_count, const std::vector<LawlerArc>& arcs,
    std::size_t source, std::size_t sink) {
    if (vertex_count == 0 || source >= vertex_count || sink >= vertex_count ||
        source == sink)
        throw std::invalid_argument("DINIC_INVALID_TERMINALS");
    for (std::size_t index = 0; index < arcs.size(); ++index) {
        if (arcs[index].from >= vertex_count || arcs[index].to >= vertex_count)
            throw std::invalid_argument(
                "DINIC_INVALID_ARC index=" + std::to_string(index));
    }

    deterministic_dinic_detail::Solver solver(vertex_count, source, sink);
    for (const LawlerArc& arc : arcs)
        solver.addArc(arc.from, arc.to, arc.capacity);
    const CutWeight max_flow = solver.run();
    std::vector<unsigned char> source_reachable = solver.sourceReachable();
    const CutWeight cut_capacity =
        deterministic_dinic_detail::cutCapacity(arcs, source_reachable);
    return {max_flow,
            cut_capacity,
            std::move(source_reachable),
            solver.successfulBfsRounds(),
            solver.augmentations(),
            solver.reverseArcAugmentations()};
}

inline DeterministicMinCutResult solveDeterministicMinCut(
    const PairLawlerNetwork& network) {
    return solveDeterministicMinCut(network.vertices.size(), network.arcs,
                                    network.source, network.sink);
}

}  // namespace cf
