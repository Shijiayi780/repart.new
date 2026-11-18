// Compile with the frozen detector flags plus -fno-lto, never with -Ofast.
#ifdef __FAST_MATH__
#error "The detector bridge must not be compiled with fast-math"
#endif
#define REPART_DETECTOR_TYPES_ONLY
#include "detector_bridge.h"
#include "multilevel_detector.h"
#include <cfenv>
#include <exception>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace cf_bridge {
namespace {
class ScopedDetectorEnvironment {
    std::fenv_t saved_;
#if defined(__SSE__)
    unsigned saved_mxcsr_;
#endif
public:
    ScopedDetectorEnvironment() {
        if (std::fegetenv(&saved_) != 0) throw std::runtime_error("DETECTOR_SAVE_FENV");
#if defined(__SSE__)
        saved_mxcsr_ = _mm_getcsr();
#endif
        if (std::fesetenv(FE_DFL_ENV) != 0) {
            if (std::fesetenv(&saved_) != 0) std::terminate();
            throw std::runtime_error("DETECTOR_SET_FENV");
        }
    }
    ~ScopedDetectorEnvironment() noexcept {
        if (std::fesetenv(&saved_) != 0) std::terminate();
#if defined(__SSE__)
        _mm_setcsr(saved_mxcsr_);
#endif
    }
    ScopedDetectorEnvironment(const ScopedDetectorEnvironment&) = delete;
    ScopedDetectorEnvironment& operator=(const ScopedDetectorEnvironment&) = delete;
};

Result exportResult(cf::MultilevelResult&& source) {
    Result result;
    result.node_labels = std::move(source.node_labels);
    result.initial_vertex_labels = std::move(source.initial_vertex_labels);
    result.initial_to_final = std::move(source.initial_to_final);
    result.final_level_labels = std::move(source.final_level_labels);
    result.aggregation_maps = std::move(source.aggregation_maps);
    for (const auto& l : source.levels)
        result.levels.push_back({l.level,l.vertices,l.edges,l.communities,l.accepted_moves,l.rounds,
                                 l.last_round_moves,l.q_before,l.q_after,l.action});
    return result;
}
} // namespace

Result detect(const Hypergraph& graph) {
    ScopedDetectorEnvironment environment;
    return exportResult(cf::detectStaticCommunities(graph));
}

Result detectGraph(const Graph& input) {
    ScopedDetectorEnvironment environment;
    cf::IncidenceBipartite graph;
    graph.node_slots = input.node_slots;
    graph.active = input.active;
    const auto n = graph.active.size();
    graph.degree.assign(n,0.0);
    graph.adjacency.resize(n);
    for (const auto& e : input.edges) {
        if (e.u >= n || e.v >= n || !graph.active[e.u] || !graph.active[e.v])
            throw std::invalid_argument("BRIDGE_EDGE_ENDPOINT");
        if (!std::isfinite(e.weight) || e.weight < 0.0)
            throw std::invalid_argument("BRIDGE_EDGE_WEIGHT");
        graph.edges.push_back({e.u,e.v,e.weight});
        graph.total_edge_weight += e.weight;
        graph.degree[e.u] += e.weight;
        graph.degree[e.v] += e.weight;
        if (e.u == e.v) graph.adjacency[e.u].push_back({e.u,2.0*e.weight});
        else {
            graph.adjacency[e.u].push_back({e.v,e.weight});
            graph.adjacency[e.v].push_back({e.u,e.weight});
        }
        if (!std::isfinite(graph.total_edge_weight) || !std::isfinite(graph.degree[e.u]) ||
            !std::isfinite(graph.degree[e.v])) throw std::overflow_error("BRIDGE_SUM_OVERFLOW");
    }
    return exportResult(cf::detectCommunities(graph));
}

Environment strictEnvironment() {
    ScopedDetectorEnvironment environment;
    Environment result{std::fegetround(),0,0};
#if defined(__SSE__)
    result.mxcsr = _mm_getcsr();
#endif
#if defined(__i386__) || defined(__x86_64__)
    asm volatile("fnstcw %0" : "=m"(result.x87_control));
#endif
    return result;
}
} // namespace cf_bridge
