#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

class Hypergraph;
namespace cf_bridge {
struct Level {
    std::size_t level, vertices, edges, communities, accepted_moves, rounds;
    std::optional<std::size_t> last_round_moves;
    std::optional<double> q_before, q_after;
    std::string action;
};
struct Result {
    std::vector<int> node_labels, initial_vertex_labels, initial_to_final, final_level_labels;
    std::vector<std::vector<int>> aggregation_maps;
    std::vector<Level> levels;
};
// Declarations only. All incidence weights, Q, gains and detection run in the .cpp.
Result detect(const Hypergraph& graph);

// Physical graph interface for independent integration regression fixtures.
struct Edge { std::size_t u, v; double weight; };
struct Graph { std::size_t node_slots = 0; std::vector<unsigned char> active; std::vector<Edge> edges; };
Result detectGraph(const Graph& graph);
struct Environment { int rounding; unsigned mxcsr; unsigned short x87_control; };
Environment strictEnvironment();
} // namespace cf_bridge
