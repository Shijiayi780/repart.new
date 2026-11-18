#pragma once

// Included only in REPART_CF_OBSERVE builds. No algorithm state is owned here.
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace cf_observe {
inline std::ofstream& output() {
    static std::ofstream stream([] {
        const char* path = std::getenv("REPART_CF_LOG");
        return path ? path : "cf_observation.tsv";
    }());
    return stream;
}

template <class... Values>
void record(const char* event, const Values&... values) {
    auto& stream = output();
    stream << std::setprecision(17) << event;
    using Expand = int[];
    (void)Expand{0, ((void)(stream << '\t' << values), 0)...};
    stream << '\n';
}

inline void flush() {
    output().flush();
    if (!output()) std::cerr << "CF observation output failed\n";
}

template <class Seed>
Seed seed(int logical_thread, Seed value) {
    record("assignment_seed", logical_thread, "mt19937", 42, value);
    return value;
}

template <class Graph>
void graph(const char* event, const Graph& hg) {
    long long weight = 0;
    int nodes = 0, edges = 0;
    for (const auto& node : hg.hypernodes) if (!node.deleted) ++nodes;
    for (const auto& edge : hg.hyperedges) {
        if (!edge.deleted) { ++edges; weight += edge.weight; }
    }
    record(event, hg._num_vertices_current, nodes, hg._num_hyperedges, edges, weight);
}

template <class Graph>
void edge(const char* event, const Graph& hg, int id) {
    const auto& e = hg.hyperedges[id];
    auto& stream = output();
    stream << event << '\t' << id << '\t' << e.weight << '\t' << e.deleted
           << '\t' << hg._incidence_array[e._begin] << '\t';
    // Sort a copy solely for observation; never reorder the live incidence array.
    std::vector<int> sinks(hg._incidence_array.begin() + e._begin + 1,
                           hg._incidence_array.begin() + e._begin + e._size);
    std::sort(sinks.begin(), sinks.end());
    for (size_t i = 0; i < sinks.size(); ++i) {
        if (i) stream << ',';
        stream << sinks[i];
    }
    stream << '\n';
}
} // namespace cf_observe
