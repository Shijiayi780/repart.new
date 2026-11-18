#include <cstdlib>
#include <cmath>
#include <iostream>
#include "link_load_refiner.h"

using namespace link_load_v1;

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << "CHECK failed at line " << __LINE__ << ": " #condition "\n"; \
        return EXIT_FAILURE; \
    } \
} while (false)

static FPGAManager manager(int n, const std::vector<std::pair<int,int>>& edges, int max_hop = 10) {
    FPGAManager m;
    m.fpga_num = n;
    m.max_hop_distance = max_hop;
    m.adj_matrix.assign(n, std::vector<int>(n, 0));
    m.fpga_info.assign(n, std::vector<int>(9, 1000));
    m.id_to_name.resize(n);
    for (int i = 0; i < n; ++i) {
        m.id_to_name[i] = "F" + std::to_string(i);
        m.name_to_id[m.id_to_name[i]] = i;
    }
    for (auto [u,v] : edges) m.adj_matrix[u][v] = m.adj_matrix[v][u] = 1;
    m.computeHopDistances();
    return m;
}

static Hypergraph graph(const FPGAManager& m, const std::vector<int>& labels,
                        const std::vector<std::pair<int,std::vector<int>>>& nets) {
    Hypergraph hg;
    hg.fpga_manager = m;
    for (int i = 0; i < static_cast<int>(labels.size()); ++i) {
        hg.hypernodes.emplace_back(i, std::vector<int>{}, std::vector<int>(8, 1));
        hg.hypernodes.back().label = labels[i];
    }
    int begin = 0, id = 0;
    for (const auto& spec : nets) {
        int source = spec.second.front();
        hg._incidence_array.insert(hg._incidence_array.end(), spec.second.begin(), spec.second.end());
        hg.hyperedges.emplace_back(begin, static_cast<int>(spec.second.size()), spec.first, id, m.fpga_num);
        hg.hypernodes[source].incident_nets_as_source.insert(id);
        for (size_t i = 1; i < spec.second.size(); ++i) hg.hypernodes[spec.second[i]].incident_nets_as_target.insert(id);
        begin += static_cast<int>(spec.second.size()); ++id;
    }
    hg._num_vertices_initial = hg._num_vertices_current = labels.size();
    hg._num_hyperedges = nets.size();
    return hg;
}

int main() {
    {
        auto m = manager(4, {{0,1},{0,2},{1,3},{2,3}});
        Model model(m);
        // Sorted-neighbor BFS deterministically chooses 0-1-3 over 0-2-3.
        CHECK(model.paths()[0][3].hop_count == 2);
        CHECK(model.paths()[0][3].physical_link_ids == std::vector<int>({0,2}));
    }
    {
        auto m = manager(3, {{0,1},{1,2}});
        // node 0 is source; nodes 1 and 2 are two sinks on the same destination FPGA.
        auto hg = graph(m, {0,2,2}, {{7,{0,1,2}}});
        auto metrics = Model(m).evaluate(hg);
        CHECK(metrics.demands.size() == 1);                 // sink-FPGA deduplication
        CHECK(metrics.total_routed_demand_weight == 7);
        CHECK(metrics.thd == 14);                           // two-hop expansion
        CHECK(metrics.links[0].load == 7 && metrics.links[1].load == 7);
        hg.hypernodes[1].label = 0;
        hg.hypernodes[2].label = 0;
        metrics = Model(m).evaluate(hg);
        CHECK(metrics.demands.empty());                     // same-FPGA traffic is free
    }
    {
        auto m = manager(3, {{0,1}});
        auto hg = graph(m, {0,2}, {{3,{0,1}}});
        CHECK(Model(m).evaluate(hg).unreachable_demands == 1);
    }
    {
        Metrics base, trial;
        base.unreachable_demands = 0;
        trial.resource_violations = 1;
        CHECK(Refiner::feasibilityFailure(trial, base, 3, 102) == Refiner::FeasibilityFailure::Resource);
        trial = Metrics{}; trial.max_hop = 4;
        CHECK(Refiner::feasibilityFailure(trial, base, 3, 102) == Refiner::FeasibilityFailure::MaximumHop);
        trial = Metrics{}; trial.communication_violations = 1;
        CHECK(Refiner::feasibilityFailure(trial, base, 3, 102) == Refiner::FeasibilityFailure::Communication);
        trial = Metrics{}; trial.thd = 103;
        CHECK(Refiner::feasibilityFailure(trial, base, 3, 102) == Refiner::FeasibilityFailure::Thd);
        trial = Metrics{}; trial.unreachable_demands = 1;
        CHECK(Refiner::feasibilityFailure(trial, base, 3, 102) == Refiner::FeasibilityFailure::Unreachable);
    }
    {
        Metrics current, lower_max, lower_square;
        current.max_link_load = 10; current.squared_load_cost = 150; current.load_stddev = 2;
        lower_max = current; lower_max.max_link_load = 9;
        lower_square = current; lower_square.squared_load_cost = 149;
        CHECK(lexicographicallyBetter(lower_max, current));
        CHECK(lexicographicallyBetter(lower_square, current));
    }
    {
        auto m = manager(3, {{0,1},{1,2}});
        auto hg = graph(m, {0,2}, {{5,{0,1}}});
        const int original = hg.hypernodes[1].label;
        hg.hypernodes[1].label = 1;
        auto trial = Model(m).evaluate(hg);
        hg.hypernodes[1].label = original;
        CHECK(trial.max_link_load == 5);
        CHECK(hg.hypernodes[1].label == original);           // rejected transaction restores label
    }
    std::cout << "link_load_tests: all checks passed\n";
}
