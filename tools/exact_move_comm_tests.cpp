#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../clean/baseline/RePart/datastructure/hypergraph.h"
#include "../clean/baseline/RePart/refinement_move/exact_move_comm.h"
#include "../clean/baseline/RePart/refinement_comm_probe.h"

namespace {

constexpr int kBoards = 32;

int add_node(Hypergraph& g, int id, int board,
             std::vector<int> replicas = {}) {
    while (static_cast<int>(g.hypernodes.size()) <= id) {
        g.hypernodes.emplace_back();
        Hypernode& node = g.hypernodes.back();
        node.id = static_cast<int>(g.hypernodes.size()) - 1;
        node.weights.assign(8, 0);
        node.deleted = true;
    }
    Hypernode& node = g.hypernodes[id];
    node.id = id;
    node.label = board;
    node.weights.assign(8, 0);
    node.deleted = false;
    node.replication_fpga_labels.clear();
    for (int replica : replicas) node.replication_fpga_labels.insert(replica);
    return id;
}

void add_net(Hypergraph& g, int id, int source, const std::vector<int>& sinks,
             int weight) {
    while (static_cast<int>(g.hyperedges.size()) <= id) {
        const int index = static_cast<int>(g.hyperedges.size());
        g.hyperedges.emplace_back(0, 0, 0, index, kBoards);
        g.hyperedges.back().deleted = true;
    }
    Hyperedge& edge = g.hyperedges[id];
    edge.id = id;
    edge.deleted = false;
    edge.weight = weight;
    edge._begin = static_cast<int>(g._incidence_array.size());
    g._incidence_array.push_back(source);
    for (int sink : sinks) g._incidence_array.push_back(sink);
    edge._size = static_cast<int>(g._incidence_array.size()) - edge._begin;
    edge.source_part_id = g.hypernodes[source].label;
    g.hypernodes[source].incident_nets_as_source.insert(id);
    for (int sink : sinks) {
        g.hypernodes[sink].incident_nets_as_target.insert(id);
    }
}

Hypergraph make_graph() {
    Hypergraph g;
    g.fpga_manager.fpga_num = kBoards;
    g.fpga_manager.fpga_info.assign(kBoards, std::vector<int>(9, 100000));
    g.fpga_manager.hop_distances.assign(kBoards, std::vector<int>(kBoards, 1));
    return g;
}

std::vector<int> expected_delta(
    const std::vector<std::pair<int, int>>& entries) {
    std::vector<int> delta(kBoards, 0);
    for (const auto& entry : entries) delta[entry.first] += entry.second;
    return delta;
}

int failures = 0;

void expect_equal(const std::string& name, const std::vector<int>& actual,
                  const std::vector<int>& expected) {
    if (actual.size() != expected.size()) {
        std::cerr << "FAIL " << name << ": size " << actual.size()
                  << " != " << expected.size() << '\n';
        ++failures;
        return;
    }
    for (int board = 0; board < kBoards; ++board) {
        if (actual[board] != expected[board]) {
            std::cerr << "FAIL " << name << ": board " << board << " actual "
                      << actual[board] << " expected " << expected[board]
                      << '\n';
            ++failures;
            return;
        }
    }
    std::cout << "PASS " << name << '\n';
}

void test_recorded_sink_move() {
    Hypergraph g = make_graph();
    add_node(g, 5410, 23);
    add_node(g, 4822, 15);
    add_node(g, 9001, 23);
    add_node(g, 9002, 27);
    add_net(g, 4687, 5410, {4822, 9001, 9002}, 1);

    const std::vector<int> before = refinement_comm_probe::reference_comm(g);
    const std::vector<int> delta = exact_move_comm::move_delta(g, 4822, 23);
    g.hypernodes[4822].label = 23;
    const std::vector<int> after = refinement_comm_probe::reference_comm(g);
    const std::vector<int> full_delta = expected_delta({{15, -1}});
    const std::vector<int> expected_after = expected_delta({{23, 1}, {27, 1}});

    expect_equal("recorded sink move exact delta", delta, full_delta);
    expect_equal("recorded sink move before", before,
                 expected_delta({{15, 1}, {23, 1}, {27, 1}}));
    expect_equal("recorded sink move after", after, expected_after);
    std::cout << "recorded sink move expected board15=-1 board23=0\n";
}

void test_recorded_source_move_with_replica() {
    Hypergraph g = make_graph();
    add_node(g, 2432, 11, {19, 22, 23, 29});
    add_node(g, 9003, 11);
    add_net(g, 7098, 2432, {9003}, 1);

    const std::vector<int> delta = exact_move_comm::move_delta(g, 2432, 27);
    const std::vector<int> expected = expected_delta({{11, 1}, {27, 1}});
    expect_equal("recorded source move exact delta", delta, expected);
}

void test_feasible_and_apply() {
    Hypergraph g = make_graph();
    add_node(g, 5410, 23);
    add_node(g, 4822, 15);
    add_node(g, 9001, 23);
    add_node(g, 9002, 27);
    add_net(g, 4687, 5410, {4822, 9001, 9002}, 1);

    std::vector<int> comm(kBoards, 10);
    std::vector<int> delta;
    if (!exact_move_comm::feasible(g, comm, 4822, 23, delta)) {
        std::cerr << "FAIL feasible rejected a legal move\n";
        ++failures;
        return;
    }
    exact_move_comm::apply(comm, delta);
    if (comm[15] != 9 || comm[23] != 10 || comm[27] != 10) {
        std::cerr << "FAIL apply produced " << comm[15] << ',' << comm[23]
                  << ',' << comm[27] << '\n';
        ++failures;
        return;
    }
    std::cout << "PASS feasible/apply comm 10,10,10 -> 9,10,10\n";
}

}  // namespace

int main() {
    test_recorded_sink_move();
    test_recorded_source_move_with_replica();
    test_feasible_and_apply();
    if (failures != 0) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ALL_EXACT_MOVE_COMM_TESTS_PASS\n";
    return 0;
}
