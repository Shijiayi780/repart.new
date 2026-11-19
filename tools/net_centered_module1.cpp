#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../RePart/datastructure/hypergraph.h"
#include "../RePart/io/hypergraph_readin.h"
#include "../RePart/refinement_move/kway_fm_refiner.h"
#include "../RePart/refinement_native_move_probe.h"

namespace fs = std::filesystem;

namespace {

struct Options {
    std::string case_dir;
    std::string placement;
    std::string snapshot;
    std::string output_dir;
};

struct NetStat {
    int id = -1;
    int source = -1;
    int source_board = -1;
    int unique_sink_count = 0;
    int cross_sink_count = 0;
    int d_e = 0;
    std::vector<int> c_by_board;
    std::vector<int> native_target_pins;
    std::vector<int> v2_target_boards;
    int v2_target_count = 0;
    int duplicate_pin_count = 0;
    std::int64_t v2_cost = 0;
    bool v2_cost_valid = false;
    bool has_sink_replica = false;
    bool has_source_replica = false;
    int native_source_board_pins = 0;
};

struct NodeStat {
    int id = -1;
    int board = -1;
    std::vector<int> replicas;
    int net_degree = 0;
};

struct Check {
    std::string name;
    std::string status;
    std::string detail;
};

struct State {
    Hypergraph g;
    std::unordered_map<std::string, int> name_to_id;
    std::unordered_map<int, std::string> id_to_name;
    std::vector<std::vector<int>> usage;
    std::vector<int> comm;
    std::vector<std::vector<int>> target_pins_ref;
    std::vector<int> net_degree;
    std::vector<NetStat> nets;
    std::vector<NodeStat> nodes;
    std::vector<Check> checks;

    bool has_snapshot = false;
    long long snapshot_meta_cost = -1;
    std::vector<std::vector<int>> snapshot_usage;
    std::vector<int> snapshot_comm;
    std::vector<long long> snapshot_source_part;
    std::vector<int> snapshot_weight;
    std::vector<std::vector<int>> snapshot_pins;

    int board_count = 0;
    std::int64_t v2_cost = 0;
    bool v2_cost_valid = false;
    bool placement_valid = false;
};

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> split_ws(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> fields;
    std::string token;
    while (stream >> token) fields.push_back(token);
    return fields;
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (char c : value) {
        switch (c) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out << "\\u" << std::hex << static_cast<int>(c) << std::dec;
                } else {
                    out << c;
                }
        }
    }
    return out.str();
}

std::string join_ints(const std::vector<int>& values, char sep = ',') {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << sep;
        out << values[i];
    }
    return out.str();
}

std::string nonzero_counts(const std::vector<int>& counts) {
    std::ostringstream out;
    bool first = true;
    for (std::size_t b = 0; b < counts.size(); ++b) {
        if (counts[b] <= 0) continue;
        if (!first) out << ';';
        first = false;
        out << b << ':' << counts[b];
    }
    if (first) out << '-';
    return out.str();
}

void add_check(State& state, const std::string& name, const std::string& status,
               const std::string& detail) {
    state.checks.push_back({name, status, detail});
}

void load_case(State& state, const std::string& case_dir) {
    FPGAManager manager;
    if (!manager.init(case_dir + "/design.info", case_dir + "/design.topo")) {
        throw std::runtime_error("cannot initialize FPGA manager from " + case_dir);
    }
    read_in(case_dir + "/design.are", case_dir + "/design.net", state.name_to_id,
            state.id_to_name, state.g, manager);
    state.board_count = state.g.fpga_manager.fpga_num;
}

void load_names_only(State& state, const std::string& case_dir) {
    std::ifstream are(case_dir + "/design.are");
    if (!are) throw std::runtime_error("cannot open " + case_dir + "/design.are");
    int id = 0;
    std::string name;
    while (are >> name) {
        int ignored = 0;
        for (int i = 0; i < 8; ++i) are >> ignored;
        if (id < static_cast<int>(state.g.hypernodes.size())) {
            state.id_to_name[id] = name;
            state.name_to_id[name] = id;
        }
        ++id;
    }
    if (id != static_cast<int>(state.g.hypernodes.size())) {
        add_check(state, "case_node_count_matches_snapshot", "DIFF",
                  "case_nodes=" + std::to_string(id) +
                      " snapshot_nodes=" + std::to_string(state.g.hypernodes.size()));
    }
}

void load_placement(State& state, const std::string& path) {
    const int node_count = static_cast<int>(state.g.hypernodes.size());
    const int board_count = state.g.fpga_manager.fpga_num;
    std::vector<int> master(node_count, -1);
    std::vector<char> board_seen(board_count, 0);

    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open placement " + path);
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty()) continue;
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("invalid placement line " + std::to_string(line_number));
        }
        const std::string board_name = trim(line.substr(0, colon));
        const auto board_it = state.g.fpga_manager.name_to_id.find(board_name);
        if (board_it == state.g.fpga_manager.name_to_id.end()) {
            throw std::runtime_error("unknown board " + board_name);
        }
        const int board = board_it->second;
        if (board_seen[board]++) {
            throw std::runtime_error("duplicate board line " + board_name);
        }
        std::istringstream stream(line.substr(colon + 1));
        std::string token;
        while (stream >> token) {
            const bool replica = !token.empty() && token.back() == '*';
            if (replica) token.pop_back();
            const auto node_it = state.name_to_id.find(token);
            if (node_it == state.name_to_id.end()) {
                throw std::runtime_error("unknown node " + token);
            }
            const int node = node_it->second;
            if (replica) {
                if (master[node] == board ||
                    state.g.hypernodes[node].replication_fpga_labels.count(board)) {
                    throw std::runtime_error("duplicate/overlapping replica " + token);
                }
                state.g.hypernodes[node].replication_fpga_labels.insert(board);
            } else {
                if (master[node] != -1) {
                    throw std::runtime_error("multiple masters for " + token);
                }
                if (state.g.hypernodes[node].replication_fpga_labels.count(board)) {
                    throw std::runtime_error("master/replica overlap for " + token);
                }
                master[node] = board;
            }
        }
    }
    for (int board = 0; board < board_count; ++board) {
        if (!board_seen[board]) {
            throw std::runtime_error("placement lacks board line " +
                                     state.g.fpga_manager.getName(board));
        }
    }
    for (int node = 0; node < node_count; ++node) {
        if (master[node] < 0) {
            throw std::runtime_error("node " + std::to_string(node) + " lacks a master");
        }
        state.g.hypernodes[node].label = master[node];
    }
    state.placement_valid = true;
}

void load_snapshot(State& state, const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open snapshot " + path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) lines.push_back(line);

    long long meta_cost = -1;
    int board_count = -1;
    int max_hop = -1;
    for (const auto& current : lines) {
        const auto fields = split_ws(current);
        if (fields.empty()) continue;
        if (fields[0] == "META") {
            if (fields.size() < 4) throw std::runtime_error("invalid META line");
            meta_cost = std::stoll(fields[1]);
            board_count = std::stoi(fields[2]);
            max_hop = std::stoi(fields[3]);
            break;
        }
    }
    if (board_count < 0) throw std::runtime_error("snapshot lacks META");

    state.g = Hypergraph();
    state.g.fpga_manager.fpga_num = board_count;
    state.g.fpga_manager.max_hop_distance = max_hop;
    state.g.fpga_manager.fpga_info.assign(board_count, std::vector<int>(9, 0));
    state.g.fpga_manager.hop_distances.assign(board_count,
                                              std::vector<int>(board_count, -1));
    state.g.fpga_manager.id_to_name.resize(board_count);
    for (int board = 0; board < board_count; ++board) {
        state.g.fpga_manager.id_to_name[board] = "B" + std::to_string(board);
    }
    state.snapshot_usage.assign(board_count, std::vector<int>(8, 0));
    state.snapshot_comm.assign(board_count, 0);
    state.has_snapshot = true;
    state.snapshot_meta_cost = meta_cost;

    for (const auto& current : lines) {
        const auto fields = split_ws(current);
        if (fields.empty()) continue;
        const std::string& kind = fields[0];
        if (kind == "META") continue;
        if (kind == "BOARD") {
            if (fields.size() < 11) throw std::runtime_error("invalid BOARD line");
            const int board = std::stoi(fields[1]);
            if (board < 0 || board >= board_count) throw std::runtime_error("BOARD out of range");
            for (int i = 0; i < 9; ++i) {
                state.g.fpga_manager.fpga_info[board][i] = std::stoi(fields[2 + i]);
            }
        } else if (kind == "HOP") {
            if (fields.size() < static_cast<std::size_t>(2 + board_count)) {
                throw std::runtime_error("invalid HOP line");
            }
            const int board = std::stoi(fields[1]);
            for (int other = 0; other < board_count; ++other) {
                state.g.fpga_manager.hop_distances[board][other] =
                    std::stoi(fields[2 + other]);
            }
        } else if (kind == "USAGE") {
            if (fields.size() < 10) throw std::runtime_error("invalid USAGE line");
            const int board = std::stoi(fields[1]);
            for (int i = 0; i < 8; ++i) {
                state.snapshot_usage[board][i] = std::stoi(fields[2 + i]);
            }
        } else if (kind == "COMM") {
            if (fields.size() < 3) throw std::runtime_error("invalid COMM line");
            state.snapshot_comm[std::stoi(fields[1])] = std::stoi(fields[2]);
        } else if (kind == "NODE") {
            if (fields.size() < 12) throw std::runtime_error("invalid NODE line");
            const int node = std::stoi(fields[1]);
            while (static_cast<int>(state.g.hypernodes.size()) <= node) {
                const int index = static_cast<int>(state.g.hypernodes.size());
                state.g.hypernodes.emplace_back();
                Hypernode& created = state.g.hypernodes.back();
                created.id = index;
                created.label = -1;
                created.weights.assign(8, 0);
                created.deleted = true;
            }
            Hypernode& value = state.g.hypernodes[node];
            value.id = node;
            value.deleted = false;
            value.label = std::stoi(fields[2]);
            value.weights.assign(8, 0);
            for (int i = 0; i < 8; ++i) value.weights[i] = std::stoi(fields[3 + i]);
            const int replica_count = std::stoi(fields[11]);
            if (fields.size() < static_cast<std::size_t>(12 + replica_count)) {
                throw std::runtime_error("invalid NODE replica list");
            }
            value.replication_fpga_labels.clear();
            for (int i = 0; i < replica_count; ++i) {
                value.replication_fpga_labels.insert(std::stoi(fields[12 + i]));
            }
        } else if (kind == "NET") {
            if (fields.size() < 5) throw std::runtime_error("invalid NET line");
            const int net = std::stoi(fields[1]);
            while (static_cast<int>(state.g.hyperedges.size()) <= net) {
                const int index = static_cast<int>(state.g.hyperedges.size());
                state.g.hyperedges.emplace_back(0, 0, 0, index, board_count);
                state.g.hyperedges.back().deleted = true;
            }
            Hyperedge& edge = state.g.hyperedges[net];
            edge.id = net;
            edge.deleted = false;
            edge.weight = std::stoi(fields[2]);
            edge._begin = static_cast<int>(state.g._incidence_array.size());
            for (std::size_t i = 4; i < fields.size(); ++i) {
                state.g._incidence_array.push_back(std::stoi(fields[i]));
            }
            edge._size = static_cast<int>(state.g._incidence_array.size()) - edge._begin;
            if (static_cast<int>(state.snapshot_source_part.size()) <= net) {
                state.snapshot_source_part.resize(net + 1, -1);
                state.snapshot_weight.resize(net + 1, 0);
                state.snapshot_pins.resize(net + 1);
            }
            state.snapshot_source_part[net] = std::stoll(fields[3]);
            state.snapshot_weight[net] = edge.weight;
            state.snapshot_pins[net].assign(
                state.g._incidence_array.begin() + edge._begin,
                state.g._incidence_array.begin() + edge._begin + edge._size);
        }
    }

    state.g._num_vertices_initial = static_cast<int>(state.g.hypernodes.size());
    state.g._num_vertices_current = 0;
    for (const auto& node : state.g.hypernodes) {
        if (!node.deleted) ++state.g._num_vertices_current;
    }
    state.g._num_hyperedges = 0;
    for (const auto& edge : state.g.hyperedges) {
        if (!edge.deleted) ++state.g._num_hyperedges;
    }
    for (auto& node : state.g.hypernodes) {
        node.incident_nets_as_source.clear();
        node.incident_nets_as_target.clear();
    }
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        const int source = state.g._incidence_array[edge._begin];
        state.g.hypernodes[source].incident_nets_as_source.insert(edge.id);
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            state.g.hypernodes[state.g._incidence_array[i]].incident_nets_as_target.insert(
                edge.id);
        }
    }
    state.board_count = board_count;
}

void build_target_pins_reference(State& state) {
    const int board_count = state.board_count;
    state.target_pins_ref.assign(state.g.hyperedges.size(),
                                 std::vector<int>(board_count, 0));
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        const int source = state.g._incidence_array[edge._begin];
        const auto& source_replicas =
            state.g.hypernodes[source].replication_fpga_labels;
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const Hypernode& sink = state.g.hypernodes[state.g._incidence_array[i]];
            if (!source_replicas.count(sink.label)) {
                ++state.target_pins_ref[edge.id][sink.label];
            }
            for (int replica : sink.replication_fpga_labels) {
                if (!source_replicas.count(replica)) {
                    ++state.target_pins_ref[edge.id][replica];
                }
            }
        }
    }
}

void build_usage_and_comm(State& state) {
    const int board_count = state.board_count;
    state.usage.assign(board_count, std::vector<int>(8, 0));
    for (const auto& node : state.g.hypernodes) {
        if (node.deleted) continue;
        std::vector<int> boards;
        boards.push_back(node.label);
        for (int replica : node.replication_fpga_labels) boards.push_back(replica);
        for (int board : boards) {
            if (board < 0 || board >= board_count) {
                throw std::runtime_error("node placement board out of range");
            }
            for (int i = 0; i < 8; ++i) state.usage[board][i] += node.weights[i];
        }
    }

    state.comm.assign(board_count, 0);
    std::vector<int> mark(board_count, -1);
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        const int source = state.g._incidence_array[edge._begin];
        const int source_board = state.g.hypernodes[source].label;
        const auto& source_replicas =
            state.g.hypernodes[source].replication_fpga_labels;
        std::vector<int> targets;
        auto add_target = [&](int board) {
            if (board < 0 || board >= board_count) {
                throw std::runtime_error("net pin board out of range");
            }
            if (mark[board] == edge.id) return;
            mark[board] = edge.id;
            targets.push_back(board);
        };
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const Hypernode& sink = state.g.hypernodes[state.g._incidence_array[i]];
            add_target(sink.label);
            for (int replica : sink.replication_fpga_labels) add_target(replica);
        }
        std::vector<int> destinations;
        for (int board : targets) {
            if (board == source_board) continue;
            if (source_replicas.count(board)) continue;
            destinations.push_back(board);
        }
        if (!destinations.empty()) {
            state.comm[source_board] += edge.weight;
            for (int board : destinations) state.comm[board] += edge.weight;
        }
    }
}

void build_net_stats(State& state) {
    const int node_count = static_cast<int>(state.g.hypernodes.size());
    const int board_count = state.board_count;
    std::vector<int> node_mark(node_count, -1);
    std::vector<int> board_mark(board_count, -1);
    state.nets.clear();
    state.nets.reserve(state.g.hyperedges.size());

    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        NetStat stat;
        stat.id = edge.id;
        stat.source = state.g._incidence_array[edge._begin];
        stat.source_board = state.g.hypernodes[stat.source].label;
        stat.c_by_board.assign(board_count, 0);
        stat.native_target_pins = edge.target_pins_in_part;

        std::vector<int> unique_sinks;
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const int sink = state.g._incidence_array[i];
            if (node_mark[sink] == edge.id) continue;
            node_mark[sink] = edge.id;
            unique_sinks.push_back(sink);
        }
        const int sink_occurrences = edge._size - 1;
        stat.unique_sink_count = static_cast<int>(unique_sinks.size());
        stat.duplicate_pin_count = sink_occurrences - stat.unique_sink_count;

        for (int sink : unique_sinks) {
            const Hypernode& node = state.g.hypernodes[sink];
            ++stat.c_by_board[node.label];
            if (node.label != stat.source_board) ++stat.cross_sink_count;
            if (!node.replication_fpga_labels.empty()) stat.has_sink_replica = true;
        }
        stat.has_source_replica =
            !state.g.hypernodes[stat.source].replication_fpga_labels.empty();
        for (int board = 0; board < board_count; ++board) {
            if (board != stat.source_board && stat.c_by_board[board] > 0) ++stat.d_e;
        }

        for (int sink : unique_sinks) {
            const Hypernode& node = state.g.hypernodes[sink];
            auto add_board = [&](int board) {
                if (board < 0 || board >= board_count) {
                    throw std::runtime_error("net pin board out of range");
                }
                if (board_mark[board] == edge.id) return;
                board_mark[board] = edge.id;
                stat.v2_target_boards.push_back(board);
            };
            add_board(node.label);
            for (int replica : node.replication_fpga_labels) add_board(replica);
        }
        std::vector<int> filtered_targets;
        filtered_targets.reserve(stat.v2_target_boards.size());
        for (int board : stat.v2_target_boards) {
            if (board == stat.source_board) continue;
            if (state.g.hypernodes[stat.source].replication_fpga_labels.count(board)) {
                continue;
            }
            filtered_targets.push_back(board);
        }
        std::sort(filtered_targets.begin(), filtered_targets.end());
        filtered_targets.erase(std::unique(filtered_targets.begin(), filtered_targets.end()),
                               filtered_targets.end());
        stat.v2_target_boards = std::move(filtered_targets);
        stat.v2_target_count = static_cast<int>(stat.v2_target_boards.size());
        stat.native_source_board_pins =
            (stat.source_board >= 0 && stat.source_board < board_count)
                ? stat.native_target_pins[stat.source_board]
                : 0;

        try {
            stat.v2_cost = refinement_native_probe::netCost(state.g, edge.id);
            stat.v2_cost_valid = true;
        } catch (const std::exception&) {
            stat.v2_cost = -1;
            stat.v2_cost_valid = false;
        }
        state.nets.push_back(std::move(stat));
    }
}

void build_node_stats(State& state) {
    const int node_count = static_cast<int>(state.g.hypernodes.size());
    std::vector<std::vector<int>> incident(node_count);
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        incident[state.g._incidence_array[edge._begin]].push_back(edge.id);
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            incident[state.g._incidence_array[i]].push_back(edge.id);
        }
    }
    state.nodes.clear();
    state.nodes.reserve(node_count);
    for (int node = 0; node < node_count; ++node) {
        if (state.g.hypernodes[node].deleted) continue;
        NodeStat stat;
        stat.id = node;
        stat.board = state.g.hypernodes[node].label;
        stat.replicas.assign(state.g.hypernodes[node].replication_fpga_labels.begin(),
                             state.g.hypernodes[node].replication_fpga_labels.end());
        std::sort(stat.replicas.begin(), stat.replicas.end());
        std::sort(incident[node].begin(), incident[node].end());
        incident[node].erase(std::unique(incident[node].begin(), incident[node].end()),
                             incident[node].end());
        stat.net_degree = static_cast<int>(incident[node].size());
        state.nodes.push_back(std::move(stat));
    }
}

void compute_total_cost(State& state) {
    try {
        state.v2_cost = refinement_native_probe::cost(state.g);
        state.v2_cost_valid = true;
    } catch (const std::exception&) {
        state.v2_cost = -1;
        state.v2_cost_valid = false;
    }
}

void check_state(State& state) {
    const int board_count = state.board_count;

    bool placement_ok = true;
    std::string placement_detail;
    for (const auto& node : state.g.hypernodes) {
        if (node.deleted) continue;
        if (node.label < 0 || node.label >= board_count) {
            placement_ok = false;
            placement_detail += "node " + std::to_string(node.id) + " label out of range; ";
        }
        if (node.replication_fpga_labels.count(node.label)) {
            placement_ok = false;
            placement_detail += "node " + std::to_string(node.id) + " master/replica overlap; ";
        }
        for (int replica : node.replication_fpga_labels) {
            if (replica < 0 || replica >= board_count) {
                placement_ok = false;
                placement_detail += "node " + std::to_string(node.id) +
                                    " replica out of range; ";
            }
        }
    }
    add_check(state, "placement_valid", placement_ok ? "PASS" : "FAIL",
              placement_ok ? "all nodes have one in-range master and no overlap"
                           : placement_detail);

    bool resources_ok = true;
    std::ostringstream resource_detail;
    for (int board = 0; board < board_count; ++board) {
        for (int dim = 0; dim < 8; ++dim) {
            const int capacity = state.g.fpga_manager.fpga_info[board][dim + 1];
            if (state.usage[board][dim] > capacity) {
                if (resources_ok) {
                    resource_detail << "board=" << board << " dim=" << dim
                                    << " usage=" << state.usage[board][dim]
                                    << " capacity=" << capacity;
                }
                resources_ok = false;
            }
        }
    }
    add_check(state, "resource_capacity", resources_ok ? "PASS" : "FAIL",
              resources_ok ? "all 8 resource dimensions within capacity"
                           : resource_detail.str());

    bool comm_ok = true;
    std::ostringstream comm_detail;
    for (int board = 0; board < board_count; ++board) {
        const int capacity = state.g.fpga_manager.fpga_info[board][0];
        if (state.comm[board] > capacity) {
            if (comm_ok) {
                comm_detail << "board=" << board << " usage=" << state.comm[board]
                            << " capacity=" << capacity;
            }
            comm_ok = false;
        }
    }
    add_check(state, "communication_capacity", comm_ok ? "PASS" : "FAIL",
              comm_ok ? "all communication usages within capacity" : comm_detail.str());

    bool hop_ok = true;
    std::ostringstream hop_detail;
    for (const auto& net : state.nets) {
        if (!net.v2_cost_valid) continue;
        for (int board : net.v2_target_boards) {
            if (state.g.fpga_manager.hop_distances[net.source_board][board] < 0) {
                if (hop_ok) {
                    hop_detail << "net=" << net.id << " source_board="
                               << net.source_board << " target_board=" << board;
                }
                hop_ok = false;
            }
        }
    }
    add_check(state, "hop_reachable", hop_ok ? "PASS" : "FAIL",
              hop_ok ? "all V2 destination boards reachable within max hop"
                     : hop_detail.str());

    bool costs_ok = true;
    std::ostringstream cost_detail;
    for (const auto& net : state.nets) {
        if (!net.v2_cost_valid) {
            if (costs_ok) cost_detail << "first invalid net=" << net.id;
            costs_ok = false;
        }
    }
    add_check(state, "v2_net_costs", costs_ok ? "PASS" : "FAIL",
              costs_ok ? "netCost succeeded for every non-deleted net"
                       : cost_detail.str());
    add_check(state, "v2_total_cost", state.v2_cost_valid ? "PASS" : "FAIL",
              state.v2_cost_valid ? "cost=" + std::to_string(state.v2_cost)
                                  : "refinement_native_probe::cost failed");

    bool target_pins_ok = true;
    std::ostringstream target_detail;
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        for (int board = 0; board < board_count; ++board) {
            if (edge.target_pins_in_part[board] != state.target_pins_ref[edge.id][board]) {
                if (target_pins_ok) {
                    target_detail << "net=" << edge.id << " board=" << board
                                  << " native=" << edge.target_pins_in_part[board]
                                  << " reference=" << state.target_pins_ref[edge.id][board];
                }
                target_pins_ok = false;
            }
        }
    }
    add_check(state, "native_target_pins_recompute", target_pins_ok ? "PASS" : "FAIL",
              target_pins_ok ? "Hypergraph::initPinsInPart matches independent reference"
                             : target_detail.str());

    if (!state.has_snapshot) return;

    bool usage_match = true;
    std::ostringstream usage_detail;
    int usage_diff = 0;
    for (int board = 0; board < board_count; ++board) {
        for (int dim = 0; dim < 8; ++dim) {
            if (state.usage[board][dim] != state.snapshot_usage[board][dim]) {
                ++usage_diff;
                if (usage_match) {
                    usage_detail << "board=" << board << " dim=" << dim
                                 << " reconstructed=" << state.usage[board][dim]
                                 << " snapshot=" << state.snapshot_usage[board][dim];
                }
                usage_match = false;
            }
        }
    }
    add_check(state, "snapshot_usage_match", usage_match ? "PASS" : "FAIL",
              usage_match ? "all board/dimension usages match snapshot"
                          : "diffs=" + std::to_string(usage_diff) + " " +
                                usage_detail.str());

    bool comm_match = true;
    std::ostringstream comm_snapshot_detail;
    int comm_diff = 0;
    for (int board = 0; board < board_count; ++board) {
        if (state.comm[board] != state.snapshot_comm[board]) {
            ++comm_diff;
            if (comm_match) {
                comm_snapshot_detail << "board=" << board
                                     << " reconstructed=" << state.comm[board]
                                     << " snapshot=" << state.snapshot_comm[board];
            }
            comm_match = false;
        }
    }
    add_check(state, "snapshot_comm_match", comm_match ? "PASS" : "FAIL",
              comm_match ? "all board communication usages match snapshot"
                         : "diffs=" + std::to_string(comm_diff) + " " +
                               comm_snapshot_detail.str());

    bool source_part_match = true;
    std::ostringstream source_detail;
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        if (edge.id >= static_cast<int>(state.snapshot_source_part.size())) continue;
        if (edge.source_part_id != state.snapshot_source_part[edge.id]) {
            if (source_part_match) {
                source_detail << "net=" << edge.id << " reconstructed="
                              << edge.source_part_id << " snapshot="
                              << state.snapshot_source_part[edge.id];
            }
            source_part_match = false;
        }
    }
    add_check(state, "snapshot_source_part_match", source_part_match ? "PASS" : "FAIL",
              source_part_match ? "all net source boards match snapshot"
                                : source_detail.str());

    bool net_structure_match = true;
    std::ostringstream structure_detail;
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        if (edge.id >= static_cast<int>(state.snapshot_weight.size())) continue;
        if (edge.weight != state.snapshot_weight[edge.id]) {
            if (net_structure_match) {
                structure_detail << "net=" << edge.id << " weight mismatch";
            }
            net_structure_match = false;
        }
        if (edge.id < static_cast<int>(state.snapshot_pins.size())) {
            const auto& pins = state.snapshot_pins[edge.id];
            if (static_cast<int>(pins.size()) != edge._size) {
                if (net_structure_match) {
                    structure_detail << "net=" << edge.id << " pin count mismatch";
                }
                net_structure_match = false;
            } else {
                for (int i = 0; i < edge._size; ++i) {
                    if (pins[i] != state.g._incidence_array[edge._begin + i]) {
                        if (net_structure_match) {
                            structure_detail << "net=" << edge.id << " pin mismatch";
                        }
                        net_structure_match = false;
                    }
                }
            }
        }
    }
    add_check(state, "snapshot_net_structure_match", net_structure_match ? "PASS" : "FAIL",
              net_structure_match ? "all net weights and pin orders match snapshot"
                                  : structure_detail.str());

    if (state.v2_cost_valid) {
        if (state.snapshot_meta_cost == state.v2_cost) {
            add_check(state, "snapshot_meta_cost_vs_v2", "PASS",
                      "snapshot META cost=" + std::to_string(state.snapshot_meta_cost) +
                          " equals V2 cost");
        } else {
            add_check(state, "snapshot_meta_cost_vs_v2", "DIFF",
                      "snapshot META cost=" + std::to_string(state.snapshot_meta_cost) +
                          " V2 cost=" + std::to_string(state.v2_cost) +
                          " (different definitions; do not overwrite)");
        }
    }
}

void write_usage_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "board\tcomm\tnative_comm";
    for (int dim = 0; dim < 8; ++dim) out << "\tusage" << dim;
    for (int dim = 0; dim < 8; ++dim) out << "\tcapacity" << dim;
    if (state.has_snapshot) {
        out << "\tsnapshot_comm";
        for (int dim = 0; dim < 8; ++dim) out << "\tsnapshot_usage" << dim;
    }
    out << '\n';
    for (int board = 0; board < state.board_count; ++board) {
        out << board << '\t' << state.comm[board] << '\t'
            << state.g.fpga_manager.fpga_info[board][0];
        for (int dim = 0; dim < 8; ++dim) out << '\t' << state.usage[board][dim];
        for (int dim = 0; dim < 8; ++dim) {
            out << '\t' << state.g.fpga_manager.fpga_info[board][dim + 1];
        }
        if (state.has_snapshot) {
            out << '\t' << state.snapshot_comm[board];
            for (int dim = 0; dim < 8; ++dim) {
                out << '\t' << state.snapshot_usage[board][dim];
            }
        }
        out << '\n';
    }
}

void write_nets_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "net_id\tsource_node\tsource_node_name\tsource_board\tunique_sink_count"
           "\tcross_sink_count\td_e\tnc_c_counts\tnative_target_pins"
           "\tv2_target_boards\tv2_target_count\tv2_cost\tduplicate_pin_count"
           "\tnc_vs_v2_delta\thas_sink_replica\thas_source_replica"
           "\tnative_source_board_pins\n";
    for (const auto& net : state.nets) {
        const auto name_it = state.id_to_name.find(net.source);
        const std::string name = name_it == state.id_to_name.end()
                                     ? "n" + std::to_string(net.source)
                                     : name_it->second;
        out << net.id << '\t' << net.source << '\t' << name << '\t'
            << net.source_board << '\t' << net.unique_sink_count << '\t'
            << net.cross_sink_count << '\t' << net.d_e << '\t'
            << nonzero_counts(net.c_by_board) << '\t'
            << nonzero_counts(net.native_target_pins) << '\t'
            << join_ints(net.v2_target_boards, ',') << '\t'
            << net.v2_target_count << '\t' << net.v2_cost << '\t'
            << net.duplicate_pin_count << '\t'
            << (net.d_e - net.v2_target_count) << '\t'
            << (net.has_sink_replica ? 1 : 0) << '\t'
            << (net.has_source_replica ? 1 : 0) << '\t'
            << net.native_source_board_pins << '\n';
    }
}

void write_nodes_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "node_id\tnode_name\tboard\treplicas\tnet_degree\n";
    for (const auto& node : state.nodes) {
        const auto name_it = state.id_to_name.find(node.id);
        const std::string name = name_it == state.id_to_name.end()
                                     ? "n" + std::to_string(node.id)
                                     : name_it->second;
        out << node.id << '\t' << name << '\t' << node.board << '\t'
            << join_ints(node.replicas, ',') << '\t' << node.net_degree << '\n';
    }
}

void write_target_pins_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "net_id\tboard\tnative_count\treference_count\n";
    for (const auto& edge : state.g.hyperedges) {
        if (edge.deleted) continue;
        for (int board = 0; board < state.board_count; ++board) {
            out << edge.id << '\t' << board << '\t' << edge.target_pins_in_part[board]
                << '\t' << state.target_pins_ref[edge.id][board] << '\n';
        }
    }
}

void write_checks_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "check\tstatus\tdetail\n";
    for (const auto& check : state.checks) {
        out << check.name << '\t' << check.status << '\t' << check.detail << '\n';
    }
}

void write_semantic_differences_tsv(const State& state, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "net_id\tsource_node\tsource_board\tunique_sink_count\tcross_sink_count"
           "\td_e\tv2_target_count\tnc_vs_v2_delta\tduplicate_pin_count"
           "\tnative_source_board_pins\thas_sink_replica\thas_source_replica"
           "\tnc_c_counts\tnative_target_pins\n";
    for (const auto& net : state.nets) {
        const bool differs = (net.d_e != net.v2_target_count) ||
                             net.duplicate_pin_count > 0 ||
                             net.has_sink_replica || net.has_source_replica;
        if (!differs) continue;
        out << net.id << '\t' << net.source << '\t' << net.source_board << '\t'
            << net.unique_sink_count << '\t' << net.cross_sink_count << '\t'
            << net.d_e << '\t' << net.v2_target_count << '\t'
            << (net.d_e - net.v2_target_count) << '\t'
            << net.duplicate_pin_count << '\t' << net.native_source_board_pins
            << '\t' << (net.has_sink_replica ? 1 : 0) << '\t'
            << (net.has_source_replica ? 1 : 0) << '\t'
            << nonzero_counts(net.c_by_board) << '\t'
            << nonzero_counts(net.native_target_pins) << '\n';
    }
}

void write_summary_json(const State& state, const std::string& path,
                        const Options& options) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    long long sum_d = 0;
    long long sum_cross = 0;
    int cross_board_nets = 0;
    int max_d = 0;
    int duplicate_nets = 0;
    for (const auto& net : state.nets) {
        sum_d += net.d_e;
        sum_cross += net.cross_sink_count;
        if (net.d_e > 0) ++cross_board_nets;
        max_d = std::max(max_d, net.d_e);
        if (net.duplicate_pin_count > 0) ++duplicate_nets;
    }
    int replicated_nodes = 0;
    int replica_locations = 0;
    for (const auto& node : state.g.hypernodes) {
        if (node.deleted) continue;
        if (!node.replication_fpga_labels.empty()) ++replicated_nodes;
        replica_locations += static_cast<int>(node.replication_fpga_labels.size());
    }

    std::vector<const NetStat*> top;
    for (const auto& net : state.nets) top.push_back(&net);
    std::sort(top.begin(), top.end(), [](const NetStat* a, const NetStat* b) {
        if (a->d_e != b->d_e) return a->d_e > b->d_e;
        return a->id < b->id;
    });
    if (top.size() > 10) top.resize(10);

    out << "{\n";
    out << "  \"input_mode\": \""
        << (state.has_snapshot ? "snapshot" : "placement") << "\",\n";
    out << "  \"case_dir\": \"" << json_escape(options.case_dir) << "\",\n";
    out << "  \"placement\": \"" << json_escape(options.placement) << "\",\n";
    out << "  \"snapshot\": \"" << json_escape(options.snapshot) << "\",\n";
    out << "  \"boards\": " << state.board_count << ",\n";
    out << "  \"nodes\": " << state.g._num_vertices_current << ",\n";
    out << "  \"nets\": " << state.g._num_hyperedges << ",\n";
    out << "  \"replicated_nodes\": " << replicated_nodes << ",\n";
    out << "  \"replica_locations\": " << replica_locations << ",\n";
    out << "  \"duplicate_pin_nets\": " << duplicate_nets << ",\n";
    out << "  \"v2_cost\": " << state.v2_cost << ",\n";
    out << "  \"snapshot_meta_cost\": ";
    if (state.has_snapshot) out << state.snapshot_meta_cost;
    else out << "null";
    out << ",\n";
    out << "  \"sum_d_e\": " << sum_d << ",\n";
    out << "  \"sum_cross_sink_count\": " << sum_cross << ",\n";
    out << "  \"cross_board_nets\": " << cross_board_nets << ",\n";
    out << "  \"max_d_e\": " << max_d << ",\n";
    out << "  \"top_d_nets\": [\n";
    for (std::size_t i = 0; i < top.size(); ++i) {
        const NetStat& net = *top[i];
        out << "    {\"net_id\": " << net.id
            << ", \"source_node\": " << net.source
            << ", \"source_board\": " << net.source_board
            << ", \"d_e\": " << net.d_e
            << ", \"cross_sink_count\": " << net.cross_sink_count
            << ", \"v2_target_count\": " << net.v2_target_count
            << ", \"v2_cost\": " << net.v2_cost
            << ", \"duplicate_pin_count\": " << net.duplicate_pin_count
            << "}";
        if (i + 1 != top.size()) out << ',';
        out << '\n';
    }
    out << "  ]\n";
    out << "}\n";
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
            return argv[++i];
        };
        if (arg == "--case") options.case_dir = value(arg);
        else if (arg == "--placement") options.placement = value(arg);
        else if (arg == "--snapshot") options.snapshot = value(arg);
        else if (arg == "--out") options.output_dir = value(arg);
        else throw std::runtime_error("unknown option " + arg);
    }
    if (options.output_dir.empty()) throw std::runtime_error("--out is required");
    if (options.placement.empty() == options.snapshot.empty()) {
        throw std::runtime_error("exactly one of --placement or --snapshot is required");
    }
    if (!options.placement.empty() && options.case_dir.empty()) {
        throw std::runtime_error("--case is required for placement mode");
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        fs::create_directories(options.output_dir);

        State state;
        if (!options.snapshot.empty()) {
            load_snapshot(state, options.snapshot);
            if (!options.case_dir.empty()) load_names_only(state, options.case_dir);
        } else {
            load_case(state, options.case_dir);
            load_placement(state, options.placement);
        }

        state.g.initPinsInPart();
        build_target_pins_reference(state);
        build_usage_and_comm(state);
        build_net_stats(state);
        build_node_stats(state);
        compute_total_cost(state);
        check_state(state);

        write_nets_tsv(state, options.output_dir + "/nets.tsv");
        write_nodes_tsv(state, options.output_dir + "/nodes.tsv");
        write_usage_tsv(state, options.output_dir + "/usage.tsv");
        write_target_pins_tsv(state, options.output_dir + "/target_pins.tsv");
        write_checks_tsv(state, options.output_dir + "/state_checks.tsv");
        write_semantic_differences_tsv(
            state, options.output_dir + "/semantic_differences.tsv");
        write_summary_json(state, options.output_dir + "/summary.json", options);

        std::cout << "module1 nodes=" << state.g._num_vertices_current
                  << " nets=" << state.g._num_hyperedges
                  << " boards=" << state.board_count
                  << " v2_cost=" << state.v2_cost
                  << " sum_d_e=";
        long long sum_d = 0;
        for (const auto& net : state.nets) sum_d += net.d_e;
        std::cout << sum_d << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "net_centered_module1 error: " << error.what() << '\n';
        return 1;
    }
}
