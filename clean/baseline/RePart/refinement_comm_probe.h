#pragma once

// Observation-only communication-cache probe.
//
// The probe never writes to the hypergraph, resource usage, communication
// cache, candidate order, or refinement state. It computes an independent
// reference communication usage from the current placement/replica state and
// compares it with the native cache passed by the caller.
//
// It is active only when REPART_COMM_PROBE is defined at compile time and
// REPART_COMM_PROBE_DIR is set at runtime. Without those, this header has no
// effect on the original program.

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "./datastructure/hypergraph.h"

namespace refinement_comm_probe {

inline void set_context(int exploration, int layer);

inline std::vector<int> reference_comm(const Hypergraph& g) {
    const int board_count = g.fpga_manager.fpga_num;
    std::vector<int> comm(board_count, 0);
    std::vector<int> mark(board_count, -1);
    for (const auto& edge : g.hyperedges) {
        if (edge.deleted) continue;
        const int source = g._incidence_array[edge._begin];
        const int source_board = g.hypernodes[source].label;
        const auto& source_replicas =
            g.hypernodes[source].replication_fpga_labels;
        std::vector<int> targets;
        auto add_target = [&](int board) {
            if (board < 0 || board >= board_count) return;
            if (mark[board] == edge.id) return;
            mark[board] = edge.id;
            targets.push_back(board);
        };
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const Hypernode& sink = g.hypernodes[g._incidence_array[i]];
            add_target(sink.label);
            for (int replica : sink.replication_fpga_labels) add_target(replica);
        }
        bool has_destination = false;
        for (int board : targets) {
            if (board == source_board) continue;
            if (source_replicas.count(board)) continue;
            has_destination = true;
        }
        if (!has_destination) continue;
        comm[source_board] += edge.weight;
        for (int board : targets) {
            if (board == source_board) continue;
            if (source_replicas.count(board)) continue;
            comm[board] += edge.weight;
        }
    }
    return comm;
}

inline std::vector<int> net_contribution(const Hypergraph& g,
                                         const Hyperedge& edge) {
    const int board_count = g.fpga_manager.fpga_num;
    std::vector<int> contribution(board_count, 0);
    if (edge.deleted) return contribution;
    const int source = g._incidence_array[edge._begin];
    const int source_board = g.hypernodes[source].label;
    const auto& source_replicas =
        g.hypernodes[source].replication_fpga_labels;
    std::vector<int> targets;
    std::vector<int> mark(board_count, -1);
    auto add_target = [&](int board) {
        if (board < 0 || board >= board_count) return;
        if (mark[board] == edge.id) return;
        mark[board] = edge.id;
        targets.push_back(board);
    };
    for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
        const Hypernode& sink = g.hypernodes[g._incidence_array[i]];
        add_target(sink.label);
        for (int replica : sink.replication_fpga_labels) add_target(replica);
    }
    bool has_destination = false;
    for (int board : targets) {
        if (board == source_board) continue;
        if (source_replicas.count(board)) continue;
        has_destination = true;
    }
    if (!has_destination) return contribution;
    contribution[source_board] += edge.weight;
    for (int board : targets) {
        if (board == source_board) continue;
        if (source_replicas.count(board)) continue;
        contribution[board] += edge.weight;
    }
    return contribution;
}

inline void append_header_if_empty(std::ofstream& out, const char* header) {
    out.seekp(0, std::ios::end);
    if (out.tellp() == 0) out << header;
}

inline void observe(const Hypergraph& g, const std::vector<int>& cache_comm,
                    int exploration, int layer, const char* stage) {
    if (std::string(stage) == "entry") set_context(exploration, layer);
    const char* directory = std::getenv("REPART_COMM_PROBE_DIR");
    if (directory == nullptr) return;
    const int board_count = g.fpga_manager.fpga_num;
    if (static_cast<int>(cache_comm.size()) != board_count) return;

    const std::vector<int> reference = reference_comm(g);
    long long cache_sum = 0;
    long long reference_sum = 0;
    long long absolute_difference = 0;
    int difference_boards = 0;
    std::vector<char> differing(board_count, 0);
    std::ostringstream detail;
    for (int board = 0; board < board_count; ++board) {
        cache_sum += cache_comm[board];
        reference_sum += reference[board];
        if (cache_comm[board] != reference[board]) {
            ++difference_boards;
            differing[board] = 1;
            absolute_difference += std::llabs(
                static_cast<long long>(cache_comm[board]) - reference[board]);
            detail << board << ':' << cache_comm[board] << ':' << reference[board]
                   << ';';
        }
    }

    const std::string summary_path =
        std::string(directory) + "/comm_probe.tsv";
    std::ofstream summary(summary_path, std::ios::app);
    if (!summary) return;
    append_header_if_empty(
        summary,
        "exploration\tlayer\tstage\tboards\tcache_sum\treference_sum"
        "\tdiff_boards\tabs_diff\tdetail\n");
    summary << exploration << '\t' << layer << '\t' << stage << '\t'
            << board_count << '\t' << cache_sum << '\t' << reference_sum << '\t'
            << difference_boards << '\t' << absolute_difference << '\t'
            << detail.str() << '\n';
    summary.close();

    if (difference_boards == 0) return;

    const std::string nets_path =
        std::string(directory) + "/comm_probe_nets.tsv";
    std::ofstream nets(nets_path, std::ios::app);
    if (!nets) return;
    append_header_if_empty(
        nets,
        "exploration\tlayer\tstage\tnet_id\tsource_node\tsource_board\tweight"
        "\tsource_replicas\ttarget_boards\tcontributions\n");
    for (const auto& edge : g.hyperedges) {
        if (edge.deleted) continue;
        const std::vector<int> contribution = net_contribution(g, edge);
        bool relevant = false;
        for (int board = 0; board < board_count; ++board) {
            if (differing[board] && contribution[board] != 0) {
                relevant = true;
                break;
            }
        }
        if (!relevant) continue;
        const int source = g._incidence_array[edge._begin];
        std::vector<int> source_replica_values(
            g.hypernodes[source].replication_fpga_labels.begin(),
            g.hypernodes[source].replication_fpga_labels.end());
        std::sort(source_replica_values.begin(), source_replica_values.end());
        std::ostringstream source_replicas;
        bool first = true;
        for (int replica : source_replica_values) {
            if (!first) source_replicas << ',';
            first = false;
            source_replicas << replica;
        }
        if (first) source_replicas << '-';
        std::ostringstream targets;
        std::vector<int> mark(board_count, -1);
        first = true;
        for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
            const Hypernode& sink = g.hypernodes[g._incidence_array[i]];
            std::vector<int> boards;
            boards.push_back(sink.label);
            for (int replica : sink.replication_fpga_labels) boards.push_back(replica);
            for (int board : boards) {
                if (mark[board] == edge.id) continue;
                mark[board] = edge.id;
                if (!first) targets << ',';
                first = false;
                targets << board;
            }
        }
        if (first) targets << '-';
        std::ostringstream contributions;
        first = true;
        for (int board = 0; board < board_count; ++board) {
            if (contribution[board] == 0) continue;
            if (!first) contributions << ';';
            first = false;
            contributions << board << ':' << contribution[board];
        }
        if (first) contributions << '-';
        nets << exploration << '\t' << layer << '\t' << stage << '\t'
             << edge.id << '\t' << source << '\t'
             << g.hypernodes[source].label << '\t' << edge.weight << '\t'
             << source_replicas.str() << '\t' << targets.str() << '\t'
             << contributions.str() << '\n';
    }
}

inline int current_exploration = -1;
inline int current_layer = -1;
inline bool first_difference_reported = false;

inline void set_context(int exploration, int layer) {
    current_exploration = exploration;
    current_layer = layer;
    first_difference_reported = false;
}

inline bool actions_enabled() {
    const char* directory = std::getenv("REPART_COMM_PROBE_DIR");
    if (directory == nullptr) return false;
    const char* actions = std::getenv("REPART_COMM_PROBE_ACTIONS");
    if (actions == nullptr || std::string(actions) != "1") return false;
    const char* exploration = std::getenv("REPART_COMM_PROBE_EXPLORATION");
    if (exploration != nullptr && std::atoi(exploration) != current_exploration) {
        return false;
    }
    const char* layer = std::getenv("REPART_COMM_PROBE_LAYER");
    if (layer != nullptr && std::atoi(layer) != current_layer) return false;
    return !first_difference_reported;
}

inline bool log_all_actions_enabled() {
    const char* value = std::getenv("REPART_COMM_PROBE_LOG_ALL_ACTIONS");
    return value != nullptr && std::string(value) == "1";
}

inline std::string contribution_string(const std::vector<int>& contribution) {
    std::ostringstream out;
    bool first = true;
    for (std::size_t board = 0; board < contribution.size(); ++board) {
        if (contribution[board] == 0) continue;
        if (!first) out << ';';
        first = false;
        out << board << ':' << contribution[board];
    }
    if (first) out << '-';
    return out.str();
}

inline std::string target_board_string(const Hypergraph& g,
                                       const Hyperedge& edge) {
    std::vector<int> boards;
    for (int i = edge._begin + 1; i < edge._begin + edge._size; ++i) {
        const Hypernode& sink = g.hypernodes[g._incidence_array[i]];
        boards.push_back(sink.label);
        for (int replica : sink.replication_fpga_labels) boards.push_back(replica);
    }
    std::sort(boards.begin(), boards.end());
    boards.erase(std::unique(boards.begin(), boards.end()), boards.end());
    std::ostringstream out;
    bool first = true;
    for (int board : boards) {
        if (!first) out << ',';
        first = false;
        out << board;
    }
    if (first) out << '-';
    return out.str();
}

class ActionProbe {
  public:
    ActionProbe(const Hypergraph& g, const std::vector<int>& cache_comm,
                int u, int v = -1)
        : g_(g), cache_(cache_comm), u_(u), v_(v), enabled_(actions_enabled()) {
        if (!enabled_) return;
        cache_before_ = cache_;
        reference_before_ = reference_comm(g_);
        std::set<int> incident;
        auto collect = [&](int node) {
            if (node < 0) return;
            for (int net : g_.hypernodes[node].incident_nets_as_source) {
                if (!g_.hyperedges[net].deleted) incident.insert(net);
            }
            for (int net : g_.hypernodes[node].incident_nets_as_target) {
                if (!g_.hyperedges[net].deleted) incident.insert(net);
            }
        };
        collect(u_);
        collect(v_);
        for (int net : incident) {
            const Hyperedge& edge = g_.hyperedges[net];
            const int source = g_._incidence_array[edge._begin];
            net_ids_.push_back(net);
            source_nodes_.push_back(source);
            source_boards_before_.push_back(g_.hypernodes[source].label);
            contributions_before_.push_back(net_contribution(g_, edge));
        }
    }

    void finish(const char* kind, int from_board, int target_board, int gain) {
        if (!enabled_ || first_difference_reported) return;
        const int board_count = g_.fpga_manager.fpga_num;
        const std::vector<int> reference_after = reference_comm(g_);

        std::vector<int> mismatch_boards;
        for (int board = 0; board < board_count; ++board) {
            const int cache_delta = cache_[board] - cache_before_[board];
            const int reference_delta =
                reference_after[board] - reference_before_[board];
            if (cache_delta != reference_delta) mismatch_boards.push_back(board);
        }
        const bool before_consistent = (cache_before_ == reference_before_);
        const char* directory = std::getenv("REPART_COMM_PROBE_DIR");
        if (directory == nullptr) return;

        long long cache_before_sum = 0;
        long long cache_after_sum = 0;
        long long reference_before_sum = 0;
        long long reference_after_sum = 0;
        for (int board = 0; board < board_count; ++board) {
            cache_before_sum += cache_before_[board];
            cache_after_sum += cache_[board];
            reference_before_sum += reference_before_[board];
            reference_after_sum += reference_after[board];
        }
        std::ostringstream mismatch;
        for (int board : mismatch_boards) {
            mismatch << board << ':' << (cache_[board] - cache_before_[board])
                     << ':' << (reference_after[board] - reference_before_[board])
                     << ':' << cache_before_[board] << ':' << cache_[board]
                     << ':' << reference_before_[board] << ':'
                     << reference_after[board] << ';';
        }

        const bool delta_match = mismatch_boards.empty();
        if (!delta_match || log_all_actions_enabled()) {
            const std::string checks_path =
                std::string(directory) + "/comm_probe_action_checks.tsv";
            std::ofstream checks(checks_path, std::ios::app);
            if (checks) {
                append_header_if_empty(
                    checks,
                    "exploration\tlayer\tkind\tu\tv\tu_board_after"
                    "\tv_board_after\tfrom_board\ttarget_board\tgain"
                    "\tcache_before_sum\tcache_after_sum\treference_before_sum"
                    "\treference_after_sum\tdelta_match\tmismatch_detail\n");
                checks << current_exploration << '\t' << current_layer << '\t'
                       << kind << '\t' << u_ << '\t' << v_ << '\t'
                       << (u_ >= 0 ? g_.hypernodes[u_].label : -1) << '\t'
                       << (v_ >= 0 ? g_.hypernodes[v_].label : -1) << '\t'
                       << from_board << '\t' << target_board << '\t' << gain
                       << '\t' << cache_before_sum << '\t' << cache_after_sum
                       << '\t' << reference_before_sum << '\t'
                       << reference_after_sum << '\t' << (delta_match ? 1 : 0)
                       << '\t' << mismatch.str() << '\n';
                checks.close();
            }
        }
        if (delta_match) return;

        const std::string actions_path =
            std::string(directory) + "/comm_probe_actions.tsv";
        std::ofstream actions(actions_path, std::ios::app);
        if (actions) {
            append_header_if_empty(
                actions,
                "exploration\tlayer\tkind\tu\tv\tu_board_after\tv_board_after"
                "\tfrom_board\ttarget_board\tgain\tcache_before_sum"
                "\tcache_after_sum\treference_before_sum\treference_after_sum"
                "\tbefore_consistent\tmismatch_detail\n");
            actions << current_exploration << '\t' << current_layer << '\t'
                    << kind << '\t' << u_ << '\t' << v_ << '\t'
                    << (u_ >= 0 ? g_.hypernodes[u_].label : -1) << '\t'
                    << (v_ >= 0 ? g_.hypernodes[v_].label : -1) << '\t'
                    << from_board << '\t' << target_board << '\t' << gain << '\t'
                    << cache_before_sum << '\t' << cache_after_sum << '\t'
                    << reference_before_sum << '\t' << reference_after_sum << '\t'
                    << (before_consistent ? 1 : 0) << '\t' << mismatch.str()
                    << '\n';
            actions.close();
        }

        const std::string nets_path =
            std::string(directory) + "/comm_probe_action_nets.tsv";
        std::ofstream nets(nets_path, std::ios::app);
        if (nets) {
            append_header_if_empty(
                nets,
                "exploration\tlayer\tkind\tu\tv\tnet_id\tsource_node"
                "\tsource_board_before\tsource_board_after\tsource_replicas_after"
                "\ttarget_boards_after\tweight\tcontribution_before"
                "\tcontribution_after\tdelta\n");
            for (std::size_t index = 0; index < net_ids_.size(); ++index) {
                const int net = net_ids_[index];
                const Hyperedge& edge = g_.hyperedges[net];
                if (edge.deleted) continue;
                const std::vector<int> contribution_after =
                    net_contribution(g_, edge);
                const std::vector<int>& contribution_before =
                    contributions_before_[index];
                bool changed = false;
                std::ostringstream delta;
                bool first = true;
                for (int board = 0; board < board_count; ++board) {
                    const int difference =
                        contribution_after[board] - contribution_before[board];
                    if (difference == 0) continue;
                    changed = true;
                    if (!first) delta << ';';
                    first = false;
                    delta << board << ':' << difference;
                }
                if (!changed) continue;
                const int source = source_nodes_[index];
                std::vector<int> source_replicas(
                    g_.hypernodes[source].replication_fpga_labels.begin(),
                    g_.hypernodes[source].replication_fpga_labels.end());
                std::sort(source_replicas.begin(), source_replicas.end());
                std::ostringstream replicas;
                bool first_replica = true;
                for (int replica : source_replicas) {
                    if (!first_replica) replicas << ',';
                    first_replica = false;
                    replicas << replica;
                }
                if (first_replica) replicas << '-';
                nets << current_exploration << '\t' << current_layer << '\t'
                     << kind << '\t' << u_ << '\t' << v_ << '\t' << net << '\t'
                     << source << '\t' << source_boards_before_[index] << '\t'
                     << g_.hypernodes[source].label << '\t' << replicas.str()
                     << '\t' << target_board_string(g_, edge) << '\t'
                     << edge.weight << '\t'
                     << contribution_string(contribution_before) << '\t'
                     << contribution_string(contribution_after) << '\t'
                     << delta.str() << '\n';
            }
        }
        first_difference_reported = true;
    }

  private:
    const Hypergraph& g_;
    const std::vector<int>& cache_;
    int u_;
    int v_;
    bool enabled_;
    std::vector<int> cache_before_;
    std::vector<int> reference_before_;
    std::vector<int> net_ids_;
    std::vector<int> source_nodes_;
    std::vector<int> source_boards_before_;
    std::vector<std::vector<int>> contributions_before_;
};

}  // namespace refinement_comm_probe
