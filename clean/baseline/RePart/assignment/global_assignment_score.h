#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <vector>

namespace global_assignment_score {
using Resources = std::array<std::int64_t, 8>;
struct Parameters {
    double w_ref, theta, eta, lambda, mu, epsilon;
    int d_max;
};

// Trial defaults, not paper parameters. Freeze the result for one layer search.
inline Parameters trial_defaults(std::vector<double> strengths,
                                 const std::vector<int>& physical_degrees) {
    for (double a : strengths)
        if (!std::isfinite(a) || a < 0) throw std::invalid_argument("invalid strength");
    strengths.erase(std::remove(strengths.begin(), strengths.end(), 0.0), strengths.end());
    std::sort(strengths.begin(), strengths.end());
    double reference = 1.0;
    if (!strengths.empty()) {
        const auto n = strengths.size();
        reference = n % 2 ? strengths[n / 2]
                         : strengths[n / 2 - 1] / 2.0 + strengths[n / 2] / 2.0;
    }
    int maximum = 0;
    for (int d : physical_degrees) {
        if (d < 0) throw std::invalid_argument("negative physical degree");
        maximum = std::max(maximum, d);
    }
    return {reference, 0.01 * reference, 0.1 * reference / std::max(1, maximum),
            0.0, 0.1, 1e-3, maximum};
}

struct AssignedNeighbor {
    double weight; // Aggregated bidirectional Driver-Sink weight, once per neighbor.
    int fpga;
    int hop;       // Distance from candidate FPGA; -1 means unreachable.
};
struct Result {
    enum class Rejection { None, Resource, Hop, Communication };
    bool feasible = false;
    Rejection rejection = Rejection::None;
    int resource_dimension = -1;
    double capacity = 0, connection = 0, resource_penalty = 0;
    double topology_reward = 0, hop_penalty = 0, score = 0;
};

// Communication capacity and fixed-position legality are supplied by the caller.
// This first version always enforces hard Hop feasibility, even if lambda changes.
inline Result evaluate(const Resources& demand, const Resources& capacity,
                       const Resources& used, int target, int degree, int max_hop,
                       const std::vector<AssignedNeighbor>& neighbors,
                       bool communication_and_fixed_feasible, const Parameters& p) {
    for (double a : {p.theta, p.eta, p.lambda, p.mu})
        if (!std::isfinite(a) || a < 0) throw std::invalid_argument("invalid coefficient");
    if (!std::isfinite(p.epsilon) || p.epsilon <= 0 || p.epsilon > 1 ||
        target < 0 || degree < 0 || max_hop < 0)
        throw std::invalid_argument("invalid score configuration");
    Result out;
    if (!communication_and_fixed_feasible) { out.rejection=Result::Rejection::Communication; return out; }
    double minimum = 1.0; // Also defines c=1 when all demand entries are zero.
    for (std::size_t k = 0; k < demand.size(); ++k) {
        if (demand[k] < 0 || capacity[k] < 0 || used[k] < 0)
            throw std::invalid_argument("negative resource");
        // Subtraction avoids overflow in used+demand.
        if (used[k] > capacity[k] || demand[k] > capacity[k] - used[k]) {
            out.rejection=Result::Rejection::Resource;
            out.resource_dimension=static_cast<int>(k);
            return out;
        }
        if (demand[k] > 0)
            minimum = std::min(minimum, double(capacity[k] - used[k] - demand[k]) /
                                             double(capacity[k]));
    }
    for (const auto& v : neighbors) {
        if (!std::isfinite(v.weight) || v.weight < 0 || v.fpga < 0)
            throw std::invalid_argument("invalid neighbor");
        if (v.hop < 0 || v.hop > max_hop) { out.rejection=Result::Rejection::Hop; return out; }
        if (v.fpga == target) out.connection += v.weight;
    }
    out.capacity = std::max(p.epsilon, minimum);
    out.resource_penalty = p.theta / out.capacity;
    out.topology_reward = p.eta * degree;
    // No legal neighbor violates max_hop, hence the user's final term is zero.
    out.hop_penalty = 0;
    out.score = out.connection - out.resource_penalty + out.topology_reward;
    out.feasible = true;
    return out;
}

struct Candidate { int node, fpga; double score; };
inline bool precedes(const Candidate& a, const Candidate& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.node != b.node) return a.node < b.node;
    return a.fpga < b.fpga;
}

inline void log_parameters(std::ostream& os, const Parameters& p) {
    const auto precision = os.precision();
    os << std::setprecision(17)
       << "parameter_origin=trial_defaults_or_explicit_override"
       << " W_ref=" << p.w_ref << " d_max=" << p.d_max
       << " theta=" << p.theta << " eta=" << p.eta << " lambda=" << p.lambda
       << " mu=" << p.mu << " epsilon=" << p.epsilon << " hard_hop=1\n";
    os.precision(precision);
}
} // namespace global_assignment_score
