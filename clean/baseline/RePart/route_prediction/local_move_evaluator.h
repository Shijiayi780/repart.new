#pragma once

#include <vector>

#include "connection_projector.h"
#include "hot_candidate_types.h"
#include "prediction_metrics.h"

namespace route_prediction_b4 {

struct ConstraintData {
    std::vector<ResourceVector> node_resources;
    std::vector<ResourceVector> fpga_capacities;
    std::vector<std::int64_t> fpga_comm_capacities;
};

class LocalMoveEvaluator {
public:
    LocalMoveEvaluator(const route_prediction_v1::CaseData& data,
                       const route_prediction_v1::FpgaTopology& topology,
                       ConstraintData constraints, int hot_link_id);

    MoveEvaluation evaluate(int node, int target_fpga, EvaluationTiming* timing=nullptr) const;
    MoveEvaluation evaluateFull(int node, int target_fpga) const;
    bool equivalentToFull(const MoveEvaluation& local, const MoveEvaluation& full) const;

    const route_prediction_v1::PredictionMetrics& baselineMetrics() const { return baseline_metrics_; }
    const std::vector<route_prediction_v1::RoutedConnection>& baselineRouted() const { return baseline_routed_; }
    const std::vector<std::vector<int>>& incidentNets() const { return incident_nets_; }
    const std::vector<ResourceVector>& resourceUsage() const { return resource_usage_; }
    const std::vector<std::int64_t>& communicationUsage() const { return communication_usage_; }
    bool resourceLegal(int node, int target_fpga) const;

private:
    std::vector<route_prediction_v1::RoutedConnection> routeNets(
        const std::vector<int>& net_ids,
        const std::vector<route_prediction_v1::NodePlacement>& placements) const;
    std::vector<std::int64_t> communicationUsage(
        const std::vector<route_prediction_v1::NodePlacement>& placements) const;
    MoveEvaluation summarize(int node, int target,
        const std::vector<route_prediction_v1::RoutedConnection>& before,
        const std::vector<route_prediction_v1::RoutedConnection>& after,
        const std::vector<std::int64_t>& comm_after, bool full) const;

    const route_prediction_v1::CaseData& data_;
    const route_prediction_v1::FpgaTopology& topology_;
    ConstraintData constraints_;
    int hot_link_id_;
    std::vector<std::vector<int>> incident_nets_;
    std::vector<ResourceVector> resource_usage_;
    std::vector<std::int64_t> communication_usage_;
    std::vector<route_prediction_v1::RoutedConnection> baseline_routed_;
    route_prediction_v1::PredictionMetrics baseline_metrics_;
};

}  // namespace route_prediction_b4
