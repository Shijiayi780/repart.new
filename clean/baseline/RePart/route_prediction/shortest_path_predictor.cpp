#include "shortest_path_predictor.h"

#include <stdexcept>

namespace route_prediction_v1 {

std::vector<RoutedConnection> ShortestPathPredictor::route(const std::vector<Connection>& connections,
                                                           const FpgaTopology& topology) {
    std::vector<RoutedConnection> routed;
    routed.reserve(connections.size());
    for (const auto& connection : connections) {
        RoutedConnection item;
        item.connection = connection;
        const int hop = topology.hop(connection.source_fpga, connection.destination_fpga);
        item.reachable = hop >= 0 && hop <= topology.maximumHop();
        if (item.reachable) item.fpga_path = topology.path(connection.source_fpga, connection.destination_fpga);
        if (item.reachable) {
            topology.validatePath(connection.source_fpga, connection.destination_fpga, item.fpga_path);
            for (std::size_t i = 1; i < item.fpga_path.size(); ++i) {
                const int link = topology.linkId(item.fpga_path[i - 1], item.fpga_path[i]);
                if (link < 0) throw std::runtime_error("routed path contains an invalid link");
                item.physical_link_ids.push_back(link);
            }
        }
        routed.push_back(std::move(item));
    }
    return routed;
}

}  // namespace route_prediction_v1
