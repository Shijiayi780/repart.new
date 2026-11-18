#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "prediction_types.h"

namespace route_prediction_v1 {

class FpgaTopology {
public:
    void load(const std::string& info_path, const std::string& topo_path);
    void initialize(const std::vector<std::string>& fpga_names,
                    const std::vector<std::pair<int, int>>& undirected_edges,
                    int maximum_hop);

    int fpgaCount() const { return static_cast<int>(names_.size()); }
    int maximumHop() const { return maximum_hop_; }
    const std::string& name(int id) const;
    int id(const std::string& name) const;
    const std::vector<PhysicalLink>& links() const { return links_; }
    const std::vector<PhysicalLink>& inputPhysicalLinks() const {
        return input_physical_links_;
    }
    const std::vector<int>& adjacency(int fpga) const;
    int hop(int source, int destination) const;
    const std::vector<int>& path(int source, int destination) const;
    int linkId(int u, int v) const;
    bool connected() const;
    void validatePath(int source, int destination, const std::vector<int>& path) const;

private:
    void precomputePaths();

    int maximum_hop_ = 0;
    std::vector<std::string> names_;
    std::unordered_map<std::string, int> name_to_id_;
    std::vector<PhysicalLink> links_;
    std::vector<PhysicalLink> input_physical_links_;
    std::vector<std::vector<int>> adjacency_;
    std::vector<std::vector<int>> hops_;
    std::vector<std::vector<std::vector<int>>> paths_;
    std::unordered_map<std::uint64_t, int> link_ids_;
};

}  // namespace route_prediction_v1
