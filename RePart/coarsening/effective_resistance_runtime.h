#pragma once
#include <cstddef>
#include <string>

namespace repart_er {
extern bool enabled;
void initialize(const std::string& path, std::size_t expected_nets);
float contribution(std::size_t original_net_id, int weight);
}
