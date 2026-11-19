#include "effective_resistance_runtime.h"
#include "effective_resistance_features.h"
#include <memory>

namespace repart_er {
bool enabled = false;
namespace {
std::unique_ptr<const EffectiveResistanceFeatures> features;
}

void initialize(const std::string& path, const std::size_t expected_nets) {
  auto loaded = EffectiveResistanceFeatures::load(path, expected_nets);
  for (std::size_t i = 0; i < loaded.size(); ++i) {
    if (loaded.at(i) <= 1.0) {
      throw std::runtime_error("ER must exceed one for weight/(ER-1)");
    }
  }
  features.reset(new EffectiveResistanceFeatures(std::move(loaded)));
  enabled = true;
}

float contribution(const std::size_t original_net_id, const int weight) {
  if (!features) throw std::runtime_error("ER features not initialized");
  return static_cast<float>(static_cast<double>(weight) /
                            (features->at(original_net_id) - 1.0));
}
}
