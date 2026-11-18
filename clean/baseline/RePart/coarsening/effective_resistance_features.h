#pragma once

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace repart_er {

// Immutable, original-net-indexed features. No scoring or community policy.
class EffectiveResistanceFeatures {
 public:
  static EffectiveResistanceFeatures load(const std::string& path,
                                           const std::size_t expected_nets) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open ER file: " + path);
    std::vector<double> values;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
      ++line_number;
      std::istringstream row(line);
      double value;
      std::string extra;
      if (!(row >> value) || (row >> extra) || !std::isfinite(value) || value <= 0) {
        throw std::runtime_error("Expected one positive finite ER at line " +
                                 std::to_string(line_number));
      }
      values.push_back(value);
      if (values.size() > expected_nets) {
        throw std::runtime_error("ER count exceeds original net count");
      }
    }
    if (input.bad()) throw std::runtime_error("Failed while reading ER file: " + path);
    if (values.size() != expected_nets) {
      throw std::runtime_error("ER count does not match original net count");
    }
    EffectiveResistanceFeatures result;
    result.values_ = std::move(values);
    return result;
  }

  double at(const std::size_t original_net_id) const {
    return values_.at(original_net_id);
  }

  std::size_t size() const { return values_.size(); }

 private:
  std::vector<double> values_;
};

}  // namespace repart_er
