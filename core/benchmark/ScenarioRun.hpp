#pragma once

#include <cstdint>
#include <string>

#include "core/scenario/Truth.hpp"

namespace navtracker {
namespace benchmark {

struct ScenarioDescriptor {
  std::string label;
  bool is_multi_seed{false};
  std::uint32_t seed_count{1};
};

// Port: produces a Scenario (measurements + truth) for a given seed.
// Replays ignore the seed; synthetics use it for noise realisation.
class ScenarioRun {
 public:
  virtual ~ScenarioRun() = default;
  virtual ScenarioDescriptor descriptor() const = 0;
  virtual Scenario generate(std::uint64_t seed) = 0;
};

}  // namespace benchmark
}  // namespace navtracker
