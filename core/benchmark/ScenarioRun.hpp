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

  // Spatial clutter density (false alarms per m² per scan) for THIS
  // scenario's environment. Used by the MHT tracker's branch score
  // (log P_D + logLik − log λ_C) so confirmation/deletion match the
  // actual clutter. Synthetic scenarios are effectively clutter-free
  // (default 1e-4); real cluttered data (AutoFerry port radar/lidar)
  // declares a realistic value. This is a property of the sensor +
  // environment, not a tuning knob — a single global constant cannot
  // serve both clean synthetic and cluttered real scenes (verified:
  // the value that suppresses real false-tracks erases synthetic
  // tracks). Data-driven λ_C estimation is the adaptive follow-up.
  double clutter_density{1e-4};
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
