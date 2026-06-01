#pragma once

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "adapters/eoir/EoIrAdapter.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct EoIrTargetEntry {
  std::uint64_t truth_id{0};
  int sensor_track_id{0};
};

struct EoIrEmitterConfig {
  enum class RangeMode { BearingOnly, BearingAndRange };
  std::vector<EoIrTargetEntry> targets;
  double dt_s{0.1};
  double fov_deg{60.0};
  double boresight_relative_deg{0.0};
  double max_range_m{5000.0};
  RangeMode range_mode{RangeMode::BearingAndRange};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  double bearing_only_range_std_m{1000.0};
};

class EoIrEmitter final : public ISensorEmitter {
 public:
  EoIrEmitter(EoIrAdapter& adapter,
              EoIrEmitterConfig cfg,
              std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  EoIrAdapter& adapter_;
  EoIrEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> bearing_noise_;
  std::normal_distribution<double> range_noise_;
  Timestamp next_emit_{};
  bool initialised_{false};
};

}  // namespace navtracker::sim
