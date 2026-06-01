#pragma once

#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct AisTargetEntry {
  std::uint64_t truth_id{0};
  std::uint32_t mmsi{0};
  bool high_accuracy{true};
};

struct AisEmitterConfig {
  std::vector<AisTargetEntry> targets;
  double pos_std_m{10.0};
  // Each pair [start_s, end_s) measured relative to the first emit() call
  // (which is treated as t=0 for cadence/dropout bookkeeping).
  std::vector<std::pair<double, double>> dropout_windows_s;
};

class AisEmitter final : public ISensorEmitter {
 public:
  AisEmitter(AisAdapter& adapter,
             const geo::Datum& datum,
             AisEmitterConfig cfg,
             std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  static double cadenceSeconds(double speed_mps);
  bool inDropout(double t_relative_s) const;

  AisAdapter& adapter_;
  const geo::Datum& datum_;
  AisEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::unordered_map<std::uint64_t, Timestamp> next_emit_;
  bool initialised_{false};
  Timestamp t0_{};
};

}  // namespace navtracker::sim
