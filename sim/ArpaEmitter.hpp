#pragma once

#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct ArpaTargetEntry {
  std::uint64_t truth_id{0};
  int arpa_track_num{0};  // NMEA $RATTM target number; conventionally 0-99.
};

struct ArpaEmitterConfig {
  std::vector<ArpaTargetEntry> targets;
  double rotation_dt_s{3.0};
  double range_std_m{50.0};
  double bearing_std_deg{1.0};
  double min_range_m{50.0};
  double max_range_m{22224.0};  // 12 NM
  int clutter_per_rotation{0};       // Poisson mean N false alarms per rotation
  double clutter_min_range_m{50.0};  // clutter drawn uniformly in [min, max_range_m]
};

class ArpaEmitter final : public ISensorEmitter {
 public:
  ArpaEmitter(ArpaAdapter& adapter,
              const geo::Datum& datum,
              ArpaEmitterConfig cfg,
              std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  ArpaAdapter& adapter_;
  const geo::Datum& datum_;
  ArpaEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> range_noise_;
  std::normal_distribution<double> bearing_noise_;
  std::unordered_map<std::uint64_t, Timestamp> next_emit_;
  bool initialised_{false};
  Timestamp next_clutter_emit_{};
  bool clutter_initialised_{false};
  std::poisson_distribution<int> clutter_count_dist_;
  std::uniform_real_distribution<double> clutter_range_dist_;
  std::uniform_real_distribution<double> clutter_bearing_dist_;
};

}  // namespace navtracker::sim
