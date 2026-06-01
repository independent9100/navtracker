#include "sim/EoIrEmitter.hpp"

#include <cmath>
#include <unordered_map>
#include <utility>

namespace navtracker::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

double wrapSignedDeg(double deg) {
  double w = std::fmod(deg + 180.0, 360.0);
  if (w < 0.0) w += 360.0;
  return w - 180.0;
}

}  // namespace

EoIrEmitter::EoIrEmitter(EoIrAdapter& adapter,
                         EoIrEmitterConfig cfg,
                         std::uint32_t seed)
    : adapter_(adapter),
      cfg_(std::move(cfg)),
      rng_(seed),
      bearing_noise_(0.0, cfg_.bearing_std_deg > 0.0 ? cfg_.bearing_std_deg : 1.0),
      range_noise_(0.0, cfg_.range_std_m > 0.0 ? cfg_.range_std_m : 1.0) {}

void EoIrEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    next_emit_ = ctx.now;
    initialised_ = true;
  }

  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  const Eigen::Vector2d own = ctx.ownship_truth.position;

  while (next_emit_ <= ctx.now) {
    for (const auto& te : cfg_.targets) {
      auto it = truths.find(te.truth_id);
      if (it == truths.end()) continue;
      const Eigen::Vector2d dxy = it->second.position - own;
      const double range = dxy.norm();
      if (range > cfg_.max_range_m) continue;

      // Compass convention (matches ArpaEmitter): 0° = +y (north), CW.
      // Compass bearing = atan2(east, north) = atan2(dxy.x, dxy.y).
      // EoIrAdapter rotates the relative bearing by own-ship heading to
      // recover true bearing; v1 deploys with heading=0 (§14.9 deferral),
      // so relative == compass-from-north here.
      const double bearing_math_deg = std::atan2(dxy.x(), dxy.y()) * kRad2Deg;
      const double bearing_rel_deg = bearing_math_deg;
      const double half_fov = cfg_.fov_deg * 0.5;
      const double delta = wrapSignedDeg(bearing_rel_deg - cfg_.boresight_relative_deg);
      if (std::fabs(delta) > half_fov) continue;

      const double b_obs = bearing_rel_deg +
          (cfg_.bearing_std_deg > 0.0 ? bearing_noise_(rng_) : 0.0);

      CameraDetection d;
      d.time = next_emit_;
      d.bearing_relative_deg = b_obs;
      d.bearing_std_deg = cfg_.bearing_std_deg > 0.0 ? cfg_.bearing_std_deg : 0.1;
      if (cfg_.range_mode == EoIrEmitterConfig::RangeMode::BearingAndRange) {
        d.range_m = range +
            (cfg_.range_std_m > 0.0 ? range_noise_(rng_) : 0.0);
        d.range_std_m = cfg_.range_std_m > 0.0 ? cfg_.range_std_m : 1.0;
      } else {
        d.range_m = range;
        d.range_std_m = cfg_.bearing_only_range_std_m;
      }
      d.sensor_track_id = te.sensor_track_id;
      adapter_.ingest(d);
    }
    next_emit_ = Timestamp::fromSeconds(next_emit_.seconds() + cfg_.dt_s);
  }
}

}  // namespace navtracker::sim
