#include "core/bias/SensorBiasPairExtractor.hpp"

#include <cmath>
#include <cstdint>

#include "core/types/Ids.hpp"

namespace navtracker {
namespace {

bool isAnchorKind(SensorKind k) { return k == SensorKind::Ais; }

bool isPositionalNonAnchor(SensorKind k) {
  return k == SensorKind::ArpaTtm || k == SensorKind::ArpaTll ||
         k == SensorKind::Lidar;
}

bool isBearingNonAnchor(SensorKind k) { return k == SensorKind::EoIr; }

Eigen::Matrix2d covWithFallback(const Eigen::Matrix2d& c, double fallback_std) {
  const double tr = c.trace();
  if (tr > 0.0) return c;
  Eigen::Matrix2d out = Eigen::Matrix2d::Zero();
  const double v = fallback_std * fallback_std;
  out(0, 0) = v;
  out(1, 1) = v;
  return out;
}

}  // namespace

std::vector<PositionBiasPairObservation> extractPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg) {
  std::vector<PositionBiasPairObservation> out;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(cfg.cycle_window_seconds * 1e9);
  for (const Track& tr : tracks) {
    const Track::SourceTouch* anchor = nullptr;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (isAnchorKind(t.sensor)) {
        anchor = &t;
        break;
      }
    }
    if (anchor == nullptr) continue;

    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (!isPositionalNonAnchor(t.sensor)) continue;

      PositionBiasPairObservation obs;
      obs.time = cycle_time;
      obs.key.sensor = t.sensor;
      obs.key.source_id = t.source_id;
      obs.z_sensor_enu = t.value_enu;
      obs.z_anchor_enu = anchor->value_enu;
      obs.R_sensor =
          covWithFallback(t.covariance, cfg.sensor_position_std_fallback_m);
      obs.R_anchor =
          covWithFallback(anchor->covariance,
                          cfg.anchor_position_std_fallback_m);
      obs.own_position_enu = t.sensor_position_enu;
      out.push_back(std::move(obs));
    }
  }
  return out;
}

std::vector<BearingBiasPairObservation> extractBearingPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg) {
  std::vector<BearingBiasPairObservation> out;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(cfg.cycle_window_seconds * 1e9);
  for (const Track& tr : tracks) {
    const Track::SourceTouch* anchor = nullptr;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (isAnchorKind(t.sensor)) {
        anchor = &t;
        break;
      }
    }
    if (anchor == nullptr) continue;

    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (!isBearingNonAnchor(t.sensor)) continue;
      if (!std::isfinite(t.alpha_rad)) continue;  // not a bearing touch

      BearingBiasPairObservation obs;
      obs.time = cycle_time;
      obs.key.sensor = t.sensor;
      obs.key.source_id = t.source_id;
      obs.sensor_position_enu = t.sensor_position_enu;
      obs.anchor_target_position_enu = anchor->value_enu;
      obs.alpha_observed_rad = t.alpha_rad;
      obs.alpha_meas_var_rad2 = t.alpha_var_rad2;
      // Project the anchor's position uncertainty onto the
      // bearing axis at the camera-to-anchor range; this drives
      // the estimator's R_anchor for the bearing update.
      const double tr_anchor =
          std::sqrt(anchor->covariance.trace() * 0.5);
      obs.anchor_position_std_m =
          tr_anchor > 0.0 ? tr_anchor
                          : cfg.anchor_position_std_fallback_m;
      out.push_back(std::move(obs));
    }
  }
  return out;
}

}  // namespace navtracker
