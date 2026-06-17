#include "core/bias/SensorBiasPairExtractor.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_map>

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

std::vector<PositionBiasPairObservation> extractCrossSensorPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    const ISensorBiasProvider* bias_provider,
    SensorBiasPairExtractorConfig cfg,
    CrossSensorEligibilityConfig gates) {
  std::vector<PositionBiasPairObservation> out;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(cfg.cycle_window_seconds * 1e9);
  for (const Track& tr : tracks) {
    // Track-quality gates first — cheapest filter.
    if (tr.existence_probability < gates.min_existence_probability) continue;
    // Position covariance: state is [px, py, ...], position block is the
    // upper-left 2x2. Tracks without a populated covariance (rows < 2)
    // are too tentative to consider anyway.
    if (tr.covariance.rows() < 2 || tr.covariance.cols() < 2) continue;
    const double pos_cov_trace =
        tr.covariance(0, 0) + tr.covariance(1, 1);
    if (pos_cov_trace > gates.max_position_cov_trace_m2) continue;

    // Skip tracks with an AIS contribution in the cycle window — those
    // are strictly more informative through the AIS-anchored path.
    bool has_ais = false;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (isAnchorKind(t.sensor)) {
        has_ais = true;
        break;
      }
    }
    if (has_ais) continue;

    // Dedupe by SensorBiasKey: at most one contribution per key per
    // cycle (avoid double-anchoring through the same sensor twice).
    // When multiple are present we keep the most recent.
    std::unordered_map<SensorBiasKey, const Track::SourceTouch*> latest;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (!isPositionalNonAnchor(t.sensor)) continue;
      SensorBiasKey k{t.sensor, t.source_id};
      auto it = latest.find(k);
      if (it == latest.end() || it->second->time.nanos() < t.time.nanos()) {
        latest[k] = &t;
      }
    }
    if (latest.size() < 2) continue;  // need two distinct keys to pair

    // Ordered pairs over distinct keys: each direction emits one
    // observation. (X, Y) gives the bias estimator a measurement of
    // b_X anchored on (Y − b_Y); (Y, X) the mirror. The coordinate
    // descent across the two unknown biases needs both.
    for (auto it_x = latest.begin(); it_x != latest.end(); ++it_x) {
      for (auto it_y = latest.begin(); it_y != latest.end(); ++it_y) {
        if (it_x == it_y) continue;  // no self-anchoring

        const SensorBiasKey& key_x = it_x->first;
        const SensorBiasKey& key_y = it_y->first;
        const Track::SourceTouch& x = *it_x->second;
        const Track::SourceTouch& y = *it_y->second;

        // Schmidt-KF fold: subtract Y's current bias estimate from
        // the anchor measurement and inflate R_anchor by Y's
        // bias covariance. When provider is null or Y has not
        // published, b̂_Y = 0 and P_b_Y = 0 — this collapses to a
        // naive pair, which is what we want at cold start.
        Eigen::Vector2d b_anchor = Eigen::Vector2d::Zero();
        Eigen::Matrix2d P_anchor = Eigen::Matrix2d::Zero();
        if (bias_provider != nullptr) {
          const auto est_y = bias_provider->positionBias(key_y);
          if (est_y.is_published) {
            b_anchor = est_y.bias_enu_m;
            P_anchor = est_y.covariance_m2;
          }
        }

        PositionBiasPairObservation obs;
        obs.time = cycle_time;
        obs.key = key_x;
        obs.z_sensor_enu = x.value_enu;
        obs.z_anchor_enu = y.value_enu - b_anchor;
        obs.R_sensor =
            covWithFallback(x.covariance, cfg.sensor_position_std_fallback_m);
        // R_anchor = R_Y_measurement + P_b_Y. The first term is Y's
        // per-contribution noise; the second is Schmidt-KF residual
        // bias treatment so that uncertainty in our knowledge of b_Y
        // does not pretend to be measurement signal.
        obs.R_anchor =
            covWithFallback(y.covariance, cfg.sensor_position_std_fallback_m) +
            P_anchor;
        obs.own_position_enu = x.sensor_position_enu;
        out.push_back(std::move(obs));
      }
    }
  }
  return out;
}

}  // namespace navtracker
