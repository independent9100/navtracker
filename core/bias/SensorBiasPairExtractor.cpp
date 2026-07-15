#include "core/bias/SensorBiasPairExtractor.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>

#include "core/types/Ids.hpp"

namespace navtracker {
namespace {

bool isAnchorKind(SensorKind k) {
  return k == SensorKind::Ais || k == SensorKind::Cooperative;
}

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
      // W3.2: reconstruct the RAW sensor measurement by adding back any
      // published position bias the pipeline already subtracted
      // (applyBiasCorrection), so the estimator's r = z_sensor − z_anchor − b̂
      // measures the full bias and does not subtract b̂ a second time in the
      // closed loop.
      obs.z_sensor_enu = t.value_enu + t.applied_position_bias_enu;
      // The anchor is the TRUTH reference: use its CORRECTED position directly
      // (do NOT add the anchor's applied bias back — that would restore the raw,
      // biased position and drive the sensor estimate to b_sensor − b_anchor).
      // Anchor kinds are never fed to any per-sensor estimator so they never
      // publish a position bias (applied stays 0), making this byte-identical to
      // the raw anchor today; using value_enu also closes that latent hole if an
      // anchor is ever position-calibrated.
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
      // W3.2: reconstruct the RAW camera bearing by adding back the bearing
      // bias the pipeline already applied, so the closed loop does not
      // double-subtract b̂. The anchor is the TRUTH reference — use its CORRECTED
      // position directly (no add-back; see the positional path above for why).
      obs.anchor_target_position_enu = anchor->value_enu;
      obs.alpha_observed_rad = t.alpha_rad + t.applied_bearing_bias_rad;
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
    //
    // std::map (not unordered_map) so the emission order is the keys'
    // operator< order — deterministic across STL implementations. The
    // sequential KF folds observations in this order; the outlier gate
    // (which branches on running state) makes the result order-sensitive
    // at the margins, and CLAUDE.md invariant #4 requires deterministic
    // replay. The sibling extractors already iterate a vector in order.
    std::map<SensorBiasKey, const Track::SourceTouch*> latest;
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
    // Need two distinct *source_ids* to pair, not just two keys: the
    // same-source_id guard below (TTM+TLL on one radar share a source_id)
    // rejects every pair from a single hardware source, so a track whose
    // keys all share one source_id would pass a `latest.size() < 2` gate
    // yet emit nothing. Count distinct source_ids to skip that wasted work.
    std::set<std::string> distinct_sources;
    for (const auto& kv : latest) distinct_sources.insert(kv.first.source_id);
    if (distinct_sources.size() < 2) continue;

    // One observation per calibrated key per cycle. The sensor's own
    // contribution `x` is a *single* measurement; pairing it against
    // every other key and folding each as an independent KF update
    // would replay the same sample N−1 times and over-shrink the bias
    // covariance (the N−1 residuals all share x's noise — they are
    // correlated, not independent). Instead each key X is calibrated
    // against its single most-trusted partner Y — the one with the
    // smallest effective anchor covariance trace after the Schmidt
    // fold. For N=2 this reduces to the original symmetric pair (X←Y
    // and Y←X); for N≥3 every sample is used exactly once.
    for (auto it_x = latest.begin(); it_x != latest.end(); ++it_x) {
      const SensorBiasKey& key_x = it_x->first;
      const Track::SourceTouch& x = *it_x->second;

      // Select the best anchor Y ≠ X. "Best" = smallest trace of
      // R_anchor = R_Y_measurement + P_b_Y, i.e. the reference we
      // trust most. The Schmidt fold subtracts Y's current bias
      // estimate from the anchor measurement and inflates R_anchor by
      // Y's bias covariance; when the provider is null or Y has not
      // published, b̂_Y = 0 and P_b_Y = 0 — a naive pair, which is
      // what we want at cold start.
      const Track::SourceTouch* best_y = nullptr;
      Eigen::Vector2d best_b_anchor = Eigen::Vector2d::Zero();
      Eigen::Matrix2d best_R_anchor = Eigen::Matrix2d::Zero();
      double best_trace = 0.0;
      bool best_anchor_published = false;
      for (auto it_y = latest.begin(); it_y != latest.end(); ++it_y) {
        if (it_y == it_x) continue;  // no self-anchoring
        const SensorBiasKey& key_y = it_y->first;
        // No anchoring across the *same physical sensor*. ARPA TTM and
        // TLL share a source_id but are distinct SensorKinds; they are
        // the same hardware and share one mounting/registration bias.
        // Pairing them gives r ≈ noise no matter the true common
        // offset, which would mask it. Cross-sensor calibration is only
        // observable in *relative* bias between independent sensors;
        // any common-mode component is unobservable and pinned to zero
        // by the estimator's prior, so we must not let a shared-hardware
        // pair pretend to measure it.
        if (key_y.source_id == key_x.source_id) continue;
        const Track::SourceTouch& y = *it_y->second;

        Eigen::Vector2d b_anchor = Eigen::Vector2d::Zero();
        Eigen::Matrix2d P_anchor = Eigen::Matrix2d::Zero();
        bool y_published = false;
        if (bias_provider != nullptr) {
          const auto est_y = bias_provider->positionBias(key_y);
          if (est_y.is_published) {
            b_anchor = est_y.bias_enu_m;
            P_anchor = est_y.covariance_m2;
            y_published = true;
          }
        }
        const Eigen::Matrix2d R_anchor =
            covWithFallback(y.covariance, cfg.sensor_position_std_fallback_m) +
            P_anchor;
        const double tr_anchor = R_anchor.trace();
        if (best_y == nullptr || tr_anchor < best_trace) {
          best_y = &y;
          best_b_anchor = b_anchor;
          best_R_anchor = R_anchor;
          best_trace = tr_anchor;
          best_anchor_published = y_published;
        }
      }
      if (best_y == nullptr) continue;  // no eligible partner

      PositionBiasPairObservation obs;
      obs.time = cycle_time;
      obs.key = key_x;
      // W3.2 / cross-sensor Schmidt fold: reconstruct the RAW touches (add
      // back any bias applyBiasCorrection already removed) BEFORE subtracting
      // the partner's published bias. Otherwise the anchor is debiased twice —
      // once by the pipeline, once by best_b_anchor here.
      obs.z_sensor_enu = x.value_enu + x.applied_position_bias_enu;
      obs.z_anchor_enu =
          best_y->value_enu + best_y->applied_position_bias_enu - best_b_anchor;
      obs.R_sensor =
          covWithFallback(x.covariance, cfg.sensor_position_std_fallback_m);
      obs.R_anchor = best_R_anchor;
      obs.own_position_enu = x.sensor_position_enu;
      // A cross-sensor pair "inherits" anchor status from its partner:
      // when the partner's own bias has already been published (seeded
      // with treat_as_anchored=true, or refined via AIS-anchored pairs),
      // the partner's contribution is debiased and behaves as an
      // absolute-position anchor for X. When the partner is itself
      // unpublished, the pair is a symmetric coordinate-descent step on
      // a relative offset and must NOT, on its own, let X's estimate
      // flow back into applyBiasCorrection (sc2/13/22 unanchored NEES
      // diagnosis, 2026-06-19).
      obs.is_anchor_source = best_anchor_published;
      out.push_back(std::move(obs));
    }
  }
  return out;
}

}  // namespace navtracker
