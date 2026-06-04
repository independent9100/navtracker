#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "core/bias/HeadingBiasObservations.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IBearingInnovationSink.hpp"
#include "ports/IHeadingBiasProvider.hpp"

namespace navtracker {

struct HeadingBiasEstimatorConfig {
  // Initial estimate and variance (rad, rad^2).
  double initial_bias_rad{0.0};
  double initial_variance_rad2{(5.0 * 3.14159265358979323846 / 180.0)
                               * (5.0 * 3.14159265358979323846 / 180.0)};
  // Random-walk process noise (rad^2/s).
  double process_noise_var_per_sec{9.4e-11};  // ~2 deg/hr 1-sigma
  // Gating thresholds.
  double publish_variance_threshold_rad2{(0.3 * 3.14159265358979323846 / 180.0)
                                         * (0.3 * 3.14159265358979323846 / 180.0)};
  double stale_seconds{30.0};

  // Bearing-innovation observation gates (spec §4).
  double bi_min_range_m{50.0};        // G1
  double bi_state_var_ratio_max{1.0}; // G2: predicted_state_var <= k * R
  double bi_outlier_sigma{5.0};       // G3: |r| <= N * sqrt(S + P_b)

  // Multi-heading-source path (v3, spec 2026-06-04).
  double cog_min_sog_mps{3.0};
  double cog_max_gyro_rate_rad_per_s{0.5 * 3.14159265358979323846 / 180.0};
  double cog_crab_budget_rad{5.0 * 3.14159265358979323846 / 180.0};
  double mag_deviation_budget_rad{3.0 * 3.14159265358979323846 / 180.0};
  double mhs_outlier_sigma{5.0};
};

// One pair observation: AIS-derived target position and ARPA-derived
// bearing-projected target position relative to the same own-ship
// origin at the same cycle. The estimator computes the angular
// disagreement and uses it as a scalar measurement of b.
struct AisArpaPairObservation {
  Timestamp time;
  Eigen::Vector2d own_position_enu;
  Eigen::Vector2d ais_target_position_enu;
  Eigen::Vector2d arpa_target_position_enu;
  // 1-sigma bearing noise contributed by the ARPA measurement, rad.
  double arpa_bearing_std_rad{1.0 * 3.14159265358979323846 / 180.0};
  // 1-sigma isotropic position noise on the AIS report, m.
  double ais_position_std_m{10.0};
  // 1-sigma own-ship GPS position noise at observation time, m.
  double own_position_std_m{0.0};
};

// === v2 observation kind (bearing innovations from Tracker) ===
//
// The estimator also implements IBearingInnovationSink. When wired into a
// Tracker, every successful hard-match update on Bearing2D or
// RangeBearing2D produces a scalar measurement
//   r ~ N(b, S),   S = H·P·Hᵀ + R
// of the heading bias b, where (β̂, H, P) come from the PRE-update
// predicted track state and R is the measurement noise. Sequentially
// fusing innovations from independent tracks is mathematically correct
// because the projected state errors are conditionally independent given
// b (independent CV processes, independent associations).
//
// Three observability gates protect against state-error contamination
// (predicted_state_var <= k * R), short range (range >= min_range_m),
// and outliers (|r| <= N * sqrt(S + P_b)). See spec
// 2026-06-04-multi-track-bearing-bias-observer-design.md for defaults.
class HeadingBiasEstimator : public IHeadingBiasProvider,
                             public IBearingInnovationSink {
 public:
  explicit HeadingBiasEstimator(HeadingBiasEstimatorConfig cfg = {});

  // Time-only predict step. Safe to call repeatedly; idempotent if
  // `to` does not advance.
  void predictTo(Timestamp to);

  // Apply one AIS+ARPA pair observation. Internally calls predictTo
  // first, then performs a scalar KF update.
  void observe(const AisArpaPairObservation& obs);

  // Apply one bearing-domain innovation produced by the Tracker.
  // Predicts to obs.time, then applies the three observability gates
  // (range / state-variance-dominance / outlier). If all gates pass,
  // performs a scalar KF update on b with measurement r ~ N(b, S).
  void observe(const BearingInnovation& obs);

  // IHeadingBiasProvider
  HeadingBiasEstimate current() const override;

  // IBearingInnovationSink — dispatches to observe(BearingInnovation).
  void onBearingInnovation(const BearingInnovation& obs) override {
    observe(obs);
  }

  // Multi-heading-source observation kinds. Each is a scalar KF update
  // on the same bias state b. See spec 2026-06-04-multi-heading-sources
  // for math, gates, defaults.
  void observe(const GyroVsGpsHeadingObservation& obs);
  void observe(const GyroVsGpsCogObservation& obs);
  void observe(const GyroVsMagneticObservation& obs);

  // Diagnostics (not part of IHeadingBiasProvider).
  double biasRad() const { return b_hat_; }
  double varianceRad2() const { return p_b_; }
  Timestamp lastUpdateTime() const { return last_update_; }
  std::size_t acceptedBearingObs() const { return accepted_bi_; }
  std::size_t rejectedByRange() const { return rej_range_; }
  std::size_t rejectedByStateVar() const { return rej_state_var_; }
  std::size_t rejectedByOutlier() const { return rej_outlier_; }
  std::size_t acceptedGpsHeading() const { return acc_gps_hdg_; }
  std::size_t acceptedGpsCog() const { return acc_cog_; }
  std::size_t acceptedMagnetic() const { return acc_mag_; }
  std::size_t rejectedCogBySog() const { return rej_cog_sog_; }
  std::size_t rejectedCogByGyroRate() const { return rej_cog_rate_; }
  std::size_t rejectedMhsByOutlier() const { return rej_mhs_outlier_; }

 private:
  HeadingBiasEstimatorConfig cfg_;
  double b_hat_;
  double p_b_;
  Timestamp last_predict_{};
  Timestamp last_update_{};
  bool has_any_update_{false};
  std::size_t accepted_bi_{0};
  std::size_t rej_range_{0};
  std::size_t rej_state_var_{0};
  std::size_t rej_outlier_{0};
  std::size_t acc_gps_hdg_{0};
  std::size_t acc_cog_{0};
  std::size_t acc_mag_{0};
  std::size_t rej_cog_sog_{0};
  std::size_t rej_cog_rate_{0};
  std::size_t rej_mhs_outlier_{0};
};

}  // namespace navtracker
