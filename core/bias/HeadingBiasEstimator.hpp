#pragma once

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"
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

class HeadingBiasEstimator : public IHeadingBiasProvider {
 public:
  explicit HeadingBiasEstimator(HeadingBiasEstimatorConfig cfg = {});

  // Time-only predict step. Safe to call repeatedly; idempotent if
  // `to` does not advance.
  void predictTo(Timestamp to);

  // Apply one AIS+ARPA pair observation. Internally calls predictTo
  // first, then performs a scalar KF update.
  void observe(const AisArpaPairObservation& obs);

  // IHeadingBiasProvider
  HeadingBiasEstimate current() const override;

  // Diagnostics (not part of IHeadingBiasProvider).
  double biasRad() const { return b_hat_; }
  double varianceRad2() const { return p_b_; }
  Timestamp lastUpdateTime() const { return last_update_; }

 private:
  HeadingBiasEstimatorConfig cfg_;
  double b_hat_;
  double p_b_;
  Timestamp last_predict_{};
  Timestamp last_update_{};
  bool has_any_update_{false};
};

}  // namespace navtracker
