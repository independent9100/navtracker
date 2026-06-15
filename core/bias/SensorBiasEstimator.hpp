#pragma once

#include <cstddef>
#include <unordered_map>

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"
#include "ports/ISensorBiasProvider.hpp"

namespace navtracker {

struct SensorBiasEstimatorConfig {
  // Initial 1-sigma prior on the bias (per axis, isotropic).
  double initial_pos_std_m{5.0};
  // Random-walk process noise on the bias position (m^2/s).
  double pos_process_noise_var_per_sec{(0.1 * 0.1) / 3600.0};

  // Initial 1-sigma prior on the bearing bias (rad).
  double initial_brg_std_rad{5.0 * 3.14159265358979323846 / 180.0};
  // Random-walk process noise on the bearing bias (rad^2/s).
  double brg_process_noise_var_per_sec{
      (0.05 * 0.05) / 3600.0};

  // Publish (deterministic-apply) thresholds.
  double publish_pos_var_threshold_m2{1.0 * 1.0};   // ≤1 m 1-sigma per axis
  double publish_brg_var_threshold_rad2{
      (0.3 * 3.14159265358979323846 / 180.0)
      * (0.3 * 3.14159265358979323846 / 180.0)};

  // Observation gates (spec §3, mirrors HeadingBiasEstimator G1-G2-G3).
  double max_time_skew_seconds{1.0};
  double min_range_m{50.0};
  double outlier_sigma{5.0};
};

// One paired observation for a position-bias estimator. z_sensor and
// z_anchor are both target-position measurements in the common ENU
// frame (z_anchor from AIS, z_sensor from the biased radar/lidar).
// Their disagreement is a direct 2D measurement of b. Range gate
// uses target-relative range from own_position_enu.
struct PositionBiasPairObservation {
  Timestamp time;
  SensorBiasKey key;
  Eigen::Vector2d z_sensor_enu{Eigen::Vector2d::Zero()};
  Eigen::Vector2d z_anchor_enu{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d R_sensor{Eigen::Matrix2d::Zero()};
  Eigen::Matrix2d R_anchor{Eigen::Matrix2d::Zero()};
  // For the range gate (target range from own-ship).
  Eigen::Vector2d own_position_enu{Eigen::Vector2d::Zero()};
};

// One paired observation for a bearing-bias estimator. anchor_pos_enu
// is the anchor-reported target position; sensor_pos_enu is the
// camera's mounting position; alpha_observed is the bearing the
// camera reported. The bias b enters as α_predicted = atan2(anchor−
// sensor) + b, so r = wrap(α_obs − α_predicted) is a scalar
// measurement of b.
struct BearingBiasPairObservation {
  Timestamp time;
  SensorBiasKey key;
  Eigen::Vector2d sensor_position_enu{Eigen::Vector2d::Zero()};
  Eigen::Vector2d anchor_target_position_enu{Eigen::Vector2d::Zero()};
  double alpha_observed_rad{0.0};
  double alpha_meas_var_rad2{0.0};   // σ²_α from the sensor
  // 1-sigma isotropic position noise on the anchor report (m).
  double anchor_position_std_m{10.0};
};

// Per-(sensor, source_id) bias estimator. Each key has its own KF;
// position and bearing biases are independent state vectors. Behaves
// as ISensorBiasProvider once any observation has updated a given key.
class SensorBiasEstimator : public ISensorBiasProvider {
 public:
  explicit SensorBiasEstimator(SensorBiasEstimatorConfig cfg = {});

  // Idempotent if `to` does not advance any key.
  void predictTo(Timestamp to);

  // Observation entry points. Each calls predictTo for the keyed entry,
  // applies the three gates, and (if accepted) updates the bias state.
  void observe(const PositionBiasPairObservation& obs);
  void observe(const BearingBiasPairObservation& obs);

  // Seed a per-key prior from external knowledge (calibration
  // documentation, factory survey, plan drawings). The bias is
  // immediately treated as "has_update == true", so it publishes
  // through ISensorBiasProvider as soon as the covariance is below
  // the publish threshold. Subsequent observations continue to
  // refine it via the KF update path; the entry behaves identically
  // to one built from observations once seeded.
  void setKnownPositionBias(const SensorBiasKey& key,
                            const Eigen::Vector2d& bias_enu_m,
                            const Eigen::Matrix2d& covariance_m2);
  void setKnownBearingBias(const SensorBiasKey& key, double bias_rad,
                           double variance_rad2);

  // ISensorBiasProvider
  PositionBiasEstimate positionBias(const SensorBiasKey& key) const override;
  BearingBiasEstimate bearingBias(const SensorBiasKey& key) const override;

  // Diagnostics.
  std::size_t acceptedPosObs() const { return acc_pos_; }
  std::size_t acceptedBrgObs() const { return acc_brg_; }
  std::size_t rejectedByTime() const { return rej_time_; }
  std::size_t rejectedByRange() const { return rej_range_; }
  std::size_t rejectedByOutlier() const { return rej_outlier_; }

 private:
  struct PositionState {
    Eigen::Vector2d b_hat{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d P{Eigen::Matrix2d::Zero()};
    Timestamp last_predict{};
    bool has_update{false};
  };
  struct BearingState {
    double b_hat{0.0};
    double P{0.0};
    Timestamp last_predict{};
    bool has_update{false};
  };

  PositionState& posStateFor(const SensorBiasKey& key);
  BearingState& brgStateFor(const SensorBiasKey& key);

  SensorBiasEstimatorConfig cfg_;
  std::unordered_map<SensorBiasKey, PositionState> pos_;
  std::unordered_map<SensorBiasKey, BearingState> brg_;
  std::size_t acc_pos_{0};
  std::size_t acc_brg_{0};
  std::size_t rej_time_{0};
  std::size_t rej_range_{0};
  std::size_t rej_outlier_{0};
};

}  // namespace navtracker
