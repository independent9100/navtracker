#pragma once

#include <memory>
#include <vector>

#include "ports/IEstimator.hpp"
#include "ports/IMeasurementNoiseModel.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

/**
 * Extended Kalman Filter. Linear-Gaussian prediction via the supplied motion
 * model; nonlinear measurement updates via Jacobian linearization.
 *
 * An optional IMeasurementNoiseModel robustifies the update: null (default)
 * = ordinary Gaussian update; a StudentTNoiseModel down-weights outlier
 * measurements. The strategy is injected so it can be swapped per
 * deployment without touching the filter.
 */
class EkfEstimator : public IEstimator {
 public:
  /**
   * Wire the filter with its `motion` model. `init_speed_std` seeds velocity
   * uncertainty on initiate; the optional `noise` model robustifies updates;
   * `bearing_range_guard` enables the along-LOS variance non-decrease clamp
   * for bearing-only updates (see `applyBearingRangeGuard`).
   */
  EkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0,
               std::shared_ptr<const IMeasurementNoiseModel> noise = nullptr,
               bool bearing_range_guard = false);

  /** Advance the track's state and covariance to time `to`. */
  void predict(Track& track, Timestamp to) const override;
  /** Fold measurement `z` into the track via Jacobian-linearized update. */
  void update(Track& track, const Measurement& z) const override;

  /** Create a new Tentative track seeded from a position-type measurement. */
  Track initiate(const Measurement& z) const override;

  /**
   * PDAF/JPDA soft update: fold a gated cluster of measurements weighted by
   * association probabilities `betas` (and miss probability `beta_0`) into
   * the track. `ctx` carries the associator parameters (see PdaContext).
   */
  void softUpdate(Track& track,
                  const std::vector<Measurement>& gated_measurements,
                  const Eigen::VectorXd& betas,
                  double beta_0,
                  const PdaContext& ctx = {}) const override;

 private:
  std::shared_ptr<const IMotionModel> motion_;
  double init_speed_std_;
  std::shared_ptr<const IMeasurementNoiseModel> noise_;
  bool bearing_range_guard_;
};

}  // namespace navtracker
