#pragma once

#include <memory>
#include <vector>

#include "ports/IEstimator.hpp"
#include "ports/IMeasurementNoiseModel.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Extended Kalman Filter. Linear-Gaussian prediction via the supplied motion
// model; nonlinear measurement updates via Jacobian linearization.
//
// An optional IMeasurementNoiseModel robustifies the update: null (default)
// = ordinary Gaussian update; a StudentTNoiseModel down-weights outlier
// measurements. The strategy is injected so it can be swapped per
// deployment without touching the filter.
class EkfEstimator : public IEstimator {
 public:
  EkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0,
               std::shared_ptr<const IMeasurementNoiseModel> noise = nullptr,
               bool bearing_range_guard = false);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

  // Create a new Tentative track seeded from a position-type measurement.
  Track initiate(const Measurement& z) const override;

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
