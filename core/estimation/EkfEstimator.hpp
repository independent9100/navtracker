#pragma once

#include <memory>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Extended Kalman Filter. Linear-Gaussian prediction via the supplied motion
// model; nonlinear measurement updates via Jacobian linearization.
class EkfEstimator : public IEstimator {
 public:
  EkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

  // Create a new Tentative track seeded from a position-type measurement.
  Track initiate(const Measurement& z) const override;

 private:
  std::shared_ptr<const IMotionModel> motion_;
  double init_speed_std_;
};

}  // namespace navtracker
