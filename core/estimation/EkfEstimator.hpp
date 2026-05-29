#pragma once

#include <memory>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Extended Kalman Filter. Linear-Gaussian prediction via the supplied motion
// model; nonlinear measurement updates via Jacobian linearization.
class EkfEstimator : public IEstimator {
 public:
  explicit EkfEstimator(std::shared_ptr<const IMotionModel> motion);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

 private:
  std::shared_ptr<const IMotionModel> motion_;
};

}  // namespace navtracker
