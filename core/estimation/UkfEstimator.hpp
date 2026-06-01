#pragma once

#include <memory>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Unscented Kalman Filter behind IEstimator. Predict uses the supplied
// motion model linearly (F(dt)·x for each sigma point); update propagates
// sigma points through h(x) and reconstructs mean / covariance via the
// unscented weighted sums.
class UkfEstimator : public IEstimator {
 public:
  UkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0,
               double alpha = 1e-3,
               double beta = 2.0,
               double kappa = 0.0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;
  Track initiate(const Measurement& z) const override;

 private:
  std::shared_ptr<const IMotionModel> motion_;
  double init_speed_std_;
  double alpha_;
  double beta_;
  double kappa_;
};

}  // namespace navtracker
