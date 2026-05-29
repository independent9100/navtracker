#pragma once

#include <Eigen/Core>

namespace navtracker {

// Linear-Gaussian motion model for a fixed state layout: supplies the state
// transition matrix F(dt) and process-noise covariance Q(dt).
class IMotionModel {
 public:
  virtual ~IMotionModel() = default;
  virtual int stateDim() const = 0;
  virtual Eigen::MatrixXd transitionMatrix(double dt) const = 0;  // F(dt)
  virtual Eigen::MatrixXd processNoise(double dt) const = 0;      // Q(dt)
};

}  // namespace navtracker
