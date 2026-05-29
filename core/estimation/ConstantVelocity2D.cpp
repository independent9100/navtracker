#include "core/estimation/ConstantVelocity2D.hpp"

namespace navtracker {

ConstantVelocity2D::ConstantVelocity2D(double accel_psd) : q_(accel_psd) {}

Eigen::MatrixXd ConstantVelocity2D::transitionMatrix(double dt) const {
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(4, 4);
  f(0, 2) = dt;
  f(1, 3) = dt;
  return f;
}

Eigen::MatrixXd ConstantVelocity2D::processNoise(double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double a = q_ * dt3 / 3.0;
  const double b = q_ * dt2 / 2.0;
  const double c = q_ * dt;
  Eigen::MatrixXd qm = Eigen::MatrixXd::Zero(4, 4);
  qm(0, 0) = a; qm(0, 2) = b;
  qm(1, 1) = a; qm(1, 3) = b;
  qm(2, 0) = b; qm(2, 2) = c;
  qm(3, 1) = b; qm(3, 3) = c;
  return qm;
}

}  // namespace navtracker
