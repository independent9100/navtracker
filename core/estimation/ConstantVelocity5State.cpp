#include "core/estimation/ConstantVelocity5State.hpp"

namespace navtracker {

ConstantVelocity5State::ConstantVelocity5State(double accel_psd, double omega_psd)
    : q_a_(accel_psd), q_omega_(omega_psd) {}

Eigen::MatrixXd ConstantVelocity5State::transitionMatrix(double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

Eigen::MatrixXd ConstantVelocity5State::processNoise(double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double a = q_a_ * dt3 / 3.0;
  const double b = q_a_ * dt2 / 2.0;
  const double c = q_a_ * dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
  Q(0, 0) = a; Q(0, 2) = b;
  Q(1, 1) = a; Q(1, 3) = b;
  Q(2, 0) = b; Q(2, 2) = c;
  Q(3, 1) = b; Q(3, 3) = c;
  Q(4, 4) = q_omega_ * dt;
  return Q;
}

}  // namespace navtracker
