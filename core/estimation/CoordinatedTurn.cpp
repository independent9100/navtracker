#include "core/estimation/CoordinatedTurn.hpp"

#include <cmath>

namespace navtracker {

CoordinatedTurn::CoordinatedTurn(double accel_psd, double omega_psd)
    : q_a_(accel_psd), q_omega_(omega_psd) {}

Eigen::MatrixXd CoordinatedTurn::transitionMatrix(double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
  const double w = omega_;
  if (std::abs(w) < 1e-6) {
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
  }
  const double wdt = w * dt;
  const double s = std::sin(wdt);
  const double c = std::cos(wdt);
  F(0, 2) =  s / w;
  F(0, 3) = -(1.0 - c) / w;
  F(1, 2) =  (1.0 - c) / w;
  F(1, 3) =  s / w;
  F(2, 2) =  c;
  F(2, 3) = -s;
  F(3, 2) =  s;
  F(3, 3) =  c;
  return F;
}

Eigen::MatrixXd CoordinatedTurn::processNoise(double dt) const {
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
