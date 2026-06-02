#include "sim/TruthTrajectory.hpp"

#include <cassert>
#include <cmath>

namespace navtracker::sim {

ConstantVelocityTrajectory::ConstantVelocityTrajectory(Eigen::Vector2d p0,
                                                       Eigen::Vector2d v,
                                                       Timestamp t0)
    : p0_(std::move(p0)), v_(std::move(v)), t0_(t0) {}

TruthState ConstantVelocityTrajectory::eval(Timestamp t) const {
  const double dt = t.secondsSince(t0_);
  return TruthState{p0_ + v_ * dt, v_};
}

ManeuveringTrajectory::ManeuveringTrajectory(Eigen::Vector2d start,
                                             Eigen::Vector2d velocity,
                                             double straight_duration_s,
                                             double turn_duration_s,
                                             double omega_rad_s,
                                             Timestamp t0)
    : start_(std::move(start)),
      v0_(std::move(velocity)),
      t_straight_(straight_duration_s),
      t_turn_(turn_duration_s),
      omega_(omega_rad_s),
      t0_(t0) {
  assert(omega_ != 0.0 && "ManeuveringTrajectory omega_rad_s must be non-zero; "
                         "use ConstantVelocityTrajectory for zero turn rate.");
}

TruthState ManeuveringTrajectory::eval(Timestamp t) const {
  double tau = t.secondsSince(t0_);
  if (tau < 0.0) tau = 0.0;

  // Leg 1: straight.
  if (tau <= t_straight_) {
    return TruthState{start_ + v0_ * tau, v0_};
  }

  // Leg 1 end-state.
  const Eigen::Vector2d p1 = start_ + v0_ * t_straight_;

  // Leg 2: constant-rate turn.
  if (tau <= t_straight_ + t_turn_) {
    const double tau2 = tau - t_straight_;
    const double speed = v0_.norm();
    const double theta0 = std::atan2(v0_.y(), v0_.x());
    const double theta = theta0 + omega_ * tau2;
    // Position by integrating velocity vector with constant turn rate:
    //   x(t) = x0 + (speed/omega) * (sin(theta) - sin(theta0))
    //   y(t) = y0 - (speed/omega) * (cos(theta) - cos(theta0))
    // (works for omega != 0; the test never hits omega==0).
    const Eigen::Vector2d p_turn(
        p1.x() + (speed / omega_) * (std::sin(theta) - std::sin(theta0)),
        p1.y() - (speed / omega_) * (std::cos(theta) - std::cos(theta0)));
    const Eigen::Vector2d v_turn(speed * std::cos(theta),
                                 speed * std::sin(theta));
    return TruthState{p_turn, v_turn};
  }

  // Leg 3: straight at post-turn velocity.
  const double speed = v0_.norm();
  const double theta0 = std::atan2(v0_.y(), v0_.x());
  const double theta_end = theta0 + omega_ * t_turn_;
  const Eigen::Vector2d v_post(speed * std::cos(theta_end),
                               speed * std::sin(theta_end));
  // Position at end of leg 2:
  const Eigen::Vector2d p2(
      p1.x() + (speed / omega_) * (std::sin(theta_end) - std::sin(theta0)),
      p1.y() - (speed / omega_) * (std::cos(theta_end) - std::cos(theta0)));
  const double tau3 = tau - t_straight_ - t_turn_;
  return TruthState{p2 + v_post * tau3, v_post};
}

}  // namespace navtracker::sim
