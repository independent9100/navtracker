#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

ConstantVelocityTrajectory::ConstantVelocityTrajectory(Eigen::Vector2d p0,
                                                       Eigen::Vector2d v,
                                                       Timestamp t0)
    : p0_(std::move(p0)), v_(std::move(v)), t0_(t0) {}

TruthState ConstantVelocityTrajectory::eval(Timestamp t) const {
  const double dt = t.secondsSince(t0_);
  return TruthState{p0_ + v_ * dt, v_};
}

}  // namespace navtracker::sim
