#pragma once

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"

namespace navtracker::sim {

struct TruthState {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // ENU metres
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};  // m/s
};

class ITruthTrajectory {
 public:
  virtual ~ITruthTrajectory() = default;
  virtual TruthState eval(Timestamp t) const = 0;
};

class ConstantVelocityTrajectory final : public ITruthTrajectory {
 public:
  ConstantVelocityTrajectory(Eigen::Vector2d p0,
                             Eigen::Vector2d v,
                             Timestamp t0);

  TruthState eval(Timestamp t) const override;

 private:
  Eigen::Vector2d p0_;
  Eigen::Vector2d v_;
  Timestamp t0_;
};

class ManeuveringTrajectory final : public ITruthTrajectory {
 public:
  // Three-leg: straight at `velocity` from `start` for `straight_duration_s`,
  // then a constant-rate turn at `omega_rad_s` for `turn_duration_s`
  // (positive omega = left turn in ENU), then straight at the post-turn
  // heading and speed indefinitely. t0 anchors the first leg's start.
  // Precondition: omega_rad_s != 0 (use ConstantVelocityTrajectory for a
  // pure straight trajectory).
  ManeuveringTrajectory(Eigen::Vector2d start,
                        Eigen::Vector2d velocity,
                        double straight_duration_s,
                        double turn_duration_s,
                        double omega_rad_s,
                        Timestamp t0);

  TruthState eval(Timestamp t) const override;

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d v0_;
  double t_straight_;
  double t_turn_;
  double omega_;
  Timestamp t0_;
};

}  // namespace navtracker::sim
