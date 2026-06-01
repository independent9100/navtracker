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

}  // namespace navtracker::sim
