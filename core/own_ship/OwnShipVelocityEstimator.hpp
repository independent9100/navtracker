#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipVelocityEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  double min_sigma_v_m_per_s{0.05};
};

struct OwnShipVelocityEstimate {
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
  double sigma_v_m_per_s{0.0};
  bool is_published{false};
};

// Online estimator for own-ship ENU velocity from successive GGA-derived
// position fixes. Fits constant-velocity per axis over a sliding window;
// the slopes are the published velocity and the slope standard error is
// the published sigma_v. Mirrors UereEstimator's sliding-window pattern,
// including the noise-aware two-halves maneuver gate. When the window is
// not yet full or a maneuver is detected, the estimate is unpublished.
class OwnShipVelocityEstimator {
 public:
  struct Sample { double t; double x; double y; };

  explicit OwnShipVelocityEstimator(OwnShipVelocityEstimatorConfig cfg = {});

  // Push one GGA-derived ENU sample.
  void observe(Timestamp t, double x_enu, double y_enu);

  OwnShipVelocityEstimate current() const;

  // Diagnostics for tests.
  std::size_t windowSize() const { return samples_.size(); }

 private:
  OwnShipVelocityEstimatorConfig cfg_;
  std::deque<Sample> samples_;
};

}  // namespace navtracker
