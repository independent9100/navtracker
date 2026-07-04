#pragma once

#include <cstddef>
#include <deque>

#include "core/types/Timestamp.hpp"

namespace navtracker {

/** Tuning for `UereEstimator`: sliding-window size, maneuver-gate threshold, and sigma floor. */
struct UereEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  double min_sigma_m{0.05};
};

/** Published own-ship GPS position 1-σ (UERE); is_published is false until the window fills and no maneuver is active. */
struct UereEstimate {
  double sigma_m{0.0};
  bool is_published{false};
};

/**
 * Online observer for own-ship GPS position sigma from successive ENU
 * fixes. Fits constant velocity over a sliding window and uses residual
 * variance as a direct sigma_pos estimate. Suppresses publication during
 * maneuvering windows (delta-v between halves > threshold).
 */
class UereEstimator {
 public:
  struct Sample { double t; double x; double y; };

  explicit UereEstimator(UereEstimatorConfig cfg = {});

  /**
   * Push one GGA-derived ENU sample. t/x/y in seconds and meters; the
   * implementation operates entirely on dt offsets internally.
   */
  void observe(Timestamp t, double x_enu, double y_enu);

  /** Current UERE estimate (unpublished until the window fills and no maneuver is detected). */
  UereEstimate current() const;

  /** Diagnostics for tests: number of samples currently in the window. */
  std::size_t windowSize() const { return samples_.size(); }

 private:
  UereEstimatorConfig cfg_;
  std::deque<Sample> samples_;
};

}  // namespace navtracker
