#pragma once

namespace navtracker {

/**
 * Snapshot of the heading-bias estimator's current published estimate.
 * is_published is false when the estimator has not yet converged or
 * has lost its anchor; consumers treat that as "use b = 0" (the
 * pre-estimator behavior).
 */
struct HeadingBiasEstimate {
  double bias_rad{0.0};
  double variance_rad2{0.0};
  bool is_published{false};
};

/**
 * Read-only port exposing the current heading/gyro-bias estimate to
 * consumers (adapters, bearing projection). Null providers behave as
 * "bias = 0, not published".
 */
class IHeadingBiasProvider {
 public:
  virtual ~IHeadingBiasProvider() = default;
  /** Current published heading-bias estimate (or not-published). */
  virtual HeadingBiasEstimate current() const = 0;
};

}  // namespace navtracker
