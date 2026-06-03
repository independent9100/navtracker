#pragma once

namespace navtracker {

// Snapshot of the heading-bias estimator's current published estimate.
// is_published is false when the estimator has not yet converged or
// has lost its anchor; consumers treat that as "use b = 0" (the
// pre-estimator behavior).
struct HeadingBiasEstimate {
  double bias_rad{0.0};
  double variance_rad2{0.0};
  bool is_published{false};
};

class IHeadingBiasProvider {
 public:
  virtual ~IHeadingBiasProvider() = default;
  virtual HeadingBiasEstimate current() const = 0;
};

}  // namespace navtracker
