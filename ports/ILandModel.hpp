#pragma once

#include <Eigen/Core>

namespace navtracker {

// Continuous spatial clutter/land prior, queried by the tracker at birth time.
// Pure, zero-I/O. Nullable in use: if no land model is wired, behaviour is
// exactly today's. See docs/superpowers/specs/2026-06-30-pmbm-land-clutter-prior-design.md
class ILandModel {
 public:
  virtual ~ILandModel() = default;

  // Prior that a detection at this ENU position (metres) is shore/structure
  // clutter rather than a real new vessel:
  //   0.0  = open water        (no birth suppression)
  //   ~0.5 = at the waterline  (soft suppression)
  //   1.0  = well inside land  (hard-gate region)
  // Positions outside the loaded coastline's coverage return 0.0.
  virtual double clutterPrior(const Eigen::Vector2d& enu_xy) const = 0;
};

}  // namespace navtracker
