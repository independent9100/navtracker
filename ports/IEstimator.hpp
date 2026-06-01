#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Recursive state estimator strategy. Implementations advance and correct a
// track's kinematic state/covariance.
class IEstimator {
 public:
  virtual ~IEstimator() = default;

  // Advance the track's state and covariance to time `to`.
  virtual void predict(Track& track, Timestamp to) const = 0;

  // Fold a measurement into the track. Assumes the track was already
  // predicted to z.time.
  virtual void update(Track& track, const Measurement& z) const = 0;

  // Create a new Tentative track seeded from a position-type measurement.
  virtual Track initiate(const Measurement& z) const = 0;

  // Soft update for probabilistic data association (PDAF / JPDA). Default
  // is no-op; estimators that support soft updates override this.
  // `betas(j)` = P(measurement j came from this track | data),
  // `beta_0` = P(no measurement assigned to this track this scan).
  // sum_j betas(j) + beta_0 == 1.
  virtual void softUpdate(Track& /*track*/,
                          const std::vector<Measurement>& /*gated_measurements*/,
                          const Eigen::VectorXd& /*betas*/,
                          double /*beta_0*/) const {}
};

}  // namespace navtracker
