#pragma once

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
};

}  // namespace navtracker
