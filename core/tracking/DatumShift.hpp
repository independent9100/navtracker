#pragma once

#include "core/geo/Datum.hpp"
#include "core/geo/AxisRotation.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

// Mutate every track in `mgr` so that its state and covariance are
// expressed in `new_datum`'s ENU frame instead of `old_datum`'s.
// Position is re-projected via geodetic; velocity is rotated by the
// ENU axis-convergence angle; covariance is block-rotated.
// Multi-mode carriers (IMM means/covariances, particles) are also
// shifted.
void shiftTracksOnDatumChange(TrackManager& mgr,
                              const geo::Datum& old_datum,
                              const geo::Datum& new_datum);

}  // namespace navtracker
