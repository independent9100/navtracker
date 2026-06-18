#pragma once

#include <vector>

#include <Eigen/Core>

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

// Shift one kinematic carrier (state vector + covariance + optional IMM
// mode means/covariances) from old_datum's ENU frame into new_datum's.
// Position rows (0,1) are re-projected via geodetic; velocity rows (2,3)
// are rotated by the ENU axis-convergence angle; the 4x4 kinematic
// covariance block is rotated. Carriers smaller than a given block are
// skipped. Particles are NOT handled here (Track-only carrier); see
// shiftTracksOnDatumChange. Used by both the TrackManager path and the
// MhtTracker's per-node tree shift so the two stay in lockstep.
void shiftStateOnDatumChange(Eigen::VectorXd& state,
                             Eigen::MatrixXd& covariance,
                             Eigen::MatrixXd& imm_means,
                             std::vector<Eigen::MatrixXd>& imm_covariances,
                             const geo::Datum& old_datum,
                             const geo::Datum& new_datum);

}  // namespace navtracker
