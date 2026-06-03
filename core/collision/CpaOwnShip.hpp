#pragma once

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Build a Track from own-ship state for use with computeCpaWithUncertainty.
// state = [ex, ey, vx, vy] in ENU; covariance = diag(σ_pos², σ_pos², 0, 0).
// id is the reserved sentinel TrackId{0}; not entered into TrackManager.
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum);

}  // namespace navtracker
