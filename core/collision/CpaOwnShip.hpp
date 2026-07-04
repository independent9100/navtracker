#pragma once

#include "core/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Build a Track from own-ship state for use with computeCpaWithUncertainty.
 * state = [ex, ey, vx, vy] in ENU; covariance = diag(sigma_pos^2, sigma_pos^2,
 * sigma_v^2, sigma_v^2), where sigma_pos is taken from pose.position_std_m and
 * sigma_v is pose.velocity_std_m_per_s when pose.velocity_is_valid is true, or
 * zero otherwise. Velocity is read from pose.velocity_enu.
 * id is the reserved sentinel TrackId{0}; not entered into TrackManager.
 */
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const OwnShipProvider& provider);

}  // namespace navtracker
