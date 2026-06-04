#pragma once

#include <string>

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Range + RELATIVE bearing (the radar / EO-IR / sonar common case).
// Looks up the most recent OwnShipPose with pose.time <= t via
// `provider.poseAtOrBefore(t)`, adds own-ship heading to the relative
// bearing, and projects to ENU Position2D via projectRangeBearingToEnu.
// Composes own-ship GPS position uncertainty from the looked-up pose
// (sigma_heading is 0 for now — see spec §5 / plan Task 4 step 2).
//
// If no pose at-or-before t is available, returns a Measurement with
// empty value/covariance and `covariance_is_default == false`; callers
// should drop or buffer these (the situation indicates the sensor
// arrived before any GPS fix).
//
// All angles in radians. Range in meters.
Measurement makeMeasurementFromRelativeBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double relative_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints = {});

// Range + TRUE bearing (already-projected, world-frame). Useful when the
// sensor pipeline pre-computes true bearings outside this library.
// Otherwise identical to the relative-bearing variant.
Measurement makeMeasurementFromTrueBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double true_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints = {});

// Absolute ENU position (AIS-style). No pose lookup or projection — the
// ENU position is already in the working frame. Exposes a uniform
// construction surface so SensorDefaults composition is consistent.
// Pass an empty 2x2 covariance (default-constructed Eigen::Matrix2d is
// uninitialized — see implementation) when no uncertainty info is
// available; the result is a Measurement whose covariance is empty and
// can be filled by applyDefaultsIfEmpty.
Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    Eigen::Vector2d enu_xy,
    Eigen::Matrix2d covariance,
    AssociationHints hints = {});

}  // namespace navtracker
