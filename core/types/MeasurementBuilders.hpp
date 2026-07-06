#pragma once

#include <string>

#include <Eigen/Core>

#include "core/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * Range + RELATIVE bearing (the radar / EO-IR / sonar common case).
 * Looks up the most recent OwnShipPose with pose.time <= t via
 * `provider.poseAtOrBefore(t)`, adds own-ship heading to the relative
 * bearing, and projects to ENU Position2D via projectRangeBearingToEnu.
 * Composes own-ship GPS position uncertainty from the looked-up pose.
 *
 * Heading (gyro/compass) σ composition (#16): the effective heading
 * uncertainty folded into the angular covariance (in quadrature with
 * bearing_std_rad) is `max(pose.heading_std_deg, heading_std_floor_deg)`.
 * The pose's per-fix `heading_std_deg` (optional) lets a nav source report
 * heading quality that varies fix-to-fix; `heading_std_floor_deg` is the
 * caller's static trust bound so an implausibly tight per-fix σ cannot make
 * measurements overconfident (the floor only widens, never tightens). With
 * both at their defaults (pose σ absent, floor 0) the heading contribution is
 * 0 — bit-identical to the prior behaviour, where heading σ came only from
 * bearing_std_rad (which the ArpaAdapter / EoIrAdapter pre-inflate via a wired
 * HeadingBiasEstimator, or the caller supplies). This is the composition
 * helper the bearing-wedge hazard (#17) refers to: pass the wedge a
 * bearing_sigma composed the same way (camera ⊕ heading).
 *
 * If no pose at-or-before t is available, returns a Measurement with
 * empty value/covariance and `covariance_is_default == false`; callers
 * should drop or buffer these (the situation indicates the sensor
 * arrived before any GPS fix).
 *
 * All angles in radians. Range in meters. `heading_std_floor_deg` in DEGREES
 * (matching OwnShipPose.heading_std_deg).
 */
Measurement makeMeasurementFromRelativeBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double relative_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints = {},
    double heading_std_floor_deg = 0.0);

/**
 * Range + TRUE bearing (already-projected, world-frame). Useful when the
 * sensor pipeline pre-computes true bearings outside this library.
 * Otherwise identical to the relative-bearing variant, including the #16
 * `max(pose.heading_std_deg, heading_std_floor_deg)` heading-σ composition.
 */
Measurement makeMeasurementFromTrueBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double true_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints = {},
    double heading_std_floor_deg = 0.0);

/**
 * Absolute ENU position (AIS-style). No pose lookup or projection — the
 * ENU position is already in the working frame. Exposes a uniform
 * construction surface so SensorDefaults composition is consistent.
 * Pass `Eigen::Matrix2d::Zero()` when no uncertainty info is available: an
 * all-zero matrix is treated as the empty/unknown sentinel (the impl tests
 * `!covariance.isZero()`), so the result's covariance is left empty for
 * `applyDefaultsIfEmpty` to fill. Do NOT pass a default-constructed
 * `Eigen::Matrix2d`: it is uninitialized garbage, never reliably zero.
 */
Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    Eigen::Vector2d enu_xy,
    Eigen::Matrix2d covariance,
    AssociationHints hints = {});

}  // namespace navtracker
