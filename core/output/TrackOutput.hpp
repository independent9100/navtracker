#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Position in geodetic coordinates (lat, lon) with covariance in m^2
 * expressed in the target's local NED frame (north-east). The rotation
 * from datum-ENU to target-local-NED is applied automatically per Option A
 * of the output-interface design (2026-06-04). For tracks within 30 km
 * of the current datum, this rotation is < 0.5° and the magnitudes
 * of sigma_north / sigma_east match the datum-ENU magnitudes within
 * numerical precision.
 */
struct PositionGeodeticWithCov {
  double lat_deg;
  double lon_deg;
  Eigen::Matrix2d position_covariance_m2;   // local NED at target, m²
};

/**
 * Velocity in operator-facing form: SOG (m/s) + COG (deg, true).
 * COG is measured clockwise from true north, in [0, 360).
 * sigma values are derived from the velocity covariance via the
 * standard polar Jacobian. When sog < 0.01 m/s the COG direction is
 * not meaningful and sigma_cog is set to 0.
 * is_valid carries the caller's signal whether the velocity is
 * populated at all (a track with 2D state has no velocity).
 */
struct VelocityGeodeticWithSigma {
  double sog_m_per_s{0.0};
  double cog_deg{0.0};
  double sigma_sog_m_per_s{0.0};
  double sigma_cog_deg{0.0};
  bool is_valid{false};
};

/**
 * Canonical drainable output for one Track. Position in lat/lon;
 * covariance in m²; velocity in SOG/COG. Bundles metadata: stable
 * track id, lifecycle status, last-update timestamp, sensor-derived
 * attributes (mmsi, vessel type, ...), provenance, and a diagnostic
 * flag indicating whether the underlying covariance came from
 * SensorDefaults rather than a real sensor uncertainty.
 */
struct TrackOutput {
  TrackId id;
  TrackStatus status;
  Timestamp last_update;
  PositionGeodeticWithCov position;
  VelocityGeodeticWithSigma velocity;
  TrackAttributes attributes;
  std::vector<std::string> contributing_sources;
  bool covariance_is_default{false};
};

/**
 * Convert a 2D ENU position + 2×2 covariance into geodetic
 * coordinates with covariance rotated into the target's local NED
 * frame.
 */
PositionGeodeticWithCov toGeodeticWithCov(
    const Eigen::Vector2d& enu_xy,
    const Eigen::Matrix2d& cov_enu_m2,
    const geo::Datum& datum);

/**
 * Convert a 2D ENU velocity + 2×2 covariance into SOG/COG with
 * scalar sigmas via polar Jacobian. is_valid is carried through
 * from the caller's signal.
 */
VelocityGeodeticWithSigma toVelocityOutput(
    const Eigen::Vector2d& v_enu,
    const Eigen::Matrix2d& v_cov_m2_per_s2,
    bool is_valid);

/**
 * Build a TrackOutput from a Track and the current datum. Drives
 * the two helpers above and copies metadata fields verbatim.
 */
TrackOutput toTrackOutput(const Track& track,
                          const geo::Datum& datum);

}  // namespace navtracker
