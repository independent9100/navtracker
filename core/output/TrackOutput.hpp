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
 * Ordering convention of a TrackOutput's position covariance. Stamped by the
 * producing function (toTrackOutputENU / toTrackOutputNED) so a struct passed
 * around retains its convention and an axis-sensitive consumer may assert on
 * it. See docs/output-contract.md.
 *   Enu — slot (0,0) = east variance, (1,1) = north variance.
 *   Ned — slot (0,0) = north variance, (1,1) = east variance.
 */
enum class CovarianceFrame { Enu, Ned };

/**
 * Position in geodetic coordinates (lat, lon) with covariance in m² expressed
 * in the target's local frame. `toGeodeticWithCov` applies only the datum→
 * target meridian-convergence rotation (Option A of the output-interface
 * design, 2026-06-04) — a small rotation between two ENU frames, NEVER an axis
 * relabel — so the ordering it emits is **ENU (east, north)**: slot (0,0) is
 * east variance, (1,1) is north. For tracks within 30 km of the current datum
 * the rotation is < 0.5°, so the σ_east / σ_north magnitudes match the
 * datum-ENU magnitudes within numerical precision. The operator-facing
 * north-first (NED) ordering is available via toTrackOutputNED.
 */
struct PositionGeodeticWithCov {
  double lat_deg;
  double lon_deg;
  Eigen::Matrix2d position_covariance_m2;   // local ENU at target (east,north), m²
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
  // Ordering of position.position_covariance_m2, stamped by the producer.
  CovarianceFrame covariance_frame{CovarianceFrame::Enu};
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
 * Build a TrackOutput from a Track and the current datum. Drives the two
 * helpers above and copies metadata fields verbatim.
 *
 * Two entry points, one per covariance-ordering convention — a caller MUST
 * choose (there is deliberately no ambiguous `toTrackOutput`; the compile-time
 * break at every call site is the consumer audit — no caller can flip
 * silently). Position (lat/lon), velocity (SOG/COG) and all metadata are
 * identical between the two; ONLY the position-covariance axis ordering and
 * the stamped `covariance_frame` differ.
 *
 *   toTrackOutputENU — position_covariance_m2 in ENU order: (0,0)=east
 *                      variance, (1,1)=north. This is what the internal ENU
 *                      state carries; frame = CovarianceFrame::Enu.
 *   toTrackOutputNED — the operator-facing north-first copy: (0,0)=north
 *                      variance, (1,1)=east; frame = CovarianceFrame::Ned.
 *
 * See docs/output-contract.md for unit semantics and a worked example.
 */
TrackOutput toTrackOutputENU(const Track& track, const geo::Datum& datum);
TrackOutput toTrackOutputNED(const Track& track, const geo::Datum& datum);

}  // namespace navtracker
