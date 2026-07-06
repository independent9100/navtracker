#pragma once

#include <cmath>

#include <Eigen/Core>

namespace navtracker {

// Shared SOG/COG → ENU-velocity conversion for target-reported kinematics
// (backlog #20). BOTH the NMEA AisAdapter and the replay loadAisCsv build
// PositionVelocity2D measurement content from a target's own-GPS SOG/COG; this
// is the single source of truth for that math so the two paths cannot drift.
//
// Math. COG is marine/true (radians, N = 0, clockwise), so the ENU velocity is
//   v = SOG · [sin(COG), cos(COG)]                      (east, north).
// The covariance is propagated from the (SOG, COG) 1-σ pair through the polar
// Jacobian of v w.r.t. (SOG, COG):
//   J = [[sin(COG),  SOG·cos(COG)],
//        [cos(COG), −SOG·sin(COG)]]   (columns d/dSOG, d/dCOG)
//   cov = J · diag(σ_SOG², σ_COG²) · Jᵀ  +  I · σ_floor².
// Assumptions. SOG/COG errors are independent and Gaussian; COG is only
// meaningful above a small SOG (see kAisSogVelocityMinMps at the call sites).
// At low SOG the d/dCOG column shrinks, leaving a near rank-1 (over-confident
// cross-track) velocity block — the isotropic floor σ_floor prevents that
// degeneracy. Rationale / alternatives are in docs/algorithms and the #20
// backlog entry; the learning intro is docs/learning/17-multi-sensor-and-bias.

// Default 1-σ / threshold values for building AIS SOG/COG velocity content.
// Single source shared by AisAdapterConfig (its per-deployment knobs default to
// these) and the replay loader, so "the SAME rules" the ticket asks for cannot
// diverge between the NMEA and replay paths.
inline constexpr double kAisSogVelocityMinMps = 0.5;   // below this: Position2D
inline constexpr double kAisSogStdMps = 0.5;           // SOG 1-σ (m/s)
inline constexpr double kAisCogStdDeg = 5.0;           // COG 1-σ (deg)
inline constexpr double kAisVelocityIsoFloorMps = 0.3; // isotropic vel-σ floor (m/s)

struct EnuVelocity2D {
  Eigen::Vector2d velocity;    // ENU (east, north), m/s
  Eigen::Matrix2d covariance;  // 2×2, m²/s²
};

/**
 * Convert a polar SOG (m/s) + COG (radians, marine/true) report into an ENU
 * velocity and its covariance. `sog_std_mps` / `cog_std_rad` are the reporting
 * 1-σ; `velocity_iso_floor_mps` is the isotropic floor added to the propagated
 * block. Callers assemble the [e, n, ve, vn] value vector and the block-diagonal
 * 4×4 R (position block differs per source).
 */
inline EnuVelocity2D sogCogToEnuVelocity(double sog_mps, double cog_rad,
                                         double sog_std_mps, double cog_std_rad,
                                         double velocity_iso_floor_mps) {
  const double s = std::sin(cog_rad), c = std::cos(cog_rad);
  Eigen::Matrix2d J;
  J << s, sog_mps * c, c, -sog_mps * s;
  Eigen::Matrix2d D =
      Eigen::Vector2d(sog_std_mps * sog_std_mps, cog_std_rad * cog_std_rad)
          .asDiagonal();
  EnuVelocity2D out;
  out.velocity = Eigen::Vector2d(sog_mps * s, sog_mps * c);
  out.covariance = J * D * J.transpose() +
                   Eigen::Matrix2d::Identity() *
                       (velocity_iso_floor_mps * velocity_iso_floor_mps);
  return out;
}

}  // namespace navtracker
