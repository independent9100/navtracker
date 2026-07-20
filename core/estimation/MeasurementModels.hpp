#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "core/types/Ids.hpp"  // MeasurementModel

namespace navtracker {

/** Predicted measurement h(x) and its Jacobian H = dh/dx at a state. */
struct MeasurementPrediction {
  Eigen::VectorXd z_pred;
  Eigen::MatrixXd H;
};

/** Wrap an angle in radians to the interval (-pi, pi]. */
double wrapAngle(double radians);

/**
 * Whether a single measurement of this model carries enough information to
 * initiate a new track (i.e. observe Cartesian position). Active sensors
 * (Position2D / PositionVelocity2D / RangeBearing2D) do; a bearing-only
 * measurement does not — its range is unobservable from one look, so it
 * can only refine an existing track. Track-birth paths use this to drop
 * non-gating passive-sensor measurements instead of seeding garbage state.
 * Matches the standard maritime-fusion convention (Helgesen et al. 2022,
 * §4.4.1: "only active sensors are used in track initialization").
 */
inline bool canInitiateTrack(MeasurementModel model) {
  return model != MeasurementModel::Bearing2D;
}

/**
 * h(x) and H for `model` evaluated at `state`, whose first two entries are
 * [px, py] (ENU position, m). The function is state-dimension-agnostic: H is
 * sized to `state.size()`, so 4-state [px,py,vx,vy] and 5-state
 * [px,py,vx,vy,ω] inputs are both valid.
 * `sensor_position_enu` is the sensor's ENU position; defaults to the origin.
 * Range/bearing and bearing-only models compute dx = px - sx, dy = py - sy so
 * that measurements are relative to the sensor rather than the ENU origin.
 */
MeasurementPrediction predictMeasurement(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());

/**
 * h(x) only, no Jacobian. Cheaper than predictMeasurement for callers
 * (e.g. particle filter) that only need the predicted measurement value.
 * Evaluated at `state` whose first two entries are [px, py] (ENU, m);
 * state-dimension-agnostic (4- or 5-state).
 * `sensor_position_enu` defaults to the origin; see predictMeasurement.
 */
Eigen::VectorXd predictMeasurementValue(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());

/**
 * Measurement residual z - h(x); the bearing component is angle-wrapped for
 * both models that carry a bearing — index 1 for RangeBearing2D, index 0 for
 * Bearing2D.
 */
Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred);

/**
 * Reject measurements whose covariance is non-finite (NaN/Inf), the
 * wrong shape, or not strictly positive-definite. Adapters should
 * validate at the edge per CLAUDE.md invariant 6, but a defensive
 * guard at estimator entry prevents one bad NMEA frame from
 * permanently poisoning a track's covariance.
 *
 * Phase 8 R3 introduced this guard with a cheap diagonal-positivity
 * check; Phase 8 iter 4 R3-strengthen promotes it to a real PSD test
 * via LDLT, which catches the diagonal-positive / off-diagonal-
 * dominated case (e.g. a near-singular bias-covariance from an
 * uncalibrated sensor) that the cheap check let through.
 */
inline bool isMeasurementCovariancePsd(const Eigen::MatrixXd& R) {
  if (R.rows() == 0 || R.cols() == 0) return false;
  if (R.rows() != R.cols()) return false;
  if (!R.allFinite()) return false;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(R);
  if (ldlt.info() != Eigen::Success) return false;
  const Eigen::VectorXd d = ldlt.vectorD();
  for (Eigen::Index i = 0; i < d.size(); ++i) {
    if (!(d(i) > 0.0)) return false;  // strict positivity (rejects 0 and NaN)
  }
  return true;
}

/**
 * Dimension-aware guard (#35 M1). The single-argument form above never sees the
 * measurement it accompanies, so a square-but-wrong-size R (e.g. a 3×3 R on a
 * 2-D Position2D measurement) passed the PSD check and then mismatched in
 * `H·P·Hᵀ + z.cov` — an `eigen_assert` abort in debug, an out-of-bounds read
 * under NDEBUG. Estimator/tracker entry points that have the measurement pass
 * its dimension (`z.dim()` == the measurement value size) so a mis-sized R is
 * rejected before the update runs. Composes with the PSD/finite checks; on a
 * correctly-sized R the result is identical to the single-argument form
 * (byte-identical downstream).
 */
inline bool isMeasurementCovariancePsd(const Eigen::MatrixXd& R,
                                       int expected_dim) {
  if (R.rows() != expected_dim) return false;
  return isMeasurementCovariancePsd(R);
}

}  // namespace navtracker
