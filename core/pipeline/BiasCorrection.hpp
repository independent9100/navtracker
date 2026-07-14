#pragma once

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "ports/ISensorBiasProvider.hpp"

namespace navtracker {

/**
 * Apply the published per-sensor bias to an incoming measurement and
 * inflate its covariance to reflect the bias estimator's residual
 * uncertainty (Schmidt-KF / "considered" approximation).
 *
 * Math:
 *   - Position2D: z' = z - b, R'[0:2,0:2] = R + H_b P_b H_bᵀ. The
 *     additive bias model gives H_b = I_2, so R' = R + P_b.
 *   - Bearing2D:  β' = β - b_β, R'[0,0] = R + σ_b² (scalar; H_b = 1).
 *   - RangeBearing2D: β-component shifted by b_β only (range not
 *     biased here); R'[1,1] += σ_b². Range row/col untouched.
 *
 * Assumptions:
 *   - The provider's (b, P_b) is computed externally (item 9: from
 *     AIS-anchored pairs the per-track filter never sees), so its
 *     covariance is uncorrelated with any single track's posterior.
 *     This justifies dropping the state-bias cross-covariance — the
 *     standard "Schmidt-KF without considered cross-cov" trick.
 *   - z.covariance is the sensor's measurement-noise covariance for
 *     `z.model`. If it is the wrong shape for the model, R is not
 *     touched (defensive).
 *   - is_published = false → behaves identical to having no provider:
 *     no shift, no R inflation. Caller-side null-provider check is
 *     handled by the caller (see Tracker / MhtTracker hot paths).
 *
 * Rationale:
 *   - The shared bias estimator publishes a single (b, P_b) per
 *     (sensor, source_id), with P_b shrinking as the estimator sees
 *     more anchored pairs. Inflating R with H_b P_b H_bᵀ tells every
 *     consuming filter "trust this measurement *up to* the bias
 *     uncertainty"; without this, applying only the mean b would make
 *     the filter overconfident exactly when the bias estimator is not
 *     yet converged, leading to over-tight tracks and NIS << 1 right
 *     after publish.
 *
 * Improve next:
 *   - Carry the state-bias cross-covariance ("considered Kalman")
 *     when the bias is observable through the same measurement the
 *     track sees — needed if a single sensor's bias is calibrated
 *     only via its own tracks (no AIS anchor). Adds bookkeeping
 *     proportional to (target_count × sensor_count); skipped here
 *     because all production wirings use the AIS-anchored path.
 *   - For RangeBearing2D, model an optional range bias b_r when a
 *     sensor's range scale is suspected of drift.
 */
inline Measurement applyBiasCorrection(const Measurement& z,
                                       const ISensorBiasProvider* provider) {
  if (provider == nullptr) return z;
  const SensorBiasKey key{z.sensor, z.source_id};
  Measurement out = z;
  if (z.model == MeasurementModel::Position2D && z.value.size() >= 2) {
    const auto pb = provider->positionBias(key);
    if (pb.is_published) {
      out.value(0) -= pb.bias_enu_m.x();
      out.value(1) -= pb.bias_enu_m.y();
      // Record what was subtracted so the bias pair extractor can reconstruct
      // the raw position and not subtract b̂ a second time (W3.2).
      out.applied_position_bias_enu = pb.bias_enu_m;
      if (out.covariance.rows() >= 2 && out.covariance.cols() >= 2) {
        out.covariance.block<2, 2>(0, 0) += pb.covariance_m2;
      }
    }
  } else if (z.model == MeasurementModel::Bearing2D && z.value.size() >= 1) {
    const auto bb = provider->bearingBias(key);
    if (bb.is_published) {
      out.value(0) -= bb.bias_rad;
      out.applied_bearing_bias_rad = bb.bias_rad;
      if (out.covariance.rows() >= 1 && out.covariance.cols() >= 1) {
        out.covariance(0, 0) += bb.variance_rad2;
      }
    }
  } else if (z.model == MeasurementModel::RangeBearing2D &&
             z.value.size() >= 2) {
    const auto bb = provider->bearingBias(key);
    if (bb.is_published) {
      out.value(1) -= bb.bias_rad;
      out.applied_bearing_bias_rad = bb.bias_rad;
      if (out.covariance.rows() >= 2 && out.covariance.cols() >= 2) {
        out.covariance(1, 1) += bb.variance_rad2;
      }
    }
  }
  return out;
}

}  // namespace navtracker
