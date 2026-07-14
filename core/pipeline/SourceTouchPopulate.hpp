#pragma once

#include <cmath>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Populate the ENU position + 2D covariance fields of a SourceTouch
 * from a Measurement, projecting (range, bearing) into ENU when the
 * measurement model carries no direct ENU coordinates. Used by
 * Tracker / MhtTracker so the provenance side-channel (consumed by
 * AisArpaPairExtractor and SensorBiasPairExtractor) is non-empty
 * regardless of the measurement model.
 *
 * Bearing2D is left at (0,0) — no observable range, no ENU position
 * the touch can carry. Consumers that handle bearings (the heading-
 * bias estimator's bearing variant) read the bearing from the
 * measurement directly, not from the touch.
 */
inline void fillSourceTouchEnu(Track::SourceTouch& touch,
                               const Measurement& z) {
  // Carry any bias corrections the adapter / pipeline already applied so the
  // bias pair extractors can reconstruct the raw observation (W3.1/W3.2).
  // Model-independent — set once regardless of the value/covariance layout.
  touch.applied_heading_bias_rad = z.applied_heading_bias_rad;
  touch.applied_position_bias_enu = z.applied_position_bias_enu;
  touch.applied_bearing_bias_rad = z.applied_bearing_bias_rad;
  switch (z.model) {
    case MeasurementModel::Position2D:
      if (z.value.size() >= 2) {
        touch.value_enu = z.value.head<2>();
        if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
          touch.covariance = z.covariance.topLeftCorner<2, 2>();
        }
      }
      break;
    case MeasurementModel::PositionVelocity2D:
      if (z.value.size() >= 2) {
        touch.value_enu = z.value.head<2>();
        if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
          touch.covariance = z.covariance.topLeftCorner<2, 2>();
        }
      }
      break;
    case MeasurementModel::RangeBearing2D:
      if (z.value.size() >= 2) {
        const double r = z.value(0);
        const double b = z.value(1);
        const double cb = std::cos(b);
        const double sb = std::sin(b);
        touch.value_enu = z.sensor_position_enu
            + Eigen::Vector2d(r * cb, r * sb);
        if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
          // J = [[cos b, -r sin b], [sin b, r cos b]]; touch_cov = J R Jᵀ.
          Eigen::Matrix2d J;
          J << cb, -r * sb,
               sb,  r * cb;
          touch.covariance =
              J * z.covariance.topLeftCorner<2, 2>() * J.transpose();
        }
      }
      break;
    case MeasurementModel::Bearing2D:
      // No range observation, so value_enu stays at the default
      // zero. We do carry the bearing itself so the bias-pair
      // extractor can compose (AIS-anchor position) × (bearing
      // contribution) into a BearingBiasPairObservation downstream.
      if (z.value.size() >= 1) {
        touch.alpha_rad = z.value(0);
        if (z.covariance.rows() >= 1 && z.covariance.cols() >= 1) {
          touch.alpha_var_rad2 = z.covariance(0, 0);
        }
      }
      break;
  }
}

}  // namespace navtracker
