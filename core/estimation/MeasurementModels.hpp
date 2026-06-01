#pragma once

#include <Eigen/Core>

#include "core/types/Ids.hpp"  // MeasurementModel

namespace navtracker {

// Predicted measurement h(x) and its Jacobian H = dh/dx at a state.
struct MeasurementPrediction {
  Eigen::VectorXd z_pred;
  Eigen::MatrixXd H;
};

// Wrap an angle in radians to the interval (-pi, pi].
double wrapAngle(double radians);

// h(x) and H for `model` evaluated at state = [px, py, vx, vy].
// `sensor_position_enu` is the sensor's ENU position; defaults to the origin.
// Range/bearing and bearing-only models compute dx = px - sx, dy = py - sy so
// that measurements are relative to the sensor rather than the ENU origin.
MeasurementPrediction predictMeasurement(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());

// h(x) only, no Jacobian. Cheaper than predictMeasurement for callers
// (e.g. particle filter) that only need the predicted measurement value.
// `sensor_position_enu` defaults to the origin; see predictMeasurement.
Eigen::VectorXd predictMeasurementValue(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());

// Measurement residual z - h(x); bearing component is angle-wrapped for
// the RangeBearing2D model.
Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred);

}  // namespace navtracker
