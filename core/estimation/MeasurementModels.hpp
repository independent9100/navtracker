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
MeasurementPrediction predictMeasurement(MeasurementModel model,
                                         const Eigen::VectorXd& state);

// Measurement residual z - h(x); bearing component is angle-wrapped for
// the RangeBearing2D model.
Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred);

}  // namespace navtracker
