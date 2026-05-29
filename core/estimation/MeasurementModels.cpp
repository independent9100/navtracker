#include "core/estimation/MeasurementModels.hpp"

#include <cmath>

namespace navtracker {
namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double wrapAngle(double radians) {
  double a = std::fmod(radians + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

MeasurementPrediction predictMeasurement(MeasurementModel model,
                                         const Eigen::VectorXd& state) {
  const double px = state(0);
  const double py = state(1);
  MeasurementPrediction out;
  switch (model) {
    case MeasurementModel::Position2D: {
      out.z_pred = Eigen::Vector2d(px, py);
      out.H = Eigen::MatrixXd::Zero(2, 4);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      break;
    }
    case MeasurementModel::PositionVelocity2D: {
      out.z_pred = state.head<4>();
      out.H = Eigen::MatrixXd::Identity(4, 4);
      break;
    }
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      out.z_pred = Eigen::Vector2d(r, std::atan2(py, px));
      out.H = Eigen::MatrixXd::Zero(2, 4);
      out.H(0, 0) = px / r;
      out.H(0, 1) = py / r;
      out.H(1, 0) = -py / (r * r);
      out.H(1, 1) = px / (r * r);
      break;
    }
  }
  return out;
}

Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred) {
  Eigen::VectorXd y = z - z_pred;
  if (model == MeasurementModel::RangeBearing2D) {
    y(1) = wrapAngle(y(1));
  }
  return y;
}

}  // namespace navtracker
