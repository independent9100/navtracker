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

Eigen::VectorXd predictMeasurementValue(MeasurementModel model,
                                        const Eigen::VectorXd& state) {
  const double px = state(0);
  const double py = state(1);
  switch (model) {
    case MeasurementModel::Position2D:
      return Eigen::Vector2d(px, py);
    case MeasurementModel::PositionVelocity2D:
      return state.head<4>();
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      return Eigen::Vector2d(r, std::atan2(py, px));
    }
    case MeasurementModel::Bearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      Eigen::VectorXd v(1);
      v(0) = std::atan2(py, px);
      return v;
    }
  }
  return Eigen::VectorXd();
}

MeasurementPrediction predictMeasurement(MeasurementModel model,
                                         const Eigen::VectorXd& state) {
  MeasurementPrediction out;
  out.z_pred = predictMeasurementValue(model, state);
  const int n = static_cast<int>(state.size());
  const double px = state(0);
  const double py = state(1);
  switch (model) {
    case MeasurementModel::Position2D: {
      out.H = Eigen::MatrixXd::Zero(2, n);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      break;
    }
    case MeasurementModel::PositionVelocity2D: {
      out.H = Eigen::MatrixXd::Zero(4, n);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      out.H(2, 2) = 1.0;
      out.H(3, 3) = 1.0;
      break;
    }
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      out.H = Eigen::MatrixXd::Zero(2, n);
      out.H(0, 0) = px / r;
      out.H(0, 1) = py / r;
      out.H(1, 0) = -py / (r * r);
      out.H(1, 1) = px / (r * r);
      break;
    }
    case MeasurementModel::Bearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      out.H = Eigen::MatrixXd::Zero(1, n);
      out.H(0, 0) = -py / (r * r);
      out.H(0, 1) =  px / (r * r);
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
  } else if (model == MeasurementModel::Bearing2D) {
    y(0) = wrapAngle(y(0));
  }
  return y;
}

}  // namespace navtracker
