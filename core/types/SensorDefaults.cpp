#include "core/types/SensorDefaults.hpp"

#include <cmath>

namespace navtracker {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

Eigen::MatrixXd SensorDefaults::covarianceFor(SensorKind sensor,
                                              MeasurementModel model) const {
  auto pos2D = [](double sigma) {
    Eigen::Matrix2d c = Eigen::Matrix2d::Zero();
    c(0, 0) = sigma * sigma;
    c(1, 1) = sigma * sigma;
    return c;
  };
  auto rb2D = [](double sigma_r, double sigma_b) {
    Eigen::Matrix2d c = Eigen::Matrix2d::Zero();
    c(0, 0) = sigma_r * sigma_r;
    c(1, 1) = sigma_b * sigma_b;
    return c;
  };
  auto b1D = [](double sigma_b) {
    Eigen::Matrix<double, 1, 1> c;
    c(0, 0) = sigma_b * sigma_b;
    return c;
  };

  if (sensor == SensorKind::Ais && model == MeasurementModel::Position2D)
    return pos2D(ais_position.sigma_pos_m);
  if (sensor == SensorKind::Cooperative && model == MeasurementModel::Position2D)
    return pos2D(cooperative_position.sigma_pos_m);
  if (sensor == SensorKind::ArpaTll && model == MeasurementModel::Position2D)
    return pos2D(arpa_tll_position.sigma_pos_m);
  if (sensor == SensorKind::ArpaTtm && model == MeasurementModel::RangeBearing2D)
    return rb2D(arpa_ttm_range_bearing.sigma_range_m,
                arpa_ttm_range_bearing.sigma_bearing_rad);
  if (sensor == SensorKind::EoIr && model == MeasurementModel::RangeBearing2D)
    return rb2D(eoir_range_bearing.sigma_range_m,
                eoir_range_bearing.sigma_bearing_rad);
  if (sensor == SensorKind::EoIr && model == MeasurementModel::Bearing2D)
    return b1D(eoir_bearing_only.sigma_bearing_rad);

  return Eigen::MatrixXd{};  // empty: unknown combination
}

SensorDefaults pessimisticSensorDefaults() {
  SensorDefaults d;
  d.ais_position.sigma_pos_m              = 30.0;
  // Fleet-partner GNSS. Tighter than AIS — no transponder/aggregation
  // chain and partners often run DGPS/RTK — but still pessimistic
  // enough for consumer-grade receivers without RTK.
  d.cooperative_position.sigma_pos_m      = 10.0;
  d.arpa_tll_position.sigma_pos_m         = 50.0;
  d.arpa_ttm_range_bearing.sigma_range_m   = 75.0;
  d.arpa_ttm_range_bearing.sigma_bearing_rad = 1.5 * kDeg2Rad;
  d.eoir_range_bearing.sigma_range_m       = 50.0;
  d.eoir_range_bearing.sigma_bearing_rad   = 1.0 * kDeg2Rad;
  d.eoir_bearing_only.sigma_bearing_rad    = 1.5 * kDeg2Rad;
  return d;
}

void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d) {
  if (m.covariance.size() != 0) return;
  Eigen::MatrixXd cov = d.covarianceFor(m.sensor, m.model);
  if (cov.size() == 0) return;  // unknown combination, leave empty
  m.covariance = std::move(cov);
  m.covariance_is_default = true;
}

}  // namespace navtracker
