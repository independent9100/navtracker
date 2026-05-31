#include "adapters/eoir/EoIrAdapter.hpp"

#include <utility>

#include "adapters/util/Projection.hpp"

namespace navtracker {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}

EoIrAdapter::EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship)
    : datum_(std::move(datum)), own_ship_(own_ship) {}

void EoIrAdapter::ingest(const CameraDetection& d) {
  const auto own_opt = own_ship_.latest();
  if (!own_opt) return;
  const double bearing_true_rad =
      (d.bearing_relative_deg + own_opt->heading_true_deg) * kDeg2Rad;

  const Eigen::Vector3d own_enu = datum_.toEnu({own_opt->lat_deg, own_opt->lon_deg, 0.0});
  const Eigen::Vector2d own_xy(own_enu.x(), own_enu.y());

  const PointAndCov2D out = projectRangeBearingToEnu(
      d.range_m, bearing_true_rad, d.range_std_m, d.bearing_std_deg * kDeg2Rad, own_xy);

  Measurement m;
  m.time = d.time;
  m.sensor = SensorKind::EoIr;
  m.source_id = d.source_id;
  m.model = MeasurementModel::Position2D;
  m.value = out.pos_enu;
  m.covariance = out.cov;
  if (d.sensor_track_id) m.hints.sensor_track_id = d.sensor_track_id;
  buffer_.push_back(std::move(m));
}

std::vector<Measurement> EoIrAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
