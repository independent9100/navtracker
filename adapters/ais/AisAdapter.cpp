#include "adapters/ais/AisAdapter.hpp"

#include <utility>

namespace navtracker {

AisAdapter::AisAdapter(geo::Datum datum) : datum_(std::move(datum)) {}

void AisAdapter::ingest(const AisDynamicReport& r) {
  const Eigen::Vector3d enu = datum_.toEnu({r.lat_deg, r.lon_deg, 0.0});
  Measurement m;
  m.time = r.time;
  m.sensor = SensorKind::Ais;
  m.source_id = r.source_id;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(enu.x(), enu.y());
  const double sigma = r.high_accuracy ? 10.0 : 30.0;
  m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
  if (r.mmsi != 0) m.hints.mmsi = r.mmsi;
  buffer_.push_back(std::move(m));
}

std::vector<Measurement> AisAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
