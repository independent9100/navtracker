#include "adapters/ais/AisAdapter.hpp"

#include <utility>

#include "adapters/util/EdgeValidation.hpp"

namespace navtracker {

AisAdapter::AisAdapter(geo::Datum datum) : datum_(std::move(datum)) {}

void AisAdapter::ingest(const AisDynamicReport& r) {
  // Invariant #6: reject implausible / sentinel / NaN fixes at the edge
  // (AIS lat 91° / lon 181° "not available", garbled positions) before
  // they become phantom tracks.
  if (!edge::isPlausibleLatLon(r.lat_deg, r.lon_deg)) return;
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

  // Target-reported kinematics (backlog #20). Validate at the edge
  // (invariant #6): drop AIS "not available" sentinels / out-of-range values
  // rather than let a 511° heading or an "undefined" nav-status reach the core.
  // heading is an attribute (true heading in [0,360)); nav_status is the
  // corroboration cue (0..14; 15 = undefined is dropped).
  if (r.heading_deg.has_value() && *r.heading_deg >= 0.0 &&
      *r.heading_deg < 360.0) {
    m.hints.heading_deg = *r.heading_deg;
  }
  if (r.nav_status.has_value() && *r.nav_status <= 14) {
    m.hints.nav_status = *r.nav_status;
  }
  buffer_.push_back(std::move(m));
}

std::vector<Measurement> AisAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
