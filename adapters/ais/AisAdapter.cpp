#include "adapters/ais/AisAdapter.hpp"

#include <cmath>
#include <utility>

#include "adapters/util/EdgeValidation.hpp"
#include "core/estimation/PolarVelocity.hpp"

namespace navtracker {

namespace {
constexpr double kKnotsToMps = 0.514444;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}  // namespace

AisAdapter::AisAdapter(geo::Datum datum, AisAdapterConfig cfg)
    : datum_(std::move(datum)), cfg_(cfg) {}

void AisAdapter::ingest(const AisDynamicReport& r) {
  // Invariant #6: reject implausible / sentinel / NaN fixes at the edge
  // (AIS lat 91° / lon 181° "not available", garbled positions) before
  // they become phantom tracks.
  if (!edge::isPlausibleLatLon(r.lat_deg, r.lon_deg)) return;
  const Eigen::Vector3d enu = datum_.toEnu({r.lat_deg, r.lon_deg, 0.0});
  const double sigma =
      r.high_accuracy ? cfg_.position_std_high_accuracy_m : cfg_.position_std_standard_m;

  Measurement m;
  m.time = r.time;
  m.sensor = SensorKind::Ais;
  m.source_id = r.source_id;

  // #20: emit velocity content (PositionVelocity2D) when the report carries a
  // usable SOG (and COG). Below sog_velocity_min_mps the COG is meaningless, so
  // fall back to Position2D — "COG down-weighted at low SOG". SOG/COG come from
  // the target's own GPS (independent witness), so this is not double-counting.
  const bool has_speed = r.sog_knots.has_value() && *r.sog_knots >= 0.0 &&
                         *r.sog_knots < 1023.0;  // 1023 = SOG not available
  const bool has_course = r.cog_deg.has_value() && *r.cog_deg >= 0.0 &&
                          *r.cog_deg < 360.0;  // 3600 sentinel already > 360
  const double sog_mps = has_speed ? *r.sog_knots * kKnotsToMps : 0.0;
  if (cfg_.emit_velocity_from_sog_cog && has_speed && has_course &&
      sog_mps >= cfg_.sog_velocity_min_mps) {
    // SOG/COG → ENU velocity + covariance via the shared polar-Jacobian helper
    // (core/estimation/PolarVelocity.hpp) — the identical math the replay
    // loadAisCsv path uses, so the two cannot drift (backlog #20).
    const double cog = *r.cog_deg * kDeg2Rad;  // true, clockwise from north
    const EnuVelocity2D vel =
        sogCogToEnuVelocity(sog_mps, cog, cfg_.sog_std_mps,
                            cfg_.cog_std_deg * kDeg2Rad,
                            cfg_.velocity_iso_floor_mps);
    m.model = MeasurementModel::PositionVelocity2D;
    Eigen::VectorXd v(4);
    v << enu.x(), enu.y(), vel.velocity.x(), vel.velocity.y();
    m.value = v;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
    R.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() * (sigma * sigma);
    R.bottomRightCorner<2, 2>() = vel.covariance;
    m.covariance = R;
  } else {
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
  }
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
