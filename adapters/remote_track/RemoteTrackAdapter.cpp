#include "adapters/remote_track/RemoteTrackAdapter.hpp"

#include <utility>

#include "adapters/util/EdgeValidation.hpp"

namespace navtracker {

RemoteTrackAdapter::RemoteTrackAdapter(geo::Datum datum,
                                       RemoteTrackAdapterConfig config)
    : datum_(std::move(datum)), config_(config) {}

void RemoteTrackAdapter::ingest(const RemoteTrackReport& r) {
  // Invariant #6: reject implausible / sentinel / NaN fixes at the edge before
  // they can become a phantom track.
  if (!edge::isPlausibleLatLon(r.lat_deg, r.lon_deg)) {
    ++rejected_;
    return;
  }

  // The feed carries this vessel's identity regardless of thinning — record it
  // for the circular-AIS guard even if this particular update is thinned.
  if (r.mmsi) seen_mmsis_.insert(*r.mmsi);

  // Rate thinning: consecutive filtered outputs for one remote track id are
  // correlated. Keep at most one per min_update_interval_s (per station).
  const std::pair<std::string, std::int32_t> key{r.source_id, r.remote_track_id};
  auto it = last_emit_.find(key);
  if (it != last_emit_.end()) {
    const double gap_s = r.time.seconds() - it->second.seconds();
    if (gap_s < config_.min_update_interval_s) {
      ++thinned_;
      return;
    }
  }
  last_emit_[key] = r.time;

  const Eigen::Vector3d enu = datum_.toEnu({r.lat_deg, r.lon_deg, 0.0});

  // Position R: inflate the STATED covariance, or fall back to the pessimistic
  // absolute default when none is stated (never both).
  const bool pos_default = r.position_covariance.isZero();
  Eigen::Matrix2d posR;
  if (pos_default) {
    const double s = config_.default_position_std_m;
    posR = Eigen::Matrix2d::Identity() * (s * s);
  } else {
    posR = r.position_covariance * config_.r_inflation_factor;
  }

  Measurement m;
  m.time = r.time;
  m.sensor = SensorKind::RemoteTrack;
  m.source_id = r.source_id;
  m.hints.sensor_track_id = r.remote_track_id;
  if (r.mmsi) m.hints.mmsi = *r.mmsi;
  m.covariance_is_default = pos_default;

  if (config_.accept_velocity && r.has_velocity) {
    // Velocity opt-in: PositionVelocity2D value = [e, n, ve, vn]; R is the
    // block-diagonal of the (inflated / default) position and velocity blocks.
    Eigen::Matrix2d velR;
    if (r.velocity_covariance.isZero()) {
      const double sv = config_.default_velocity_std_mps;
      velR = Eigen::Matrix2d::Identity() * (sv * sv);
    } else {
      velR = r.velocity_covariance * config_.r_inflation_factor;
    }
    m.model = MeasurementModel::PositionVelocity2D;
    Eigen::VectorXd v(4);
    v << enu.x(), enu.y(), r.velocity_enu.x(), r.velocity_enu.y();
    m.value = std::move(v);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
    R.topLeftCorner<2, 2>() = posR;
    R.bottomRightCorner<2, 2>() = velR;
    m.covariance = std::move(R);
  } else {
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    m.covariance = posR;
  }

  buffer_.push_back(std::move(m));
}

std::vector<Measurement> RemoteTrackAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

std::vector<std::uint32_t> RemoteTrackAdapter::circularAisMmsis(
    const std::set<std::uint32_t>& raw_ais_mmsis) const {
  std::vector<std::uint32_t> overlap;
  for (std::uint32_t mmsi : seen_mmsis_) {
    if (raw_ais_mmsis.count(mmsi)) overlap.push_back(mmsi);
  }
  return overlap;  // sorted ascending (seen_mmsis_ is a std::set)
}

}  // namespace navtracker
