#include "core/output/TrackOutput.hpp"

#include <cmath>

#include "core/geo/AxisRotation.hpp"

namespace navtracker {

PositionGeodeticWithCov toGeodeticWithCov(
    const Eigen::Vector2d& enu_xy,
    const Eigen::Matrix2d& cov_enu_m2,
    const geo::Datum& datum) {
  const auto geo_target = datum.toGeodetic(
      Eigen::Vector3d(enu_xy.x(), enu_xy.y(), 0.0));
  const geo::Datum target_datum(
      geo::Geodetic{geo_target.lat_deg, geo_target.lon_deg, 0.0});
  const Eigen::Matrix2d R = geo::datumAxisRotation(datum, target_datum);
  PositionGeodeticWithCov out;
  out.lat_deg = geo_target.lat_deg;
  out.lon_deg = geo_target.lon_deg;
  out.position_covariance_m2 = R * cov_enu_m2 * R.transpose();
  return out;
}

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kSogEpsilon = 0.01;   // m/s — below this, COG is undefined
}

VelocityGeodeticWithSigma toVelocityOutput(
    const Eigen::Vector2d& v_enu,
    const Eigen::Matrix2d& v_cov_m2_per_s2,
    bool is_valid) {
  VelocityGeodeticWithSigma out;
  out.is_valid = is_valid;
  if (!is_valid) return out;
  const double sog = v_enu.norm();
  const double cog_rad = std::atan2(v_enu.x(), v_enu.y());
  double cog_deg = cog_rad * kRad2Deg;
  if (cog_deg < 0.0) cog_deg += 360.0;
  if (cog_deg >= 360.0) cog_deg -= 360.0;
  out.sog_m_per_s = sog;
  out.cog_deg = (sog < kSogEpsilon) ? 0.0 : cog_deg;

  // Polar Jacobian on (v_east, v_north) -> (sog, cog).
  if (sog < kSogEpsilon) {
    // Direction undefined; σ_sog still well-defined.
    // Limit: as sog -> 0, ∂sog/∂v_east, ∂sog/∂v_north depend on direction;
    // safest practical choice is the isotropic trace bound.
    const double sigma_sog2 = 0.5 * v_cov_m2_per_s2.trace();
    out.sigma_sog_m_per_s = std::sqrt(std::max(sigma_sog2, 0.0));
    out.sigma_cog_deg = 0.0;
    return out;
  }
  const double s = std::sin(cog_rad);
  const double c = std::cos(cog_rad);
  Eigen::Matrix2d J;
  J << s,         c,
       c / sog,  -s / sog;
  const Eigen::Matrix2d cov_polar = J * v_cov_m2_per_s2 * J.transpose();
  out.sigma_sog_m_per_s = std::sqrt(std::max(cov_polar(0, 0), 0.0));
  out.sigma_cog_deg     = std::sqrt(std::max(cov_polar(1, 1), 0.0)) * kRad2Deg;
  return out;
}

TrackOutput toTrackOutput(const Track& track,
                          const geo::Datum& datum) {
  TrackOutput out;
  out.id = track.id;
  out.status = track.status;
  out.last_update = track.last_update;
  out.attributes = track.attributes;
  out.contributing_sources = track.contributing_sources;
  out.covariance_is_default = false;  // v1: forwarding from recent_contributions deferred

  Eigen::Vector2d pos_enu = Eigen::Vector2d::Zero();
  Eigen::Matrix2d pos_cov = Eigen::Matrix2d::Zero();
  if (track.state.size() >= 2) {
    pos_enu = Eigen::Vector2d(track.state(0), track.state(1));
  }
  if (track.covariance.rows() >= 2 && track.covariance.cols() >= 2) {
    pos_cov = track.covariance.topLeftCorner<2, 2>();
  }
  out.position = toGeodeticWithCov(pos_enu, pos_cov, datum);

  if (track.state.size() >= 4 && track.covariance.rows() >= 4 &&
      track.covariance.cols() >= 4) {
    const Eigen::Vector2d v_enu(track.state(2), track.state(3));
    const Eigen::Matrix2d v_cov = track.covariance.block<2, 2>(2, 2);
    const bool v_valid = v_cov.trace() > 0.0;
    out.velocity = toVelocityOutput(v_enu, v_cov, v_valid);
  } else {
    out.velocity = VelocityGeodeticWithSigma{};  // default: is_valid=false, zeros
  }
  return out;
}

}  // namespace navtracker
