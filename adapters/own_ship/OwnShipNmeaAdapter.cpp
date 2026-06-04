#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

#include "adapters/util/Nmea.hpp"

namespace navtracker {
namespace {

double parseDdmm(const std::string& s) {
  if (s.empty()) return 0.0;
  const auto dot = s.find('.');
  const std::size_t mm_digits = 2;
  const std::size_t deg_end = (dot == std::string::npos ? s.size() : dot) - mm_digits;
  if (deg_end > s.size()) return 0.0;
  const double deg = std::strtod(s.substr(0, deg_end).c_str(), nullptr);
  const double min = std::strtod(s.substr(deg_end).c_str(), nullptr);
  return deg + min / 60.0;
}

// WGS-84 semi-major axis. Used only as the meter-per-radian scale in the
// local equirectangular projection that feeds the UereEstimator. We do not
// need a true ENU frame here — the estimator is invariant to translation
// and to a coordinate rotation, and an axis-wise linear scaling preserves
// residual variance in meters. (We avoid pulling in geo::Datum to keep
// this adapter dependency-free.)
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

OwnShipNmeaAdapter::OwnShipNmeaAdapter(OwnShipProvider& provider,
                                       OwnShipNmeaAdapterConfig cfg)
    : provider_(provider),
      cfg_(cfg),
      uere_estimator_(cfg.uere_estimator_cfg),
      velocity_estimator_(cfg.velocity_estimator_cfg) {}

void OwnShipNmeaAdapter::setPositionStd(double sigma_m) {
  position_std_m_ = sigma_m;
}

bool OwnShipNmeaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;
  OwnShipPose pose = provider_.latest().value_or(OwnShipPose{});
  pose.time = t;

  if (parsed->formatter == "GGA") {
    // GGA is the only message that updates position uncertainty: clear
    // before reapplying via the precedence rule below. Note: HDT messages
    // intentionally DO NOT touch position_std_m — they preserve whatever
    // the most recent GGA established, so an adaptive estimate is not
    // clobbered by an interleaved HDT.
    pose.position_std_m = position_std_m_;
    if (parsed->fields.size() < 5) return false;
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    pose.lat_deg = lat;
    pose.lon_deg = lon;

    // Compute an equirectangular ENU offset about the first GGA fix. Used
    // by both the UERE estimator (sigma_pos) and the velocity estimator
    // (sigma_v). The reference is captured unconditionally so the velocity
    // estimator can run even when the adaptive UERE path is disabled.
    // Over the timescale of either sliding window (~8 s) the cos(lat)
    // scale change is negligible.
    if (!enu_ref_set_) {
      enu_ref_lat_deg_ = lat;
      enu_ref_lon_deg_ = lon;
      enu_ref_set_ = true;
    }
    const double cos_ref = std::cos(enu_ref_lat_deg_ * kDegToRad);
    const double enu_x_m = (lon - enu_ref_lon_deg_) * kDegToRad *
                           kEarthRadiusM * cos_ref;
    const double enu_y_m = (lat - enu_ref_lat_deg_) * kDegToRad *
                           kEarthRadiusM;
    if (cfg_.enable_adaptive_uere) {
      uere_estimator_.observe(t, enu_x_m, enu_y_m);
    }
    if (cfg_.enable_velocity_estimator) {
      velocity_estimator_.observe(t, enu_x_m, enu_y_m);
    }

    // sigma precedence: adaptive (when published) > HDOP * UERE_static
    // (when HDOP > 0) > sticky setter (already in pose.position_std_m).
    double hdop = 0.0;
    if (parsed->fields.size() > 7 && !parsed->fields[7].empty()) {
      hdop = std::strtod(parsed->fields[7].c_str(), nullptr);
    }
    const UereEstimate adaptive = uere_estimator_.current();
    if (cfg_.enable_adaptive_uere && adaptive.is_published) {
      pose.position_std_m = adaptive.sigma_m;
    } else if (hdop > 0.0) {
      pose.position_std_m = hdop * cfg_.uere_m;
    }
    // else: keep the sticky-setter value loaded into pose.position_std_m
    // at the top of this function.

    // Velocity precedence (spec §4.4): prefer a fresh RMC if configured;
    // otherwise fall back to the GGA-derived estimator; otherwise leave
    // velocity invalid.
    const double dt_rmc_s = rmc_buffer_.has_value
        ? t.secondsSince(rmc_buffer_.time)
        : std::numeric_limits<double>::infinity();
    const bool rmc_fresh = cfg_.prefer_rmc_velocity
                        && rmc_buffer_.has_value
                        && dt_rmc_s >= 0.0
                        && dt_rmc_s <= cfg_.rmc_stale_seconds;
    if (rmc_fresh) {
      pose.velocity_enu = rmc_buffer_.velocity_enu;
      pose.velocity_std_m_per_s = rmc_buffer_.sigma_v_m_per_s;
      pose.velocity_is_valid = true;
    } else {
      const auto v_est = velocity_estimator_.current();
      if (v_est.is_published) {
        pose.velocity_enu = v_est.velocity_enu;
        pose.velocity_std_m_per_s = v_est.sigma_v_m_per_s;
        pose.velocity_is_valid = true;
      } else {
        pose.velocity_enu = Eigen::Vector2d::Zero();
        pose.velocity_std_m_per_s = 0.0;
        pose.velocity_is_valid = false;
      }
    }
    provider_.update(pose);
    return true;
  }
  if (parsed->formatter == "RMC") {
    // RMC layout after $xxRMC,: [0]=time, [1]=status(A/V), [2]=lat,
    // [3]=N/S, [4]=lon, [5]=E/W, [6]=SOG(knots), [7]=COG(deg true),
    // [8]=date, [9]=magvar, [10]=E/W, [11]=mode(optional).
    if (parsed->fields.size() < 8) return false;
    if (parsed->fields[1] != "A") return false;  // V = navigation receiver warning
    const double sog_knots = std::strtod(parsed->fields[6].c_str(), nullptr);
    const double sog_m_per_s = sog_knots * 0.514444;  // 1 knot = 1852/3600 m/s
    const double cog_deg = std::strtod(parsed->fields[7].c_str(), nullptr);
    const double cog_rad = cog_deg * kDegToRad;
    // ENU: east = SOG · sin(COG_true), north = SOG · cos(COG_true)
    // (COG is measured clockwise from true north).
    rmc_buffer_.time = t;
    rmc_buffer_.velocity_enu = Eigen::Vector2d(sog_m_per_s * std::sin(cog_rad),
                                               sog_m_per_s * std::cos(cog_rad));
    // First-order error propagation: independent sigma_SOG (m/s) and
    // sigma_COG (rad). The bearing term scales with speed, so at SOG=0
    // sigma_v reduces to sigma_SOG.
    const double sigma_cog_rad = cfg_.sigma_cog_deg * kDegToRad;
    rmc_buffer_.sigma_v_m_per_s = std::sqrt(
        cfg_.sigma_sog_m_per_s * cfg_.sigma_sog_m_per_s
        + (sog_m_per_s * sigma_cog_rad) * (sog_m_per_s * sigma_cog_rad));
    rmc_buffer_.has_value = true;
    return true;
  }
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    pose.heading_true_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    provider_.update(pose);
    return true;
  }
  return false;
}

}  // namespace navtracker
