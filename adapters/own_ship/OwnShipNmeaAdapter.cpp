#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>

#include "adapters/util/EdgeValidation.hpp"
#include "adapters/util/Nmea.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/bias/HeadingBiasObservations.hpp"

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

double signedFromDir(double magnitude, const std::string& dir) {
  if (dir == "E") return magnitude;
  if (dir == "W") return -magnitude;
  return std::nan("");
}

// #26 M16/M17: parse a REQUIRED numeric NMEA field. Returns nullopt when the
// field is empty or does not parse to a FINITE value — both the empty-field
// strtod("")==0.0 case (which used to publish a bogus due-north / zero-SOG
// sample) and the strtod("1e400")==+Inf case (which used to hang wrapDegToPi's
// wrap loop, freezing the ingest thread). Callers must treat nullopt as
// "reject this sentence", never as 0.
std::optional<double> parseFiniteField(const std::string& s) {
  if (s.empty()) return std::nullopt;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) return std::nullopt;   // no digits consumed
  if (!std::isfinite(v)) return std::nullopt;  // Inf / NaN / overflow
  return v;
}

double wrapDegToPi(double a) {
  double rad = a * kDegToRad;
  // #26 M17: never spin on a non-finite input. Inf - 2π == Inf, so the wrap
  // loops below never terminate on Inf; NaN falls straight through. Required
  // callers gate on parseFiniteField, but keep the wrap itself total.
  if (!std::isfinite(rad)) return rad;
  constexpr double kPi = 3.14159265358979323846;
  while (rad > kPi) rad -= 2.0 * kPi;
  while (rad <= -kPi) rad += 2.0 * kPi;
  return rad;
}

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

void OwnShipNmeaAdapter::pushGyroSample(Timestamp t, double heading_deg) {
  if (gyro_history_count_ < gyro_history_.size()) {
    const std::size_t idx =
        (gyro_history_head_ + gyro_history_count_) % gyro_history_.size();
    gyro_history_[idx].t = t;
    gyro_history_[idx].heading_rad = wrapDegToPi(heading_deg);
    ++gyro_history_count_;
  } else {
    gyro_history_[gyro_history_head_].t = t;
    gyro_history_[gyro_history_head_].heading_rad = wrapDegToPi(heading_deg);
    gyro_history_head_ = (gyro_history_head_ + 1) % gyro_history_.size();
  }
}

std::optional<double> OwnShipNmeaAdapter::latestGyroRad(
    Timestamp t, double max_age_s) const {
  if (gyro_history_count_ == 0) return std::nullopt;
  const std::size_t last_idx =
      (gyro_history_head_ + gyro_history_count_ - 1) % gyro_history_.size();
  const auto& s = gyro_history_[last_idx];
  const double age = t.secondsSince(s.t);
  if (age < 0.0 || age > max_age_s) return std::nullopt;
  return s.heading_rad;
}

double OwnShipNmeaAdapter::gyroRateRadPerSec(Timestamp t,
                                             double max_dt_s) const {
  (void)t;
  if (gyro_history_count_ < 2) return 0.0;
  const std::size_t n = gyro_history_count_;
  const std::size_t last_idx =
      (gyro_history_head_ + n - 1) % gyro_history_.size();
  const std::size_t prev_idx =
      (gyro_history_head_ + n - 2) % gyro_history_.size();
  const auto& a = gyro_history_[prev_idx];
  const auto& b = gyro_history_[last_idx];
  const double dt = b.t.secondsSince(a.t);
  if (dt <= 0.0 || dt > max_dt_s) return 0.0;
  constexpr double kPi = 3.14159265358979323846;
  double dh = b.heading_rad - a.heading_rad;
  // #26 M17: the stored headings come from parseFiniteField-gated wrapDegToPi,
  // so dh is finite; guard anyway so a non-finite slip can never spin the wrap.
  if (!std::isfinite(dh)) return 0.0;
  while (dh > kPi) dh -= 2.0 * kPi;
  while (dh < -kPi) dh += 2.0 * kPi;
  return dh / dt;
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
    // Validate at the edge (invariant #6): a GGA must carry a valid fix and a
    // plausible position, or it produces NO pose. Fix quality is field[5]
    // (0 = invalid / no-fix). Without this, a standard no-fix GGA (empty
    // lat/lon fields, quality 0) parses to (0,0) and publishes a null-island
    // pose that initializes — or auto-recenters — the datum there, silently
    // corrupting every downstream ENU conversion. The RMC branch already
    // rejects its "V" navigation-warning status; this is the GGA counterpart.
    // Rejections are counted (skippedNoFixGga) so the drop is observable, not
    // silent — and we return BEFORE any datum/estimator/provider side effect.
    if (parsed->fields.size() < 6                                    // no quality
        || parsed->fields[5].empty()
        || std::strtol(parsed->fields[5].c_str(), nullptr, 10) <= 0  // 0 = no fix
        || parsed->fields[1].empty() || parsed->fields[3].empty()) { // no position
      ++skip_no_fix_gga_;
      return false;
    }
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    if (!edge::isPlausibleLatLon(lat, lon)) {  // out of range / NaN
      ++skip_no_fix_gga_;
      return false;
    }
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
    // #26 M16: SOG (field 6) and COG (field 7) are required. A bare strtod
    // turned an empty field into 0.0 — a bogus zero-velocity buffered as a real
    // measurement — and an overflow into Inf. Reject the sentence instead.
    const auto sog_knots_opt = parseFiniteField(parsed->fields[6]);
    const auto cog_deg_opt = parseFiniteField(parsed->fields[7]);
    if (!sog_knots_opt || !cog_deg_opt) {
      ++skip_non_finite_;
      return false;
    }
    const double sog_m_per_s = *sog_knots_opt * 0.514444;  // 1 kn = 1852/3600 m/s
    const double cog_deg = *cog_deg_opt;
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

    // Forward magnetic variation if present (fields 9, 10).
    if (parsed->fields.size() > 10 && !parsed->fields[9].empty()
                                   && !parsed->fields[10].empty()) {
      const double var_mag = std::strtod(parsed->fields[9].c_str(), nullptr);
      const double var_signed = signedFromDir(var_mag, parsed->fields[10]);
      if (!std::isnan(var_signed)) cached_variation_deg_ = var_signed;
    }

    if (bias_estimator_ != nullptr) {
      const auto gyro_rad = latestGyroRad(t, cfg_.gyro_max_age_s);
      if (!gyro_rad.has_value()) {
        ++skip_stale_;
      } else {
        GyroVsGpsCogObservation obs;
        obs.time = t;
        obs.gyro_rad = *gyro_rad;
        obs.gps_cog_rad = wrapDegToPi(cog_deg);
        obs.gps_cog_std_rad = cfg_.gps_cog_sigma_deg * kDegToRad;
        obs.sog_mps = sog_m_per_s;
        obs.gyro_rate_rad_per_s = gyroRateRadPerSec(t, cfg_.gyro_max_age_s);
        bias_estimator_->observe(obs);
        ++d_cog_;
      }
    }
    return true;
  }
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    // #26 M16/M17: an empty heading field parsed to 0.0 (published as an
    // authoritative due-north sample) and an overflowed field to Inf (which
    // hung wrapDegToPi's wrap loop). Reject a non-finite heading at the edge.
    const auto heading_deg_opt = parseFiniteField(parsed->fields[0]);
    if (!heading_deg_opt) {
      ++skip_non_finite_;
      return false;
    }
    const double heading_deg = *heading_deg_opt;
    const bool routed_gps =
        cfg_.gps_heading_talkers.count(parsed->talker) > 0;
    if (routed_gps) {
      pose.gps_true_heading_deg = heading_deg;
      pose.gps_true_heading_std_deg = cfg_.gps_heading_sigma_deg;
      provider_.update(pose);
      if (bias_estimator_ != nullptr) {
        const auto gyro_rad = latestGyroRad(t, cfg_.gyro_max_age_s);
        if (gyro_rad.has_value()) {
          GyroVsGpsHeadingObservation obs;
          obs.time = t;
          obs.gyro_rad = *gyro_rad;
          obs.gps_true_heading_rad = wrapDegToPi(heading_deg);
          obs.gps_true_heading_std_rad =
              cfg_.gps_heading_sigma_deg * kDegToRad;
          bias_estimator_->observe(obs);
          ++d_gps_hdg_;
        } else {
          ++skip_stale_;
        }
      }
    } else {
      pose.heading_true_deg = heading_deg;
      pushGyroSample(t, heading_deg);
      provider_.update(pose);
    }
    return true;
  }
  if (parsed->formatter == "HDG") {
    if (parsed->fields.empty()) return false;
    // #26 M16: the magnetic heading (field 0) is required; reject empty /
    // non-finite rather than publish a 0.0 due-north sample.
    const auto mag_raw_deg_opt = parseFiniteField(parsed->fields[0]);
    if (!mag_raw_deg_opt) {
      ++skip_non_finite_;
      return false;
    }
    const double mag_raw_deg = *mag_raw_deg_opt;
    double dev_deg = 0.0;
    if (parsed->fields.size() > 2 && !parsed->fields[1].empty()) {
      const double dev_mag = std::strtod(parsed->fields[1].c_str(), nullptr);
      const double dev_signed = signedFromDir(dev_mag, parsed->fields[2]);
      if (!std::isnan(dev_signed)) dev_deg = dev_signed;
    }
    const double mag_corr_deg = mag_raw_deg + dev_deg;
    pose.magnetic_heading_deg = mag_corr_deg;
    pose.magnetic_heading_std_deg = cfg_.magnetic_heading_sigma_deg;

    double variation_deg = std::nan("");
    if (parsed->fields.size() > 4 && !parsed->fields[3].empty()
                                  && !parsed->fields[4].empty()) {
      const double var_mag = std::strtod(parsed->fields[3].c_str(), nullptr);
      variation_deg = signedFromDir(var_mag, parsed->fields[4]);
    }
    if (!std::isnan(variation_deg)) {
      pose.magnetic_variation_deg = variation_deg;
      cached_variation_deg_ = variation_deg;
    }
    provider_.update(pose);

    if (bias_estimator_ != nullptr) {
      const auto gyro_rad = latestGyroRad(t, cfg_.gyro_max_age_s);
      if (!gyro_rad.has_value()) {
        ++skip_stale_;
      } else {
        double var_use_deg = variation_deg;
        if (std::isnan(var_use_deg)) var_use_deg = cached_variation_deg_;
        if (std::isnan(var_use_deg)) {
          var_use_deg = cfg_.magnetic_variation_fallback_deg;
        }
        if (std::isnan(var_use_deg)) {
          ++skip_mag_var_;
        } else {
          GyroVsMagneticObservation obs;
          obs.time = t;
          obs.gyro_rad = *gyro_rad;
          obs.magnetic_heading_rad = wrapDegToPi(mag_corr_deg);
          obs.magnetic_heading_std_rad =
              cfg_.magnetic_heading_sigma_deg * kDegToRad;
          obs.magnetic_variation_rad = var_use_deg * kDegToRad;
          bias_estimator_->observe(obs);
          ++d_mag_;
        }
      }
    }
    return true;
  }
  return false;
}

}  // namespace navtracker
