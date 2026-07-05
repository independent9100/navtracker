#include "adapters/arpa/ArpaAdapter.hpp"

#include <cmath>
#include <cstdlib>
#include <utility>

#include "adapters/util/EdgeValidation.hpp"
#include "adapters/util/Nmea.hpp"
#include "core/projection/Projection.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

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

double distanceToMeters(double value, const std::string& units) {
  if (units == "N") return value * 1852.0;
  if (units == "K") return value * 1000.0;
  if (units == "S") return value * 1609.344;
  return value;
}

// TTM speed units (field 10, same K/N/S code as distance): N = knots,
// K = km/h, S = mph. → m/s.
double speedToMps(double value, const std::string& units) {
  if (units == "N") return value * 0.514444;
  if (units == "K") return value / 3.6;
  if (units == "S") return value * 0.44704;
  return value;  // assume m/s if unlabelled
}

// Smallest absolute difference between two true-course angles, degrees [0,180].
double courseDeltaDeg(double a_deg, double b_deg) {
  double d = std::fmod(std::abs(a_deg - b_deg), 360.0);
  return d > 180.0 ? 360.0 - d : d;
}

}  // namespace

ArpaAdapter::ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
                         ArpaAdapterConfig cfg,
                         const IHeadingBiasProvider* bias_provider)
    : datum_(std::move(datum)),
      own_ship_(own_ship),
      cfg_(cfg),
      bias_provider_(bias_provider) {}

bool ArpaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;

  if (parsed->formatter == "TLL") {
    if (parsed->fields.size() < 5) return false;
    const int target_num = std::atoi(parsed->fields[0].c_str());
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    // Invariant #6: drop malformed / out-of-range lat/lon (parseDdmm
    // silently yields 0.0 on a parse failure) before projecting.
    if (!edge::isPlausibleLatLon(lat, lon)) return false;
    const Eigen::Vector3d enu = datum_.toEnu({lat, lon, 0.0});

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::ArpaTll;
    m.source_id = "arpa";
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    const double sigma = cfg_.position_std_m;
    m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
    m.hints.sensor_track_id = target_num;
    buffer_.push_back(std::move(m));
    return true;
  }

  if (parsed->formatter == "TTM") {
    if (parsed->fields.size() < 10) return false;
    const auto own_opt = own_ship_.poseAtOrBefore(t);
    if (!own_opt) return false;
    const int target_num = std::atoi(parsed->fields[0].c_str());
    const double dist = std::strtod(parsed->fields[1].c_str(), nullptr);
    const double bearing = std::strtod(parsed->fields[2].c_str(), nullptr);
    const std::string bearing_units = parsed->fields[3];
    const std::string dist_units = parsed->fields[9];
    const double range_m = distanceToMeters(dist, dist_units);
    // Invariant #6: a strtod parse failure maps range/bearing to 0.0,
    // which would place a target at own-ship. Reject non-positive /
    // non-finite range and non-finite bearing.
    if (!edge::isPlausibleRange(range_m) || !edge::isFiniteValue(bearing))
      return false;
    double bearing_true_deg = bearing;
    if (bearing_units == "R") bearing_true_deg += own_opt->heading_true_deg;
    const double bearing_true_rad = bearing_true_deg * kDeg2Rad;

    const Eigen::Vector3d own_enu = datum_.toEnu({own_opt->lat_deg, own_opt->lon_deg, 0.0});
    const Eigen::Vector2d own_xy(own_enu.x(), own_enu.y());

    HeadingBiasEstimate bias_est = bias_provider_
                                       ? bias_provider_->current()
                                       : HeadingBiasEstimate{};
    const double b_hat = bias_est.is_published ? bias_est.bias_rad : 0.0;
    const double var_b_hat =
        bias_est.is_published ? bias_est.variance_rad2 : 0.0;

    const double bearing_true_rad_corrected = bearing_true_rad - b_hat;

    const double sigma_heading_cfg = cfg_.heading_std_deg * kDeg2Rad;
    const double sigma_heading_eff =
        std::sqrt(sigma_heading_cfg * sigma_heading_cfg + var_b_hat);

    const double sigma_gps_pos = own_opt->position_std_m;
    const PointAndCov2D out =
        projectRangeBearingToEnu(range_m, bearing_true_rad_corrected,
                                 cfg_.position_std_m,
                                 cfg_.bearing_std_deg * kDeg2Rad,
                                 sigma_heading_eff,
                                 sigma_gps_pos,
                                 own_xy);

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::ArpaTtm;
    m.source_id = "arpa";
    m.model = MeasurementModel::Position2D;
    m.value = out.pos_enu;
    m.covariance = out.cov;
    m.hints.sensor_track_id = target_num;
    m.sensor_position_std_m = own_opt->position_std_m;

    // #20: TTM speed/course. This is the radar's OWN smoothed derivative of the
    // range/bearing detections we already feed, so it is NEVER a recurring
    // measurement (double-counting; guide §3). Two legitimate uses only:
    //   (a) a one-shot birth-velocity prior (used once at initiate, discarded);
    //   (b) a target-swap diagnostic (course jump ⇒ distrust sensor_track_id).
    // Fields (0-indexed after the formatter): [4]=speed, [5]=course,
    // [6]=course T/R, [9]=speed/distance units (same K/N/S code as distance).
    const double speed_raw = std::strtod(parsed->fields[4].c_str(), nullptr);
    const double course_raw = std::strtod(parsed->fields[5].c_str(), nullptr);
    const std::string course_units = parsed->fields[6];
    const bool speed_ok = edge::isFiniteValue(speed_raw) && speed_raw >= 0.0;
    const bool course_ok = edge::isFiniteValue(course_raw) &&
                           course_raw >= 0.0 && course_raw < 360.0;
    if (speed_ok && course_ok) {
      const double speed_mps = speedToMps(speed_raw, dist_units);
      // Course to TRUE (relative course is water/ground-stabilised relative to
      // own heading — mirror the bearing convention; deployment must confirm the
      // radar's stabilisation mode, ground vs water, per the #20 note).
      double course_true_deg = course_raw;
      if (course_units == "R") course_true_deg += own_opt->heading_true_deg;

      if (cfg_.seed_birth_velocity_from_ttm && speed_mps > 0.0) {
        const double cr = course_true_deg * kDeg2Rad;  // true, CW from north
        m.hints.birth_velocity_enu =
            Eigen::Vector2d(speed_mps * std::sin(cr),   // east
                            speed_mps * std::cos(cr));  // north
      }
      // Swap diagnostic: a discontinuous course jump for the same target number
      // (while moving) is the target-number-reuse signature.
      if (cfg_.swap_course_jump_deg > 0.0 && speed_mps >= cfg_.swap_min_speed_mps) {
        auto it = last_ttm_course_deg_.find(target_num);
        if (it != last_ttm_course_deg_.end() &&
            courseDeltaDeg(course_true_deg, it->second) > cfg_.swap_course_jump_deg)
          m.hints.sensor_track_id_suspect = true;
        last_ttm_course_deg_[target_num] = course_true_deg;
      }
    }

    buffer_.push_back(std::move(m));
    return true;
  }

  return false;
}

std::vector<Measurement> ArpaAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
