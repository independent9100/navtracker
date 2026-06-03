#include "adapters/arpa/ArpaAdapter.hpp"

#include <cmath>
#include <cstdlib>
#include <utility>

#include "adapters/util/Nmea.hpp"
#include "adapters/util/Projection.hpp"

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
    const Eigen::Vector3d enu = datum_.toEnu({lat, lon, 0.0});

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::ArpaTll;
    m.source_id = "arpa";
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    const double sigma = 50.0;
    m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
    m.hints.sensor_track_id = target_num;
    buffer_.push_back(std::move(m));
    return true;
  }

  if (parsed->formatter == "TTM") {
    if (parsed->fields.size() < 10) return false;
    const auto own_opt = own_ship_.latest();
    if (!own_opt) return false;
    const int target_num = std::atoi(parsed->fields[0].c_str());
    const double dist = std::strtod(parsed->fields[1].c_str(), nullptr);
    const double bearing = std::strtod(parsed->fields[2].c_str(), nullptr);
    const std::string bearing_units = parsed->fields[3];
    const std::string dist_units = parsed->fields[9];
    const double range_m = distanceToMeters(dist, dist_units);
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
                                 50.0, 1.0 * kDeg2Rad,
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
