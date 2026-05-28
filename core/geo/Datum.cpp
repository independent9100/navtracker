#include "core/geo/Datum.hpp"

#include <cmath>

namespace navtracker::geo {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double d) { return d * kPi / 180.0; }
}  // namespace

Datum::Datum(const Geodetic& origin) : origin_(origin) {
  origin_ecef_ = geodeticToEcef(origin);
  const double lat = deg2rad(origin.lat_deg);
  const double lon = deg2rad(origin.lon_deg);
  const double s_lat = std::sin(lat);
  const double c_lat = std::cos(lat);
  const double s_lon = std::sin(lon);
  const double c_lon = std::cos(lon);
  ecef_to_enu_ << -s_lon,          c_lon,         0.0,
                  -s_lat * c_lon,  -s_lat * s_lon, c_lat,
                   c_lat * c_lon,   c_lat * s_lon, s_lat;
}

Eigen::Vector3d Datum::toEnu(const Geodetic& g) const {
  return ecef_to_enu_ * (geodeticToEcef(g) - origin_ecef_);
}

Geodetic Datum::toGeodetic(const Eigen::Vector3d& enu) const {
  const Eigen::Vector3d ecef = ecef_to_enu_.transpose() * enu + origin_ecef_;
  return ecefToGeodetic(ecef);
}

}  // namespace navtracker::geo
