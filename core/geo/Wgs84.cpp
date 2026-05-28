#include "core/geo/Wgs84.hpp"

#include <cmath>

namespace navtracker::geo {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0;                 // semi-major axis (m)
constexpr double kF = 1.0 / 298.257223563;       // flattening
constexpr double kE2 = kF * (2.0 - kF);          // first eccentricity squared
constexpr double kB = kA * (1.0 - kF);           // semi-minor axis (m)
constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);  // second eccentricity squared

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }

}  // namespace

Eigen::Vector3d geodeticToEcef(const Geodetic& g) {
  const double lat = deg2rad(g.lat_deg);
  const double lon = deg2rad(g.lon_deg);
  const double s = std::sin(lat);
  const double c = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * s * s);
  const double x = (n + g.alt_m) * c * std::cos(lon);
  const double y = (n + g.alt_m) * c * std::sin(lon);
  const double z = (n * (1.0 - kE2) + g.alt_m) * s;
  return {x, y, z};
}

Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef) {
  const double x = ecef.x();
  const double y = ecef.y();
  const double z = ecef.z();

  const double lon = std::atan2(y, x);
  const double p = std::hypot(x, y);
  const double theta = std::atan2(z * kA, p * kB);
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double lat = std::atan2(z + kEp2 * kB * st * st * st,
                                p - kE2 * kA * ct * ct * ct);
  const double sl = std::sin(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
  const double alt = p / std::cos(lat) - n;

  return {rad2deg(lat), rad2deg(lon), alt};
}

}  // namespace navtracker::geo
