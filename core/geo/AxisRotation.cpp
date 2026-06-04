#include "core/geo/AxisRotation.hpp"

#include <cmath>

namespace navtracker::geo {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

Eigen::Matrix2d datumAxisRotation(const Datum& old_datum,
                                  const Datum& new_datum) {
  const auto& o = old_datum.origin();
  const auto& n = new_datum.origin();
  const double delta_lon_rad = (n.lon_deg - o.lon_deg) * kDeg2Rad;
  const double mean_lat_rad = 0.5 * (o.lat_deg + n.lat_deg) * kDeg2Rad;
  const double gamma = delta_lon_rad * std::sin(mean_lat_rad);
  const double c = std::cos(gamma), s = std::sin(gamma);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

}  // namespace navtracker::geo
