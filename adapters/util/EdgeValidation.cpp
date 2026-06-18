#include "adapters/util/EdgeValidation.hpp"

#include <cmath>

namespace navtracker::edge {

bool isPlausibleLatLon(double lat_deg, double lon_deg) {
  if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg)) return false;
  if (std::abs(lat_deg) > 90.0) return false;
  if (std::abs(lon_deg) > 180.0) return false;
  return true;
}

bool isPlausibleRange(double range_m) {
  return std::isfinite(range_m) && range_m > 0.0;
}

bool isFiniteValue(double v) { return std::isfinite(v); }

}  // namespace navtracker::edge
