#include "core/output/BearingWedgeOutput.hpp"

#include <cmath>

#include "core/static/BearingWedgeModel.hpp"  // BearingWedge

namespace navtracker {
namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

// Internal math angle (CCW from east) → true bearing (CW from north), [0, 360).
double mathToTrueDeg(double math_rad) {
  double t = 90.0 - math_rad * kRad2Deg;  // β = 90° − α
  t = std::fmod(t, 360.0);
  if (t < 0.0) t += 360.0;
  return t;
}
}  // namespace

BearingWedgeOutput toBearingWedgeOutput(const BearingWedge& wedge,
                                        const geo::Datum& anchor) {
  const geo::Geodetic g = anchor.toGeodetic(
      Eigen::Vector3d(wedge.apex_enu.x(), wedge.apex_enu.y(), 0.0));
  BearingWedgeOutput out;
  out.hazard_id = wedge.wedge_id;
  out.apex_lat_deg = g.lat_deg;
  out.apex_lon_deg = g.lon_deg;
  out.bearing_true_deg = mathToTrueDeg(wedge.bearing_math_rad);
  out.half_width_deg = wedge.half_width_rad * kRad2Deg;
  out.max_range_m = wedge.max_range_m;
  out.is_charted = false;
  out.source_id = wedge.source_id;
  return out;
}

}  // namespace navtracker
