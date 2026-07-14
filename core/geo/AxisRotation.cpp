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
  // W2.2: wrap Δlongitude into [−180°, 180°] so a recenter that crosses the
  // ±180° antimeridian uses the SMALL true Δλ, not the ~±360° raw difference
  // (which would otherwise fabricate a huge, wildly wrong rotation).
  const double delta_lon_rad =
      std::remainder(n.lon_deg - o.lon_deg, 360.0) * kDeg2Rad;
  const double mean_lat_rad = 0.5 * (o.lat_deg + n.lat_deg) * kDeg2Rad;
  // W2.3: the rotation that re-expresses a vector from the OLD datum's ENU axes
  // into the NEW datum's ENU axes is by −γ, where γ = Δλ·sin(φ_mean) is the
  // meridian convergence angle. This equals the E,N block of the exact map
  // R_new·R_oldᵀ (verified to first order). The previous code applied +γ, which
  // rotated velocity / covariance / IMM means / particles the WRONG way — worse
  // than not rotating at all. Sign convention documented in AxisRotation.hpp.
  const double gamma = -delta_lon_rad * std::sin(mean_lat_rad);
  const double c = std::cos(gamma), s = std::sin(gamma);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

}  // namespace navtracker::geo
