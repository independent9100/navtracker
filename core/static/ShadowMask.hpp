#pragma once
//
// LOS/shadow-guard geometry for coverage-aware occupancy decay.
//
// Coverage-aware decay (LiveOccupancyModel::observe) forgets a cell only when it
// was OBSERVABLE this scan — inside some bundle's swept coverage sector. The
// 2026-07-06 LOS/shadow probe (eval-log, verdict b) showed the gap: a cell can
// be geometrically inside the sector yet physically UNREACHABLE because a large
// vessel is between it and the sensor. Its returns truncate at the occluder, so
// "no return at the cell" is not evidence of emptiness — but decay counts it as
// observed-empty and erodes a real moored vessel's occupancy mass on every close
// passage.
//
// This is the pure geometry of the fix: from the returns a scanning sensor
// produced this scan, find the STRONG closer occluders (dense return clusters)
// and the angular wedge each shadows. A cell that falls in a wedge, beyond the
// occluder, has blocked line of sight and must NOT be decayed.
//
// Math / conventions:
//   - All angles are ENU "math" bearings about the sensor: atan2(dy, dx), CCW
//     from +x, in radians, wrapped to (-pi, pi]. Same convention as
//     ISensorDetectionModel::CoverageSector.
//   - An occluder is a contiguous cluster of >= min_occluder_returns return
//     bearings (split at gaps > cluster_gap_rad, wraparound-aware). Its wedge
//     arc is the cluster's own angular extent widened by wedge_pad_rad (the
//     tolerance derives from the occluder, not a magic constant); its shadow
//     starts at the cluster's NEAREST return range (line of sight is blocked
//     beyond the closest occluding surface).
//   - A point is shadowed by a wedge iff its bearing is within the wedge arc AND
//     its range exceeds occluder_range + range_margin_m. Equal/closer range is
//     never shadowed (a point at or before the occluder still has line of sight).
//
// Assumptions:
//   - Returns and the query point are in the SAME ENU frame as `sensor` (the
//     caller re-expresses everything in one frame; the occupancy model uses the
//     grid's fixed anchor frame). A single fixed datum makes this exact.
//   - "Strong" is measured by return COUNT, not amplitude: raw n_cells/amp does
//     not survive to the occupancy feed, but a large vessel produces a dense
//     bearing cluster. Under-detecting an occluder (too-high floor) simply leaves
//     the standing decay behaviour; over-detecting only HOLDS a cell's mass
//     longer — both are the safe direction (never a spurious decay).
//   - Occluders and the shadowed cell are seen in the SAME scan/burst: a burst
//     that sweeps the occluder's bearing is exactly the burst whose LOS to the
//     cell behind it is blocked.
//
// Pure: no I/O, no state, deterministic.
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace navtracker {

// Per-instance tuning for the LOS/shadow guard (no globals — threaded through
// LiveOccupancyParams). Disabled by default so a default-constructed occupancy
// model is byte-identical to pre-guard behaviour.
struct ShadowGuardParams {
  bool enabled = false;
  // A cluster must hold at least this many return bearings to count as a "strong"
  // occluder. Below it, the region behind is NOT shielded (a lone echo / small
  // craft must not freeze the map behind it).
  int min_occluder_returns = 6;
  // Bearing gap that separates distinct occluders within one burst (returns more
  // than this apart are different echo clusters that merely share a timestamp).
  double cluster_gap_rad = 0.105;  // ~6 deg
  // Widen each occluder's own angular extent by this, covering beam/edge slop.
  double wedge_pad_rad = 0.035;  // ~2 deg
  // A cell must lie at least this far BEYOND the occluder's nearest surface to be
  // shadowed — guards the occluder's own far side from being clipped.
  double range_margin_m = 50.0;
};

// One shadow wedge cast by a strong occluder: an angular arc
// [center - half_width, center + half_width] about the sensor, blocked beyond
// occluder_range_m.
struct ShadowWedge {
  double center_rad = 0.0;
  double half_width_rad = 0.0;
  double occluder_range_m = 0.0;  // nearest occluding surface; shadow starts past it
};

namespace shadow_detail {
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kPi = 3.14159265358979323846;
}  // namespace shadow_detail

// Build the shadow wedges cast by the strong occluders among `returns`
// (ENU points) about `sensor`. Empty when disabled, no returns, or no cluster
// reaches the strength floor.
inline std::vector<ShadowWedge> computeShadowWedges(
    const Eigen::Vector2d& sensor,
    const std::vector<Eigen::Vector2d>& returns, const ShadowGuardParams& p) {
  using namespace shadow_detail;
  std::vector<ShadowWedge> out;
  if (!p.enabled || returns.empty()) return out;

  // (bearing, range) per return, sorted by bearing.
  std::vector<std::pair<double, double>> br;
  br.reserve(returns.size());
  for (const auto& q : returns) {
    const Eigen::Vector2d d = q - sensor;
    if (d.squaredNorm() <= 0.0) continue;  // a return at the sensor has no bearing
    br.emplace_back(std::atan2(d.y(), d.x()), d.norm());
  }
  const std::size_t n = br.size();
  if (n == 0) return out;
  std::sort(br.begin(), br.end(),
            [](const std::pair<double, double>& a,
               const std::pair<double, double>& b) {
              return a.first < b.first;
            });

  // Cut the circle at the LARGEST bearing gap so no cluster wraps, then walk CCW,
  // unwrapping bearings into a monotone-increasing sequence (mirrors
  // CoverageSector::fromReturns so wraparound is handled once, consistently).
  std::size_t gmax_i = 0;
  double gmax = -1.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double g = (i + 1 < n) ? br[i + 1].first - br[i].first
                                 : br[0].first + kTwoPi - br[n - 1].first;
    if (g > gmax) {
      gmax = g;
      gmax_i = i;
    }
  }
  std::vector<double> az(n), rng(n);
  for (std::size_t s = 0; s < n; ++s) {
    const std::size_t idx = (gmax_i + 1 + s) % n;
    double a = br[idx].first;
    if (s > 0)
      while (a < az[s - 1]) a += kTwoPi;  // unwrap: monotone increasing
    az[s] = a;
    rng[s] = br[idx].second;
  }

  // Split into contiguous clusters (internal gap > cluster_gap_rad); each cluster
  // with >= min_occluder_returns members casts a wedge.
  std::size_t lo = 0;
  for (std::size_t i = 1; i <= n; ++i) {
    const bool split = (i == n) || (az[i] - az[i - 1] > p.cluster_gap_rad);
    if (!split) continue;
    const std::size_t cnt = i - lo;
    if (static_cast<int>(cnt) >= p.min_occluder_returns) {
      const double amin = az[lo], amax = az[i - 1];
      double rnear = rng[lo];
      for (std::size_t k = lo; k < i; ++k) rnear = std::min(rnear, rng[k]);
      ShadowWedge w;
      // Wrap the center back to (-pi, pi]; isShadowed uses a wrapped angular
      // difference so the exact representative does not matter, but keep it tidy.
      w.center_rad = std::remainder(0.5 * (amin + amax), kTwoPi);
      w.half_width_rad = std::min(0.5 * (amax - amin) + p.wedge_pad_rad, kPi);
      w.occluder_range_m = rnear;
      out.push_back(w);
    }
    lo = i;
  }
  return out;
}

// Is `point` (ENU) shadowed by any wedge about `sensor` — bearing within the
// wedge arc and range beyond occluder + margin?
inline bool isShadowed(const Eigen::Vector2d& sensor,
                       const Eigen::Vector2d& point,
                       const std::vector<ShadowWedge>& wedges,
                       double range_margin_m) {
  using namespace shadow_detail;
  const Eigen::Vector2d d = point - sensor;
  const double range = d.norm();
  if (range <= 0.0) return false;
  const double bearing = std::atan2(d.y(), d.x());
  for (const auto& w : wedges) {
    if (range <= w.occluder_range_m + range_margin_m) continue;
    const double off = std::remainder(bearing - w.center_rad, kTwoPi);
    if (std::abs(off) <= w.half_width_rad) return true;
  }
  return false;
}

}  // namespace navtracker
