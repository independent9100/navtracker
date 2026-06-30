#include "core/land/CoastlineGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegLat = 111320.0;

bool pointInRing(double lon, double lat, const std::vector<Eigen::Vector2d>& ring) {
  bool inside = false;
  const std::size_t n = ring.size();
  if (n < 3) return false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = ring[i].x(), yi = ring[i].y();
    const double xj = ring[j].x(), yj = ring[j].y();
    if (((yi > lat) != (yj > lat)) &&
        (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

// distance (m) from point (lon,lat) to segment a-b, in local equirectangular m.
double segDistM(double lon, double lat, const Eigen::Vector2d& a,
                const Eigen::Vector2d& b) {
  const double mlon = kMetersPerDegLat * std::cos(lat * kPi / 180.0);
  const double px = 0.0, py = 0.0;
  const double ax = (a.x() - lon) * mlon, ay = (a.y() - lat) * kMetersPerDegLat;
  const double bx = (b.x() - lon) * mlon, by = (b.y() - lat) * kMetersPerDegLat;
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = (len2 > 0.0) ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double cx = ax + t * dx, cy = ay + t * dy;
  return std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

double minEdgeDistM(double lon, double lat, const std::vector<Eigen::Vector2d>& ring) {
  double best = std::numeric_limits<double>::infinity();
  if (ring.size() < 2) return best;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    best = std::min(best, segDistM(lon, lat, ring[i], ring[i + 1]));
  }
  return best;
}

}  // namespace

CoastlineGeometry::CoastlineGeometry(std::vector<LandPolygon> polys,
                                     CoastlinePriorParams params)
    : polys_(std::move(polys)), params_(params) {}

double CoastlineGeometry::priorAtGeodetic(double lat_deg, double lon_deg) const {
  if (polys_.empty()) return 0.0;

  bool inside = false;
  double dist = std::numeric_limits<double>::infinity();
  for (const auto& p : polys_) {
    bool in_outer = pointInRing(lon_deg, lat_deg, p.outer);
    bool in_hole = false;
    for (const auto& h : p.holes)
      if (pointInRing(lon_deg, lat_deg, h)) { in_hole = true; break; }
    if (in_outer && !in_hole) inside = true;
    dist = std::min(dist, minEdgeDistM(lon_deg, lat_deg, p.outer));
    for (const auto& h : p.holes)
      dist = std::min(dist, minEdgeDistM(lon_deg, lat_deg, h));
  }

  const double d = inside ? -dist : dist;            // signed: <0 inland
  const double w = params_.offshore_halfwidth_m + params_.inland_halfwidth_m;
  if (w <= 0.0) return inside ? 1.0 : 0.0;
  const double c = (params_.offshore_halfwidth_m - d) / w;
  return std::clamp(c, 0.0, 1.0);
}

}  // namespace navtracker
