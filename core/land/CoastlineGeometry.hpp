#pragma once

#include <vector>
#include <Eigen/Core>

namespace navtracker {

struct LandPolygon {
  std::vector<Eigen::Vector2d> outer;                       // (lon_deg, lat_deg)
  std::vector<std::vector<Eigen::Vector2d>> holes;          // each ring (lon,lat)
};

struct CoastlinePriorParams {
  double inland_halfwidth_m = 50.0;     // W_in: ramp reaches 1.0 this far inland
  double offshore_halfwidth_m = 50.0;   // W_off: ramp reaches 0.0 this far offshore
};

// Pure geometry: land polygons (geodetic) -> signed-distance shoreline ramp.
// Prior c(d) with d = signed distance to nearest shore edge (d<0 inland):
//   c = clamp((W_off - d) / (W_off + W_in), 0, 1)
// => c=1 for d<=-W_in (plateau, hard-gate region), ~0.5 at d=0 (waterline),
//    c=0 for d>=+W_off (open water). Distance computed in local-equirectangular
//    metres about the query point.
class CoastlineGeometry {
 public:
  CoastlineGeometry() = default;
  CoastlineGeometry(std::vector<LandPolygon> polys, CoastlinePriorParams params);

  double priorAtGeodetic(double lat_deg, double lon_deg) const;
  bool empty() const { return polys_.empty(); }

 private:
  std::vector<LandPolygon> polys_;
  CoastlinePriorParams params_{};
};

}  // namespace navtracker
