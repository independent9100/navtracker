#pragma once

#include <Eigen/Core>

#include "core/geo/Wgs84.hpp"

namespace navtracker::geo {

// Local East-North-Up tangent plane about a fixed geodetic origin.
// The common working frame for the fusion engine.
class Datum {
 public:
  explicit Datum(const Geodetic& origin);

  Eigen::Vector3d toEnu(const Geodetic& g) const;
  Geodetic toGeodetic(const Eigen::Vector3d& enu) const;

  const Geodetic& origin() const { return origin_; }

 private:
  Geodetic origin_;
  Eigen::Vector3d origin_ecef_;
  Eigen::Matrix3d ecef_to_enu_;  // rotation R
};

}  // namespace navtracker::geo
