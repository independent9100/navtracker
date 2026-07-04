#pragma once

#include <Eigen/Core>

#include "core/geo/Wgs84.hpp"

namespace navtracker::geo {

/**
 * Local East-North-Up tangent plane about a fixed geodetic origin.
 * The common working frame for the fusion engine.
 */
class Datum {
 public:
  explicit Datum(const Geodetic& origin);

  /** Project a geodetic coordinate into this datum's ENU frame (meters). */
  Eigen::Vector3d toEnu(const Geodetic& g) const;
  /** Inverse of toEnu: map an ENU vector (meters) back to geodetic. */
  Geodetic toGeodetic(const Eigen::Vector3d& enu) const;

  /** The geodetic origin this tangent plane is anchored at. */
  const Geodetic& origin() const { return origin_; }

 private:
  Geodetic origin_;
  Eigen::Vector3d origin_ecef_;
  Eigen::Matrix3d ecef_to_enu_;  // rotation R
};

}  // namespace navtracker::geo
