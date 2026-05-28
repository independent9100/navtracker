#pragma once

#include <Eigen/Core>

namespace navtracker::geo {

// Geodetic coordinate on the WGS-84 ellipsoid. Degrees, degrees, meters.
struct Geodetic {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

// Earth-Centered Earth-Fixed position in meters, as an Eigen vector (X, Y, Z).
Eigen::Vector3d geodeticToEcef(const Geodetic& g);

// Inverse of geodeticToEcef using Bowring's method.
Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef);

}  // namespace navtracker::geo
