#include <gtest/gtest.h>
#include "core/geo/Wgs84.hpp"

using navtracker::geo::Geodetic;
using navtracker::geo::geodeticToEcef;
using navtracker::geo::ecefToGeodetic;

TEST(Wgs84, OriginOnEquatorPrimeMeridian) {
  const Eigen::Vector3d ecef = geodeticToEcef({0.0, 0.0, 0.0});
  EXPECT_NEAR(ecef.x(), 6378137.0, 1e-3);
  EXPECT_NEAR(ecef.y(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.z(), 0.0, 1e-3);
}

TEST(Wgs84, NorthPoleHitsSemiMinorAxis) {
  const Eigen::Vector3d ecef = geodeticToEcef({90.0, 0.0, 0.0});
  EXPECT_NEAR(ecef.x(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.y(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.z(), 6356752.314245, 1e-3);  // b = a(1-f)
}

TEST(Wgs84, RoundTripGeodetic) {
  const Geodetic g{53.5, 8.2, 25.0};
  const Geodetic back = ecefToGeodetic(geodeticToEcef(g));
  EXPECT_NEAR(back.lat_deg, g.lat_deg, 1e-9);
  EXPECT_NEAR(back.lon_deg, g.lon_deg, 1e-9);
  EXPECT_NEAR(back.alt_m, g.alt_m, 1e-6);
}
