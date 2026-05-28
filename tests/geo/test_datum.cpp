#include <gtest/gtest.h>
#include "core/geo/Datum.hpp"

using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

TEST(Datum, OriginMapsToZero) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.5, 8.0, 0.0});
  EXPECT_NEAR(enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(enu.y(), 0.0, 1e-6);
  EXPECT_NEAR(enu.z(), 0.0, 1e-6);
}

TEST(Datum, NorthwardPointHasPositiveNorthZeroEast) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.51, 8.0, 0.0});  // ~0.01 deg north
  EXPECT_NEAR(enu.x(), 0.0, 1e-3);          // east ~ 0
  EXPECT_GT(enu.y(), 1000.0);               // ~1.1 km north
  EXPECT_LT(enu.y(), 1200.0);
}

TEST(Datum, EastwardPointHasPositiveEast) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.5, 8.01, 0.0});  // ~0.01 deg east
  EXPECT_GT(enu.x(), 500.0);
  EXPECT_NEAR(enu.y(), 0.0, 1.0);
}

TEST(Datum, RoundTripEnuGeodetic) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu(1234.0, -5678.0, 12.0);
  const Geodetic g = datum.toGeodetic(enu);
  const Eigen::Vector3d back = datum.toEnu(g);
  EXPECT_NEAR(back.x(), enu.x(), 1e-4);
  EXPECT_NEAR(back.y(), enu.y(), 1e-4);
  EXPECT_NEAR(back.z(), enu.z(), 1e-4);
}
