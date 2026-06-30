#include <gtest/gtest.h>
#include "core/land/CoastlineModel.hpp"
#include "core/geo/Datum.hpp"

using navtracker::CoastlineGeometry;
using navtracker::CoastlineModel;
using navtracker::CoastlinePriorParams;
using navtracker::LandPolygon;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {
CoastlineGeometry island() {
  LandPolygon p;
  p.outer = { {-71.06,42.36}, {-71.04,42.36}, {-71.04,42.38}, {-71.06,42.38}, {-71.06,42.36} };
  return CoastlineGeometry({p}, CoastlinePriorParams{});
}
}  // namespace

TEST(CoastlineModel, QueryConvertsEnuToGeodeticPrior) {
  Datum d{Geodetic{42.37, -71.05, 0.0}};            // datum inside the island
  CoastlineModel m{island(), d};
  // ENU origin == datum origin == deep inland -> ~1.0
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 1.0, 1e-6);
  // 5 km east -> far open water -> 0
  EXPECT_NEAR(m.clutterPrior({5000.0, 0.0}), 0.0, 1e-6);
}

TEST(CoastlineModel, DatumRecenterSwapsQueryFrameKeepsGeographicPrior) {
  Datum d0{Geodetic{42.37, -71.05, 0.0}};
  CoastlineModel m{island(), d0};
  const double before = m.clutterPrior({0.0, 0.0});   // origin = inland point
  // Recenter datum to a point 10 km away; the SAME geographic inland point is
  // now at a non-zero ENU; querying that ENU must still give the inland prior.
  Datum d1{Geodetic{42.46, -71.05, 0.0}};             // ~10 km north
  Eigen::Vector3d enu_of_old_origin = d1.toEnu(Geodetic{42.37, -71.05, 0.0});
  m.onDatumRecentered(d0, d1);
  const double after = m.clutterPrior({enu_of_old_origin.x(), enu_of_old_origin.y()});
  EXPECT_NEAR(before, after, 1e-6);
  // And the new ENU origin (10 km north, open water) is now 0.
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 0.0, 1e-6);
}

TEST(CoastlineModel, SetCoastlineSwapsGeometry) {
  Datum d{Geodetic{42.37, -71.05, 0.0}};
  CoastlineModel m{CoastlineGeometry{}, d};           // empty -> all water
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 0.0, 1e-9);
  m.setCoastline(island());
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 1.0, 1e-6);
}
