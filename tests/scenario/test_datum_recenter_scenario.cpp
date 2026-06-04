#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/tracking/DatumShift.hpp"
#include "core/tracking/TrackManager.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

namespace {
class TrackShifterSink : public IDatumChangeSink {
 public:
  TrackManager* mgr;
  explicit TrackShifterSink(TrackManager* m) : mgr(m) {}
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    shiftTracksOnDatumChange(*mgr, o, n);
  }
};
}  // namespace

TEST(DatumRecenterScenario, GeodeticPositionPreservedAcrossRecenter) {
  OwnShipProvider provider;
  TrackManager mgr(2, 3);
  TrackShifterSink sink(&mgr);
  provider.registerDatumSink(&sink);

  // Initial pose -> datum at (53.5, 8.0).
  OwnShipPose start;
  start.time = Timestamp::fromSeconds(0.0);
  start.lat_deg = 53.5;
  start.lon_deg = 8.0;
  provider.update(start);

  // Place a track at a known target lat/lon (a bit north-east).
  const double target_lat = 53.6, target_lon = 8.2;
  const auto enu_target = provider.datum().toEnu({target_lat, target_lon, 0.0});
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << enu_target.x(), enu_target.y(), 0.0, 0.0;
  t.covariance = Eigen::Matrix4d::Identity() * 25.0;
  mgr.add(t);

  // Push poses crossing the 30 km threshold so recenter fires.
  OwnShipPose far;
  far.time = Timestamp::fromSeconds(60.0);
  far.lat_deg = 53.5;
  far.lon_deg = 9.0;  // ~66 km east — well past 30 km
  provider.update(far);

  // Confirm datum moved to the far pose.
  const auto& new_datum = provider.datum();
  const auto enu_far_in_new = new_datum.toEnu({far.lat_deg, far.lon_deg, 0.0});
  EXPECT_NEAR(enu_far_in_new.x(), 0.0, 1e-3);

  // Verify the track's geodetic position is preserved.
  const Track& t_after = mgr.tracks()[0];
  const auto geo_after = new_datum.toGeodetic(
      Eigen::Vector3d(t_after.state(0), t_after.state(1), 0.0));
  EXPECT_NEAR(geo_after.lat_deg, target_lat, 1e-4);
  EXPECT_NEAR(geo_after.lon_deg, target_lon, 1e-4);
}

}  // namespace navtracker
