#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(EndToEnd, MultiSensorMergesIntoSingleTrack) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.4955;  // ~500 m south of datum
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);

  AisAdapter ais(datum);
  ArpaAdapter arpa(datum, own);
  EoIrAdapter eo(datum, own);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator estimator(motion, 5.0);
  GnnAssociator associator(30.0);
  TrackManager manager(2, 3);
  Tracker tracker(estimator, associator, manager, 10.0);

  // AIS at t=0: lat 53.51 -> ~1.11 km north of datum origin.
  AisDynamicReport a;
  a.time = Timestamp::fromSeconds(0.0);
  a.mmsi = 200000001u;
  a.lat_deg = 53.51;
  a.lon_deg = 8.0;
  a.high_accuracy = true;
  ais.ingest(a);
  for (auto& m : ais.poll()) tracker.process(m);

  // ARPA TLL at t=1, same target (53.51, 8.0).
  arpa.ingest(navtracker_test::makeNmea("RATLL,1,5330.6,N,00800.0,E,T1,123456,T,R"),
              Timestamp::fromSeconds(1.0));
  for (auto& m : arpa.poll()) tracker.process(m);

  // EO/IR at t=2, bearing straight ahead, range ~1.6 km (target north of own-ship).
  CameraDetection d;
  d.time = Timestamp::fromSeconds(2.0);
  d.bearing_relative_deg = 0.0;
  d.range_m = 1600.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 50.0;
  eo.ingest(d);
  for (auto& m : eo.poll()) tracker.process(m);

  ASSERT_EQ(manager.size(), 1u);
  const Track& t = manager.tracks()[0];
  EXPECT_EQ(t.status, TrackStatus::Confirmed);
  EXPECT_GE(t.contributing_sources.size(), 2u);
}
