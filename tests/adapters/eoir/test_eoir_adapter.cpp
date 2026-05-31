#include <cmath>

#include <gtest/gtest.h>
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

using navtracker::CameraDetection;
using navtracker::EoIrAdapter;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;

TEST(EoIrAdapter, IngestProducesPositionAheadOfOwnShip) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow north
  provider.update(pose);

  Datum datum({53.5, 8.0, 0.0});
  EoIrAdapter adapter(datum, provider);

  CameraDetection d;
  d.time = Timestamp::fromSeconds(2.0);
  d.bearing_relative_deg = 0.0;
  d.range_m = 500.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 20.0;
  d.sensor_track_id = 7;
  adapter.ingest(d);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::EoIr);
  EXPECT_NEAR(out[0].value(0), 0.0, 1.0);
  EXPECT_NEAR(out[0].value(1), 500.0, 1.0);
  ASSERT_TRUE(out[0].hints.sensor_track_id.has_value());
  EXPECT_EQ(*out[0].hints.sensor_track_id, 7);
}
