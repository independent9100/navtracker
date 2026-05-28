#include <gtest/gtest.h>
#include "core/types/Measurement.hpp"

using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;

TEST(Measurement, HoldsPosition2D) {
  Measurement m;
  m.time = Timestamp::fromSeconds(100.0);
  m.sensor = SensorKind::ArpaTll;
  m.source_id = "radar_fwd";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(1500.0, -300.0);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;

  EXPECT_EQ(m.dim(), 2);
  EXPECT_EQ(m.value.x(), 1500.0);
  EXPECT_EQ(m.covariance(0, 0), 25.0);
  EXPECT_FALSE(m.hints.mmsi.has_value());
}

TEST(Measurement, CarriesAssociationHints) {
  Measurement m;
  m.hints.mmsi = 211234560u;
  m.hints.sensor_track_id = 42;

  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 211234560u);
  ASSERT_TRUE(m.hints.sensor_track_id.has_value());
  EXPECT_EQ(*m.hints.sensor_track_id, 42);
}
