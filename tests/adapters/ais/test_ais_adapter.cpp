#include <gtest/gtest.h>
#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"

using navtracker::AisAdapter;
using navtracker::AisDynamicReport;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;

TEST(AisAdapter, IngestProducesPosition2DAtOrigin) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(100.0);
  r.mmsi = 211000000u;
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.high_accuracy = true;
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  const Measurement& m = out[0];
  EXPECT_EQ(m.sensor, SensorKind::Ais);
  EXPECT_EQ(m.model, MeasurementModel::Position2D);
  EXPECT_NEAR(m.value(0), 0.0, 1e-6);
  EXPECT_NEAR(m.value(1), 0.0, 1e-6);
  EXPECT_EQ(m.covariance(0, 0), 100.0);
  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 211000000u);
  EXPECT_TRUE(adapter.poll().empty());
}

TEST(AisAdapter, LowAccuracyHasLargerCovariance) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.high_accuracy = false;
  adapter.ingest(r);
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].covariance(0, 0), 900.0);
}
