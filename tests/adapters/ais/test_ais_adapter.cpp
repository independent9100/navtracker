#include <limits>

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

TEST(AisAdapter, DropsSentinelAndNaNPositions) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  // AIS "position not available" sentinel (lat 91, lon 181).
  AisDynamicReport sentinel;
  sentinel.time = Timestamp::fromSeconds(1.0);
  sentinel.lat_deg = 91.0;
  sentinel.lon_deg = 181.0;
  adapter.ingest(sentinel);

  // NaN position.
  AisDynamicReport nan_fix;
  nan_fix.time = Timestamp::fromSeconds(2.0);
  nan_fix.lat_deg = std::numeric_limits<double>::quiet_NaN();
  nan_fix.lon_deg = 8.0;
  adapter.ingest(nan_fix);

  EXPECT_TRUE(adapter.poll().empty());

  // A valid fix interleaved still gets through.
  AisDynamicReport good;
  good.time = Timestamp::fromSeconds(3.0);
  good.lat_deg = 53.5;
  good.lon_deg = 8.0;
  adapter.ingest(good);
  EXPECT_EQ(adapter.poll().size(), 1u);
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
