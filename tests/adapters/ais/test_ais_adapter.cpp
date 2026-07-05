#include <limits>

#include <Eigen/Dense>  // Matrix2d::determinant() for the velocity-block check
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

// #20: SOG/COG become PositionVelocity2D measurement content (AIS is an
// independent witness). COG true (clockwise from north) → ENU velocity.
TEST(AisAdapter, EmitsPositionVelocityFromSogCogAboveThreshold) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.sog_knots = 10.0;  // ~5.14 m/s
  r.cog_deg = 90.0;    // due east (true) → v_east = SOG, v_north ≈ 0
  adapter.ingest(r);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, navtracker::MeasurementModel::PositionVelocity2D);
  ASSERT_EQ(out[0].value.size(), 4);
  const double sog_mps = 10.0 * 0.514444;
  EXPECT_NEAR(out[0].value(2), sog_mps, 1e-6);  // v_east
  EXPECT_NEAR(out[0].value(3), 0.0, 1e-6);      // v_north
  // 4x4 R; the velocity block is non-degenerate thanks to the isotropic floor.
  ASSERT_EQ(out[0].covariance.rows(), 4);
  const Eigen::Matrix2d cov_v = out[0].covariance.bottomRightCorner<2, 2>();
  EXPECT_GT(cov_v(0, 0), 0.0);
  EXPECT_GT(cov_v(1, 1), 0.0);
  EXPECT_GT(cov_v.determinant(), 0.0) << "velocity block must not be rank-1";
  // Position block unchanged (standard-accuracy 30 m).
  EXPECT_DOUBLE_EQ(out[0].covariance(0, 0), 900.0);
}

// Near-stationary target: COG is meaningless, so no velocity content.
TEST(AisAdapter, LowSogFallsBackToPosition2D) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.sog_knots = 0.2;  // ~0.1 m/s, below the 0.5 m/s threshold
  r.cog_deg = 123.0;
  adapter.ingest(r);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, navtracker::MeasurementModel::Position2D)
      << "low-SOG COG must be down-weighted → Position2D";
  EXPECT_EQ(out[0].value.size(), 2);
}

// A consumer that distrusts AIS velocity can turn it off.
TEST(AisAdapter, VelocityEmissionCanBeDisabled) {
  Datum datum({53.5, 8.0, 0.0});
  navtracker::AisAdapterConfig cfg;
  cfg.emit_velocity_from_sog_cog = false;
  AisAdapter adapter(datum, cfg);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.sog_knots = 10.0;
  r.cog_deg = 90.0;
  adapter.ingest(r);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, navtracker::MeasurementModel::Position2D);
}

// #20: AIS self-report heading + nav-status flow onto the measurement hints,
// with AIS "not available" sentinels dropped at the edge (invariant #6).
TEST(AisAdapter, ParsesHeadingAndNavStatusAndDropsSentinels) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.heading_deg = 47.0;
  r.nav_status = std::uint8_t{1};  // at anchor
  adapter.ingest(r);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  ASSERT_TRUE(out[0].hints.heading_deg.has_value());
  EXPECT_DOUBLE_EQ(*out[0].hints.heading_deg, 47.0);
  ASSERT_TRUE(out[0].hints.nav_status.has_value());
  EXPECT_EQ(*out[0].hints.nav_status, 1u);

  // Sentinels: heading 511 = "not available", nav-status 15 = "undefined".
  AisDynamicReport s = r;
  s.heading_deg = 511.0;
  s.nav_status = std::uint8_t{15};
  adapter.ingest(s);
  auto out2 = adapter.poll();
  ASSERT_EQ(out2.size(), 1u);
  EXPECT_FALSE(out2[0].hints.heading_deg.has_value())
      << "511 heading sentinel must be dropped, not surfaced as a bearing";
  EXPECT_FALSE(out2[0].hints.nav_status.has_value())
      << "nav-status 15 (undefined) must be dropped";

  // A report that carries neither leaves both hints empty.
  AisDynamicReport bare = r;
  bare.heading_deg.reset();
  bare.nav_status.reset();
  adapter.ingest(bare);
  auto out3 = adapter.poll();
  ASSERT_EQ(out3.size(), 1u);
  EXPECT_FALSE(out3[0].hints.heading_deg.has_value());
  EXPECT_FALSE(out3[0].hints.nav_status.has_value());
}
