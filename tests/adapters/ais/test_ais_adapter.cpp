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

// W5.6.1: the SOG gate must reject not just the 1023 "not available" sentinel
// but the whole physically-impossible band above the AIS valid maximum
// (102.2 kn, the 0.1-knot field's 1022). A raw or mis-scaled SOG in
// (102.2, 1023) kn would otherwise be multiplied into a ~100+ m/s velocity that
// yanks the estimator. The literal 1023 / 3600 not-available sentinels are
// already dropped (regression-pinned here, green both ways — the Section-D
// invariant the ticket names); the impossible band is the live teeth.
TEST(AisAdapter, ImpossibleSogBandAndSentinelsFallBackToPosition2D) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  auto pollOne = [&](double sog, double cog, double t) {
    AisDynamicReport r;
    r.time = Timestamp::fromSeconds(t);
    r.lat_deg = 53.5;
    r.lon_deg = 8.0;
    r.sog_knots = sog;
    r.cog_deg = cog;
    adapter.ingest(r);
    return adapter.poll();
  };
  // TEETH: impossible-band SOG (>102.2, <1023 kn) must NOT become velocity.
  const auto band = pollOne(200.0, 90.0, 0.0);  // ~102.9 m/s if wrongly accepted
  ASSERT_EQ(band.size(), 1u);
  EXPECT_EQ(band[0].model, navtracker::MeasurementModel::Position2D)
      << "SOG above 102.2 kn is physically impossible; must not be velocity";
  EXPECT_EQ(band[0].value.size(), 2);
  // Regression pins (green both ways): the literal not-available sentinels.
  const auto sog_na = pollOne(1023.0, 90.0, 1.0);  // SOG not available
  ASSERT_EQ(sog_na.size(), 1u);
  EXPECT_EQ(sog_na[0].model, navtracker::MeasurementModel::Position2D);
  const auto cog_na = pollOne(10.0, 3600.0, 2.0);  // COG not available, valid SOG
  ASSERT_EQ(cog_na.size(), 1u);
  EXPECT_EQ(cog_na[0].model, navtracker::MeasurementModel::Position2D)
      << "COG 3600 = not available → no course → no velocity content";
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

// #20 sub-item b: an anchored (nav_status 1) or moored (nav_status 5) vessel
// swinging within its watch circle can report SOG above the velocity threshold,
// but that swing is NOT a track velocity — feeding it destabilizes the
// (correctly near-static) track. Suppress it to Position2D even above threshold.
TEST(AisAdapter, AnchoredNavStatusSuppressesVelocity) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.sog_knots = 2.0;  // ~1.03 m/s, well above the 0.5 m/s threshold
  r.cog_deg = 90.0;
  r.nav_status = std::uint8_t{1};  // at anchor
  adapter.ingest(r);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, navtracker::MeasurementModel::Position2D)
      << "anchored (nav_status 1) must not emit watch-circle SOG as velocity";
  EXPECT_EQ(out[0].value.size(), 2);
  ASSERT_TRUE(out[0].hints.nav_status.has_value());  // still surfaced
  EXPECT_EQ(*out[0].hints.nav_status, 1u);
}

// The gate is narrow: nav_status 5 (moored) suppresses too, but an underway
// vessel (nav_status 0) above threshold still emits legitimate velocity.
TEST(AisAdapter, MooredSuppressesButUnderwayStillEmitsVelocity) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  AisDynamicReport moored;
  moored.time = Timestamp::fromSeconds(0.0);
  moored.lat_deg = 53.5;
  moored.lon_deg = 8.0;
  moored.sog_knots = 2.0;
  moored.cog_deg = 90.0;
  moored.nav_status = std::uint8_t{5};  // moored
  adapter.ingest(moored);
  auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, navtracker::MeasurementModel::Position2D);

  AisDynamicReport underway;
  underway.time = Timestamp::fromSeconds(1.0);
  underway.lat_deg = 53.5;
  underway.lon_deg = 8.0;
  underway.sog_knots = 2.0;
  underway.cog_deg = 90.0;
  underway.nav_status = std::uint8_t{0};  // underway using engine
  adapter.ingest(underway);
  auto out2 = adapter.poll();
  ASSERT_EQ(out2.size(), 1u);
  EXPECT_EQ(out2[0].model, navtracker::MeasurementModel::PositionVelocity2D)
      << "underway vessel above threshold still emits velocity";
}
