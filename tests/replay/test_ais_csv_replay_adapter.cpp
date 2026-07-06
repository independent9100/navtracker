// Unit tests for the replay AIS CSV loader's #20 velocity path.
//
// Covers: default-off byte-identical Position2D; opt-in PositionVelocity2D above
// the SOG threshold; low-SOG fallback; nav_status surfacing + sentinel drop;
// tolerance of missing sog/cog columns; and the load-bearing no-drift proof —
// loadAisCsv and the NMEA AisAdapter produce IDENTICAL velocity content for the
// same SOG/COG (they share core/estimation/PolarVelocity.hpp).
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Ids.hpp"

using namespace navtracker;
using navtracker::replay::loadAisCsv;

namespace {

constexpr double kKnotsToMps = 0.514444;

std::string writeTemp(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + "/" + name;
  std::ofstream f(path);
  f << body;
  return path;
}

geo::Datum testDatum() { return geo::Datum(geo::Geodetic{63.4, 10.4, 0.0}); }

// One vessel, moving due east at ~5.14 m/s (10 kn), nav_status underway.
const char* kCsv =
    "unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name\n"
    "0.000,257000001,63.4010,10.4000,5.144440,90.0,0,A\n";

}  // namespace

// Default OFF: even with sog/cog columns present, output is Position2D — the
// byte-identical historical behaviour every existing call site relies on.
TEST(AisCsvReplay, VelocityOffByDefaultIsPosition2D) {
  const std::string p = writeTemp("ais_off.csv", kCsv);
  const auto ms = loadAisCsv(p, testDatum(), "ais");  // emit_velocity defaulted
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_EQ(ms[0].model, MeasurementModel::Position2D);
  EXPECT_EQ(ms[0].value.size(), 2);
  EXPECT_EQ(ms[0].covariance.rows(), 2);
  EXPECT_DOUBLE_EQ(ms[0].covariance(0, 0), 900.0);  // 30 m default
  EXPECT_FALSE(ms[0].hints.nav_status.has_value());  // gated off with velocity
}

// ON + above threshold: PositionVelocity2D with the target-reported velocity.
TEST(AisCsvReplay, EmitsPositionVelocityWhenOnAndAboveThreshold) {
  const std::string p = writeTemp("ais_on.csv", kCsv);
  const auto ms = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_EQ(ms[0].model, MeasurementModel::PositionVelocity2D);
  ASSERT_EQ(ms[0].value.size(), 4);
  EXPECT_NEAR(ms[0].value(2), 5.144440, 1e-6);  // due east -> ve = SOG
  EXPECT_NEAR(ms[0].value(3), 0.0, 1e-6);        // vn ~ 0
  ASSERT_EQ(ms[0].covariance.rows(), 4);
  EXPECT_DOUBLE_EQ(ms[0].covariance(0, 0), 900.0);        // position block
  const Eigen::Matrix2d vcov = ms[0].covariance.bottomRightCorner<2, 2>();
  EXPECT_GT(vcov.determinant(), 0.0);  // isotropic floor => not rank-1
  EXPECT_EQ(ms[0].hints.nav_status.value_or(255), 0u);
}

// Below the SOG threshold COG is meaningless -> Position2D even when ON.
TEST(AisCsvReplay, LowSogFallsBackToPosition2D) {
  const std::string p = writeTemp("ais_lowsog.csv",
      "unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name\n"
      "0.000,257000001,63.4010,10.4000,0.200,90.0,0,A\n");
  const auto ms = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_EQ(ms[0].model, MeasurementModel::Position2D);
}

// nav_status 1 (at anchor) surfaces on hints; 15 (undefined) is dropped.
TEST(AisCsvReplay, NavStatusSurfacedAndSentinelDropped) {
  const std::string p = writeTemp("ais_nav.csv",
      "unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name\n"
      "0.000,257000001,63.4010,10.4000,0.100,90.0,1,ANCH\n"
      "1.000,257000002,63.4020,10.4000,0.100,90.0,15,UNK\n");
  const auto ms = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 2u);
  EXPECT_EQ(ms[0].hints.nav_status.value_or(255), 1u);   // at anchor kept
  EXPECT_FALSE(ms[1].hints.nav_status.has_value());       // 15 dropped
}

// Old fixtures without sog/cog stay valid when velocity is requested.
TEST(AisCsvReplay, ToleratesMissingSogCogColumns) {
  const std::string p = writeTemp("ais_nocols.csv",
      "unix_time,mmsi,lat,lon\n"
      "0.000,257000001,63.4010,10.4000\n");
  const auto ms = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_EQ(ms[0].model, MeasurementModel::Position2D);
}

// No-drift proof: for the SAME SOG/COG the replay loader and the NMEA
// AisAdapter emit IDENTICAL velocity content + covariance (shared helper).
TEST(AisCsvReplay, MatchesAisAdapterVelocityContent) {
  const auto datum = testDatum();
  const double sog_mps = 5.144440, cog_deg = 90.0, lat = 63.4010, lon = 10.4000;

  const std::string p = writeTemp("ais_match.csv",
      "unix_time,mmsi,lat,lon,sog_mps,cog_deg\n"
      "0.000,257000001,63.4010,10.4000,5.144440,90.0\n");
  const auto ms = loadAisCsv(p, datum, "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 1u);

  AisAdapter adapter(datum);  // default config: standard 30 m, same helper
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.mmsi = 257000001;
  r.lat_deg = lat;
  r.lon_deg = lon;
  r.sog_knots = sog_mps / kKnotsToMps;  // adapter takes knots
  r.cog_deg = cog_deg;
  adapter.ingest(r);
  const auto am = adapter.poll();
  ASSERT_EQ(am.size(), 1u);

  ASSERT_EQ(ms[0].model, MeasurementModel::PositionVelocity2D);
  ASSERT_EQ(am[0].model, MeasurementModel::PositionVelocity2D);
  // Velocity content + covariance must match to floating tolerance (the only
  // difference is the sog_mps<->knots round-trip on the adapter input).
  EXPECT_NEAR(ms[0].value(2), am[0].value(2), 1e-6);
  EXPECT_NEAR(ms[0].value(3), am[0].value(3), 1e-6);
  const Eigen::Matrix2d v_replay = ms[0].covariance.bottomRightCorner<2, 2>();
  const Eigen::Matrix2d v_nmea = am[0].covariance.bottomRightCorner<2, 2>();
  EXPECT_LT((v_replay - v_nmea).cwiseAbs().maxCoeff(), 1e-6);
  // Position blocks are both the 30 m default.
  EXPECT_DOUBLE_EQ(ms[0].covariance(0, 0), am[0].covariance(0, 0));
}

// #20 sub-item b: nav_status 1 (at anchor) / 5 (moored) forces Position2D even
// above the SOG threshold when velocity is ON — the watch-circle swing is not a
// track velocity. An underway vessel (nav_status 0) above threshold still emits
// it. Shares the AisAdapter gate via core/estimation/PolarVelocity.hpp.
TEST(AisCsvReplay, AnchoredNavStatusSuppressesVelocityWhenOn) {
  const std::string p = writeTemp("ais_anchored.csv",
      "unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name\n"
      "0.000,257000001,63.4010,10.4000,1.000,90.0,1,ANCH\n"    // at anchor
      "1.000,257000002,63.4020,10.4000,1.000,90.0,5,MOOR\n"    // moored
      "2.000,257000003,63.4030,10.4000,1.000,90.0,0,UNDER\n"); // underway
  const auto ms = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(ms.size(), 3u);
  EXPECT_EQ(ms[0].model, MeasurementModel::Position2D) << "anchor suppressed";
  EXPECT_EQ(ms[1].model, MeasurementModel::Position2D) << "moored suppressed";
  EXPECT_EQ(ms[2].model, MeasurementModel::PositionVelocity2D)
      << "underway above threshold still emits velocity";
  EXPECT_EQ(ms[0].hints.nav_status.value_or(255), 1u);  // still surfaced
}

// #20 sub-item b — the load-bearing invariant (arbiter ruling 2026-07-06): for
// an anchored vessel the gate makes velocity-ON KINEMATICALLY identical to
// velocity-OFF (same model/value/covariance), even on rows above the SOG
// threshold. So an anchored vessel cannot destabilize its OWN track via velocity
// — measurement-level inertness, stronger than any aggregate metric row. The
// only ON-vs-OFF difference is the surfaced nav_status hint (metadata for the
// veto path; it carries no kinematics).
TEST(AisCsvReplay, AnchoredVesselKinematicsIdenticalOnVsOff) {
  const std::string p = writeTemp("ais_anchor_identity.csv",
      "unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name\n"
      "0.000,257000001,63.4010,10.4000,0.510,64.0,1,ANCH\n"     // above threshold
      "1.000,257000001,63.4020,10.4010,0.590,351.6,1,ANCH\n");  // above threshold
  const auto off = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/false);
  const auto on = loadAisCsv(p, testDatum(), "ais", /*emit_velocity=*/true);
  ASSERT_EQ(off.size(), 2u);
  ASSERT_EQ(on.size(), 2u);
  for (std::size_t i = 0; i < off.size(); ++i) {
    EXPECT_EQ(on[i].model, MeasurementModel::Position2D);
    EXPECT_EQ(on[i].model, off[i].model);
    ASSERT_EQ(on[i].value.size(), off[i].value.size());
    EXPECT_TRUE(on[i].value.isApprox(off[i].value)) << "position content differs";
    EXPECT_TRUE(on[i].covariance.isApprox(off[i].covariance))
        << "covariance content differs";
  }
  // The one intended difference: ON surfaces the nav_status hint, OFF does not.
  EXPECT_EQ(on[0].hints.nav_status.value_or(255), 1u);
  EXPECT_FALSE(off[0].hints.nav_status.has_value());
}
