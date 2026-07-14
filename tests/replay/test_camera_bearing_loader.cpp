// Unit + smoke tests for the philos camera-bearing replay loader.
//
// Unit tests exercise parsing, heading composition, the atan2(dN,dE) value
// convention, wrap handling, the drop-on-no-pose rule, invalid-row rejection,
// and per-camera source ids — against small temp CSVs (no fixtures needed).
//
// The smoke test (skip-guarded on the gitignored fixture) feeds ais_ferry_near
// radar plots + camera bearings through the v2 Tracker and asserts the two
// MECHANICS properties the ticket requires — a Bearing2D update lands on a
// (radar-born) track, and no track is ever initiated by a camera measurement.
// No accuracy assertions (circularity rule: camera detections were derived
// from the same videos as the existence labels).
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include "adapters/replay/CameraBearingCsvReader.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/MeasurementModels.hpp"  // canInitiateTrack, wrapAngle
#include "core/own_ship/OwnShipProvider.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "ports/IBearingInnovationSink.hpp"

using namespace navtracker;
using navtracker::replay::CameraBearingLoadStats;
using navtracker::replay::loadCameraBearingsCsv;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

std::string writeTemp(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + "/" + name;
  std::ofstream f(path);
  f << body;
  return path;
}

// Provider with a single known pose established at t0 (datum origin = pose).
OwnShipProvider providerWithPose(double t0, double lat, double lon,
                                 double heading_deg) {
  OwnShipProvider p(8);
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(t0);
  pose.lat_deg = lat;
  pose.lon_deg = lon;
  pose.heading_true_deg = heading_deg;
  p.update(pose);
  return p;
}

const char* kHeader =
    "unix_time,camera,bearing_rel_deg,sigma_deg,confidence,u_px,v_px,w_px,h_px,frame\n";

}  // namespace

TEST(CameraBearingLoader, HeadingCompositionAndValueConvention) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 240.0);
  const std::string csv = writeTemp("cam_basic.csv",
      std::string(kHeader) +
      "100.000000,center,6.0000,2.0000,0.9,780,357,300,150,f.jpg\n");

  CameraBearingLoadStats st;
  auto ms = loadCameraBearingsCsv(csv, prov, "philos_cam", &st);
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_EQ(st.emitted, 1u);
  const auto& m = ms.front();
  EXPECT_EQ(m.model, MeasurementModel::Bearing2D);
  EXPECT_EQ(m.sensor, SensorKind::EoIr);
  EXPECT_EQ(m.source_id, "philos_cam_center");
  ASSERT_EQ(m.value.size(), 1);

  // marine_true = 240 + 6 = 246 deg; value(0) = atan2(dN, dE).
  const double marine = 246.0 * kDeg2Rad;
  const double expected = wrapAngle(kPi / 2.0 - marine);
  EXPECT_NEAR(m.value(0), expected, 1e-9);
  // Same as atan2(cos(marine), sin(marine)).
  EXPECT_NEAR(m.value(0), std::atan2(std::cos(marine), std::sin(marine)), 1e-9);

  // R(0,0) = (2 deg)^2 in rad^2.
  ASSERT_EQ(m.covariance.rows(), 1);
  EXPECT_NEAR(m.covariance(0, 0), std::pow(2.0 * kDeg2Rad, 2), 1e-15);

  // Sensor position is the own-ship ENU at the pose (datum origin) ~ (0,0).
  EXPECT_NEAR(m.sensor_position_enu.x(), 0.0, 1e-3);
  EXPECT_NEAR(m.sensor_position_enu.y(), 0.0, 1e-3);
}

TEST(CameraBearingLoader, WrapHandlingAcrossNorth) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 350.0);
  const std::string csv = writeTemp("cam_wrap.csv",
      std::string(kHeader) +
      "100.000000,center,30.0000,1.0000,0.9,700,357,10,10,f.jpg\n");
  auto ms = loadCameraBearingsCsv(csv, prov);
  ASSERT_EQ(ms.size(), 1u);
  // marine 350+30 = 380 == 20 deg. Value must be wrapped and equal the 20-deg
  // result.
  const double expected = wrapAngle(kPi / 2.0 - 20.0 * kDeg2Rad);
  EXPECT_NEAR(ms.front().value(0), expected, 1e-9);
  EXPECT_LE(ms.front().value(0), kPi);
  EXPECT_GT(ms.front().value(0), -kPi);
}

TEST(CameraBearingLoader, DropsRowWithNoPose) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 240.0);
  // Row at t=50 precedes the only pose (t=100) -> no pose at-or-before.
  const std::string csv = writeTemp("cam_nopose.csv",
      std::string(kHeader) +
      "50.000000,center,6.0000,2.0000,0.9,780,357,300,150,f.jpg\n"
      "100.000000,center,6.0000,2.0000,0.9,780,357,300,150,g.jpg\n");
  CameraBearingLoadStats st;
  auto ms = loadCameraBearingsCsv(csv, prov, "philos_cam", &st);
  EXPECT_EQ(ms.size(), 1u);
  EXPECT_EQ(st.rows_read, 2u);
  EXPECT_EQ(st.dropped_no_pose, 1u);
  EXPECT_EQ(st.emitted, 1u);
}

TEST(CameraBearingLoader, RejectsInvalidSigma) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 240.0);
  const std::string csv = writeTemp("cam_badsigma.csv",
      std::string(kHeader) +
      "100.000000,center,6.0000,0.0000,0.9,780,357,300,150,a.jpg\n"    // sigma=0
      "100.000000,center,6.0000,-1.0000,0.9,780,357,300,150,b.jpg\n"   // sigma<0
      "100.000000,center,6.0000,2.0000,0.9,780,357,300,150,c.jpg\n");  // ok
  CameraBearingLoadStats st;
  auto ms = loadCameraBearingsCsv(csv, prov, "philos_cam", &st);
  EXPECT_EQ(ms.size(), 1u);
  EXPECT_EQ(st.dropped_invalid, 2u);
}

TEST(CameraBearingLoader, PerCameraSourceIds) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 240.0);
  const std::string csv = writeTemp("cam_multi.csv",
      std::string(kHeader) +
      "100.000000,center,6.0000,2.0,0.9,780,357,300,150,a.jpg\n"
      "100.000000,left,-40.0000,3.0,0.9,100,357,50,40,b.jpg\n"
      "100.000000,right,45.0000,3.0,0.9,900,357,50,40,c.jpg\n");
  auto ms = loadCameraBearingsCsv(csv, prov, "philos_cam");
  ASSERT_EQ(ms.size(), 3u);
  std::vector<std::string> ids;
  for (const auto& m : ms) ids.push_back(m.source_id);
  EXPECT_NE(std::find(ids.begin(), ids.end(), "philos_cam_center"), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), "philos_cam_left"), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), "philos_cam_right"), ids.end());
}

TEST(CameraBearingLoader, LoadedMeasurementCannotInitiateTrack) {
  auto prov = providerWithPose(100.0, 42.371, -71.054, 240.0);
  const std::string csv = writeTemp("cam_init.csv",
      std::string(kHeader) +
      "100.000000,center,6.0000,2.0,0.9,780,357,300,150,a.jpg\n");
  auto ms = loadCameraBearingsCsv(csv, prov);
  ASSERT_EQ(ms.size(), 1u);
  EXPECT_FALSE(canInitiateTrack(ms.front().model));
}

// ---------------------------------------------------------------------------
// Skip-guarded replay smoke test on the real (gitignored) fixture.
// ---------------------------------------------------------------------------
namespace {

std::string srcDir() { return std::string(NAVTRACKER_SOURCE_DIR); }
std::string ferryDir() {
  return srcDir() + "/tests/fixtures/philos/out/ais_ferry_near";
}
bool fileExists(const std::string& p) {
  std::ifstream f(p);
  return static_cast<bool>(f);
}

class CountingBearingSink : public IBearingInnovationSink {
 public:
  std::vector<BearingInnovation> events;
  void onBearingInnovation(const BearingInnovation& obs) override {
    events.push_back(obs);
  }
};

}  // namespace

TEST(CameraBearingSmoke, CameraOnlyInitiatesNoTracks) {
  const std::string cam = ferryDir() + "/camera_bearings.csv";
  const std::string own = ferryDir() + "/ownship.csv";
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !fileExists(cam) || !fileExists(own),
      "camera_bearings.csv fixture absent (regenerate with "
      "extract_camera_bearings.py)");
  const auto poses = navtracker::replay::loadOwnshipCsv(own);
  ASSERT_FALSE(poses.empty());
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);

  auto bearings = loadCameraBearingsCsv(cam, provider, "philos_cam");
  ASSERT_FALSE(bearings.empty());

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(600.0);
  TrackManager mgr(1, 4);
  Tracker tracker(est, assoc, mgr, 60.0);

  for (const auto& m : bearings) tracker.process(m);

  // Bearing-only stream cannot birth a track (canInitiateTrack==false).
  EXPECT_TRUE(mgr.tracks().empty());
}

TEST(CameraBearingSmoke, RadarBornTrackReceivesBearingUpdate) {
  const std::string cam = ferryDir() + "/camera_bearings.csv";
  const std::string own = ferryDir() + "/ownship.csv";
  const std::string plots = ferryDir() + "/radar_plots.csv";
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !fileExists(cam) || !fileExists(own) || !fileExists(plots),
      "ais_ferry_near fixtures absent (regenerate)");
  const auto poses = navtracker::replay::loadOwnshipCsv(own);
  ASSERT_FALSE(poses.empty());
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);

  auto radar = navtracker::replay::loadPlotCsvBodyFrame(
      plots, provider, SensorKind::ArpaTtm, "philos_radar");
  auto bearings = loadCameraBearingsCsv(cam, provider, "philos_cam");
  ASSERT_FALSE(radar.empty());
  ASSERT_FALSE(bearings.empty());

  // Merge both streams in strict time order (Tracker rejects stale input).
  std::vector<Measurement> scan;
  scan.reserve(radar.size() + bearings.size());
  scan.insert(scan.end(), radar.begin(), radar.end());
  scan.insert(scan.end(), bearings.begin(), bearings.end());
  std::stable_sort(scan.begin(), scan.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time < b.time;
                   });

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(600.0);
  TrackManager mgr(1, 4);
  Tracker tracker(est, assoc, mgr, 60.0);
  CountingBearingSink sink;
  tracker.setBearingInnovationSink(&sink);

  for (const auto& m : scan) tracker.process(m);

  // At least one camera Bearing2D measurement gated into and updated a
  // radar-born track (the sink only fires on Bearing2D/RangeBearing2D updates;
  // radar here is Position2D, so every event is camera-driven).
  EXPECT_GT(sink.events.size(), 0u);
}
