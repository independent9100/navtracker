// Unit tests for the AutoFerry JSON loader. The synthetic-fixture tests
// write tiny JSON files to a temp dir and pin the format mapping (2×M
// active points, NED→ENU axis swap, bearing convention, graceful-empty).
// A second test loads the real data/autoferry/scenario2 fixture if it is
// reachable, otherwise it skips.

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "adapters/replay/AutoferryJsonReplay.hpp"

namespace navtracker::replay {
namespace {

std::string tmpDir() {
  const char* t = std::getenv("TMPDIR");
  return (t && *t) ? std::string(t) : std::string("/tmp");
}

void writeFile(const std::string& path, const std::string& body) {
  std::ofstream f(path);
  f << body;
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

// Detections: one radar (2×1 flat), one lidar (2×2 nested), one EO bearing.
constexpr const char* kDetections = R"JSON(
[
  {"sensorID":2,"ownshipPosition":[10,20],"time":1.0,"measurement":[3,4]},
  {"sensorID":1,"ownshipPosition":[0,0],"time":2.0,"measurement":[[1,2],[3,4]]},
  {"sensorID":4,"ownshipPosition":[0,0],"time":3.0,"measurement":0.0}
]
)JSON";

// Ground truth: two scans, one target each.
constexpr const char* kGroundTruth = R"JSON(
[
  [{"targetID":1,"position":[100,200,0],"time":1.0}],
  [{"targetID":1,"position":[110,205,0],"time":2.0}]
]
)JSON";

std::string makeFixture(const std::string& label) {
  const std::string dir = tmpDir() + "/" + label;
  std::string mk = "mkdir -p '" + dir + "'";
  (void)std::system(mk.c_str());
  writeFile(dir + "/" + label + "_detections.json", kDetections);
  writeFile(dir + "/" + label + "_groundTruth.json", kGroundTruth);
  return dir;
}

TEST(AutoferryJsonReplay, ActiveSensorsMapToEnuPositions) {
  const std::string dir = makeFixture("af_test_active");
  const Scenario s = loadAutoferryScenario(dir, "af_test_active");

  // Default opts: bearings excluded → radar(1) + lidar(2) = 3 measurements.
  ASSERT_EQ(s.measurements.size(), 3u);
  for (const auto& m : s.measurements)
    EXPECT_EQ(m.model, MeasurementModel::Position2D);

  // Sorted by time: radar (t=1) first. ENU (E,N) = ownship(E,N)+(e,n)
  // where ownship=[N=10,E=20], meas=(n=3,e=4) → (E=24, N=13).
  EXPECT_DOUBLE_EQ(s.measurements[0].value(0), 24.0);
  EXPECT_DOUBLE_EQ(s.measurements[0].value(1), 13.0);
  EXPECT_EQ(s.measurements[0].sensor, SensorKind::ArpaTtm);

  // Lidar 2×M: norths=[1,2] easts=[3,4] → points (n,e)=(1,3),(2,4)
  // → ENU (E,N)=(3,1),(4,2).
  EXPECT_DOUBLE_EQ(s.measurements[1].value(0), 3.0);
  EXPECT_DOUBLE_EQ(s.measurements[1].value(1), 1.0);
  EXPECT_DOUBLE_EQ(s.measurements[2].value(0), 4.0);
  EXPECT_DOUBLE_EQ(s.measurements[2].value(1), 2.0);
  EXPECT_EQ(s.measurements[1].sensor, SensorKind::Lidar);

  // Active sensors carry the ownship ENU position as sensor position so
  // the MHT miss branch can range-condition per-sensor P_D (a lidar with
  // ~140 m coverage must not penalise a track at 500 m). Radar ownship
  // NED [10, 20] → ENU (20, 10).
  EXPECT_DOUBLE_EQ(s.measurements[0].sensor_position_enu(0), 20.0);
  EXPECT_DOUBLE_EQ(s.measurements[0].sensor_position_enu(1), 10.0);
  EXPECT_DOUBLE_EQ(s.measurements[1].sensor_position_enu(0), 0.0);
}

TEST(AutoferryJsonReplay, GroundTruthSwapsNedToEnu) {
  const std::string dir = makeFixture("af_test_gt");
  const Scenario s = loadAutoferryScenario(dir, "af_test_gt");
  ASSERT_EQ(s.truth.size(), 2u);
  // GT position NED [N=100,E=200] → ENU (E=200,N=100).
  EXPECT_DOUBLE_EQ(s.truth[0].position(0), 200.0);
  EXPECT_DOUBLE_EQ(s.truth[0].position(1), 100.0);
  EXPECT_EQ(s.truth[0].truth_id, 1u);
}

TEST(AutoferryJsonReplay, BearingsEmittedOnlyWhenEnabled) {
  const std::string dir = makeFixture("af_test_bearing");
  AutoferryLoadOptions opts;
  opts.include_bearings = true;
  const Scenario s = loadAutoferryScenario(dir, "af_test_bearing", opts);
  // Now 3 active... wait: radar+lidar=3 plus 1 EO bearing = 4.
  ASSERT_EQ(s.measurements.size(), 4u);
  const auto& b = s.measurements.back();
  EXPECT_EQ(b.model, MeasurementModel::Bearing2D);
  ASSERT_EQ(b.value.size(), 1);
  // NED bearing 0 (due north) → ENU atan2 from east = π/2.
  EXPECT_NEAR(b.value(0), M_PI / 2.0, 1e-12);
  EXPECT_EQ(b.sensor, SensorKind::EoIr);
}

// The real dataset gives every target its OWN timestamp inside a ground-
// truth scan (skews of ~0.1 s). If the loader passes those through, the
// bench harness fragments each 2-target scan into two 1-target steps and
// every cardinality/identity metric collapses. The loader must therefore
// unify each scan's samples onto a single timestamp (the scan's latest
// target time) and keep the flattened list time-sorted.
constexpr const char* kSkewedGroundTruth = R"JSON(
[
  [{"targetID":1,"position":[100,200,0],"time":1.08},
   {"targetID":2,"position":[300,400,0],"time":1.0}],
  [{"targetID":1,"position":[110,205,0],"time":2.08},
   {"targetID":2,"position":[310,405,0],"time":2.0}]
]
)JSON";

TEST(AutoferryJsonReplay, TruthScansShareOneTimestampAndStaySorted) {
  const std::string label = "af_test_skewed_gt";
  const std::string dir = tmpDir() + "/" + label;
  std::string mk = "mkdir -p '" + dir + "'";
  (void)std::system(mk.c_str());
  writeFile(dir + "/" + label + "_detections.json", kDetections);
  writeFile(dir + "/" + label + "_groundTruth.json", kSkewedGroundTruth);

  const Scenario s = loadAutoferryScenario(dir, label);
  ASSERT_EQ(s.truth.size(), 4u);
  // Both targets of one scan share a single timestamp (the scan max).
  EXPECT_TRUE(s.truth[0].time == s.truth[1].time);
  EXPECT_TRUE(s.truth[2].time == s.truth[3].time);
  EXPECT_TRUE(s.truth[0].time == Timestamp::fromSeconds(1.08));
  EXPECT_TRUE(s.truth[2].time == Timestamp::fromSeconds(2.08));
  // Flattened list is sorted by time (BenchRunner::groupTruth precondition).
  EXPECT_TRUE(s.truth[1].time < s.truth[2].time);

  // The dataset carries no truth velocity; the loader derives it by
  // finite differences per target so SOG/COG RMSE compare against real
  // kinematics instead of a zero vector. Target 1 moves ENU (200,100) →
  // (205,110) over the unified dt of 1.0 s → v = (5, 10).
  ASSERT_EQ(s.truth[0].truth_id, 1u);
  EXPECT_NEAR(s.truth[0].velocity(0), 5.0, 1e-9);
  EXPECT_NEAR(s.truth[0].velocity(1), 10.0, 1e-9);
  EXPECT_NEAR(s.truth[2].velocity(0), 5.0, 1e-9);
  EXPECT_NEAR(s.truth[2].velocity(1), 10.0, 1e-9);
}

TEST(AutoferryJsonReplay, MissingFilesReturnEmpty) {
  const Scenario s =
      loadAutoferryScenario(tmpDir() + "/af_does_not_exist", "nope");
  EXPECT_TRUE(s.measurements.empty());
  EXPECT_TRUE(s.truth.empty());
}

TEST(AutoferryJsonReplay, LoadsRealScenario2FixtureIfPresent) {
  const std::string dir = "data/autoferry/scenario2";
  if (!fileExists(dir + "/scenario2_detections.json")) {
    GTEST_SKIP() << "AutoFerry scenario2 fixture not reachable from cwd";
  }
  const Scenario s = loadAutoferryScenario(dir, "scenario2");
  EXPECT_GT(s.measurements.size(), 100u);
  EXPECT_GT(s.truth.size(), 100u);
  for (const auto& m : s.measurements)
    EXPECT_EQ(m.model, MeasurementModel::Position2D);
  // Two reference targets in this scenario (Havfruen, Gunnerus).
  std::uint64_t max_id = 0;
  for (const auto& t : s.truth) max_id = std::max(max_id, t.truth_id);
  EXPECT_GE(max_id, 2u);
}

}  // namespace
}  // namespace navtracker::replay
