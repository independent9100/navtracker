#include <fstream>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/RadarTruthCsvReader.hpp"

// Fixture paths are relative to project root (the bench/test runner cwd).
static constexpr const char* kOwnship =
    "tests/fixtures/philos/out/ais_ferry_near/ownship.csv";
static constexpr const char* kRadarTruth =
    "tests/fixtures/philos/out/ais_ferry_near/radar_truth.csv";

TEST(RadarTruthLoader, ProjectsBodyFrameToEnuWithinRange) {
  if (!std::ifstream(kRadarTruth)) GTEST_SKIP() << "fixture absent";
  if (!std::ifstream(kOwnship)) GTEST_SKIP() << "ownship fixture absent";

  const auto poses = navtracker::replay::loadOwnshipCsv(kOwnship);
  ASSERT_FALSE(poses.empty());

  navtracker::OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);

  const auto truth =
      navtracker::replay::loadRadarTruthCsv(kRadarTruth, provider);

  ASSERT_FALSE(truth.empty());
  for (const auto& t : truth) {
    EXPECT_TRUE(t.position.allFinite())
        << "non-finite ENU position for truth_id=" << t.truth_id;
    // radar_truth.csv reports ranges up to ~15 800 m (the fixture has a few
    // distant vessels); 20 km is a safe upper bound in ENU distance from the
    // datum (which is set from the first ownship fix).
    EXPECT_LT(t.position.norm(), 20000.0)
        << "position too far from datum for truth_id=" << t.truth_id;
    EXPECT_NE(t.truth_id, 0u) << "uid/MMSI was not parsed";
  }
}

TEST(RadarTruthLoader, TimeStampsAreMonotonicAndNonZero) {
  if (!std::ifstream(kRadarTruth)) GTEST_SKIP() << "fixture absent";
  if (!std::ifstream(kOwnship)) GTEST_SKIP() << "ownship fixture absent";

  const auto poses = navtracker::replay::loadOwnshipCsv(kOwnship);
  ASSERT_FALSE(poses.empty());

  navtracker::OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);

  const auto truth =
      navtracker::replay::loadRadarTruthCsv(kRadarTruth, provider);
  ASSERT_GE(truth.size(), 2u);

  // Output must be sorted by time (loader sorts).
  for (std::size_t i = 1; i < truth.size(); ++i) {
    EXPECT_LE(truth[i - 1].time, truth[i].time)
        << "truth not sorted at index " << i;
  }
  // Timestamps should be non-trivially large (epoch seconds, ~2022).
  EXPECT_GT(truth.front().time.seconds(), 1e9);
}
