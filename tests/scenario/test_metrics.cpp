#include <gtest/gtest.h>
#include "core/scenario/Metrics.hpp"

using navtracker::countIdSwitches;
using navtracker::ScenarioStep;
using navtracker::TrackId;
using navtracker::TrackSnapshot;

namespace {
ScenarioStep step(double t,
                  const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<TrackSnapshot>& tracks) {
  ScenarioStep s;
  s.time = navtracker::Timestamp::fromSeconds(t);
  s.truth = truth;
  s.tracks = tracks;
  return s;
}
}  // namespace

TEST(CountIdSwitches, NoSwitchesWhenSameTrackTracks) {
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(1.1, 0.0)}}),
      step(2.0, {Eigen::Vector2d(2.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(2.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 0);
}

TEST(CountIdSwitches, OneSwitchWhenIdChangesOnce) {
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)},
           {TrackSnapshot{TrackId{9}, Eigen::Vector2d(1.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 1);
}

TEST(CountIdSwitches, TrackLossDoesNotCount) {
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)}, {}),
      step(2.0, {Eigen::Vector2d(2.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(2.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 0);
}
