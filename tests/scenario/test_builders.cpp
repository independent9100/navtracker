#include <gtest/gtest.h>
#include "core/scenario/Builders.hpp"

using navtracker::buildParallelTargetsScenario;
using navtracker::buildStraightLineScenario;
using navtracker::Scenario;

TEST(Builders, StraightLineProducesTruthAndMeasurementsAtSameTimes) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 0.0, 1, 42);
  ASSERT_EQ(s.truth.size(), 3u);
  ASSERT_EQ(s.measurements.size(), 3u);
  EXPECT_DOUBLE_EQ(s.truth[1].position.x(), 20.0);
  EXPECT_DOUBLE_EQ(s.truth[1].position.y(), 0.0);
  EXPECT_EQ(s.truth[1].truth_id, 42u);
  EXPECT_DOUBLE_EQ(s.measurements[1].value(0), 20.0);
  EXPECT_DOUBLE_EQ(s.measurements[1].time.seconds(), 2.0);
}

TEST(Builders, ParallelTargetsEmitsTwoTruthPerTime) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 500.0),
      Eigen::Vector2d(10.0, 0.0), times, 0.0, 1);
  ASSERT_EQ(s.truth.size(), 4u);
  ASSERT_EQ(s.measurements.size(), 4u);
  EXPECT_NE(s.truth[0].truth_id, s.truth[1].truth_id);
}

TEST(Builders, DeterministicForSameSeed) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario a = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  const Scenario b = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.measurements[i].value(0), b.measurements[i].value(0));
    EXPECT_DOUBLE_EQ(a.measurements[i].value(1), b.measurements[i].value(1));
  }
}
