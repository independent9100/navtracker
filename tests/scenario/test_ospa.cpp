#include <gtest/gtest.h>
#include "core/scenario/Ospa.hpp"

using navtracker::ospaGreedy;

TEST(Ospa, ZeroWhenBothEmpty) {
  EXPECT_DOUBLE_EQ(
      ospaGreedy(std::vector<Eigen::Vector2d>{},
                 std::vector<Eigen::Vector2d>{}, 10.0),
      0.0);
}

TEST(Ospa, SinglePairDistance) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0)};
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 3.0, 1e-12);
}

TEST(Ospa, UnmatchedCountsAsCutoff) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{};
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 10.0, 1e-12);
}

TEST(Ospa, TwoPairsAverage) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                      Eigen::Vector2d(100.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(5.0, 0.0),
                                    Eigen::Vector2d(105.0, 0.0)};
  EXPECT_NEAR(ospaGreedy(truth, est, 50.0), 5.0, 1e-12);
}

TEST(Ospa, DistancesAboveCutoffAreClipped) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(1000.0, 0.0)};
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 10.0, 1e-12);
}
