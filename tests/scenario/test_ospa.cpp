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

// Review #17: optimal (min-cost) assignment, not greedy NN. Geometry where
// the globally-smallest edge is NOT in the optimal matching:
//   truth A=(0,0), B=(0,3); est P=(0,2), Q=(0,5).
//   greedy locks the min edge B-P (d=1) → leaves A-Q (d=5):
//     OSPA_greedy = sqrt((1²+5²)/2) = sqrt(13) ≈ 3.606.
//   optimal pairs A-P, B-Q (d=2 each):
//     OSPA_opt = sqrt((2²+2²)/2) = 2.0.
TEST(Ospa, UsesOptimalAssignmentNotGreedy) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                      Eigen::Vector2d(0.0, 3.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(0.0, 2.0),
                                    Eigen::Vector2d(0.0, 5.0)};
  EXPECT_NEAR(ospaGreedy(truth, est, 50.0), 2.0, 1e-12);
}
