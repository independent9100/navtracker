#include <gtest/gtest.h>
#include <cmath>
#include "core/scenario/Gospa.hpp"

using navtracker::gospaGreedy;

TEST(Gospa, ZeroWhenBothEmpty) {
  EXPECT_DOUBLE_EQ(
      gospaGreedy(std::vector<Eigen::Vector2d>{},
                  std::vector<Eigen::Vector2d>{}, 10.0),
      0.0);
}

TEST(Gospa, IdenticalSetsAreZero) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                     Eigen::Vector2d(10.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, truth, 30.0), 0.0, 1e-12);
}

// Single pair within cutoff: GOSPA = (d^p)^(1/p) = d.
TEST(Gospa, SinglePairDistanceUnderCutoff) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), 3.0, 1e-12);
}

// Missed truth only (|X|=1, |Y|=0): cost = c^p / α; under p=α=2 that is
// (c² / 2)^(1/2) = c / √2.
TEST(Gospa, MissedTruthChargesHalfCutoffSquared) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), 10.0 / std::sqrt(2.0), 1e-12);
}

// False track only (|X|=0, |Y|=1): symmetric, same penalty.
TEST(Gospa, FalseTrackChargesHalfCutoffSquared) {
  std::vector<Eigen::Vector2d> truth{};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(0.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), 10.0 / std::sqrt(2.0), 1e-12);
}

// Pair outside cutoff is left unmatched and charged as one miss + one
// false: cost = 2 · c²/α = c² → √(c²) = c. (Same shape OSPA returns,
// but the *unbounded* growth shows up the next test.)
TEST(Gospa, FarPairChargesAsMissPlusFalse) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(1000.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), 10.0, 1e-12);
}

// Distinguishing GOSPA from OSPA: doubling truth + est (both missed)
// should DOUBLE the unbounded cost-squared inside the root → GOSPA
// grows ~√2. OSPA would stay at c.
TEST(Gospa, CardinalityErrorGrowsWithSetSize) {
  const double c = 10.0;
  const Eigen::Vector2d zero(0.0, 0.0);

  const double one_pair = gospaGreedy({zero}, {}, c);
  const double two_pairs = gospaGreedy({zero, zero}, {}, c);
  // Two missed truths: cost = 2·c²/α with α=2 → c² → root c.
  // sqrt(2) ratio over the single-truth case.
  EXPECT_NEAR(two_pairs, one_pair * std::sqrt(2.0), 1e-12);
  // Concretely: 2 missed @ c=10 → √(2·100/2) = 10.
  EXPECT_NEAR(two_pairs, 10.0, 1e-12);
}

// alpha=1 (the "punish both miss and false at full c^p" extreme):
// single missed truth charges c^p / 1 → root = c.
TEST(Gospa, AlphaOneChargesFullCutoff) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0, /*p=*/2.0, /*alpha=*/1.0),
              10.0, 1e-12);
}

// Asymmetric: one matched pair + one extra estimate.
// Cost = d² + c²/α = 9 + 50 = 59 → √59 ≈ 7.681.
TEST(Gospa, MatchedPairPlusExtraEstimate) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0),
                                    Eigen::Vector2d(500.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), std::sqrt(59.0), 1e-12);
}
