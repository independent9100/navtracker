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

// Review #17: optimal (min-cost) assignment, not greedy NN. Same geometry
// as the OSPA test: truth A=(0,0), B=(0,3); est P=(0,2), Q=(0,5).
//   greedy locks min edge B-P (d=1), then A-Q (d=5<cutoff): cost=1+25=26 →
//     GOSPA_greedy = √26 ≈ 5.099.
//   optimal pairs A-P, B-Q (d=2 each): cost=4+4=8 → GOSPA_opt = √8 ≈ 2.828.
TEST(Gospa, UsesOptimalAssignmentNotGreedy) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                     Eigen::Vector2d(0.0, 3.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(0.0, 2.0),
                                   Eigen::Vector2d(0.0, 5.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 50.0), std::sqrt(8.0), 1e-12);
}

// Asymmetric: one matched pair + one extra estimate.
// Cost = d² + c²/α = 9 + 50 = 59 → √59 ≈ 7.681.
TEST(Gospa, MatchedPairPlusExtraEstimate) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0),
                                    Eigen::Vector2d(500.0, 0.0)};
  EXPECT_NEAR(gospaGreedy(truth, est, 10.0), std::sqrt(59.0), 1e-12);
}

// ---------------------------------------------------------------------------
// GospaComponents decomposition tests
// ---------------------------------------------------------------------------
// truth = {(0,0),(100,0)}, est = {(3,0),(500,500)}, c=20, p=2, α=2
//   Optimal assignment: (0,0)↔(3,0) d=3 (matched), (100,0) missed,
//   (500,500) false.
//   localization = 3² = 9
//   missed       = c²/α = 400/2 = 200  (1 missed truth)
//   false_       = c²/α = 400/2 = 200  (1 false est)
//   total        = 9+200+200 = 409 → GOSPA = √409
//   n_missed=1, n_false=1
TEST(GospaComponents, DecomposesCorrectly) {
  using navtracker::gospaComponents;
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                     Eigen::Vector2d(100.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0),
                                   Eigen::Vector2d(500.0, 500.0)};
  const auto g = gospaComponents(truth, est, /*cutoff=*/20.0, /*p=*/2.0,
                                 /*alpha=*/2.0);
  EXPECT_NEAR(g.localization, 9.0, 1e-9);
  EXPECT_NEAR(g.missed, 200.0, 1e-9);
  EXPECT_NEAR(g.false_, 200.0, 1e-9);
  EXPECT_EQ(g.n_missed, 1);
  EXPECT_EQ(g.n_false, 1);
  // total must equal sum of components
  EXPECT_NEAR(g.total, 409.0, 1e-9);
  // scalar gospaGreedy is consistent: returns pow(total,1/p) = √409
  EXPECT_NEAR(gospaGreedy(truth, est, 20.0), std::sqrt(409.0), 1e-9);
}

// Both empty: all components zero.
TEST(GospaComponents, BothEmpty) {
  using navtracker::gospaComponents;
  const auto g = gospaComponents({}, {}, 20.0);
  EXPECT_DOUBLE_EQ(g.total, 0.0);
  EXPECT_DOUBLE_EQ(g.localization, 0.0);
  EXPECT_DOUBLE_EQ(g.missed, 0.0);
  EXPECT_DOUBLE_EQ(g.false_, 0.0);
  EXPECT_EQ(g.n_missed, 0);
  EXPECT_EQ(g.n_false, 0);
}

// Single truth, no est: pure missed cost.
TEST(GospaComponents, SingleMiss) {
  using navtracker::gospaComponents;
  const auto g = gospaComponents({Eigen::Vector2d(0.0, 0.0)}, {}, 10.0);
  // c²/α = 100/2 = 50; total=50; localization=0; false_=0
  EXPECT_NEAR(g.missed, 50.0, 1e-9);
  EXPECT_DOUBLE_EQ(g.localization, 0.0);
  EXPECT_DOUBLE_EQ(g.false_, 0.0);
  EXPECT_EQ(g.n_missed, 1);
  EXPECT_EQ(g.n_false, 0);
  EXPECT_NEAR(g.total, 50.0, 1e-9);
}

// Single est, no truth: pure false cost.
TEST(GospaComponents, SingleFalse) {
  using navtracker::gospaComponents;
  const auto g = gospaComponents({}, {Eigen::Vector2d(0.0, 0.0)}, 10.0);
  EXPECT_NEAR(g.false_, 50.0, 1e-9);
  EXPECT_DOUBLE_EQ(g.localization, 0.0);
  EXPECT_DOUBLE_EQ(g.missed, 0.0);
  EXPECT_EQ(g.n_missed, 0);
  EXPECT_EQ(g.n_false, 1);
}
