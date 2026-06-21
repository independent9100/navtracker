#include <gtest/gtest.h>

#include <cmath>

#include "core/scenario/TGospa.hpp"

using navtracker::Trajectory;
using navtracker::tgospa;

namespace {

Trajectory makeTraj(std::uint64_t id, std::initializer_list<int> ks,
                    std::initializer_list<std::pair<double, double>> xys) {
  Trajectory t;
  t.id = id;
  auto k_it = ks.begin();
  auto xy_it = xys.begin();
  for (; k_it != ks.end() && xy_it != xys.end(); ++k_it, ++xy_it) {
    t.samples[*k_it] = Eigen::Vector2d(xy_it->first, xy_it->second);
  }
  return t;
}

}  // namespace

TEST(TGospa, ZeroWhenBothEmpty) {
  EXPECT_DOUBLE_EQ(tgospa({}, {}, 10.0), 0.0);
}

TEST(TGospa, IdenticalTrajectoriesAreZero) {
  std::vector<Trajectory> truth{
      makeTraj(1, {1, 2, 3}, {{0, 0}, {1, 0}, {2, 0}})};
  std::vector<Trajectory> est = truth;
  EXPECT_NEAR(tgospa(truth, est, 10.0), 0.0, 1e-12);
}

// Single trajectory matched at every scan, displaced by d at each
// scan, no switches → sum of d^p over T scans, ^(1/p). With T=3,
// d=1, p=2: total = sqrt(3·1) = sqrt(3).
TEST(TGospa, PerScanErrorAccumulatesAcrossTime) {
  std::vector<Trajectory> truth{
      makeTraj(1, {1, 2, 3}, {{0, 0}, {0, 0}, {0, 0}})};
  std::vector<Trajectory> est{
      makeTraj(99, {1, 2, 3}, {{1, 0}, {1, 0}, {1, 0}})};
  EXPECT_NEAR(tgospa(truth, est, 10.0), std::sqrt(3.0), 1e-12);
}

// Same truth + est mid-track id swap (e.g. tracker fragments the
// trajectory into two ids). Per-scan GOSPA distances are zero
// throughout (the position is right), but the id of the matched
// estimated trajectory changes at scan 2 → one switch penalty.
// Default switch_penalty = cutoff = 10, p = 2 ⇒ contribution
// = sqrt(10²) = 10.
TEST(TGospa, SwitchPenaltyChargedWhenAssignmentFlips) {
  std::vector<Trajectory> truth{
      makeTraj(1, {1, 2}, {{0, 0}, {0, 0}})};
  std::vector<Trajectory> est{
      makeTraj(50, {1}, {{0, 0}}),
      makeTraj(51, {2}, {{0, 0}})};
  EXPECT_NEAR(tgospa(truth, est, 10.0), 10.0, 1e-12);
}

// Missed truth at every scan: T=3 missed targets × c²/α = 3·50 = 150.
// √150 ≈ 12.247.
TEST(TGospa, AllMissedTruthChargesCardinality) {
  std::vector<Trajectory> truth{
      makeTraj(1, {1, 2, 3}, {{0, 0}, {0, 0}, {0, 0}})};
  EXPECT_NEAR(tgospa(truth, {}, 10.0), std::sqrt(3.0 * 50.0), 1e-12);
}

// Symmetric: T=2 false tracks → 2·50 = 100, √100 = 10.
TEST(TGospa, AllFalseEstChargesCardinality) {
  std::vector<Trajectory> est{
      makeTraj(1, {1, 2}, {{0, 0}, {0, 0}})};
  EXPECT_NEAR(tgospa({}, est, 10.0), std::sqrt(100.0), 1e-12);
}
