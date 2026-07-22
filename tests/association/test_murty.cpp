#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <random>
#include <set>
#include <vector>

#include <Eigen/Core>

#include "core/association/Hungarian.hpp"
#include "core/association/Murty.hpp"

using navtracker::hungarianAssignment;
using navtracker::murtyKBest;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

double assignmentCost(const std::vector<int>& a, const Eigen::MatrixXd& C) {
  double s = 0.0;
  for (int r = 0; r < static_cast<int>(a.size()); ++r) {
    if (a[r] < 0) continue;
    s += C(r, a[r]);
  }
  return s;
}
}  // namespace

TEST(Murty, K1MatchesHungarianOnSimpleCase) {
  Eigen::MatrixXd c(3, 3);
  c << 4, 2, 8,
       3, 5, 1,
       6, 7, 9;
  const auto h = hungarianAssignment(c);
  const auto m = murtyKBest(c, 1);
  ASSERT_EQ(m.assignments.size(), 1u);
  EXPECT_EQ(m.assignments[0], h);
  EXPECT_NEAR(m.costs[0], assignmentCost(h, c), 1e-12);
}

TEST(Murty, K1MatchesHungarianOnRandomBatch) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0.0, 100.0);
  for (int trial = 0; trial < 30; ++trial) {
    const int N = 4 + (rng() % 5);
    Eigen::MatrixXd c(N, N);
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) c(i, j) = u(rng);
    const auto h = hungarianAssignment(c);
    const auto m = murtyKBest(c, 1);
    ASSERT_EQ(m.assignments.size(), 1u) << "trial=" << trial;
    EXPECT_EQ(m.assignments[0], h) << "trial=" << trial;
  }
}

TEST(Murty, CostsAreInNonDecreasingOrder) {
  // 5×5 with all-distinct integer costs so ranking is unambiguous.
  Eigen::MatrixXd c(5, 5);
  c << 17,  3, 11,  9, 25,
        4, 19,  2, 14,  8,
       22,  6, 16,  1, 13,
       10, 21,  5, 18,  7,
       15, 12, 20, 23, 24;
  const auto m = murtyKBest(c, 10);
  ASSERT_GE(m.assignments.size(), 5u);
  for (std::size_t i = 1; i < m.costs.size(); ++i) {
    EXPECT_LE(m.costs[i - 1], m.costs[i] + 1e-12)
        << "rank " << (i - 1) << " cost " << m.costs[i - 1]
        << " > rank " << i << " cost " << m.costs[i];
  }
}

TEST(Murty, EnumeratesAllPermutationsOf3x3) {
  // 3! = 6 permutations of a 3×3 with distinct costs. K=6 must
  // return all 6 in sorted-cost order.
  Eigen::MatrixXd c(3, 3);
  c << 1, 5, 9,
       4, 2, 7,
       6, 3, 8;
  const auto m = murtyKBest(c, 6);
  ASSERT_EQ(m.assignments.size(), 6u);
  // All assignments must be unique permutations.
  std::set<std::vector<int>> seen;
  for (const auto& a : m.assignments) {
    seen.insert(a);
  }
  EXPECT_EQ(seen.size(), 6u);
  // Compute brute-force expected sorted costs.
  std::vector<double> brute;
  std::vector<int> perm{0, 1, 2};
  do {
    brute.push_back(c(0, perm[0]) + c(1, perm[1]) + c(2, perm[2]));
  } while (std::next_permutation(perm.begin(), perm.end()));
  std::sort(brute.begin(), brute.end());
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(m.costs[i], brute[i], 1e-9) << "i=" << i;
  }
}

TEST(Murty, StopsAtFeasibleCount) {
  // 2×2 with only two feasible assignments. K=10 must return 2, not hang.
  Eigen::MatrixXd c(2, 2);
  c << 1, 2,
       3, 4;
  const auto m = murtyKBest(c, 10);
  EXPECT_EQ(m.assignments.size(), 2u);
  EXPECT_LE(m.costs[0], m.costs[1]);
}

TEST(Murty, HonorsForbiddenCells) {
  // Top-left and bottom-right cells forbidden. K=2 must return only
  // permutations that avoid them.
  Eigen::MatrixXd c(2, 2);
  c << kInf, 1.0,
       1.0, kInf;
  const auto m = murtyKBest(c, 3);
  ASSERT_GE(m.assignments.size(), 1u);
  // The one feasible assignment is row0→col1, row1→col0 (cost 2).
  EXPECT_EQ(m.assignments[0][0], 1);
  EXPECT_EQ(m.assignments[0][1], 0);
  EXPECT_NEAR(m.costs[0], 2.0, 1e-12);
}

TEST(Murty, RectangularSurplusCols) {
  // 2 rows × 4 cols. Top-2 assignments by total cost.
  Eigen::MatrixXd c(2, 4);
  c << 0.0, 5.0, 5.0, 5.0,
       5.0, 5.0, 0.0, 5.0;
  const auto m = murtyKBest(c, 3);
  ASSERT_GE(m.assignments.size(), 1u);
  // Best: row 0 → col 0, row 1 → col 2; total cost 0.
  EXPECT_EQ(m.assignments[0][0], 0);
  EXPECT_EQ(m.assignments[0][1], 2);
  EXPECT_NEAR(m.costs[0], 0.0, 1e-12);
  // Subsequent costs must be non-decreasing.
  for (std::size_t i = 1; i < m.costs.size(); ++i) {
    EXPECT_LE(m.costs[i - 1], m.costs[i] + 1e-12);
  }
}

TEST(Murty, ZeroOrEmptyArguments) {
  Eigen::MatrixXd c(0, 0);
  EXPECT_TRUE(murtyKBest(c, 5).assignments.empty());
  Eigen::MatrixXd c2(2, 2);
  c2.setZero();
  EXPECT_TRUE(murtyKBest(c2, 0).assignments.empty());
  EXPECT_TRUE(murtyKBest(c2, -1).assignments.empty());
}

// ── Backlog #34 M3: per-row degradation on an infeasible seed ──────────────
// When no full matching on finite edges exists, the Hungarian seed crosses a
// +∞ cell (BIG_M fallback). murtyKBest MUST drop that infeasible edge and
// return the feasible subset — NOT an empty result that silently drops the
// whole cluster's children. Both callers (PmbmTracker, MhtTracker) were built
// against this contract: they re-check isfinite per assigned cell and skip
// unassigned (-1) rows; the empty-return made those checks dead code.

TEST(Murty, DegradesToFeasibleSubsetOnAllInfeasibleRow) {
  // Row 0 is entirely forbidden (+∞): row 0 can never be feasibly assigned.
  Eigen::MatrixXd c(2, 2);
  c << kInf, kInf,
       1.0,  2.0;
  const auto m = murtyKBest(c, 3);
  ASSERT_GE(m.assignments.size(), 1u);  // NOT empty (the M3 defect)
  // Row 0 → unassigned; row 1 takes its cheapest finite column (col 0).
  EXPECT_EQ(m.assignments[0][0], -1);
  EXPECT_EQ(m.assignments[0][1], 0);
  EXPECT_NEAR(m.costs[0], 1.0, 1e-12);
  // No returned assignment crosses a +∞ edge.
  for (const auto& a : m.assignments)
    for (int r = 0; r < static_cast<int>(a.size()); ++r)
      if (a[r] >= 0) EXPECT_TRUE(std::isfinite(c(r, a[r])));
}

TEST(Murty, DegradesToFeasibleSubsetOnAllInfeasibleColumn) {
  // Column 0 is entirely forbidden: no row can feasibly claim it, so it stays
  // unmatched while the finite column is assigned to its cheapest row.
  Eigen::MatrixXd c(2, 2);
  c << kInf, 1.0,
       kInf, 2.0;
  const auto m = murtyKBest(c, 3);
  ASSERT_GE(m.assignments.size(), 1u);
  // Col 1 → row 0 (cheapest), row 1 unassigned (its only finite col is taken).
  EXPECT_EQ(m.assignments[0][0], 1);
  EXPECT_EQ(m.assignments[0][1], -1);
  EXPECT_NEAR(m.costs[0], 1.0, 1e-12);
  for (const auto& a : m.assignments)
    for (int r = 0; r < static_cast<int>(a.size()); ++r)
      if (a[r] >= 0) EXPECT_TRUE(std::isfinite(c(r, a[r])));
}

TEST(Murty, FullyFeasibleWithSomeForbiddenIsUnchanged) {
  // Byte-identical guard: when a full finite matching EXISTS despite some +∞
  // cells, degradation must not alter it (this is the common gauntlet case).
  Eigen::MatrixXd c(2, 2);
  c << kInf, 1.0,
       1.0, kInf;
  const auto m = murtyKBest(c, 1);
  ASSERT_EQ(m.assignments.size(), 1u);
  EXPECT_EQ(m.assignments[0][0], 1);
  EXPECT_EQ(m.assignments[0][1], 0);
  EXPECT_NEAR(m.costs[0], 2.0, 1e-12);
}
