#include <gtest/gtest.h>

#include <limits>

#include <Eigen/Core>

#include "core/association/Hungarian.hpp"

using navtracker::hungarianAssignment;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

TEST(Hungarian, SquareTrivialOneToOne) {
  // Identity-like cost: row i is cheapest at col i.
  Eigen::MatrixXd c(3, 3);
  c << 0.0, 5.0, 5.0,
       5.0, 0.0, 5.0,
       5.0, 5.0, 0.0;
  const auto a = hungarianAssignment(c);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_EQ(a[0], 0);
  EXPECT_EQ(a[1], 1);
  EXPECT_EQ(a[2], 2);
}

TEST(Hungarian, SquarePicksMinimumSum) {
  // Diagonal sum 0+0+0; alternative would be 1+1+1.
  Eigen::MatrixXd c(3, 3);
  c << 0.0, 1.0, 1.0,
       1.0, 0.0, 1.0,
       1.0, 1.0, 0.0;
  const auto a = hungarianAssignment(c);
  double total = 0.0;
  for (int i = 0; i < 3; ++i) total += c(i, a[i]);
  EXPECT_NEAR(total, 0.0, 1e-12);
}

TEST(Hungarian, RectangularMoreColsThanRows) {
  // 2 rows × 4 cols. Best: row 0 → col 0, row 1 → col 2.
  Eigen::MatrixXd c(2, 4);
  c << 0.0, 10.0, 10.0, 10.0,
       9.0, 10.0,  0.0, 10.0;
  const auto a = hungarianAssignment(c);
  ASSERT_EQ(a.size(), 2u);
  EXPECT_EQ(a[0], 0);
  EXPECT_EQ(a[1], 2);
}

TEST(Hungarian, ForbiddenCellsHonored) {
  // Forbid (0, 0) and (1, 1) via +∞. Optimal: row 0→col 1, row 1→col 0.
  Eigen::MatrixXd c(2, 2);
  c << kInf, 0.0,
       0.0, kInf;
  const auto a = hungarianAssignment(c);
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, DegenerateAllInfRowSurvivesWithoutHang) {
  // The pathology that caused the original MHT crash: one row has no
  // feasible column. The solver must complete (no infinite loop, no
  // out-of-bounds access) and produce SOME assignment. The MHT caller
  // detects this via std::isfinite(cost(t, a[t])) and falls back.
  Eigen::MatrixXd c(2, 2);
  c << kInf, kInf,
       0.0, 1.0;
  const auto a = hungarianAssignment(c);
  ASSERT_EQ(a.size(), 2u);
  // Row 1 must take its zero-cost col 0.
  EXPECT_EQ(a[1], 0);
  // Row 0's assignment must be the originally-infinite cell —
  // confirmed by the caller checking isfinite(cost(0, a[0])).
  ASSERT_GE(a[0], 0);
  EXPECT_FALSE(std::isfinite(c(0, a[0])));
}

TEST(Hungarian, EmptyMatrixReturnsEmpty) {
  Eigen::MatrixXd c(0, 0);
  const auto a = hungarianAssignment(c);
  EXPECT_TRUE(a.empty());
}
