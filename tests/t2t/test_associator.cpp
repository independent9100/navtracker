// Unit tests for T2tAssociator (ticket §6.3 association bullet): gate math vs
// hand chi², soft MMSI (kinematics win), graceful huge-covariance gate.

#include "core/t2t/T2tAssociator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace navtracker::t2t {
namespace {

GateCandidate cand(double x, double y, double var, std::optional<std::uint32_t> mmsi = {}) {
  GateCandidate g;
  g.position = Eigen::Vector2d(x, y);
  g.covariance = Eigen::Matrix2d::Identity() * var;
  g.mmsi = mmsi;
  return g;
}

TEST(T2tAssociator, GateDistanceMatchesHandChiSquare) {
  T2tAssociator a;
  // Separation (2,0), each cov = I -> S = 2I -> d² = 4 / 2 = 2.
  EXPECT_NEAR(a.gateDistanceSq(cand(0, 0, 1.0), cand(2, 0, 1.0)), 2.0, 1e-9);
  // Separation (10,0), each cov = I -> d² = 100/2 = 50.
  EXPECT_NEAR(a.gateDistanceSq(cand(0, 0, 1.0), cand(10, 0, 1.0)), 50.0, 1e-9);
}

TEST(T2tAssociator, HugeCovarianceGatesWideNotSingular) {
  T2tAssociator a;
  // A bearing-quality source 1 km away but with enormous covariance: S is large
  // and well-conditioned, so d² is small (gates WIDE) and finite.
  const double d2 = a.gateDistanceSq(cand(0, 0, 1.0), cand(1000, 0, 1e6));
  EXPECT_TRUE(std::isfinite(d2));
  EXPECT_LT(d2, a.config().gate_chi2_position);  // in-gate
}

TEST(T2tAssociator, DegenerateCovarianceGatesOut) {
  T2tAssociator a;
  // Both covariances zero -> S singular -> +inf -> gates out (no poison).
  const double d2 = a.gateDistanceSq(cand(0, 0, 0.0), cand(1, 0, 0.0));
  EXPECT_TRUE(std::isinf(d2));
}

TEST(T2tAssociator, MmsiTermsAreSoft) {
  T2tAssociator a;
  const double base = 1.0;
  EXPECT_NEAR(a.assignmentCost(base, 42u, 42u),
              base - a.config().shared_mmsi_cost_bonus, 1e-12);
  EXPECT_NEAR(a.assignmentCost(base, 42u, 43u),
              base + a.config().conflicting_mmsi_cost_penalty, 1e-12);
  EXPECT_NEAR(a.assignmentCost(base, std::nullopt, 43u), base, 1e-12);
}

TEST(T2tAssociator, ConflictingMmsiStillAssociatesWhenKinematicsAgree) {
  T2tAssociator a;
  // Near-coincident positions but conflicting MMSI: the soft penalty keeps the
  // cost under the gate, so kinematics win and the pair associates.
  const std::vector<GateCandidate> fused{cand(0.0, 0.0, 1.0, 100u)};
  const std::vector<GateCandidate> sources{cand(0.1, 0.0, 1.0, 200u)};
  const T2tAssignment r = a.associate(fused, sources);
  ASSERT_EQ(r.matches.size(), 1u);
  EXPECT_EQ(r.matches[0].first, 0u);
  EXPECT_EQ(r.matches[0].second, 0u);
}

TEST(T2tAssociator, OutOfGatePairsAreUnmatched) {
  T2tAssociator a;
  const std::vector<GateCandidate> fused{cand(0, 0, 1.0)};
  const std::vector<GateCandidate> sources{cand(100, 0, 1.0)};  // d²=5000
  const T2tAssignment r = a.associate(fused, sources);
  EXPECT_TRUE(r.matches.empty());
  ASSERT_EQ(r.unmatched_fused.size(), 1u);
  ASSERT_EQ(r.unmatched_sources.size(), 1u);
}

TEST(T2tAssociator, GlobalAssignmentPrefersCloserPairing) {
  T2tAssociator a;
  // All FOUR pairings are in-gate, so gating alone cannot pick the answer — only
  // cost-minimization can. With cov=I (S=2I) and gate 9.21: diagonal cells
  // d²=0.5²/2=0.125 (in-gate) AND anti-diagonal cells d²=2.5²/2=3.125 (also
  // in-gate but costlier). The diagonal (cost 0.25) beats the anti-diagonal
  // (cost 6.25). A solver that respected the gate but did NOT minimize cost
  // could legally return the anti-diagonal here — so asserting the diagonal
  // genuinely guards the Hungarian's optimization (combined-review, associator
  // lens: the prior fixture put the wrong pairing out-of-gate, making this
  // vacuous).
  const std::vector<GateCandidate> fused{cand(0, 0, 1.0), cand(3, 0, 1.0)};
  const std::vector<GateCandidate> sources{cand(0.5, 0, 1.0), cand(2.5, 0, 1.0)};
  const T2tAssignment r = a.associate(fused, sources);
  ASSERT_EQ(r.matches.size(), 2u);
  for (const auto& [fi, sj] : r.matches) EXPECT_EQ(fi, sj);  // 0-0 and 1-1
}

}  // namespace
}  // namespace navtracker::t2t
