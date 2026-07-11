// Unit tests for covariance intersection (ticket §6.3, first bullet).
// Assertions are banded/structural (#24 discipline): no exact pins on
// optimizer output; properties asserted with generous tolerances.

#include "core/t2t/CovarianceIntersection.hpp"

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <random>
#include <vector>

namespace navtracker::t2t {
namespace {

Eigen::MatrixXd diag2(double a, double b) {
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2, 2);
  m(0, 0) = a;
  m(1, 1) = b;
  return m;
}

// The overconfident control: naive fusion assuming independence,
// P_f^{-1} = P1^{-1} + P2^{-1}. Test-only, by design — the shipped surface must
// not make this reachable (it is the double-counting footgun CI prevents). Used
// here to prove trace(P_CI) >= trace(P_naive): the naive covariance is the one
// that becomes dangerously small when the inputs share a source.
CiResult naiveIndependentFuse(const Eigen::VectorXd& x1, const Eigen::MatrixXd& P1,
                              const Eigen::VectorXd& x2, const Eigen::MatrixXd& P2) {
  const Eigen::MatrixXd I1 = P1.inverse();
  const Eigen::MatrixXd I2 = P2.inverse();
  CiResult r;
  r.P = (I1 + I2).inverse();
  r.x = r.P * (I1 * x1 + I2 * x2);
  r.omega = 0.5;
  return r;
}

// A seeded random symmetric positive-definite 2x2 (A A^T + floor*I).
Eigen::MatrixXd randomSpd2(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  Eigen::MatrixXd A(2, 2);
  A << u(rng), u(rng), u(rng), u(rng);
  return A * A.transpose() + Eigen::MatrixXd::Identity(2, 2) * 0.5;
}

TEST(CovarianceIntersection, EqualCovariancesGiveOmegaOneHalfAndMidpoint) {
  const Eigen::VectorXd x1 = Eigen::Vector2d(0.0, 0.0);
  const Eigen::VectorXd x2 = Eigen::Vector2d(10.0, 0.0);
  const Eigen::MatrixXd P = diag2(4.0, 4.0);

  const CiResult r = covarianceIntersect(x1, P, x2, P);

  EXPECT_NEAR(r.omega, 0.5, 1e-9);            // symmetry
  EXPECT_NEAR(r.x(0), 5.0, 1e-6);             // midpoint mean
  EXPECT_NEAR(r.x(1), 0.0, 1e-6);
  EXPECT_TRUE(r.P.isApprox(P, 1e-9));         // fused cov == the common cov
}

TEST(CovarianceIntersection, SelfFuseIsIdempotent) {
  const Eigen::VectorXd x = Eigen::Vector2d(3.0, -7.0);
  const Eigen::MatrixXd P = diag2(2.0, 5.0);

  const CiResult r = covarianceIntersect(x, P, x, P);

  EXPECT_TRUE(r.x.isApprox(x, 1e-9));
  EXPECT_TRUE(r.P.isApprox(P, 1e-9));
}

TEST(CovarianceIntersection, OneInfinitelyUncertainInputIsPassthrough) {
  const Eigen::VectorXd x1 = Eigen::Vector2d(1.0, 2.0);
  const Eigen::MatrixXd P1 = diag2(3.0, 3.0);
  const Eigen::VectorXd x2 = Eigen::Vector2d(-1000.0, 500.0);
  const Eigen::MatrixXd P2 = diag2(1e12, 1e12);  // effectively "no information"

  const CiResult r = covarianceIntersect(x1, P1, x2, P2);

  EXPECT_GT(r.omega, 0.99);                    // nearly all weight on input 1
  EXPECT_TRUE(r.x.isApprox(x1, 1e-3));
  EXPECT_TRUE(r.P.isApprox(P1, 1e-3));
}

TEST(CovarianceIntersection, FixedWeightEndpointsSelectEachInput) {
  const Eigen::VectorXd x1 = Eigen::Vector2d(0.0, 0.0);
  const Eigen::MatrixXd P1 = diag2(1.0, 9.0);
  const Eigen::VectorXd x2 = Eigen::Vector2d(4.0, 4.0);
  const Eigen::MatrixXd P2 = diag2(9.0, 1.0);

  const CiResult at1 = covarianceIntersectAt(x1, P1, x2, P2, 1.0);
  EXPECT_TRUE(at1.x.isApprox(x1, 1e-9));
  EXPECT_TRUE(at1.P.isApprox(P1, 1e-9));

  const CiResult at0 = covarianceIntersectAt(x1, P1, x2, P2, 0.0);
  EXPECT_TRUE(at0.x.isApprox(x2, 1e-9));
  EXPECT_TRUE(at0.P.isApprox(P2, 1e-9));
}

TEST(CovarianceIntersection, FusedIsSymmetricPositiveDefinite) {
  std::mt19937 rng(12345);
  for (int i = 0; i < 200; ++i) {
    const Eigen::MatrixXd P1 = randomSpd2(rng);
    const Eigen::MatrixXd P2 = randomSpd2(rng);
    const Eigen::VectorXd x1 = Eigen::Vector2d(0.0, 0.0);
    const Eigen::VectorXd x2 = Eigen::Vector2d(1.0, -1.0);
    const CiResult r = covarianceIntersect(x1, P1, x2, P2);

    EXPECT_TRUE(r.P.isApprox(r.P.transpose(), 1e-9));  // symmetric
    Eigen::LLT<Eigen::MatrixXd> llt(r.P);
    EXPECT_EQ(llt.info(), Eigen::Success);             // positive-definite
  }
}

// The core consistency guarantee, in the two deterministic directions the
// ticket names: CI improves on (or ties) the better input, and CI is never
// more confident than the overconfident naive-independent fusion.
TEST(CovarianceIntersection, TraceIsBoundedAndNeverBelowNaiveFloor) {
  std::mt19937 rng(67890);
  for (int i = 0; i < 500; ++i) {
    const Eigen::MatrixXd P1 = randomSpd2(rng);
    const Eigen::MatrixXd P2 = randomSpd2(rng);
    const Eigen::VectorXd x1 = Eigen::Vector2d(0.0, 0.0);
    const Eigen::VectorXd x2 = Eigen::Vector2d(2.0, 3.0);

    const CiResult ci = covarianceIntersect(x1, P1, x2, P2);
    const CiResult naive = naiveIndependentFuse(x1, P1, x2, P2);

    const double t_ci = ci.P.trace();
    const double eps = 1e-6;
    // CI never worse than the better single input.
    EXPECT_LE(t_ci, std::min(P1.trace(), P2.trace()) + eps);
    // CI never smaller (more confident) than the naive-independent floor: the
    // naive covariance is the dangerously-small one; CI must stay above it.
    EXPECT_GE(t_ci, naive.P.trace() - eps);
  }
}

TEST(CovarianceIntersection, SequentialFuseMatchesPairwiseAndIsDeterministic) {
  const Eigen::VectorXd x1 = Eigen::Vector2d(0.0, 0.0);
  const Eigen::MatrixXd P1 = diag2(2.0, 4.0);
  const Eigen::VectorXd x2 = Eigen::Vector2d(6.0, 2.0);
  const Eigen::MatrixXd P2 = diag2(5.0, 1.0);

  const CiResult pairwise = covarianceIntersect(x1, P1, x2, P2);
  const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> two{{x1, P1}, {x2, P2}};
  const CiResult seq = covarianceIntersectSequential(two);
  EXPECT_TRUE(seq.x.isApprox(pairwise.x, 1e-12));
  EXPECT_TRUE(seq.P.isApprox(pairwise.P, 1e-12));

  // Determinism: identical inputs -> bit-identical output on repeat.
  const CiResult again = covarianceIntersect(x1, P1, x2, P2);
  EXPECT_EQ(pairwise.omega, again.omega);
  EXPECT_TRUE(pairwise.x == again.x);
  EXPECT_TRUE(pairwise.P == again.P);

  // Single-input sequential passthrough.
  const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> one{{x1, P1}};
  const CiResult solo = covarianceIntersectSequential(one);
  EXPECT_TRUE(solo.x.isApprox(x1, 1e-12));
  EXPECT_TRUE(solo.P.isApprox(P1, 1e-12));
}

}  // namespace
}  // namespace navtracker::t2t
