#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/collision/Cpa.hpp"

using navtracker::CpaPrediction;
using navtracker::computeCpaWithUncertainty;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackStatus;

namespace {

Track makeTrack(double px, double py, double vx, double vy,
                double sigma_pos = 0.0, double sigma_vel = 0.0,
                Timestamp last_update = Timestamp::fromSeconds(0.0)) {
  Track t;
  t.id = TrackId{1};
  t.last_update = last_update;
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << px, py, vx, vy;
  t.covariance = Eigen::Matrix4d::Zero();
  const double pp = sigma_pos * sigma_pos;
  const double vv = sigma_vel * sigma_vel;
  t.covariance.diagonal() << pp, pp, vv, vv;
  return t;
}

// Reference 1D-Gaussian CDF used by the implementation; reproduce here
// to compare against the implementation's probability output.
double cdf(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

}  // namespace

TEST(CpaUncertainty, ZeroCovGivesZeroSigma) {
  // Head-on collision, no input uncertainty -> output sigma all zero.
  // A at (0,0) moving +x at 10 m/s; B at (20, 0) moving -x at 10 m/s.
  // CPA at t=1 s, distance 0.
  const Track a = makeTrack( 0.0, 0.0,  10.0, 0.0);
  const Track b = makeTrack(20.0, 0.0, -10.0, 0.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           5.0);
  EXPECT_NEAR(r.cpa_distance_m, 0.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,   1.0, 1e-9);
  EXPECT_NEAR(r.sigma_cpa_m,    0.0, 1e-9);
  EXPECT_NEAR(r.sigma_tcpa_seconds, 0.0, 1e-9);
  // cpa = 0 < d_threshold = 5; sigma = 0 -> step function -> 1.
  EXPECT_NEAR(r.probability_below_threshold, 1.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}

TEST(CpaUncertainty, HeadOnPropagatesPositionSigma) {
  // Near head-on geometry with small lateral offset so cpa > kEpsCpa
  // and the direction-projection branch (not the isotropic fallback)
  // exercises position σ propagation. Own-ship A at (0, 0, 10, 0) with
  // zero covariance; target B at (200, 5, -10, 0) with σ_pos = 10 m.
  // dp = (-200, -5), dv = (20, 0). t_cpa = 10 s. p_cpa = (0, -5). cpa = 5.
  // J_p_cpa columns for B-position give [[0,0],[0,-1]] (the t_cpa Jacobian
  // exactly cancels the px component); cov_p_cpa = diag(0, 100); projected
  // onto (0, -1) yields σ_cpa = 10 m exactly.
  const Track a = makeTrack(  0.0, 0.0,  10.0, 0.0);
  const Track b = makeTrack(200.0, 5.0, -10.0, 0.0, /*sigma_pos=*/10.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           50.0);
  EXPECT_NEAR(r.tcpa_seconds, 10.0, 1e-9);
  EXPECT_NEAR(r.cpa_distance_m, 5.0, 1e-9);
  // Target σ_pos = 10 m projects to σ_cpa ≈ 10 m (within 20% per spec).
  EXPECT_NEAR(r.sigma_cpa_m, 10.0, 2.0);
  EXPECT_FALSE(r.is_diverging);
}

TEST(CpaUncertainty, PerpendicularPassSigmaScalesLinearlyWithTargetSigmaPos) {
  // A moves +x at 10 m/s starting at (-100, 0); B stationary at (0, 5).
  // Perpendicular pass. CPA = 5 m at t = 10 s.
  // For each σ_pos in {1, 5, 10, 20}, σ_cpa should be ≈ σ_pos within 20 %.
  for (double sp : {1.0, 5.0, 10.0, 20.0}) {
    const Track a = makeTrack(-100.0, 0.0, 10.0, 0.0);
    const Track b = makeTrack(   0.0, 5.0,  0.0, 0.0, /*sigma_pos=*/sp);
    const auto r = computeCpaWithUncertainty(a, b,
                                             Timestamp::fromSeconds(0.0),
                                             10.0);
    EXPECT_NEAR(r.tcpa_seconds, 10.0, 1e-9);
    EXPECT_NEAR(r.cpa_distance_m, 5.0, 1e-9);
    EXPECT_NEAR(r.sigma_cpa_m, sp, 0.2 * sp + 1e-9)
        << "σ_pos = " << sp;
    EXPECT_FALSE(r.is_diverging);
  }
}

TEST(CpaUncertainty, ParallelVelocitiesSigmaTcpaInfinite) {
  // Both moving +x at 10 m/s, 20 m apart on y. dv = 0 → parallel branch.
  // tcpa = 0, cpa = current distance = 20 m, σ_tcpa = +infinity.
  // σ_cpa from current dp covariance projected onto dp̂ = (0, 1):
  // cov_dp_yy = σ²_a + σ²_b = 9 + 9 = 18  → σ_cpa = √18 ≈ 4.2426.
  const Track a = makeTrack(0.0,  0.0, 10.0, 0.0, /*sigma_pos=*/3.0);
  const Track b = makeTrack(0.0, 20.0, 10.0, 0.0, /*sigma_pos=*/3.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           5.0);
  EXPECT_NEAR(r.cpa_distance_m, 20.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,    0.0, 1e-9);
  EXPECT_TRUE(std::isinf(r.sigma_tcpa_seconds));
  EXPECT_FALSE(r.is_diverging);
  EXPECT_NEAR(r.sigma_cpa_m, std::sqrt(18.0), 1e-6);
}

TEST(CpaUncertainty, PastCpaUsesCurrentDistance) {
  // A at (0,0) moving +x at 5; B at (50,0) moving +x at 10. Both heading
  // the same way, B is faster and ahead → already separating; CPA in
  // the past. Fallback: tcpa = 0, σ_tcpa = ∞, cpa = current distance,
  // is_diverging = true. σ_cpa from dp covariance projected onto (1, 0).
  const Track a = makeTrack( 0.0, 0.0,  5.0, 0.0, /*sigma_pos=*/4.0);
  const Track b = makeTrack(50.0, 0.0, 10.0, 0.0, /*sigma_pos=*/3.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           10.0);
  EXPECT_NEAR(r.cpa_distance_m, 50.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,    0.0, 1e-9);
  EXPECT_TRUE(std::isinf(r.sigma_tcpa_seconds));
  EXPECT_TRUE(r.is_diverging);
  // cov_dp_xx = σ²_a + σ²_b = 16 + 9 = 25 → σ_cpa = 5.
  EXPECT_NEAR(r.sigma_cpa_m, 5.0, 1e-9);
}

TEST(CpaUncertainty, HeadOnNearZeroCpaUsesIsotropicFallback) {
  // True head-on collision: cpa = 0 < kEpsCpa = 1 m. The implementation
  // must use the isotropic fallback σ = √(tr(cov_p_cpa)/2) rather than
  // the direction-projection (which would divide by zero).
  // A at (0,0) +x@10, B at (200,0) -x@10, target σ_pos = 10 m.
  // t_cpa = 10 s, p_cpa = (0, 0), cpa = 0.
  // J_p_cpa cols 4,5 reduce to [[0,0],[0,-1]]; cov_p_cpa = diag(0, 100).
  // tr/2 = 50 → σ_cpa = √50 ≈ 7.0711.
  const Track a = makeTrack(  0.0, 0.0,  10.0, 0.0);
  const Track b = makeTrack(200.0, 0.0, -10.0, 0.0, /*sigma_pos=*/10.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           50.0);
  EXPECT_NEAR(r.cpa_distance_m, 0.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,  10.0, 1e-9);
  EXPECT_NEAR(r.sigma_cpa_m, std::sqrt(50.0), 1e-6);
  EXPECT_FALSE(r.is_diverging);
}

TEST(CpaUncertainty, ProbabilityMatchesGaussian) {
  // Use the perpendicular-pass geometry from test 3 with σ_pos = 10
  // (so cpa = 5 m, σ_cpa ≈ 10 m). Sweep d_threshold and check that the
  // implementation matches Φ((d - cpa) / σ_cpa).
  const Track a = makeTrack(-100.0, 0.0, 10.0, 0.0);
  const Track b = makeTrack(   0.0, 5.0,  0.0, 0.0, /*sigma_pos=*/10.0);

  // First read σ_cpa at d_threshold = 5 (where P should be 0.5).
  const auto r_at_cpa = computeCpaWithUncertainty(
      a, b, Timestamp::fromSeconds(0.0), /*d=*/5.0);
  ASSERT_GT(r_at_cpa.sigma_cpa_m, 0.0);
  EXPECT_NEAR(r_at_cpa.probability_below_threshold, 0.5, 1e-9);

  const double cpa = r_at_cpa.cpa_distance_m;
  const double sigma = r_at_cpa.sigma_cpa_m;

  // d much less than cpa  → P → 0.
  {
    const auto r = computeCpaWithUncertainty(
        a, b, Timestamp::fromSeconds(0.0), cpa - 10.0 * sigma);
    EXPECT_LT(r.probability_below_threshold, 1e-9);
  }
  // d much greater than cpa → P → 1.
  {
    const auto r = computeCpaWithUncertainty(
        a, b, Timestamp::fromSeconds(0.0), cpa + 10.0 * sigma);
    EXPECT_GT(r.probability_below_threshold, 1.0 - 1e-9);
  }
  // Three intermediate offsets vs std::erfc-based reference.
  for (double k : {-1.5, -0.5, 0.5, 1.5, 2.0}) {
    const double d = cpa + k * sigma;
    const auto r = computeCpaWithUncertainty(
        a, b, Timestamp::fromSeconds(0.0), d);
    EXPECT_NEAR(r.probability_below_threshold, cdf(k), 1e-9)
        << "k = " << k;
  }
}

TEST(CpaUncertainty, MonotonicityInSigmaCpa) {
  // For fixed mean cpa < d_threshold, P should decrease monotonically
  // as σ_cpa grows (more uncertainty pulls P toward 0.5).
  // Conversely for cpa > d_threshold, P should increase toward 0.5.
  //
  // We vary the target's σ_pos to vary σ_cpa, using a perpendicular pass
  // where cpa = 5 m is the same for every variant.
  //
  // Case A: cpa = 5 < d_threshold = 20 → P should decrease with σ.
  // Case B: cpa = 5 > d_threshold = 1  → P should increase with σ.
  std::vector<double> p_below;
  std::vector<double> p_above;
  for (double sp : {1.0, 2.0, 5.0, 10.0, 20.0, 50.0}) {
    const Track a = makeTrack(-100.0, 0.0, 10.0, 0.0);
    const Track b = makeTrack(   0.0, 5.0,  0.0, 0.0, /*sigma_pos=*/sp);
    const auto r_below = computeCpaWithUncertainty(
        a, b, Timestamp::fromSeconds(0.0), /*d=*/20.0);
    const auto r_above = computeCpaWithUncertainty(
        a, b, Timestamp::fromSeconds(0.0), /*d=*/1.0);
    EXPECT_NEAR(r_below.cpa_distance_m, 5.0, 1e-9);
    p_below.push_back(r_below.probability_below_threshold);
    p_above.push_back(r_above.probability_below_threshold);
  }
  // d > cpa: as σ grows, P falls from ~1 toward 0.5 (strictly decreasing).
  for (std::size_t i = 1; i < p_below.size(); ++i) {
    EXPECT_LT(p_below[i], p_below[i - 1])
        << "p_below not monotonically decreasing at index " << i;
    EXPECT_GE(p_below[i], 0.5 - 1e-9);
  }
  // d < cpa: as σ grows, P rises from ~0 toward 0.5 (strictly increasing).
  for (std::size_t i = 1; i < p_above.size(); ++i) {
    EXPECT_GT(p_above[i], p_above[i - 1])
        << "p_above not monotonically increasing at index " << i;
    EXPECT_LE(p_above[i], 0.5 + 1e-9);
  }
}
