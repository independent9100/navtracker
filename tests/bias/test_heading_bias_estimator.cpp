#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace navtracker {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// Build a clean AIS+ARPA pair where the true ENU-math bearing is
// `beta_truth_rad` and `bias_rad` is the COMPASS gyro bias (gyro reads high
// by bias_rad, 0 = north CW+; W3.4 convention). A +bias_rad compass bias
// rotates the projected ARPA return the OTHER way in the ENU-math frame, so
// the ARPA sits at `beta_truth_rad − bias_rad`. The estimator converges to
// +bias_rad (the value the adapter subtracts). Range = 1500 m.
AisArpaPairObservation makePair(Timestamp t, double beta_truth_rad,
                                double bias_rad, double range_m = 1500.0) {
  AisArpaPairObservation o;
  o.time = t;
  o.own_position_enu = Eigen::Vector2d::Zero();
  o.ais_target_position_enu =
      range_m * Eigen::Vector2d(std::cos(beta_truth_rad), std::sin(beta_truth_rad));
  const double beta_arpa = beta_truth_rad - bias_rad;
  o.arpa_target_position_enu =
      range_m * Eigen::Vector2d(std::cos(beta_arpa), std::sin(beta_arpa));
  o.arpa_bearing_std_rad = 0.5 * kDeg2Rad;
  o.ais_position_std_m = 5.0;
  return o;
}

Timestamp tAt(double seconds) {
  return Timestamp{static_cast<std::int64_t>(seconds * 1e9)};
}

}  // namespace

TEST(HeadingBiasEstimatorTest, InitialStateUnpublished) {
  HeadingBiasEstimator est{};
  const auto e = est.current();
  EXPECT_FALSE(e.is_published);
  EXPECT_NEAR(e.bias_rad, 0.0, 1e-12);
  EXPECT_GT(e.variance_rad2, 0.0);
}

TEST(HeadingBiasEstimatorTest, SinglePairMovesBiasTowardTruth) {
  HeadingBiasEstimator est{};
  const double true_bias = 1.0 * kDeg2Rad;
  est.observe(makePair(tAt(0.0), 0.0, true_bias));
  EXPECT_GT(est.biasRad(), 0.0);
  EXPECT_LT(est.biasRad(), true_bias + 0.01);
}

TEST(HeadingBiasEstimatorTest, ConvergesOverManyUpdates) {
  HeadingBiasEstimatorConfig cfg{};
  HeadingBiasEstimator est{cfg};
  const double true_bias = 2.0 * kDeg2Rad;
  for (int i = 0; i < 30; ++i) {
    // Vary bearing across pairs so the geometry isn't degenerate.
    const double beta = (i * 17.0) * kDeg2Rad;
    est.observe(makePair(tAt(i * 1.0), beta, true_bias));
  }
  const auto e = est.current();
  EXPECT_TRUE(e.is_published);
  EXPECT_NEAR(est.biasRad(), true_bias, 0.1 * kDeg2Rad);
}

TEST(HeadingBiasEstimatorTest, TracksDriftingBias) {
  HeadingBiasEstimatorConfig cfg{};
  // Process noise tuned to track ~0.05 deg/s ramp. The standard-deviation
  // bound per dt=1s is the ramp rate; variance is its square.
  cfg.process_noise_var_per_sec =
      std::pow(0.5 * kDeg2Rad, 2.0);
  HeadingBiasEstimator est{cfg};
  // Drive 0 deg -> 3 deg over 60 s, 1 Hz pairs.
  for (int i = 0; i <= 60; ++i) {
    const double true_bias = (3.0 * i / 60.0) * kDeg2Rad;
    const double beta = (i * 17.0) * kDeg2Rad;
    est.observe(makePair(tAt(i * 1.0), beta, true_bias));
  }
  EXPECT_NEAR(est.biasRad(), 3.0 * kDeg2Rad, 0.6 * kDeg2Rad);
}

TEST(HeadingBiasEstimatorTest, GatingClosesOnAnchorLoss) {
  HeadingBiasEstimatorConfig cfg{};
  cfg.stale_seconds = 10.0;
  HeadingBiasEstimator est{cfg};
  const double true_bias = 2.0 * kDeg2Rad;
  for (int i = 0; i < 30; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  EXPECT_TRUE(est.current().is_published);
  // Advance time past stale window without any updates.
  est.predictTo(tAt(60.0));
  EXPECT_FALSE(est.current().is_published);
}

TEST(HeadingBiasEstimatorTest, GatingDelaysPublicationUntilTightEnough) {
  HeadingBiasEstimatorConfig cfg{};
  // Make the threshold tight so 1-2 updates aren't enough. Asymptotic
  // P_b approaches sigma_v^2/N for N updates; sigma_v ~ 0.5 deg so we
  // need O(25) updates to drop below (0.1 deg)^2.
  cfg.publish_variance_threshold_rad2 =
      std::pow(0.1 * kDeg2Rad, 2.0);
  HeadingBiasEstimator est{cfg};
  const double true_bias = 1.0 * kDeg2Rad;
  est.observe(makePair(tAt(0.0), 0.0, true_bias));
  EXPECT_FALSE(est.current().is_published);
  for (int i = 1; i < 200; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  EXPECT_TRUE(est.current().is_published);
}

TEST(HeadingBiasEstimatorTest, GpsFloorPreventsOverConvergence) {
  // With identical seeds/inputs but different own_position_std_m, the
  // estimator with a non-zero GPS floor must end up with a larger P_b
  // than the estimator without the floor. At r=1500 m and sigma_gps=13 m,
  // the angular floor is ~0.5 deg per update — comparable to the existing
  // ARPA bearing std (0.5 deg), so the asymptotic variance roughly doubles.
  HeadingBiasEstimator est_no_floor{};
  HeadingBiasEstimator est_with_floor{};
  const double true_bias = 1.0 * kDeg2Rad;
  for (int i = 0; i < 100; ++i) {
    auto pair_a = makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias);
    pair_a.own_position_std_m = 0.0;
    est_no_floor.observe(pair_a);

    auto pair_b = makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias);
    pair_b.own_position_std_m = 13.0;
    est_with_floor.observe(pair_b);
  }
  const double p_no_floor = est_no_floor.varianceRad2();
  const double p_with_floor = est_with_floor.varianceRad2();
  EXPECT_GT(p_with_floor, p_no_floor * 1.5);
}

TEST(HeadingBiasEstimatorTest, HandlesWrapAroundInnovation) {
  HeadingBiasEstimator est{};
  // True bias near +pi - epsilon so beta_arpa wraps.
  const double true_bias = (kPi - 0.05);
  for (int i = 0; i < 40; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  // Estimator should converge to true_bias (modulo wrap); check via cos
  // to side-step +pi/-pi sign issues.
  EXPECT_NEAR(std::cos(est.biasRad()), std::cos(true_bias), 0.05);
}

// Review #14: once converged, a single wildly-off AIS↔ARPA pair (e.g. AIS
// position jump / mis-association) is rejected by the outlier gate instead
// of corrupting the bias state. The gate is cold-start exempt, so it only
// engages after the estimate is data-backed.
TEST(HeadingBiasEstimatorTest, OutlierPairRejectedAfterConvergence) {
  HeadingBiasEstimator est{};
  const double true_bias = 2.0 * kDeg2Rad;
  for (int i = 0; i < 40; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  ASSERT_EQ(est.rejectedByOutlier(), 0u);
  const double converged = est.biasRad();

  // Inject one pair with a +90° disagreement — far outside the converged
  // innovation σ.
  est.observe(makePair(tAt(41.0), 30.0 * kDeg2Rad, 90.0 * kDeg2Rad));
  EXPECT_EQ(est.rejectedByOutlier(), 1u);
  // Bias essentially unchanged by the rejected outlier.
  EXPECT_NEAR(est.biasRad(), converged, 1e-9);
}

}  // namespace navtracker
