#include <cmath>
#include <cstdint>
#include <random>

#include <gtest/gtest.h>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IBearingInnovationSink.hpp"

using namespace navtracker;

namespace {

BearingInnovation makeBI(double t_s, double r_rad, double state_var_rad2,
                         double R_rad2, double range_m,
                         std::uint64_t tid = 1) {
  BearingInnovation obs;
  obs.time = Timestamp::fromSeconds(t_s);
  obs.track_id = TrackId{tid};
  obs.innovation_rad = r_rad;
  obs.predicted_state_var_rad2 = state_var_rad2;
  obs.variance_rad2 = state_var_rad2 + R_rad2;
  obs.range_m = range_m;
  return obs;
}

constexpr double kBiasTrue = 0.0349;  // ~2 deg

}  // namespace

TEST(BiasObsBearingInnovation, SingleObservationAppliesScalarKfUpdate) {
  HeadingBiasEstimatorConfig cfg;
  cfg.initial_bias_rad = 0.0;
  HeadingBiasEstimator est(cfg);
  const double r = 0.02;
  const double R = 1e-4;
  const double state_var = 1e-5;
  const auto obs = makeBI(1.0, r, state_var, R, 200.0);

  const double p0 = est.varianceRad2();
  est.observe(obs);

  const double S = state_var + R;
  const double s_full = S + p0;
  const double K_expected = p0 / s_full;
  // W3.4: innovation_rad is the raw ENU-math innovation; the estimator negates
  // it to the compass bias convention, so a +r innovation moves b̂ to −K·r.
  EXPECT_NEAR(est.biasRad(), -K_expected * r, 1e-9);
  EXPECT_NEAR(est.varianceRad2(), (1.0 - K_expected) * p0, 1e-9);
  EXPECT_EQ(est.acceptedBearingObs(), 1u);
}

TEST(BiasObsBearingInnovation, ManyDrawsConvergeToTruthWithin3Sigma) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  std::mt19937_64 rng(1234);
  const double R = 1e-4;
  const double state_var = 1e-5;
  std::normal_distribution<double> noise(0.0, std::sqrt(state_var + R));
  const int N = 200;
  for (int i = 0; i < N; ++i) {
    // A +kBiasTrue compass gyro bias emits a −kBiasTrue ENU-math innovation
    // (W3.4); the estimator negates it back to +kBiasTrue.
    const double r = -kBiasTrue + noise(rng);
    est.observe(makeBI(static_cast<double>(i + 1) * 0.1,
                       r, state_var, R, 500.0));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue),
            3.0 * std::sqrt(est.varianceRad2()));
  EXPECT_LT(est.varianceRad2(), cfg.initial_variance_rad2);
  EXPECT_GT(est.acceptedBearingObs(), 0u);
}

TEST(BiasObsBearingInnovation, GateOnInnovationNotAbsoluteBiasDoesNotFreeze) {
  // Finding B5 teeth: the outlier gate must key on the INNOVATION (meas − b̂),
  // not on |meas|. A large true gyro bias (~8.6°) drives |meas| ≈ |b_true|,
  // which — once the estimate is tight — exceeds bi_outlier_sigma·σ. The
  // pre-fix |meas| gate then rejected every state-consistent observation and
  // the estimate FROZE below the true bias. Gating on the innovation, a stream
  // of consistent observations converges fully and rejects nothing.
  HeadingBiasEstimatorConfig cfg;
  cfg.initial_bias_rad = 0.0;
  HeadingBiasEstimator est(cfg);
  const double b_true = 0.15;  // ~8.6°, far above 5σ once the estimate tightens
  const double R = 1e-4;
  const double state_var = 1e-5;
  for (int i = 0; i < 80; ++i) {
    // meas = −innovation_rad = +b_true (a +b_true compass gyro bias, W3.4).
    est.observe(makeBI((i + 1) * 0.1, -b_true, state_var, R, 500.0));
  }
  EXPECT_NEAR(est.biasRad(), b_true, 5e-3);  // fully reaches the large bias
  EXPECT_EQ(est.rejectedByOutlier(), 0u);    // never froze on |meas|
  EXPECT_GT(est.acceptedBearingObs(), 70u);
}

TEST(BiasObsBearingInnovation, RangeGateRejectsShortRange) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double b_before = est.biasRad();
  est.observe(makeBI(1.0, 0.05, 1e-5, 1e-4, /*range=*/10.0));
  EXPECT_EQ(est.biasRad(), b_before);
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByRange(), 1u);
}

TEST(BiasObsBearingInnovation, StateVarGateRejectsStateDominated) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double R = 1e-4;
  const double state_var = 10.0 * R;
  est.observe(makeBI(1.0, 0.02, state_var, R, 500.0));
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByStateVar(), 1u);
}

TEST(BiasObsBearingInnovation, OutlierGateRejectsHugeInnovation) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double R = 1e-4;
  const double state_var = 1e-5;
  const double S = state_var + R;
  const double sigma = std::sqrt(S + est.varianceRad2());
  const double r = 10.0 * sigma;
  est.observe(makeBI(1.0, r, state_var, R, 500.0));
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByOutlier(), 1u);
}

TEST(BiasObsBearingInnovation, LargeInnovationAppliesUnderLooseGate) {
  // Bias estimator should treat the wrapped value linearly (no further
  // wrap inside observe()) — pass r close to but inside (-π, π].
  HeadingBiasEstimatorConfig cfg;
  cfg.initial_bias_rad = 0.0;
  cfg.initial_variance_rad2 = 1.0;  // wide prior so K~1, easy to detect
  cfg.bi_outlier_sigma = 1000.0;    // disable outlier gate for this case
  HeadingBiasEstimator est(cfg);
  const double r_in = 3.0;          // < π, would be rejected at default 5σ
  est.observe(makeBI(1.0, r_in, 1e-5, 1e-4, 500.0));
  // W3.4: the estimator negates the ENU-math innovation, so b̂ moves toward
  // −r_in (substantially negative).
  EXPECT_LT(est.biasRad(), -0.5);
  EXPECT_EQ(est.acceptedBearingObs(), 1u);
}
