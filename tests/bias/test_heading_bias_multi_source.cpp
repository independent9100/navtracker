#include <cmath>
#include <cstdint>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/bias/HeadingBiasObservations.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {
constexpr double kBiasTrue = 0.0349;
constexpr double kPi = 3.14159265358979323846;

GyroVsGpsHeadingObservation makeGpsHdg(double t_s, double gyro,
                                       double gps, double sigma) {
  GyroVsGpsHeadingObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.gps_true_heading_rad = gps;
  o.gps_true_heading_std_rad = sigma;
  return o;
}

GyroVsGpsCogObservation makeCog(double t_s, double gyro, double cog,
                                double sigma, double sog, double rate) {
  GyroVsGpsCogObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.gps_cog_rad = cog;
  o.gps_cog_std_rad = sigma;
  o.sog_mps = sog;
  o.gyro_rate_rad_per_s = rate;
  return o;
}

GyroVsMagneticObservation makeMag(double t_s, double gyro, double mag,
                                  double sigma,
                                  std::optional<double> variation) {
  GyroVsMagneticObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.magnetic_heading_rad = mag;
  o.magnetic_heading_std_rad = sigma;
  o.magnetic_variation_rad = variation;
  return o;
}
}  // namespace

TEST(MhsGpsHeading, SingleObservationAppliesScalarKf) {
  HeadingBiasEstimator est({});
  const double sigma = 0.001;
  const auto obs = makeGpsHdg(1.0, 0.5 + kBiasTrue, 0.5, sigma);
  const double p0 = est.varianceRad2();
  est.observe(obs);
  const double R = sigma * sigma;
  const double K = p0 / (p0 + R);
  EXPECT_NEAR(est.biasRad(), K * kBiasTrue, 1e-9);
  EXPECT_NEAR(est.varianceRad2(), (1.0 - K) * p0, 1e-9);
  EXPECT_EQ(est.acceptedGpsHeading(), 1u);
}

TEST(MhsGpsHeading, SequenceConvergesTightly) {
  HeadingBiasEstimator est({});
  std::mt19937_64 rng(1);
  std::normal_distribution<double> n(0.0, 0.001);
  for (int i = 0; i < 50; ++i) {
    est.observe(makeGpsHdg(0.1 * (i + 1),
                           kBiasTrue + n(rng), 0.0, 0.001));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue),
            3.0 * std::sqrt(est.varianceRad2()));
  EXPECT_GT(est.acceptedGpsHeading(), 0u);
}

TEST(MhsGpsHeading, OutlierRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.mhs_outlier_sigma = 5.0;
  HeadingBiasEstimator est(cfg);
  const double r = 50.0 * std::sqrt(cfg.initial_variance_rad2);
  est.observe(makeGpsHdg(1.0, r, 0.0, 0.001));
  EXPECT_EQ(est.acceptedGpsHeading(), 0u);
  EXPECT_EQ(est.rejectedMhsByOutlier(), 1u);
}

TEST(MhsGpsCog, PassingGatesUpdatesBias) {
  HeadingBiasEstimator est({});
  est.observe(makeCog(1.0, kBiasTrue, 0.0, 0.005,
                      /*sog=*/10.0, /*rate=*/0.0));
  EXPECT_EQ(est.acceptedGpsCog(), 1u);
  EXPECT_GT(est.biasRad(), 0.0);
}

TEST(MhsGpsCog, LowSogRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.cog_min_sog_mps = 3.0;
  HeadingBiasEstimator est(cfg);
  est.observe(makeCog(1.0, kBiasTrue, 0.0, 0.005, /*sog=*/1.0, 0.0));
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogBySog(), 1u);
}

TEST(MhsGpsCog, TurningRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.cog_max_gyro_rate_rad_per_s = 0.01;
  HeadingBiasEstimator est(cfg);
  est.observe(makeCog(1.0, kBiasTrue, 0.0, 0.005,
                      /*sog=*/10.0, /*rate=*/0.05));
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogByGyroRate(), 1u);
}

TEST(MhsGpsCog, CrabRealisticSequenceConvergesWithinHalfDeg) {
  HeadingBiasEstimator est({});
  std::mt19937_64 rng(11);
  std::normal_distribution<double> crab(0.0, 3.0 * kPi / 180.0);
  std::normal_distribution<double> sensor(0.0, 0.005);
  for (int i = 0; i < 400; ++i) {
    const double cog = -kBiasTrue - crab(rng) + sensor(rng);
    est.observe(makeCog(0.5 * (i + 1),
                        /*gyro=*/0.0,
                        cog,
                        /*sigma=*/0.005,
                        /*sog=*/10.0,
                        /*rate=*/0.0));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad() << " acc=" << est.acceptedGpsCog();
}

TEST(MhsMagnetic, VariationAppliedUpdatesBias) {
  HeadingBiasEstimator est({});
  const double truth = 0.7;
  const double variation = 0.1;
  est.observe(makeMag(1.0, truth + kBiasTrue, truth - variation,
                      0.005, variation));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_GT(est.biasRad(), 0.0);
}

TEST(MhsMagnetic, NullVariationIsNoop) {
  HeadingBiasEstimator est({});
  const double b_before = est.biasRad();
  est.observe(makeMag(1.0, kBiasTrue, 0.0, 0.005, std::nullopt));
  EXPECT_EQ(est.biasRad(), b_before);
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
}

TEST(MhsMagnetic, OutlierRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.mhs_outlier_sigma = 5.0;
  HeadingBiasEstimator est(cfg);
  const double r = 50.0 * std::sqrt(cfg.initial_variance_rad2);
  est.observe(makeMag(1.0, r, 0.0, 0.005, 0.0));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(est.rejectedMhsByOutlier(), 1u);
}
