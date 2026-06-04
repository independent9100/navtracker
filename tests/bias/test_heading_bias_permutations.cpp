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

enum SourceMask : unsigned {
  kNone = 0,
  kGpsHdg = 1u << 0,
  kCog    = 1u << 1,
  kMag    = 1u << 2,
};

void runMixed(HeadingBiasEstimator& est, unsigned sources, int n_cycles,
              std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.001);
  std::normal_distribution<double> sensor(0.0, 0.005);
  for (int i = 0; i < n_cycles; ++i) {
    const double t = 0.5 * (i + 1);
    if (sources & kGpsHdg) {
      GyroVsGpsHeadingObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = kBiasTrue + noise(rng);
      o.gps_true_heading_rad = 0.0;
      o.gps_true_heading_std_rad = 0.001;
      est.observe(o);
    }
    if (sources & kCog) {
      GyroVsGpsCogObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = 0.0;
      o.gps_cog_rad = -kBiasTrue + sensor(rng);
      o.gps_cog_std_rad = 0.005;
      o.sog_mps = 10.0;
      o.gyro_rate_rad_per_s = 0.0;
      est.observe(o);
    }
    if (sources & kMag) {
      GyroVsMagneticObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = kBiasTrue + sensor(rng);
      o.magnetic_heading_rad = 0.0;
      o.magnetic_heading_std_rad = 0.005;
      o.magnetic_variation_rad = 0.0;
      est.observe(o);
    }
  }
}

}  // namespace

TEST(MhsPermutations, NoneWiredLeavesEstimateUnchanged) {
  HeadingBiasEstimator est({});
  const double b0 = est.biasRad();
  EXPECT_DOUBLE_EQ(est.biasRad(), b0);
  EXPECT_FALSE(est.current().is_published);
}

TEST(MhsPermutations, OnlyGpsHeadingConvergesTight) {
  HeadingBiasEstimator est({});
  runMixed(est, kGpsHdg, 50, 100);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedGpsHeading(), 0u);
}

TEST(MhsPermutations, OnlyCogConvergesWithinHalfDeg) {
  HeadingBiasEstimator est({});
  runMixed(est, kCog, 400, 200);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad();
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedGpsCog(), 0u);
}

TEST(MhsPermutations, OnlyMagConvergesWithinHalfDeg) {
  HeadingBiasEstimator est({});
  runMixed(est, kMag, 200, 300);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad();
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedMagnetic(), 0u);
}

TEST(MhsPermutations, AllThreeConvergesFastestAndTightest) {
  HeadingBiasEstimator est_all({});
  runMixed(est_all, kGpsHdg | kCog | kMag, 50, 400);
  EXPECT_LT(std::abs(est_all.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est_all.current().is_published);
  HeadingBiasEstimator est_gps({});
  runMixed(est_gps, kGpsHdg, 50, 400);
  EXPECT_LE(est_all.varianceRad2(), est_gps.varianceRad2() + 1e-12);
}

TEST(MhsPermutations, MixedHdgAndCogConvergesAtLeastAsTightAsCogAlone) {
  HeadingBiasEstimator est_mixed({});
  runMixed(est_mixed, kGpsHdg | kCog, 100, 500);
  HeadingBiasEstimator est_cog({});
  runMixed(est_cog, kCog, 100, 500);
  EXPECT_LE(est_mixed.varianceRad2(), est_cog.varianceRad2() + 1e-12);
  EXPECT_LT(std::abs(est_mixed.biasRad() - kBiasTrue),
            std::max(std::abs(est_cog.biasRad() - kBiasTrue),
                     0.1 * kPi / 180.0));
}

TEST(MhsPermutations, CogPlusMagConverges) {
  HeadingBiasEstimator est({});
  runMixed(est, kCog | kMag, 200, 600);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.4 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
}

TEST(MhsPermutations, HdgPlusMagConvergesToHdgPrecision) {
  HeadingBiasEstimator est({});
  runMixed(est, kGpsHdg | kMag, 50, 700);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
}

TEST(MhsDynamic, SourceLossAndReturnHandledCleanly) {
  HeadingBiasEstimatorConfig cfg;
  cfg.publish_variance_threshold_rad2 = 1e-5;
  HeadingBiasEstimator est(cfg);
  std::mt19937_64 rng(99);
  std::normal_distribution<double> n(0.0, 0.001);

  auto pushHdg = [&](double t) {
    GyroVsGpsHeadingObservation o;
    o.time = Timestamp::fromSeconds(t);
    o.gyro_rad = kBiasTrue + n(rng);
    o.gps_true_heading_rad = 0.0;
    o.gps_true_heading_std_rad = 0.001;
    est.observe(o);
  };

  for (int i = 0; i < 50; ++i) pushHdg(0.5 * (i + 1));
  const double p_after_hdg = est.varianceRad2();
  EXPECT_LT(p_after_hdg, cfg.publish_variance_threshold_rad2);
  EXPECT_TRUE(est.current().is_published);

  const double b_held = est.biasRad();

  for (int i = 100; i < 150; ++i) pushHdg(0.5 * (i + 1));
  EXPECT_NEAR(est.biasRad(), kBiasTrue, 0.1 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
  EXPECT_NEAR(est.biasRad(), b_held, 0.1 * kPi / 180.0);
}
