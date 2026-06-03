#include "core/own_ship/UereEstimator.hpp"

#include <cmath>
#include <cstdint>
#include <random>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}
}  // namespace

TEST(UereEstimatorTest, UnpublishedWhenWindowEmpty) {
  UereEstimator est{};
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, UnpublishedBelowWindowSize) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  for (std::size_t i = 0; i < cfg.window_size - 1; ++i)
    est.observe(tAt(static_cast<double>(i) * 1.0),
                5.0 * static_cast<double>(i), 0.0);
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, ConvergesOnSyntheticWhiteNoise) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  std::mt19937 rng{42};
  const double sigma = 2.0;
  std::normal_distribution<double> n(0.0, sigma);
  const double v = 5.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0),
                v * static_cast<double>(i) + n(rng),
                0.0 + n(rng));
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_GT(e.sigma_m, 0.5 * sigma);
  EXPECT_LT(e.sigma_m, 1.5 * sigma);
}

TEST(UereEstimatorTest, TracksRangeOfSigmas) {
  for (double sigma : {0.5, 2.0, 5.0, 10.0}) {
    UereEstimatorConfig cfg{};
    UereEstimator est{cfg};
    std::mt19937 rng{42};
    std::normal_distribution<double> n(0.0, sigma);
    // Stream 20 samples; on the last few, the window has settled.
    double last_sigma = -1.0;
    for (int i = 0; i < 20; ++i) {
      est.observe(tAt(static_cast<double>(i) * 1.0),
                  5.0 * static_cast<double>(i) + n(rng), n(rng));
      const auto e = est.current();
      if (e.is_published) last_sigma = e.sigma_m;
    }
    EXPECT_GT(last_sigma, 0.0);
    EXPECT_GT(last_sigma, 0.5 * sigma);
    EXPECT_LT(last_sigma, 1.5 * sigma);
  }
}

TEST(UereEstimatorTest, SuppressesDuringManeuver) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  // Window of 8: first 4 at v=(5,0), next 4 at v=(0,5). delta-v = sqrt(50)~7m/s
  // — well above threshold 0.5.
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0), x, y);
    y += 5.0;
  }
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, ResumesAfterManeuver) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  // Inject 8 maneuvering samples first.
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0), x, y);
    y += 5.0;
  }
  ASSERT_FALSE(est.current().is_published);
  // Then 8 steady samples — window slides over them.
  std::mt19937 rng{42};
  std::normal_distribution<double> n(0.0, 1.0);
  double v = 5.0;
  double t = static_cast<double>(cfg.window_size) * 1.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i, t += 1.0) {
    x += v;
    est.observe(tAt(t), x + n(rng), y + n(rng));
  }
  EXPECT_TRUE(est.current().is_published);
}

TEST(UereEstimatorTest, MinSigmaFloor) {
  UereEstimatorConfig cfg{};
  cfg.min_sigma_m = 0.5;
  UereEstimator est{cfg};
  const double v = 5.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(static_cast<double>(i) * 1.0),
                v * static_cast<double>(i), 0.0);  // perfect straight line
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_NEAR(e.sigma_m, cfg.min_sigma_m, 1e-9);
}

}  // namespace navtracker
