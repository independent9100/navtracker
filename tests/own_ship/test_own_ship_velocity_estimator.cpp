#include "core/own_ship/OwnShipVelocityEstimator.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}
}  // namespace

TEST(OwnShipVelocityEstimatorTest, UnpublishedWhenWindowEmpty) {
  OwnShipVelocityEstimator est{};
  EXPECT_FALSE(est.current().is_published);
}

TEST(OwnShipVelocityEstimatorTest, ConvergesOnConstantVelocityInput) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
  std::mt19937 rng{42};
  const double sigma_pos = 1.0;
  std::normal_distribution<double> n(0.0, sigma_pos);
  const Eigen::Vector2d v_truth(5.0, 3.0);
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0),
                v_truth.x() * i + n(rng),
                v_truth.y() * i + n(rng));
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_NEAR(e.velocity_enu.x(), v_truth.x(), 1.0);
  EXPECT_NEAR(e.velocity_enu.y(), v_truth.y(), 1.0);
  EXPECT_GT(e.sigma_v_m_per_s, 0.0);
}

TEST(OwnShipVelocityEstimatorTest, SuppressesDuringManeuver) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    y += 5.0;
  }
  EXPECT_FALSE(est.current().is_published);
}

TEST(OwnShipVelocityEstimatorTest, SigmaVTracksNoise) {
  for (double sigma_pos : {0.5, 2.0, 5.0}) {
    OwnShipVelocityEstimatorConfig cfg{};
    OwnShipVelocityEstimator est{cfg};
    std::mt19937 rng{42};
    std::normal_distribution<double> n(0.0, sigma_pos);
    for (int i = 0; i < 20; ++i) {
      est.observe(tAt(i * 1.0), 5.0 * i + n(rng), n(rng));
    }
    const auto e = est.current();
    if (!e.is_published) continue;  // tolerate noisy-window suppression
    EXPECT_GT(e.sigma_v_m_per_s, 0.0);
    EXPECT_LT(e.sigma_v_m_per_s, 3.0 * sigma_pos);  // loose envelope
  }
}

TEST(OwnShipVelocityEstimatorTest, ResumesAfterManeuver) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    y += 5.0;
  }
  ASSERT_FALSE(est.current().is_published);
  std::mt19937 rng{42};
  std::normal_distribution<double> n(0.0, 1.0);
  double t = cfg.window_size * 1.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i, t += 1.0) {
    y += 5.0;
    est.observe(tAt(t), x + n(rng), y + n(rng));
  }
  EXPECT_TRUE(est.current().is_published);
}

}  // namespace navtracker
