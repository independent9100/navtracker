#include <gtest/gtest.h>

#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::ParticleFilterEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;

namespace {

Measurement positionMeas(double x, double y, double std_m, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m);
  m.source_id = "test";
  return m;
}

}  // namespace

TEST(ParticleFilterEstimator, InitiateSeedsEnsembleAndCarrier) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 500, 5.0, 0.5, 42);
  const navtracker::Track t = pf.initiate(positionMeas(100.0, -50.0, 3.0, 1.0));

  EXPECT_EQ(t.particles.rows(), 4);
  EXPECT_EQ(t.particles.cols(), 500);
  EXPECT_EQ(t.particle_weights.size(), 500);
  EXPECT_NEAR(t.particle_weights.sum(), 1.0, 1e-9);
  for (int i = 0; i < 500; ++i)
    EXPECT_NEAR(t.particle_weights(i), 1.0 / 500.0, 1e-12);

  // Carrier mean within Monte-Carlo tolerance of the measurement.
  EXPECT_NEAR(t.state(0), 100.0, 1.5);
  EXPECT_NEAR(t.state(1), -50.0, 1.5);
  // Velocity initialized at zero (no info), spread = init_speed_std_.
  EXPECT_NEAR(t.state(2), 0.0, 1.5);
  EXPECT_NEAR(t.state(3), 0.0, 1.5);
  EXPECT_GT(t.covariance(2, 2), 1.0);
  EXPECT_GT(t.covariance(3, 3), 1.0);
}
