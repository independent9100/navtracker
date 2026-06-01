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

TEST(ParticleFilterEstimator, PredictAdvancesMeanByMotionModel) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.0);  // zero process noise
  ParticleFilterEstimator pf(motion, 2000, 5.0, 0.5, 7);
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  // Inject a deterministic velocity into every particle so the predict step
  // has something to advance.
  for (int i = 0; i < t.particles.cols(); ++i) {
    t.particles(2, i) = 5.0;
    t.particles(3, i) = -3.0;
  }
  pf.predict(t, Timestamp::fromSeconds(2.0));
  EXPECT_NEAR(t.state(0), 10.0, 0.3);
  EXPECT_NEAR(t.state(1), -6.0, 0.3);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 2.0);
}

TEST(ParticleFilterEstimator, PredictGrowsCovarianceWithProcessNoise) {
  auto motion_q0 = std::make_shared<ConstantVelocity2D>(0.0);
  auto motion_q1 = std::make_shared<ConstantVelocity2D>(1.0);
  ParticleFilterEstimator pf0(motion_q0, 2000, 5.0, 0.5, 11);
  ParticleFilterEstimator pf1(motion_q1, 2000, 5.0, 0.5, 11);

  navtracker::Track t0 = pf0.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  navtracker::Track t1 = pf1.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  pf0.predict(t0, Timestamp::fromSeconds(5.0));
  pf1.predict(t1, Timestamp::fromSeconds(5.0));

  // The q=1 predict must add appreciably more position spread than q=0.
  EXPECT_GT(t1.covariance(0, 0), t0.covariance(0, 0) + 1.0);
  EXPECT_GT(t1.covariance(1, 1), t0.covariance(1, 1) + 1.0);
}

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

TEST(ParticleFilterEstimator, PredictOnEmptyEnsembleIsNoOp) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 500, 5.0, 0.5, 0);
  // Initiate with a non-PD measurement covariance to land in the
  // fallback path (no particles populated).
  Measurement bad;
  bad.time = Timestamp::fromSeconds(0.0);
  bad.model = MeasurementModel::Position2D;
  bad.value = Eigen::Vector2d(0.0, 0.0);
  bad.covariance = Eigen::Matrix2d::Zero();  // zero matrix → LLT fails
  bad.source_id = "test";
  navtracker::Track t = pf.initiate(bad);
  ASSERT_EQ(t.particles.cols(), 0);
  // predict must not crash and must leave the track unchanged.
  const Eigen::VectorXd state_before = t.state;
  const navtracker::Timestamp time_before = t.last_update;
  pf.predict(t, Timestamp::fromSeconds(5.0));
  EXPECT_EQ(t.state.size(), state_before.size());
  for (int i = 0; i < state_before.size(); ++i)
    EXPECT_DOUBLE_EQ(t.state(i), state_before(i));
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), time_before.seconds());
}

TEST(ParticleFilterEstimator, UpdateShrinksPositionCovariance) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 2000, 10.0, 0.5, 23);
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 20.0, 0.0));
  const double var_before = t.covariance(0, 0);
  pf.update(t, positionMeas(0.0, 0.0, 1.0, 0.0));
  EXPECT_LT(t.covariance(0, 0), var_before * 0.5);
  EXPECT_LT(t.covariance(1, 1), var_before * 0.5);
  EXPECT_NEAR(t.state(0), 0.0, 1.0);
  EXPECT_NEAR(t.state(1), 0.0, 1.0);
}

TEST(ParticleFilterEstimator, UpdateResamplesWhenEssCollapses) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 1000, 50.0, 0.5, 31);
  // Wide prior so a sharp measurement gives a heavily concentrated weight
  // distribution → ESS collapses → triggers systematic resampling.
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 100.0, 0.0));
  pf.update(t, positionMeas(0.0, 0.0, 0.5, 0.0));
  // Post-resample weights are uniform.
  for (int i = 0; i < t.particle_weights.size(); ++i)
    EXPECT_NEAR(t.particle_weights(i), 1.0 / 1000.0, 1e-12);
}
