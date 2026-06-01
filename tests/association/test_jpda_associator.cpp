#include <gtest/gtest.h>
#include <memory>

#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"

using navtracker::EkfEstimator;
using navtracker::ConstantVelocity2D;
using navtracker::JpdaAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;

namespace {

Measurement positionMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "t";
  return m;
}

Track makeTrackAt(double x, double y) {
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::VectorXd::Zero(4);
  t.state(0) = x; t.state(1) = y;
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  return t;
}

}  // namespace

TEST(JpdaAssociator, ReturnsSoftFieldsNotHardMatches) {
  JpdaAssociator jpda(9.0, 0.9, 1e-5);
  std::vector<Track> tracks = {makeTrackAt(0.0, 0.0)};
  std::vector<Measurement> meas = {positionMeas(0.1, 0.0, 1.0)};
  const auto r = jpda.associate(tracks, meas);
  EXPECT_TRUE(r.matches.empty());
  EXPECT_EQ(r.betas.rows(), 1);
  EXPECT_EQ(r.betas.cols(), 1);
  EXPECT_EQ(r.beta_0.size(), 1);
  EXPECT_GT(r.betas(0, 0), 0.5);
}

TEST(JpdaAssociator, UngatedMeasurementGetsZeroBeta) {
  JpdaAssociator jpda(9.0, 0.9, 1e-5);
  std::vector<Track> tracks = {makeTrackAt(0.0, 0.0)};
  std::vector<Measurement> meas = {positionMeas(1000.0, 1000.0, 1.0)};
  const auto r = jpda.associate(tracks, meas);
  ASSERT_EQ(r.betas.rows(), 1);
  ASSERT_EQ(r.betas.cols(), 1);
  EXPECT_NEAR(r.betas(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(r.beta_0(0), 1.0, 1e-9);
}

TEST(JpdaAssociator, BetaColumnPlusBeta0EqualsOneAcrossMeasurements) {
  JpdaAssociator jpda(9.0, 0.9, 1e-3);
  std::vector<Track> tracks = {makeTrackAt(0.0, 0.0), makeTrackAt(10.0, 0.0)};
  std::vector<Measurement> meas = {
      positionMeas(0.5, 0.0, 1.0),
      positionMeas(9.5, 0.0, 1.0),
      positionMeas(5.0, 0.0, 1.0)};
  const auto r = jpda.associate(tracks, meas);
  ASSERT_EQ(r.betas.cols(), 2);
  for (int t = 0; t < 2; ++t) {
    double col_sum = 0.0;
    for (int j = 0; j < r.betas.rows(); ++j) col_sum += r.betas(j, t);
    EXPECT_NEAR(col_sum + r.beta_0(t), 1.0, 1e-9);
  }
}

TEST(JpdaAssociator, EmptyInputsGiveEmptyResult) {
  JpdaAssociator jpda(9.0, 0.9, 1e-5);
  const auto r = jpda.associate({}, {});
  EXPECT_EQ(r.betas.rows(), 0);
  EXPECT_EQ(r.betas.cols(), 0);
  EXPECT_EQ(r.beta_0.size(), 0);
}
