#include <gtest/gtest.h>
#include <memory>

#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "ports/ISensorDetectionModel.hpp"

using navtracker::DetectionParams;
using navtracker::EkfEstimator;
using navtracker::ConstantVelocity2D;
using navtracker::FixedSensorDetectionModel;
using navtracker::JpdaAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
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

namespace {

Measurement radarMeas(double x, double y, double t) {
  Measurement m = positionMeas(x, y, t);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = "radar0";
  return m;
}

Measurement lidarMeas(double x, double y, double t) {
  Measurement m = positionMeas(x, y, t);
  m.sensor = SensorKind::Lidar;
  m.source_id = "lidar0";
  return m;
}

}  // namespace

// Per-sensor mode with a single sensor and a uniform table reproduces
// the scalar-ctor result exactly (modulo the per-track miss aggregation,
// which collapses to one factor when only one sensor is present).
TEST(JpdaAssociator, PerSensorSingleSensorMatchesScalar) {
  DetectionParams dp;
  dp.probability_of_detection = 0.9;
  dp.clutter_intensity = 1e-3;
  FixedSensorDetectionModel model(dp);

  JpdaAssociator scalar(9.0, 0.9, 1e-3);
  JpdaAssociator per_sensor(9.0, &model);

  std::vector<Track> tracks = {makeTrackAt(0.0, 0.0), makeTrackAt(10.0, 0.0)};
  std::vector<Measurement> meas = {
      radarMeas(0.5, 0.0, 1.0),
      radarMeas(9.5, 0.0, 1.0),
      radarMeas(5.0, 0.0, 1.0)};

  const auto r_scalar = scalar.associate(tracks, meas);
  const auto r_persen = per_sensor.associate(tracks, meas);

  ASSERT_EQ(r_scalar.betas.rows(), r_persen.betas.rows());
  ASSERT_EQ(r_scalar.betas.cols(), r_persen.betas.cols());
  for (int j = 0; j < r_scalar.betas.rows(); ++j)
    for (int t = 0; t < r_scalar.betas.cols(); ++t)
      EXPECT_NEAR(r_scalar.betas(j, t), r_persen.betas(j, t), 1e-12);
  for (int t = 0; t < r_scalar.beta_0.size(); ++t)
    EXPECT_NEAR(r_scalar.beta_0(t), r_persen.beta_0(t), 1e-12);

  // Homogeneous batch → result.p_d carries the sensor's P_D (consumed by
  // ImmEstimator::softUpdate downstream).
  EXPECT_NEAR(r_persen.p_d, 0.9, 1e-12);
}

// Per-measurement λ_C: when sensor A's clutter density is raised by 1e3,
// only measurements from sensor A see their association weight to the
// nearby track collapse — measurements from sensor B are untouched.
TEST(JpdaAssociator, PerSensorClutterIsolatedToSourceSensor) {
  DetectionParams base;
  base.probability_of_detection = 0.9;
  base.clutter_intensity = 1e-6;

  // Two tracks far apart so each measurement gates to exactly one.
  std::vector<Track> tracks = {makeTrackAt(0.0, 0.0), makeTrackAt(100.0, 0.0)};
  std::vector<Measurement> meas = {
      radarMeas(0.3, 0.0, 1.0),         // near track 0, from radar
      lidarMeas(100.3, 0.0, 1.0)};      // near track 1, from lidar

  // Baseline: identical per-sensor params for radar and lidar.
  FixedSensorDetectionModel model_uniform(base);
  const auto r_uniform =
      JpdaAssociator(9.0, &model_uniform).associate(tracks, meas);

  // Raise lidar's λ_C by 1e3. Radar measurement's association to track 0
  // must be unaffected; lidar measurement's association to track 1 must
  // weaken toward the clutter explanation.
  FixedSensorDetectionModel model_split(base);
  DetectionParams lidar_noisy = base;
  lidar_noisy.clutter_intensity = 1e-3;
  model_split.set(SensorKind::Lidar, MeasurementModel::Position2D, lidar_noisy);
  const auto r_split =
      JpdaAssociator(9.0, &model_split).associate(tracks, meas);

  EXPECT_NEAR(r_uniform.betas(0, 0), r_split.betas(0, 0), 1e-12);
  EXPECT_GT(r_uniform.betas(1, 1), r_split.betas(1, 1));
  EXPECT_LT(r_uniform.beta_0(1), r_split.beta_0(1));

  // Heterogeneous batch → result.p_d falls back to 0 (documented).
  EXPECT_NEAR(r_split.p_d, 0.0, 1e-12);
}

// Out-of-coverage miss: a track outside a sensor's max_range_m is
// charged zero miss penalty for that sensor, exactly matching the
// MhtTracker / TrackTree::branch convention.
TEST(JpdaAssociator, PerSensorOutOfCoverageChargesNoMiss) {
  // Radar at the origin with 50 m range; one track at 1000 m, far
  // outside coverage, with a clean radar return at 1001 m so the
  // detection branch still gates.
  DetectionParams radar_dp;
  radar_dp.probability_of_detection = 0.9;
  radar_dp.clutter_intensity = 1e-6;
  radar_dp.max_range_m = 50.0;
  FixedSensorDetectionModel model(radar_dp);

  std::vector<Track> tracks = {makeTrackAt(1000.0, 0.0)};
  Measurement m = radarMeas(1000.3, 0.0, 1.0);
  m.sensor_position_enu = Eigen::Vector2d::Zero();
  std::vector<Measurement> meas = {m};

  const auto r = JpdaAssociator(9.0, &model).associate(tracks, meas);
  // The miss branch contributes log(1 − 0) = 0 → the only "no detection"
  // explanation is clutter, which at λ=1e-6 is exponentially less
  // likely than the gated detection. β should pin to ~1.
  EXPECT_GT(r.betas(0, 0), 0.99);
  EXPECT_LT(r.beta_0(0), 0.01);
}
