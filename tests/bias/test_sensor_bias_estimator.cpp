#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/bias/SensorBiasPairExtractor.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

Timestamp tsSeconds(double s) {
  return Timestamp::fromSeconds(s);
}

Eigen::Matrix2d iso(double sigma) {
  Eigen::Matrix2d m = Eigen::Matrix2d::Zero();
  m(0, 0) = sigma * sigma;
  m(1, 1) = sigma * sigma;
  return m;
}

PositionBiasPairObservation makePosObs(SensorKind k, const std::string& src,
                                        Timestamp time,
                                        const Eigen::Vector2d& true_b,
                                        const Eigen::Vector2d& truth) {
  PositionBiasPairObservation obs;
  obs.time = time;
  obs.key = {k, src};
  obs.z_anchor_enu = truth;
  obs.z_sensor_enu = truth + true_b;
  obs.R_sensor = iso(2.0);
  obs.R_anchor = iso(1.0);
  obs.own_position_enu = Eigen::Vector2d(0.0, 0.0);
  return obs;
}

// Convergence: a constant true bias is recovered within 20 noiseless
// observations.
TEST(SensorBiasEstimator, PositionConvergesToTrueBias) {
  SensorBiasEstimator est;
  const Eigen::Vector2d true_b(3.0, -2.0);
  const SensorBiasKey key{SensorKind::Lidar, "lidar0"};
  Eigen::Vector2d truth(200.0, 100.0);
  for (int i = 0; i < 20; ++i) {
    truth.x() += 1.0;
    auto obs = makePosObs(key.sensor, key.source_id,
                          tsSeconds(0.1 * (i + 1)), true_b, truth);
    est.observe(obs);
  }
  const auto pb = est.positionBias(key);
  EXPECT_NEAR(pb.bias_enu_m.x(), true_b.x(), 0.3);
  EXPECT_NEAR(pb.bias_enu_m.y(), true_b.y(), 0.3);
  EXPECT_TRUE(pb.is_published);
}

// Range gate: at near range the observation is rejected.
TEST(SensorBiasEstimator, PositionRangeGateRejectsNearRange) {
  SensorBiasEstimatorConfig cfg;
  cfg.min_range_m = 50.0;
  SensorBiasEstimator est(cfg);
  const SensorBiasKey key{SensorKind::Lidar, "lidar0"};
  const Eigen::Vector2d true_b(2.0, 0.0);
  const Eigen::Vector2d near_truth(20.0, 0.0);
  auto obs = makePosObs(key.sensor, key.source_id, tsSeconds(0.1), true_b,
                        near_truth);
  est.observe(obs);
  EXPECT_EQ(est.acceptedPosObs(), 0u);
  EXPECT_EQ(est.rejectedByRange(), 1u);
}

// Outlier gate: a wildly mismatched pair does not pull the state.
TEST(SensorBiasEstimator, PositionOutlierGateRejects) {
  SensorBiasEstimatorConfig cfg;
  cfg.outlier_sigma = 3.0;
  cfg.initial_pos_std_m = 1.0;  // tight prior so 1000-m residual is far
  SensorBiasEstimator est(cfg);
  const SensorBiasKey key{SensorKind::ArpaTtm, "radar0"};
  PositionBiasPairObservation obs;
  obs.time = tsSeconds(0.1);
  obs.key = key;
  obs.z_anchor_enu = Eigen::Vector2d(500.0, 0.0);
  obs.z_sensor_enu = Eigen::Vector2d(1500.0, 0.0);  // 1 km off
  obs.R_sensor = iso(1.0);
  obs.R_anchor = iso(1.0);
  est.observe(obs);
  const auto pb = est.positionBias(key);
  EXPECT_LT(pb.bias_enu_m.norm(), 0.5);
  EXPECT_EQ(est.rejectedByOutlier(), 1u);
}

// Independence: per-(sensor, source_id) keys do not contaminate each other.
TEST(SensorBiasEstimator, PerKeyIndependent) {
  SensorBiasEstimator est;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  const SensorBiasKey radar{SensorKind::ArpaTtm, "radar0"};
  const Eigen::Vector2d truth(300.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    Eigen::Vector2d tr(300.0 + i, 0.0);
    auto o1 = makePosObs(lidar.sensor, lidar.source_id,
                         tsSeconds(0.1 * (i + 1)),
                         Eigen::Vector2d(5.0, 0.0), tr);
    auto o2 = makePosObs(radar.sensor, radar.source_id,
                         tsSeconds(0.1 * (i + 1)),
                         Eigen::Vector2d(0.0, -3.0), tr);
    est.observe(o1);
    est.observe(o2);
  }
  EXPECT_NEAR(est.positionBias(lidar).bias_enu_m.x(), 5.0, 0.5);
  EXPECT_NEAR(est.positionBias(lidar).bias_enu_m.y(), 0.0, 0.5);
  EXPECT_NEAR(est.positionBias(radar).bias_enu_m.x(), 0.0, 0.5);
  EXPECT_NEAR(est.positionBias(radar).bias_enu_m.y(), -3.0, 0.5);
}

// Bearing-bias estimator: scalar convergence.
TEST(SensorBiasEstimator, BearingConvergesToTrueBias) {
  SensorBiasEstimator est;
  const SensorBiasKey cam{SensorKind::EoIr, "cam0"};
  const double true_b = 0.02;  // ~1.15 deg
  const Eigen::Vector2d sensor_pos(0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    Eigen::Vector2d target(200.0 + i * 5.0, 100.0 - i * 2.0);
    const double alpha_true = std::atan2(target.y() - sensor_pos.y(),
                                          target.x() - sensor_pos.x());
    BearingBiasPairObservation obs;
    obs.time = tsSeconds(0.1 * (i + 1));
    obs.key = cam;
    obs.sensor_position_enu = sensor_pos;
    obs.anchor_target_position_enu = target;
    obs.alpha_observed_rad = alpha_true + true_b;
    obs.alpha_meas_var_rad2 = (2.0 * kPi / 180.0) * (2.0 * kPi / 180.0);
    obs.anchor_position_std_m = 5.0;
    est.observe(obs);
  }
  const auto bb = est.bearingBias(cam);
  EXPECT_NEAR(bb.bias_rad, true_b, 0.01);
}

// Initial prior: positionBias returns is_published=false.
TEST(SensorBiasEstimator, UnobservedKeyReturnsNotPublished) {
  SensorBiasEstimator est;
  const SensorBiasKey k{SensorKind::Lidar, "neverseen"};
  const auto pb = est.positionBias(k);
  EXPECT_FALSE(pb.is_published);
  EXPECT_NEAR(pb.bias_enu_m.x(), 0.0, 1e-9);
  EXPECT_NEAR(pb.bias_enu_m.y(), 0.0, 1e-9);
}

// Extractor pulls (AIS, lidar) pairs from track contributions.
TEST(SensorBiasPairExtractor, EmitsPairFromAisAndLidarContributions) {
  Track tr;
  tr.id = TrackId{1};
  Track::SourceTouch ais;
  ais.sensor = SensorKind::Ais;
  ais.source_id = "ais0";
  ais.time = tsSeconds(1.0);
  ais.value_enu = Eigen::Vector2d(500.0, 100.0);
  ais.covariance = iso(8.0);
  tr.recent_contributions.push_back(ais);
  Track::SourceTouch lidar;
  lidar.sensor = SensorKind::Lidar;
  lidar.source_id = "lidar0";
  lidar.time = tsSeconds(1.1);
  lidar.value_enu = Eigen::Vector2d(502.0, 99.0);
  lidar.covariance = iso(2.0);
  tr.recent_contributions.push_back(lidar);

  std::vector<Track> tracks{tr};
  const auto pairs = extractPositionPairs(tracks, tsSeconds(1.1));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].key.sensor, SensorKind::Lidar);
  EXPECT_EQ(pairs[0].key.source_id, "lidar0");
  EXPECT_NEAR(pairs[0].z_anchor_enu.x(), 500.0, 1e-9);
  EXPECT_NEAR(pairs[0].z_sensor_enu.x(), 502.0, 1e-9);
}

// FixedSensorBiasProvider: known offsets publish immediately.
TEST(FixedSensorBiasProvider, ConfiguredKeyPublishesKnownOffset) {
  FixedSensorBiasProvider prov;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  prov.setPositionBias(lidar, Eigen::Vector2d(1.5, -0.8));
  const auto pb = prov.positionBias(lidar);
  EXPECT_TRUE(pb.is_published);
  EXPECT_NEAR(pb.bias_enu_m.x(), 1.5, 1e-12);
  EXPECT_NEAR(pb.bias_enu_m.y(), -0.8, 1e-12);

  const SensorBiasKey unknown{SensorKind::Lidar, "neverseen"};
  EXPECT_FALSE(prov.positionBias(unknown).is_published);
}

// setKnownPositionBias with tight prior: publishes immediately and the
// estimate equals the seed.
TEST(SensorBiasPairExtractor, EmitsBearingPairFromAisAndEoirContributions) {
  Track tr;
  tr.id = TrackId{42};
  // Anchor at (100, 0) ENU, camera at origin → predicted α = 0.
  Track::SourceTouch ais;
  ais.sensor = SensorKind::Ais;
  ais.source_id = "ais0";
  ais.time = tsSeconds(1.0);
  ais.value_enu = Eigen::Vector2d(100.0, 0.0);
  ais.covariance = iso(5.0);
  tr.recent_contributions.push_back(ais);
  // EO measurement reports 0.05 rad → bearing-bias observation of +0.05.
  Track::SourceTouch eo;
  eo.sensor = SensorKind::EoIr;
  eo.source_id = "eo0";
  eo.time = tsSeconds(1.05);
  eo.sensor_position_enu = Eigen::Vector2d(0.0, 0.0);
  eo.alpha_rad = 0.05;
  eo.alpha_var_rad2 = 1e-3;
  tr.recent_contributions.push_back(eo);

  std::vector<Track> tracks{tr};
  const auto pairs = extractBearingPairs(tracks, tsSeconds(1.05));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].key.sensor, SensorKind::EoIr);
  EXPECT_EQ(pairs[0].key.source_id, "eo0");
  EXPECT_NEAR(pairs[0].alpha_observed_rad, 0.05, 1e-12);
  EXPECT_NEAR(pairs[0].alpha_meas_var_rad2, 1e-3, 1e-15);
  EXPECT_NEAR(pairs[0].anchor_target_position_enu.x(), 100.0, 1e-12);
  EXPECT_GT(pairs[0].anchor_position_std_m, 0.0);
}

TEST(SensorBiasPairExtractor, SkipsBearingTouchesWithoutAlphaPayload) {
  // A Bearing2D touch whose alpha_rad is still NaN (e.g. a legacy
  // populator that never ran the bearing branch) must not crash the
  // extractor; the pair is simply skipped.
  Track tr;
  tr.id = TrackId{1};
  Track::SourceTouch ais;
  ais.sensor = SensorKind::Ais;
  ais.time = tsSeconds(1.0);
  ais.value_enu = Eigen::Vector2d(100.0, 0.0);
  ais.covariance = iso(5.0);
  tr.recent_contributions.push_back(ais);
  Track::SourceTouch eo;
  eo.sensor = SensorKind::EoIr;
  eo.source_id = "eo0";
  eo.time = tsSeconds(1.05);
  // alpha_rad left as default NaN.
  tr.recent_contributions.push_back(eo);

  std::vector<Track> tracks{tr};
  const auto pairs = extractBearingPairs(tracks, tsSeconds(1.05));
  EXPECT_EQ(pairs.size(), 0u);
}

TEST(SensorBiasEstimator, SetKnownPositionBiasTightPriorPublishes) {
  SensorBiasEstimator est;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  est.setKnownPositionBias(lidar, Eigen::Vector2d(2.0, 0.0),
                           Eigen::Matrix2d::Identity() * 0.25);
  const auto pb = est.positionBias(lidar);
  EXPECT_TRUE(pb.is_published);
  EXPECT_NEAR(pb.bias_enu_m.x(), 2.0, 1e-12);
  EXPECT_NEAR(pb.bias_enu_m.y(), 0.0, 1e-12);
}

// setKnownPositionBias with loose prior: observations dominate and pull
// the posterior to the true bias. The "I have rough numbers, let
// observations refine" workflow.
TEST(SensorBiasEstimator, SetKnownPositionBiasLoosePriorRefines) {
  SensorBiasEstimator est;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  // Loose prior: σ = 5 m per axis. Observations (σ_obs ≈ √5 ≈ 2.2 m)
  // dominate after ~20 pairs.
  est.setKnownPositionBias(lidar, Eigen::Vector2d(0.0, 0.0),
                           Eigen::Matrix2d::Identity() * 25.0);
  const Eigen::Vector2d true_b(3.0, -2.0);
  Eigen::Vector2d truth(200.0, 100.0);
  for (int i = 0; i < 20; ++i) {
    truth.x() += 1.0;
    auto obs = makePosObs(lidar.sensor, lidar.source_id,
                          tsSeconds(0.1 * (i + 1)), true_b, truth);
    est.observe(obs);
  }
  const auto pb = est.positionBias(lidar);
  EXPECT_NEAR(pb.bias_enu_m.x(), true_b.x(), 0.4);
  EXPECT_NEAR(pb.bias_enu_m.y(), true_b.y(), 0.4);
  EXPECT_TRUE(pb.is_published);
}

}  // namespace
}  // namespace navtracker
