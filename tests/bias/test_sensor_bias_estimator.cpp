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

TEST(SensorBiasPairExtractor, EmitsPairFromCooperativeAndLidarContributions) {
  Track tr;
  tr.id = TrackId{1};
  Track::SourceTouch coop;
  coop.sensor = SensorKind::Cooperative;
  coop.source_id = "consort0";
  coop.time = tsSeconds(1.0);
  coop.value_enu = Eigen::Vector2d(500.0, 100.0);
  coop.covariance = iso(2.0);
  tr.recent_contributions.push_back(coop);
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

// Tight seed alone does NOT publish (since 2026-06-19 anchor-gating):
// the seed is a hypothesis, not a measurement, so it must wait for an
// observation to confirm. Callers that *want* the seed to publish
// immediately (offline calibration trusted as ground truth) must pass
// treat_as_anchored=true.
TEST(SensorBiasEstimator, SetKnownPositionBiasTightPriorDoesNotPublishByDefault) {
  SensorBiasEstimator est;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  est.setKnownPositionBias(lidar, Eigen::Vector2d(2.0, 0.0),
                           Eigen::Matrix2d::Identity() * 0.25);
  const auto pb = est.positionBias(lidar);
  EXPECT_FALSE(pb.is_published);
  EXPECT_NEAR(pb.bias_enu_m.x(), 2.0, 1e-12);
  EXPECT_NEAR(pb.bias_enu_m.y(), 0.0, 1e-12);
}

TEST(SensorBiasEstimator, SetKnownPositionBiasTightAnchoredPriorPublishes) {
  SensorBiasEstimator est;
  const SensorBiasKey lidar{SensorKind::Lidar, "lidar0"};
  est.setKnownPositionBias(lidar, Eigen::Vector2d(2.0, 0.0),
                           Eigen::Matrix2d::Identity() * 0.25,
                           /*treat_as_anchored=*/true);
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

// --- Cross-sensor anchored extractor (backlog item 13) -------------

namespace cross_sensor {

Track::SourceTouch posTouch(SensorKind k, const std::string& src,
                            Timestamp time,
                            const Eigen::Vector2d& value) {
  Track::SourceTouch t;
  t.sensor = k;
  t.source_id = src;
  t.time = time;
  t.value_enu = value;
  t.covariance = iso(2.0);
  t.sensor_position_enu = Eigen::Vector2d::Zero();
  return t;
}

Track makeTrack(double existence, double pos_cov_per_axis,
                std::vector<Track::SourceTouch> touches) {
  Track tr;
  tr.id = TrackId{1};
  tr.existence_probability = existence;
  tr.covariance = Eigen::MatrixXd::Zero(4, 4);
  tr.covariance(0, 0) = pos_cov_per_axis;
  tr.covariance(1, 1) = pos_cov_per_axis;
  tr.recent_contributions = std::move(touches);
  return tr;
}

}  // namespace cross_sensor

// Eligibility: a converged AIS-less track with one radar + one lidar
// contribution yields the two symmetric pairs.
TEST(SensorBiasPairExtractor, CrossSensorEmitsSymmetricPairs) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  const auto pairs = extractCrossSensorPositionPairs(
      {tr}, tsSeconds(1.0), /*bias_provider=*/nullptr);
  ASSERT_EQ(pairs.size(), 2u);
  // Both directions present.
  bool saw_lidar_anchored_on_radar = false;
  bool saw_radar_anchored_on_lidar = false;
  for (const auto& p : pairs) {
    if (p.key.sensor == SensorKind::Lidar) {
      saw_radar_anchored_on_lidar = true;
      EXPECT_NEAR(p.z_sensor_enu.x(), 500.0, 1e-9);
      EXPECT_NEAR(p.z_anchor_enu.x(), 503.0, 1e-9);
    } else if (p.key.sensor == SensorKind::ArpaTtm) {
      saw_lidar_anchored_on_radar = true;
      EXPECT_NEAR(p.z_sensor_enu.x(), 503.0, 1e-9);
      EXPECT_NEAR(p.z_anchor_enu.x(), 500.0, 1e-9);
    }
  }
  EXPECT_TRUE(saw_lidar_anchored_on_radar);
  EXPECT_TRUE(saw_radar_anchored_on_lidar);
}

// 1A: each calibrated key gets exactly ONE observation per cycle, even
// when N>=3 positional contributions are present. The sensor's own
// sample is a single measurement; replaying it against every other
// anchor as independent KF updates double-counts it and over-shrinks
// the bias covariance. Three distinct keys must yield three
// observations (one per calibrated key), not N*(N-1)=6.
TEST(SensorBiasPairExtractor, CrossSensorEmitsOneObservationPerKey) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0)),
       posTouch(SensorKind::ArpaTll, "radar1", tsSeconds(1.0),
                Eigen::Vector2d(501.0, 101.0))});
  const auto pairs = extractCrossSensorPositionPairs(
      {tr}, tsSeconds(1.0), /*bias_provider=*/nullptr);
  ASSERT_EQ(pairs.size(), 3u);
  int lidar = 0, ttm = 0, tll = 0;
  for (const auto& p : pairs) {
    if (p.key.sensor == SensorKind::Lidar) ++lidar;
    else if (p.key.sensor == SensorKind::ArpaTtm) ++ttm;
    else if (p.key.sensor == SensorKind::ArpaTll) ++tll;
    // The chosen anchor must never be the calibrated key itself.
    EXPECT_NE(p.z_sensor_enu, p.z_anchor_enu);
  }
  EXPECT_EQ(lidar, 1);
  EXPECT_EQ(ttm, 1);
  EXPECT_EQ(tll, 1);
}

// 1B: contributions from the *same physical sensor* must not anchor
// each other. ARPA TTM and TLL carry distinct SensorKinds but share
// the adapter's source_id ("arpa") — they are the same hardware and
// therefore share the same mounting/registration bias. Cross-anchoring
// them yields a near-zero residual regardless of the true common
// offset, masking it. With these two as the only positional keys there
// is no valid partner for either, so no pairs are emitted.
TEST(SensorBiasPairExtractor, CrossSensorSkipsSameSourceIdHardware) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::ArpaTtm, "arpa", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTll, "arpa", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  EXPECT_TRUE(extractCrossSensorPositionPairs(
                  {tr}, tsSeconds(1.0), nullptr)
                  .empty());
}

// 1B: with a third sensor on a different source_id, the same-hardware
// pair (TTM/TLL "arpa") still never anchors itself, but each can be
// calibrated against the independent lidar. Each of the three keys
// emits one observation, and the two "arpa" keys are anchored on the
// lidar (only valid partner), never on each other.
TEST(SensorBiasPairExtractor, CrossSensorSameHardwareAnchorsOnThirdSensor) {
  using namespace cross_sensor;
  const Eigen::Vector2d lidar_val(490.0, 110.0);
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::ArpaTtm, "arpa", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTll, "arpa", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0)),
       posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0), lidar_val)});
  const auto pairs = extractCrossSensorPositionPairs(
      {tr}, tsSeconds(1.0), nullptr);
  ASSERT_EQ(pairs.size(), 3u);
  for (const auto& p : pairs) {
    if (p.key.sensor == SensorKind::ArpaTtm ||
        p.key.sensor == SensorKind::ArpaTll) {
      // Only valid anchor is the lidar (no provider bias → raw value).
      EXPECT_NEAR(p.z_anchor_enu.x(), lidar_val.x(), 1e-9);
      EXPECT_NEAR(p.z_anchor_enu.y(), lidar_val.y(), 1e-9);
    }
  }
}

// Finding 2: emission order is deterministic (the calibrated keys come
// out in SensorBiasKey operator< order, independent of the order the
// contributions appear in recent_contributions). The sequential KF
// gate makes the fold order-sensitive at the margins, so a stable order
// is required by the determinism invariant.
TEST(SensorBiasPairExtractor, CrossSensorEmitsKeysInDeterministicOrder) {
  using namespace cross_sensor;
  // Insert contributions in a deliberately unsorted order.
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(501.0, 101.0)),
       posTouch(SensorKind::ArpaTll, "radar1", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0))});
  const auto pairs = extractCrossSensorPositionPairs(
      {tr}, tsSeconds(1.0), nullptr);
  ASSERT_EQ(pairs.size(), 3u);
  for (std::size_t i = 1; i < pairs.size(); ++i) {
    EXPECT_TRUE(pairs[i - 1].key < pairs[i].key)
        << "observations must be emitted in ascending SensorBiasKey order";
  }
}

// Low-existence tracks are not eligible to anchor.
TEST(SensorBiasPairExtractor, CrossSensorSkipsLowExistenceTrack) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.5, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  EXPECT_TRUE(extractCrossSensorPositionPairs(
                  {tr}, tsSeconds(1.0), nullptr)
                  .empty());
}

// Loose-position tracks are not eligible to anchor.
TEST(SensorBiasPairExtractor, CrossSensorSkipsLoosePositionTrack) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 50.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  EXPECT_TRUE(extractCrossSensorPositionPairs(
                  {tr}, tsSeconds(1.0), nullptr)
                  .empty());
}

// AIS contribution in the window suppresses the cross-sensor path —
// the AIS-anchored extractor handles those tracks.
TEST(SensorBiasPairExtractor, CrossSensorSkipsTracksWithAis) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Ais, "ais0", tsSeconds(1.0),
                Eigen::Vector2d(501.0, 99.0)),
       posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  EXPECT_TRUE(extractCrossSensorPositionPairs(
                  {tr}, tsSeconds(1.0), nullptr)
                  .empty());
}

// A single positional contribution gives nothing to pair with.
TEST(SensorBiasPairExtractor, CrossSensorNoPairFromSingleContribution) {
  using namespace cross_sensor;
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0))});
  EXPECT_TRUE(extractCrossSensorPositionPairs(
                  {tr}, tsSeconds(1.0), nullptr)
                  .empty());
}

// Schmidt-KF fold: when the anchor's bias is published the extractor
// debiases the anchor measurement and folds the bias covariance into
// R_anchor.
TEST(SensorBiasPairExtractor, CrossSensorAppliesSchmidtFoldFromProvider) {
  using namespace cross_sensor;
  FixedSensorBiasProvider prov;
  const SensorBiasKey lidar_key{SensorKind::Lidar, "lidar0"};
  prov.setPositionBias(lidar_key, Eigen::Vector2d(2.0, -1.0),
                       Eigen::Matrix2d::Identity() * 0.5);
  auto tr = makeTrack(
      0.99, 4.0,
      {posTouch(SensorKind::Lidar, "lidar0", tsSeconds(1.0),
                Eigen::Vector2d(500.0, 100.0)),
       posTouch(SensorKind::ArpaTtm, "radar0", tsSeconds(1.0),
                Eigen::Vector2d(503.0, 98.0))});
  const auto pairs = extractCrossSensorPositionPairs(
      {tr}, tsSeconds(1.0), &prov);
  ASSERT_EQ(pairs.size(), 2u);
  // The radar-anchored-on-lidar observation has its anchor debiased.
  for (const auto& p : pairs) {
    if (p.key.sensor == SensorKind::ArpaTtm) {
      // z_anchor = z_lidar - b_lidar = (500, 100) - (2, -1) = (498, 101)
      EXPECT_NEAR(p.z_anchor_enu.x(), 498.0, 1e-9);
      EXPECT_NEAR(p.z_anchor_enu.y(), 101.0, 1e-9);
      // R_anchor includes the +0.5 inflation along the diagonal.
      EXPECT_GE(p.R_anchor(0, 0), 4.5 - 1e-9);
      EXPECT_GE(p.R_anchor(1, 1), 4.5 - 1e-9);
    }
  }
}

// Joint convergence: with the prior breaking symmetry, the bias
// estimator on cross-sensor pairs converges to a near-zero-bias
// solution when both sensors are actually unbiased (truth = no
// mounting offset). This is the symmetry-breaking sanity check the
// item-13 spec calls out.
TEST(SensorBiasEstimator, CrossSensorPairsConvergeUnderPrior) {
  using namespace cross_sensor;
  SensorBiasEstimator est;
  const SensorBiasKey lidar_key{SensorKind::Lidar, "lidar0"};
  const SensorBiasKey radar_key{SensorKind::ArpaTtm, "radar0"};

  // Walk the target through 50 cycles. Both sensors see the truth
  // perfectly. The prior pulls each bias to zero.
  Eigen::Vector2d truth(500.0, 100.0);
  for (int i = 0; i < 50; ++i) {
    truth.x() += 1.0;
    auto tr = makeTrack(
        0.99, 4.0,
        {posTouch(SensorKind::Lidar, "lidar0",
                  tsSeconds(0.1 * (i + 1)), truth),
         posTouch(SensorKind::ArpaTtm, "radar0",
                  tsSeconds(0.1 * (i + 1)), truth)});
    const auto pairs = extractCrossSensorPositionPairs(
        {tr}, tsSeconds(0.1 * (i + 1)), &est);
    est.predictTo(tsSeconds(0.1 * (i + 1)));
    for (const auto& p : pairs) est.observe(p);
  }
  // Expected: both biases land near zero (within ~0.5 m of truth).
  const auto pb_l = est.positionBias(lidar_key);
  const auto pb_r = est.positionBias(radar_key);
  EXPECT_NEAR(pb_l.bias_enu_m.norm(), 0.0, 0.5);
  EXPECT_NEAR(pb_r.bias_enu_m.norm(), 0.0, 0.5);
}

// Asymmetric bias recovery: when one sensor (radar) is unbiased and
// has a tight prior pinning it, the cross-sensor extractor recovers
// the OTHER sensor's (lidar) true bias through the cross-sensor
// pairs. This is the deployment-relevant scenario: one sensor has
// been calibrated via AIS earlier, the other is being calibrated
// against it now.
TEST(SensorBiasEstimator, CrossSensorRecoversBiasWithPinnedAnchor) {
  using namespace cross_sensor;
  SensorBiasEstimator est;
  const SensorBiasKey lidar_key{SensorKind::Lidar, "lidar0"};
  const SensorBiasKey radar_key{SensorKind::ArpaTtm, "radar0"};

  // Pin radar bias to zero via a tight seed marked as anchored
  // (simulates earlier AIS-anchored convergence — the operator is
  // asserting "this sensor's bias is known"). The treat_as_anchored
  // flag makes the seed publish immediately, which the cross-sensor
  // extractor then propagates to the lidar pairs.
  est.setKnownPositionBias(radar_key, Eigen::Vector2d::Zero(),
                           Eigen::Matrix2d::Identity() * 0.01,
                           /*treat_as_anchored=*/true);

  const Eigen::Vector2d lidar_bias(3.0, -2.0);
  Eigen::Vector2d truth(500.0, 100.0);
  for (int i = 0; i < 60; ++i) {
    truth.x() += 1.0;
    // Lidar reports truth + bias; radar reports truth.
    auto tr = makeTrack(
        0.99, 4.0,
        {posTouch(SensorKind::Lidar, "lidar0",
                  tsSeconds(0.1 * (i + 1)), truth + lidar_bias),
         posTouch(SensorKind::ArpaTtm, "radar0",
                  tsSeconds(0.1 * (i + 1)), truth)});
    const auto pairs = extractCrossSensorPositionPairs(
        {tr}, tsSeconds(0.1 * (i + 1)), &est);
    est.predictTo(tsSeconds(0.1 * (i + 1)));
    for (const auto& p : pairs) est.observe(p);
  }
  const auto pb_l = est.positionBias(lidar_key);
  // Lidar bias should be recovered (within ~0.6 m, looser tolerance
  // than the AIS-anchored case because the radar's anchor-side noise
  // still gets folded into R_anchor through Schmidt's path).
  EXPECT_NEAR(pb_l.bias_enu_m.x(), lidar_bias.x(), 0.6);
  EXPECT_NEAR(pb_l.bias_enu_m.y(), lidar_bias.y(), 0.6);
  EXPECT_TRUE(pb_l.is_published);
}

}  // namespace
}  // namespace navtracker
