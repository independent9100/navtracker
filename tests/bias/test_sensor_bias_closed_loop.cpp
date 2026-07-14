// W3.2 — per-sensor bias loop, same closed-loop double-subtraction as W3.1.
//
// In the shipped item-9 wiring the tracker subtracts the published per-sensor
// bias from every measurement (applyBiasCorrection) BEFORE the SourceTouch is
// recorded, so the pair the extractor forms shows only the residual bias. The
// estimator's r = z_sensor − z_anchor − b̂ then subtracts b̂ a second time and
// the fixed point is b̂ = b_true/2. The fix: the touch carries the applied
// correction (applied_position_bias_enu) and the extractor reconstructs the
// raw measurement.
//
// Teeth: the SAME closed loop converges to the full bias with reconstruction
// and to half without.

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/bias/SensorBiasPairExtractor.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/types/Track.hpp"

namespace navtracker {
namespace {

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

Track::SourceTouch touch(SensorKind k, Timestamp t, Eigen::Vector2d v,
                         Eigen::Vector2d applied_pos_bias) {
  Track::SourceTouch s;
  s.sensor = k;
  s.source_id = "radar";
  s.time = t;
  s.value_enu = v;
  s.sensor_position_enu = Eigen::Vector2d::Zero();
  s.covariance = Eigen::Matrix2d::Identity() * 4.0;  // 2 m std sensor noise
  s.applied_position_bias_enu = applied_pos_bias;
  return s;
}

Eigen::Vector2d runClosedLoop(
    Eigen::Vector2d b_true, bool carry,
    Eigen::Vector2d anchor_applied = Eigen::Vector2d::Zero()) {
  SensorBiasEstimator est{};
  const Eigen::Vector2d truth(1200.0, 300.0);
  Eigen::Vector2d b_pub = Eigen::Vector2d::Zero();
  const SensorBiasKey key{SensorKind::ArpaTtm, "radar"};
  for (int i = 0; i < 300; ++i) {
    // Sensor reports truth + b_true; pipeline subtracts b_pub.
    const Eigen::Vector2d sensor_corrected = truth + b_true - b_pub;
    Track tr;
    // Anchor (AIS): its CORRECTED position is truth. anchor_applied models a
    // (hypothetical) published anchor position bias the pipeline subtracted —
    // the estimator must ignore it (the anchor is the truth reference).
    tr.recent_contributions.push_back(
        touch(SensorKind::Ais, tAt(i * 1.0), truth, anchor_applied));
    tr.recent_contributions.push_back(
        touch(SensorKind::ArpaTtm, tAt(i * 1.0), sensor_corrected,
              carry ? b_pub : Eigen::Vector2d::Zero()));
    for (const auto& obs : extractPositionPairs({tr}, tAt(i * 1.0))) {
      est.observe(obs);
    }
    const auto e = est.positionBias(key);
    if (e.is_published) b_pub = e.bias_enu_m;
  }
  return est.positionBias(key).bias_enu_m;
}

}  // namespace

TEST(SensorBiasClosedLoop, ConvergesToFullBiasWithReconstruction) {
  const Eigen::Vector2d b_true(4.0, -2.0);
  const Eigen::Vector2d b_hat = runClosedLoop(b_true, /*carry=*/true);
  EXPECT_NEAR(b_hat.x(), b_true.x(), 0.2);
  EXPECT_NEAR(b_hat.y(), b_true.y(), 0.2);
}

TEST(SensorBiasClosedLoop, WithoutReconstructionConvergesToHalf) {
  const Eigen::Vector2d b_true(4.0, -2.0);
  const Eigen::Vector2d b_hat = runClosedLoop(b_true, /*carry=*/false);
  EXPECT_NEAR(b_hat.x(), 0.5 * b_true.x(), 0.2);
  EXPECT_NEAR(b_hat.y(), 0.5 * b_true.y(), 0.2);
}

TEST(SensorBiasClosedLoop, InvariantToAnchorAppliedPositionBias) {
  // The anchor is the truth reference: the estimator uses its CORRECTED
  // position, so a (hypothetical) published anchor position bias must NOT leak
  // into the sensor estimate. Adding the anchor's applied bias back (the
  // pre-guard code) drove the sensor estimate to b_sensor − b_anchor.
  const Eigen::Vector2d b_true(4.0, -2.0);
  const Eigen::Vector2d anchor_bias(6.0, 5.0);
  const Eigen::Vector2d b_hat =
      runClosedLoop(b_true, /*carry=*/true, anchor_bias);
  EXPECT_NEAR(b_hat.x(), b_true.x(), 0.2);
  EXPECT_NEAR(b_hat.y(), b_true.y(), 0.2);
}

}  // namespace navtracker
