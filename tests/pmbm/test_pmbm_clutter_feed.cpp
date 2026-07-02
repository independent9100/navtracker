#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorDetectionModel.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::ISensorDetectionModel;
using navtracker::DetectionParams;
using navtracker::pmbm::PmbmTracker;

namespace {
// Records every observe() bundle; returns a fixed baseline for paramsFor.
struct SpyDetectionModel : ISensorDetectionModel {
  int observe_calls = 0;
  std::vector<std::vector<ISensorDetectionModel::ScanObservation>> bundles;
  DetectionParams paramsFor(SensorKind /*sensor*/,
                            MeasurementModel /*model*/) const override {
    DetectionParams p;
    p.probability_of_detection = 0.9;
    p.clutter_intensity = 1e-4;
    return p;
  }
  void observe(const std::vector<ISensorDetectionModel::ScanObservation>& by_sensor) override {
    ++observe_calls;
    bundles.push_back(by_sensor);
  }
};

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}
PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  return c;
}
}  // namespace

// Flag off (default): observe() is never called → bit-identical path.
TEST(PmbmClutterFeed, FlagOffNeverCallsObserve) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker t(ekf, cfg());
  auto spy = std::make_shared<SpyDetectionModel>();
  t.setSensorDetectionModel(spy);
  for (int k = 0; k < 5; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(0.0, 0.0, k)});
  }
  EXPECT_EQ(spy->observe_calls, 0);
}

// Flag on: observe() called once per non-empty scan, with a claimed return
// carrying weight 1 − r (< 1) and clutter positions populated.
TEST(PmbmClutterFeed, FlagOnFeedsWeightedObservations) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.feed_clutter_map = true;
  PmbmTracker t(ekf, c);
  auto spy = std::make_shared<SpyDetectionModel>();
  t.setSensorDetectionModel(spy);
  // Feed a persistent target so a Bernoulli confirms and claims the return.
  for (int k = 0; k < 8; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(100.0, 0.0, k)});
  }
  EXPECT_GE(spy->observe_calls, 7);  // ~once per non-empty scan (8 scans)
  // In a late scan the target's Bernoulli should claim the return (weight < 1).
  bool saw_claimed = false;
  for (const auto& bundle : spy->bundles)
    for (const auto& obs : bundle)
      for (double w : obs.clutter_position_weights)
        if (w < 0.999) saw_claimed = true;
  EXPECT_TRUE(saw_claimed);
}

// R2 behavioral guard: with two persistent targets, BOTH real returns are
// credited to their own Bernoulli (weight 1 − r < 1) — neither is fed as
// full-weight (1.0) clutter. Under the true-assignment labeling each detected
// Bernoulli claims exactly its assigned measurement; the old NN reconstruction
// could collapse two close Bernoullis onto one return and leak the other as
// clutter. (Guard, not a RED discriminator: in this clean two-target geometry
// NN and true-assignment agree — the collapse case cannot be isolated in a unit
// test because an unclaimed return births-and-claims a new Bernoulli anyway.)
TEST(PmbmClutterFeed, TwoTargetsBothReturnsClaimed) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.feed_clutter_map = true;
  PmbmTracker t(ekf, c);
  auto spy = std::make_shared<SpyDetectionModel>();
  t.setSensorDetectionModel(spy);
  // Two targets 25 m apart (far enough not to merge), both persistent.
  for (int k = 0; k < 10; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(100.0, 0.0, k), posMeas(100.0, 25.0, k)});
  }
  ASSERT_FALSE(spy->bundles.empty());
  // Every return in this clean two-target scene is a detection or a birth, so
  // each is credited to its own Bernoulli — none is ever fed as full-weight
  // (1.0) clutter. (Once existence saturates to r≈1 the weight 1−r≈0 return is
  // dropped from the feed entirely, which is why we scan all bundles, not just
  // the last.)
  int total_weights = 0, full_weight = 0;
  for (const auto& bundle : spy->bundles)
    for (const auto& obs : bundle)
      for (double w : obs.clutter_position_weights) {
        ++total_weights;
        if (w >= 0.999) ++full_weight;
      }
  EXPECT_GT(total_weights, 0);  // the feed ran and fed weighted returns
  EXPECT_EQ(full_weight, 0);    // no real return leaked as full-weight clutter
}

// R2 finding #2: two returns close enough that their Bernoullis merge in the
// SAME scan (equal timestamps) must both stay credited. mergeBernoulliDuplicates
// used to carry the absorbed duplicate's claim only when it was strictly newer,
// so at equal timestamps the merged-away claim was dropped and its real return
// leaked to the clutter map as full-weight (1.0). The fix carries the claim of
// the higher-existence duplicate and never drops a real claim for none.
TEST(PmbmClutterFeed, MergedSameScanReturnsStayCredited) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.feed_clutter_map = true;
  PmbmTracker t(ekf, c);
  auto spy = std::make_shared<SpyDetectionModel>();
  t.setSensorDetectionModel(spy);
  // Two returns ~3 m apart every scan — close enough to fall inside the
  // Bhattacharyya merge gate once both Bernoullis converge, exercising the
  // same-scan merge path.
  for (int k = 0; k < 14; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(100.0, 0.0, k), posMeas(103.0, 0.0, k)});
  }
  ASSERT_FALSE(spy->bundles.empty());
  int total_weights = 0, full_weight = 0;
  for (const auto& bundle : spy->bundles)
    for (const auto& obs : bundle)
      for (double w : obs.clutter_position_weights) {
        ++total_weights;
        if (w >= 0.999) ++full_weight;
      }
  EXPECT_GT(total_weights, 0);
  EXPECT_EQ(full_weight, 0);  // no merged-away real return leaked as clutter
}

// Determinism: identical inputs → identical observe() call count and weights.
TEST(PmbmClutterFeed, DeterministicFeed) {
  auto run = []() {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator ekf{motion, 5.0};
    PmbmTracker::Config c = cfg();
    c.feed_clutter_map = true;
    PmbmTracker t(ekf, c);
    auto spy = std::make_shared<SpyDetectionModel>();
    t.setSensorDetectionModel(spy);
    for (int k = 0; k < 8; ++k) {
      t.predict(Timestamp::fromSeconds(k));
      t.processBatch({posMeas(100.0, 0.0, k), posMeas(-50.0, 20.0, k)});
    }
    std::vector<double> ws;
    for (const auto& bundle : spy->bundles)
      for (const auto& obs : bundle)
        for (double w : obs.clutter_position_weights) ws.push_back(w);
    return std::make_pair(spy->observe_calls, ws);
  };
  EXPECT_EQ(run(), run());
}
