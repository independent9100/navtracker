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
