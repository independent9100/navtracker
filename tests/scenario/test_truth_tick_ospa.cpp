#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

// Build a scenario with truth samples at 1 Hz (t = 0, 1, 2) and noiseless
// Position2D measurements at 10 Hz (t = 0.0, 0.1, ..., 2.0). One stationary
// target at the origin.
Scenario buildSparseTruthScenario() {
  Scenario s;
  for (int k = 0; k <= 2; ++k) {
    TruthSample ts;
    ts.time = Timestamp::fromSeconds(static_cast<double>(k));
    ts.truth_id = 1;
    ts.position = Eigen::Vector2d::Zero();
    ts.velocity = Eigen::Vector2d::Zero();
    s.truth.push_back(ts);
  }
  for (int k = 0; k <= 20; ++k) {
    Measurement m;
    m.time = Timestamp::fromSeconds(0.1 * static_cast<double>(k));
    m.sensor = SensorKind::Ais;
    m.source_id = "sim";
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d::Zero();
    m.covariance = Eigen::Matrix2d::Identity();
    s.measurements.push_back(m);
  }
  return s;
}

}  // namespace

TEST(TruthTickOspa, RunScenarioEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 5);
  Tracker tracker(ekf, gnn, mgr, 10.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}

TEST(TruthTickOspa, RunScenarioBatchedEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  JpdaAssociator jpda(20.0, 0.9, 1e-4);
  TrackManager mgr(1, 5);
  Tracker tracker(ekf, jpda, mgr, 10.0);

  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}

TEST(TruthTickOspa, RunScenarioBatchedMhtEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker tracker(ekf, cfg);

  const ScenarioResult r = runScenarioBatchedMht(s, tracker, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}
