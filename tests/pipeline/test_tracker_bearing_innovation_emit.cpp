#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IBearingInnovationSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public IBearingInnovationSink {
 public:
  std::vector<BearingInnovation> events;
  void onBearingInnovation(const BearingInnovation& obs) override {
    events.push_back(obs);
  }
};

struct Bench {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est{motion, 5.0};
  GnnAssociator assoc{600.0};
  TrackManager mgr{1, 4};
  Tracker tracker{est, assoc, mgr, 60.0};
  RecordingSink sink;
  Bench() { tracker.setBearingInnovationSink(&sink); }
};

Measurement seedPosition(double x, double y, double t_s) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "seed";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 100.0;
  return m;
}

Measurement bearing(double beta_rad, double t_s, double sigma_b_rad,
                    const Eigen::Vector2d& sensor_enu = Eigen::Vector2d::Zero()) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "eoir";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = beta_rad;
  m.value = v;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma_b_rad * sigma_b_rad;
  m.covariance = R;
  m.sensor_position_enu = sensor_enu;
  return m;
}

}  // namespace

TEST(TrackerBearingInnovationEmit, Position2DDoesNotEmit) {
  Bench b;
  b.tracker.process(seedPosition(100.0, 0.0, 1.0));
  b.tracker.process(seedPosition(101.0, 0.0, 2.0));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, NullSinkIsSafe) {
  Bench b;
  b.tracker.setBearingInnovationSink(nullptr);
  b.tracker.process(seedPosition(100.0, 0.0, 1.0));
  b.tracker.process(bearing(0.0, 2.0, 0.01));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, InitiationDoesNotEmit) {
  Bench b;
  b.tracker.process(bearing(0.0, 1.0, 0.01));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, Bearing2DEmitsCorrectFields) {
  Bench b;
  b.tracker.process(seedPosition(500.0, 0.0, 1.0));
  b.tracker.process(bearing(0.01, 2.0, 0.005));
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_NEAR(e.innovation_rad, 0.01, 5e-3);
  EXPECT_GT(e.variance_rad2, 0.0);
  EXPECT_GE(e.predicted_state_var_rad2, 0.0);
  EXPECT_LE(e.predicted_state_var_rad2, e.variance_rad2);
  EXPECT_NEAR(e.range_m, 500.0, 50.0);
}

TEST(TrackerBearingInnovationEmit, JpdaSoftBranchEmitsWeightedInnovation) {
  // Soft branch: use JpdaAssociator over a bearing-only measurement scan.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  // High P_D, low clutter density → β_target ≈ 1.0; β_clutter ≈ 0.
  navtracker::JpdaAssociator jpda(/*gate=*/16.0, /*p_d=*/0.99, /*lambda=*/1e-6);
  TrackManager mgr(1, 4);
  Tracker tracker(est, jpda, mgr, 60.0);
  RecordingSink sink;
  tracker.setBearingInnovationSink(&sink);

  // Seed a track far enough away that bearing variance is reasonable.
  tracker.process(seedPosition(500.0, 0.0, 1.0));
  // Soft scan of one bearing measurement at ~predicted bearing + 0.01.
  std::vector<Measurement> scan{bearing(0.01, 2.0, 0.005)};
  tracker.processBatch(scan);
  ASSERT_GE(sink.events.size(), 1u);
  const auto& e = sink.events.front();
  EXPECT_NEAR(e.innovation_rad, 0.01, 5e-3);
  // Variance inflated by 1/β (β ≤ 1) → variance_rad2 ≥ state_var + R.
  EXPECT_GT(e.variance_rad2, 0.0);
}

TEST(TrackerBearingInnovationEmit, RangeBearing2DEmitsBearingComponent) {
  Bench b;
  b.tracker.process(seedPosition(500.0, 0.0, 1.0));
  Measurement m;
  m.time = Timestamp::fromSeconds(2.0);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = "rb";
  m.model = MeasurementModel::RangeBearing2D;
  m.value = Eigen::Vector2d(500.0, 0.01);
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = 25.0;
  R(1, 1) = 1e-4;
  m.covariance = R;
  b.tracker.process(m);
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_NEAR(e.innovation_rad, 0.01, 5e-3);
  EXPECT_NEAR(e.range_m, 500.0, 50.0);
}
