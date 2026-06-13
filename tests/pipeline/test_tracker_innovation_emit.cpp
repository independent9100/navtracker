#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IInnovationSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public IInnovationSink {
 public:
  std::vector<InnovationEvent> events;
  void onInnovation(const InnovationEvent& e) override { events.push_back(e); }
};

struct Bench {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est{motion, 5.0};
  GnnAssociator assoc{600.0};
  TrackManager mgr{1, 4};
  Tracker tracker{est, assoc, mgr, 60.0};
  RecordingSink sink;
  Bench() { tracker.setInnovationSink(&sink); }
};

Measurement positionMeas(double x, double y, double t_s,
                         const std::string& src = "pos") {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

Measurement bearingMeas(double beta_rad, double t_s, double sigma_b,
                        const Eigen::Vector2d& sensor_enu =
                            Eigen::Vector2d::Zero()) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "cam0";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = beta_rad;
  m.value = v;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma_b * sigma_b;
  m.covariance = R;
  m.sensor_position_enu = sensor_enu;
  return m;
}

Measurement rangeBearingMeas(double r, double beta_rad, double t_s,
                             const std::string& src = "rb0") {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = src;
  m.model = MeasurementModel::RangeBearing2D;
  m.value = Eigen::Vector2d(r, beta_rad);
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = 25.0;
  R(1, 1) = 1e-4;
  m.covariance = R;
  return m;
}

}  // namespace

// Initiation is the first detection — no prior state, no innovation.
TEST(TrackerInnovationEmit, InitiationDoesNotEmit) {
  Bench b;
  b.tracker.process(positionMeas(100.0, 0.0, 1.0));
  EXPECT_TRUE(b.sink.events.empty());
}

// Null sink → zero overhead, no events. Existing behaviour bit-identical.
TEST(TrackerInnovationEmit, NullSinkIsSafe) {
  Bench b;
  b.tracker.setInnovationSink(nullptr);
  b.tracker.process(positionMeas(100.0, 0.0, 1.0));
  b.tracker.process(positionMeas(101.0, 0.0, 2.0));
  EXPECT_TRUE(b.sink.events.empty());
}

// Hard-match Position2D update emits one event with ν = z − ẑ⁻ and
// S = HPHᵀ + R recoverable to within numerical noise.
TEST(TrackerInnovationEmit, Position2DEmitsCorrectShape) {
  Bench b;
  b.tracker.process(positionMeas(100.0, 0.0, 1.0));
  b.tracker.process(positionMeas(102.0, 0.0, 2.0));
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_EQ(e.sensor, SensorKind::Ais);
  EXPECT_EQ(e.source_id, "pos");
  EXPECT_EQ(e.model, MeasurementModel::Position2D);
  EXPECT_EQ(e.dim, 2u);
  EXPECT_EQ(e.residual.size(), 2);
  EXPECT_EQ(e.S.rows(), 2);
  EXPECT_EQ(e.S.cols(), 2);
  EXPECT_EQ(e.R.rows(), 2);
  EXPECT_TRUE(e.S.isApprox(e.S.transpose(), 1e-9));
  EXPECT_GT((e.S - e.R).norm(), 0.0);  // S = HPHᵀ + R, HPHᵀ ≠ 0
}

// Bearing-only emission carries the wrapped scalar residual.
TEST(TrackerInnovationEmit, Bearing2DEmits1d) {
  Bench b;
  b.tracker.process(positionMeas(500.0, 0.0, 1.0));
  b.tracker.process(bearingMeas(0.01, 2.0, 0.005));
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_EQ(e.model, MeasurementModel::Bearing2D);
  EXPECT_EQ(e.dim, 1u);
  EXPECT_NEAR(e.residual(0), 0.01, 5e-3);
}

// Mixed sensor flow — every successful hard-match update fires.
TEST(TrackerInnovationEmit, RangeBearing2DEmits2d) {
  Bench b;
  b.tracker.process(positionMeas(500.0, 0.0, 1.0));
  b.tracker.process(rangeBearingMeas(500.0, 0.01, 2.0));
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_EQ(e.model, MeasurementModel::RangeBearing2D);
  EXPECT_EQ(e.dim, 2u);
  EXPECT_NEAR(e.residual(1), 0.01, 5e-3);
}

// Stale measurement guard drops the input before estimator.update → no
// emission. Guard is ON by default.
TEST(TrackerInnovationEmit, StaleMeasurementSilent) {
  Bench b;
  b.tracker.process(positionMeas(100.0, 0.0, 5.0));
  b.tracker.process(positionMeas(101.0, 0.0, 6.0));
  const std::size_t before = b.sink.events.size();
  // A stale measurement timestamped earlier than the high-water mark.
  b.tracker.process(positionMeas(99.0, 0.0, 2.0));
  EXPECT_EQ(b.sink.events.size(), before);
  EXPECT_GT(b.tracker.staleDropped(), 0u);
}
