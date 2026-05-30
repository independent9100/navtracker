#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::Tracker;
using navtracker::TrackManager;
using navtracker::TrackStatus;

namespace {
Measurement positionAt(double t, double x, double y, const std::string& src) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity();
  return z;
}
}  // namespace

TEST(Tracker, InitiatesAndUpdatesSingleTarget) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(2, 3);
  Tracker tracker(estimator, associator, manager, 10.0);

  tracker.process(positionAt(0.0, 10.0, 0.0, "ais"));
  tracker.process(positionAt(1.0, 12.0, 0.0, "ais"));
  tracker.process(positionAt(2.0, 14.0, 0.0, "ais"));

  ASSERT_EQ(manager.size(), 1u);
  EXPECT_EQ(manager.tracks()[0].status, TrackStatus::Confirmed);
  EXPECT_NEAR(manager.tracks()[0].state(0), 14.0, 1.0);
  EXPECT_NEAR(manager.tracks()[0].state(1), 0.0, 1.0);
}

TEST(Tracker, StaleTrackTimesOutAndIsDeleted) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(1, 2);
  Tracker tracker(estimator, associator, manager, 0.5);

  tracker.process(positionAt(0.0, 0.0, 0.0, "s"));
  ASSERT_EQ(manager.size(), 1u);
  tracker.process(positionAt(1.0, 1000.0, 0.0, "s"));
  ASSERT_EQ(manager.size(), 2u);
  tracker.process(positionAt(2.0, 1000.5, 0.0, "s"));
  EXPECT_EQ(manager.size(), 1u);
}

TEST(Tracker, ReplayIsDeterministic) {
  const std::vector<Measurement> stream{
      positionAt(0.0, 0.0, 0.0, "a"),
      positionAt(1.0, 1.0, 0.0, "a"),
      positionAt(2.0, 2.0, 0.0, "a"),
      positionAt(3.0, 3.0, 0.0, "a"),
  };

  auto run = [&stream]() {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator estimator(motion, 10.0);
    GnnAssociator associator(20.0);
    TrackManager manager(2, 3);
    Tracker tracker(estimator, associator, manager, 10.0);
    for (const auto& z : stream) tracker.process(z);
    return manager.tracks();
  };

  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].id.value, b[i].id.value);
    ASSERT_EQ(a[i].state.size(), b[i].state.size());
    for (int k = 0; k < a[i].state.size(); ++k) {
      EXPECT_DOUBLE_EQ(a[i].state(k), b[i].state(k));
    }
  }
}
