#include <gtest/gtest.h>

#include <memory>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::PmbmTracker;

namespace {

Measurement pos2d(double t, double x, double y, double sigma = 1.0) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = SensorKind::Lidar;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
  return z;
}

struct Fixture {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
};

}  // namespace

// #28 (backlog #1's PMBM face): processBatch sorts WITHIN a batch but has no
// cross-batch reject-stale / high-water guard. An out-of-order batch reaches
// predict(), whose dt<=0 branch REWINDS current_time_ and returns without
// propagating; the update then runs against newer states. Tracker/MhtTracker
// both drop stale batches — PMBM silently processed them. Mirror the
// MhtTracker guard exactly (default on).
TEST(PmbmStaleGuard, RejectsStaleBatchAndDoesNotRewindClock) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});  // reject_stale_measurements defaults true

  tracker.processBatch({pos2d(10.0, 0.0, 0.0, 0.5)});
  ASSERT_TRUE(tracker.hasCurrentTime());
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 10.0);
  EXPECT_EQ(tracker.staleDropped(), 0u);
  const std::size_t n_tracks = tracker.tracks().size();

  // A batch whose front is BEFORE the high-water mark must be dropped whole:
  // the filter clock must NOT rewind and no track state may change.
  tracker.processBatch({pos2d(5.0, 100.0, 100.0, 0.5)});
  EXPECT_EQ(tracker.staleDropped(), 1u);
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 10.0);  // not rewound to 5
  EXPECT_EQ(tracker.tracks().size(), n_tracks);
}

TEST(PmbmStaleGuard, InOrderBatchesAreNeverDropped) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(2.0, 1.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(3.0, 2.0, 0.0, 0.5)});
  EXPECT_EQ(tracker.staleDropped(), 0u);
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 3.0);
}

TEST(PmbmStaleGuard, GuardCanBeDisabledForOrderRobustCallers) {
  // The escape hatch mirrors MhtTracker: a caller that genuinely wants PMBM's
  // (order-robust) set-wise processing of a late batch can opt out. With the
  // guard off, the stale batch is NOT counted as dropped.
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.reject_stale_measurements = false;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.processBatch({pos2d(10.0, 0.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(5.0, 1.0, 0.0, 0.5)});
  EXPECT_EQ(tracker.staleDropped(), 0u);
}
