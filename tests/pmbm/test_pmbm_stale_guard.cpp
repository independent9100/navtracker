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

// #28 (adversarial-review gap): the guard must key on the batch's LATEST
// instant (t_max — what predict() advances to), not its front. predict(t_max)
// rewinds current_time_ whenever t_max < current_time_, regardless of the
// front. A front-keyed guard accepted an overlapping batch whose t_max is stale
// and then rewound the clock and updated against newer states.
TEST(PmbmStaleGuard, RejectsOverlappingBatchWhoseLatestPrecedesFilterTime) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  // Batch A spans [20, 40] -> current_time_ = 40.
  tracker.processBatch({pos2d(20.0, 0.0, 0.0, 0.5), pos2d(40.0, 1.0, 0.0, 0.5)});
  ASSERT_DOUBLE_EQ(tracker.currentTime().seconds(), 40.0);
  // Batch B = {25, 30}: its front (25) is AFTER A's front (20), so a front-keyed
  // guard would accept it — but its t_max (30) is before current_time_ (40), so
  // predict(30) would rewind the clock. It must be dropped whole.
  tracker.processBatch({pos2d(25.0, 5.0, 5.0, 0.5), pos2d(30.0, 6.0, 6.0, 0.5)});
  EXPECT_EQ(tracker.staleDropped(), 2u);
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 40.0);  // NOT rewound to 30
}

// A same-instant batch (t_max == current_time_) is NOT stale: predict() sees
// dt == 0 (no propagation, no rewind) and the update applies at the current
// instant. It must be processed, not dropped.
TEST(PmbmStaleGuard, SameInstantBatchIsNotDropped) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.processBatch({pos2d(10.0, 0.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(10.0, 1.0, 0.0, 0.5)});  // same t_max
  EXPECT_EQ(tracker.staleDropped(), 0u);
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 10.0);
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
