#include <gtest/gtest.h>
#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::MhtTracker;
using navtracker::Timestamp;

namespace {

Measurement positionMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "t";
  return m;
}

}  // namespace

TEST(MhtTracker, SingleTargetCleanlyTracked) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  for (int i = 1; i <= 10; ++i) {
    mht.processBatch({positionMeas(static_cast<double>(i) * 5.0, 0.0,
                                   static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 1u);
  EXPECT_EQ(mht.tracks().size(), 1u);
}

TEST(MhtTracker, NewMeasurementsGetTheirOwnTrees) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  mht.processBatch({positionMeas(   0.0,   0.0, 1.0),
                    positionMeas(1000.0, 1000.0, 1.0)});
  EXPECT_EQ(mht.treeCount(), 2u);
}

TEST(MhtTracker, ScoreDeltaTightThresholdDoesNotExplodeTreesOnCleanRun) {
  // Smoke test for the Score-Δ K filter: on a single-target unambiguous
  // run, the K=2 alternatives all involve "miss" branches whose cost is
  // far worse than the K=1 best. With a tight delta the filter rejects
  // them — protection collapses to the K=1 chain, and the tree should
  // not grow without bound across many scans. Without the filter, every
  // miss alternative gets protected and pruneKLocal can't demote, so
  // tree size would grow ~k_max_leaves per scan instead of staying
  // bounded.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.score_delta_threshold = 0.5;  // tight: only near-tied alternatives
  MhtTracker mht(ekf, cfg);
  for (int i = 1; i <= 30; ++i) {
    mht.processBatch({positionMeas(static_cast<double>(i) * 5.0, 0.0,
                                   static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 1u);
  EXPECT_EQ(mht.tracks().size(), 1u);
}

TEST(MhtTracker, AdaptiveNScanExtensionDoesNotBreakStableTracking) {
  // Smoke test for adaptive N-scan: a large extension shouldn't
  // destabilize a clean single-target run. Trees with one dominant
  // leaf use the base n_scan; only trees with surviving alternatives
  // see the extended depth. Verifies the new branch in processBatch
  // doesn't accidentally apply the extension to all trees.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.n_scan_extension_when_protected = 5;
  MhtTracker mht(ekf, cfg);
  for (int i = 1; i <= 20; ++i) {
    mht.processBatch({positionMeas(static_cast<double>(i) * 5.0, 0.0,
                                   static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 1u);
  EXPECT_EQ(mht.tracks().size(), 1u);
}
