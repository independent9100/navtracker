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

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "t";
  return m;
}

}  // namespace

// Cross-tree duplicate merge — two trees latched onto ONE target.
// Reproduces the duplicate mechanism seen on AutoFerry: per-scan
// measurement exclusivity is satisfied because each scan carries two
// detections of the same target (two sensors), so tree A takes one hit
// and tree B the other — both stay confirmed forever, the metric
// assignment flips between their ids (~59 residual id_switches on
// scenario2), and OSPA carries a +1 cardinality error. The merge pass
// must retire the younger tree once the selected leaves have stayed
// within the Bhattacharyya bound for M consecutive scans, keeping the
// OLDER external id (ID-stability invariant).
TEST(MhtCrossMerge, DuplicateTreesOnOneTargetMergeKeepingOlderId) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.gate_threshold = 9.0;
  MhtTracker mht(ekf, cfg);

  // Scan 1: two near-coincident detections, no trees yet -> two trees
  // spawn on the same physical target.
  mht.processBatch({posMeas(0.0, 0.0, 1.0), posMeas(0.6, 0.0, 1.0)});
  ASSERT_EQ(mht.treeCount(), 2u);

  // Subsequent scans keep feeding two detections of the one target
  // (5 m/s east) so BOTH trees receive a hit every scan.
  for (int i = 2; i <= 12; ++i) {
    const double x = 5.0 * static_cast<double>(i - 1);
    mht.processBatch({posMeas(x, 0.0, static_cast<double>(i)),
                      posMeas(x + 0.6, 0.0, static_cast<double>(i))});
  }

  EXPECT_EQ(mht.treeCount(), 1u)
      << "duplicate trees on one target must merge";
  ASSERT_EQ(mht.tracks().size(), 1u);
  EXPECT_EQ(mht.tracks()[0].id.value, 1u)
      << "the OLDER external id must survive the merge";
}

// Two genuinely separate targets must never merge, no matter how long
// they run in parallel — the Bhattacharyya bound on converged
// covariances keeps well-separated tracks far apart.
TEST(MhtCrossMerge, SeparateTargetsDoNotMerge) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.gate_threshold = 9.0;
  MhtTracker mht(ekf, cfg);

  for (int i = 1; i <= 15; ++i) {
    const double x = 5.0 * static_cast<double>(i - 1);
    mht.processBatch({posMeas(x, 0.0, static_cast<double>(i)),
                      posMeas(x, 50.0, static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 2u)
      << "parallel targets 50 m apart must not merge";
}

// Threshold <= 0 disables the pass entirely (legacy behaviour).
TEST(MhtCrossMerge, DisabledThresholdKeepsDuplicates) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.gate_threshold = 9.0;
  cfg.duplicate_merge_bhattacharyya = 0.0;  // off
  MhtTracker mht(ekf, cfg);

  mht.processBatch({posMeas(0.0, 0.0, 1.0), posMeas(0.6, 0.0, 1.0)});
  for (int i = 2; i <= 12; ++i) {
    const double x = 5.0 * static_cast<double>(i - 1);
    mht.processBatch({posMeas(x, 0.0, static_cast<double>(i)),
                      posMeas(x + 0.6, 0.0, static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 2u);
}

// The duplicate streak is TIME-based, not scan-counted: on a fast
// multi-sensor stream (16 Hz on AutoFerry) a scan-counted streak of 3
// is ~0.19 s — two real vessels passing close would merge almost
// instantly (measured: scenario6 breaks 2.5 -> 11.5 with a 3-scan
// streak). Closeness must be sustained for duplicate_merge_seconds of
// stream time before the younger tree is retired.
TEST(MhtCrossMerge, FastScanRateDoesNotMergeBriefCloseness) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.gate_threshold = 9.0;
  cfg.duplicate_merge_seconds = 3.0;
  MhtTracker mht(ekf, cfg);

  // Two duplicate trees on one target, scans at 10 Hz: 1.0 s of
  // sustained closeness (11 scans) is well past any scan-counted
  // streak of 3 but below the 3.0 s time requirement -> NO merge.
  mht.processBatch({posMeas(0.0, 0.0, 1.0), posMeas(0.6, 0.0, 1.0)});
  ASSERT_EQ(mht.treeCount(), 2u);
  for (int i = 1; i <= 10; ++i) {
    const double t = 1.0 + 0.1 * static_cast<double>(i);
    const double x = 5.0 * (t - 1.0);
    mht.processBatch({posMeas(x, 0.0, t), posMeas(x + 0.6, 0.0, t)});
  }
  EXPECT_EQ(mht.treeCount(), 2u)
      << "1.0 s of closeness at 10 Hz must not merge with a 3.0 s "
         "requirement";

  // Keep going past 3.0 s of sustained closeness -> merge fires.
  for (int i = 11; i <= 45; ++i) {
    const double t = 1.0 + 0.1 * static_cast<double>(i);
    const double x = 5.0 * (t - 1.0);
    mht.processBatch({posMeas(x, 0.0, t), posMeas(x + 0.6, 0.0, t)});
  }
  EXPECT_EQ(mht.treeCount(), 1u);
  ASSERT_EQ(mht.tracks().size(), 1u);
  EXPECT_EQ(mht.tracks()[0].id.value, 1u);
}

// Separation resets the clock: closeness must be *sustained*.
TEST(MhtCrossMerge, SeparationResetsTheMergeClock) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.gate_threshold = 20.0;
  cfg.duplicate_merge_seconds = 3.0;
  MhtTracker mht(ekf, cfg);

  // Two targets 60 m apart (own trees), 1 Hz.
  auto feed_apart = [&](double t) {
    mht.processBatch({posMeas(0.0, 0.0, t), posMeas(0.0, 60.0, t)});
  };
  // Brief 2 s closeness bursts separated by an apart scan: the clock
  // resets each time, so the pair never merges.
  auto feed_close = [&](double t) {
    mht.processBatch({posMeas(0.0, 0.0, t), posMeas(0.6, 0.0, t)});
  };
  double t = 1.0;
  feed_apart(t); t += 1.0;
  ASSERT_EQ(mht.treeCount(), 2u);
  for (int cycle = 0; cycle < 4; ++cycle) {
    feed_close(t); t += 1.0;
    feed_close(t); t += 1.0;
    feed_apart(t); t += 1.0;  // resets the streak clock
  }
  EXPECT_EQ(mht.treeCount(), 2u)
      << "interrupted closeness must never accumulate to a merge";
}
