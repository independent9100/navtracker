#include <gtest/gtest.h>
#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/tracking/SensorDetectionModels.hpp"

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

// --- Stale-input guard (default ON) -----------------------------------

TEST(MhtTracker, DropsStaleScanByDefaultAndCounts) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;

  // Twin A: clean in-order feed.
  MhtTracker clean(ekf, cfg);
  clean.processBatch({positionMeas(10.0, 0.0, 123.0)});
  clean.processBatch({positionMeas(16.0, 0.0, 126.0)});

  // Twin B: same feed with a stale scan injected.
  MhtTracker mht(ekf, cfg);
  mht.processBatch({positionMeas(10.0, 0.0, 123.0)});
  mht.processBatch({positionMeas(9.0, 0.0, 115.0)});  // stale scan
  mht.processBatch({positionMeas(16.0, 0.0, 126.0)});

  EXPECT_EQ(mht.staleDropped(), 1u);
  EXPECT_EQ(clean.staleDropped(), 0u);
  ASSERT_EQ(mht.tracks().size(), 1u);
  ASSERT_EQ(clean.tracks().size(), 1u);
  EXPECT_TRUE(mht.tracks()[0].state == clean.tracks()[0].state);
  EXPECT_EQ(mht.tracks()[0].last_update.nanos(),
            clean.tracks()[0].last_update.nanos());
}

TEST(MhtTracker, EqualTimestampScanIsNotStale) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  mht.processBatch({positionMeas(10.0, 0.0, 123.0)});
  mht.processBatch({positionMeas(10.5, 0.0, 123.0)});  // same instant: OK
  EXPECT_EQ(mht.staleDropped(), 0u);
}

TEST(MhtTracker, StaleGuardCanBeOptedOut) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.reject_stale_measurements = false;
  MhtTracker mht(ekf, cfg);

  mht.processBatch({positionMeas(10.0, 0.0, 123.0)});
  mht.processBatch({positionMeas(1000.0, 0.0, 115.0)});  // legacy: spawns
  EXPECT_EQ(mht.staleDropped(), 0u);
  EXPECT_EQ(mht.treeCount(), 2u);
}

// --- Default-detection-model footgun diagnostic ------------------------
// Running ≥2 distinct (SensorKind, MeasurementModel) keys through the
// auto-installed single-default detection model is the misconfiguration
// that produced the pre-fix AutoFerry collapse (one λ_C across units).
// The tracker flags it; the composition root decides what to do.

TEST(MhtTracker, FlagsDefaultDetectionModelOnSecondSensorKind) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);  // no detection model injected

  Measurement radar = positionMeas(10.0, 0.0, 1.0);
  radar.sensor = navtracker::SensorKind::ArpaTtm;
  mht.processBatch({radar});
  EXPECT_FALSE(mht.defaultDetectionModelWarning());

  Measurement lidar = positionMeas(11.0, 0.0, 2.0);
  lidar.sensor = navtracker::SensorKind::Lidar;
  mht.processBatch({lidar});  // second distinct key, later scan
  EXPECT_TRUE(mht.defaultDetectionModelWarning());
}

TEST(MhtTracker, NoWarningWithInjectedDetectionModel) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  auto det = std::make_shared<navtracker::FixedSensorDetectionModel>(
      navtracker::DetectionParams{0.9, 1e-4});
  det->set(navtracker::SensorKind::ArpaTtm,
           MeasurementModel::Position2D,
           navtracker::DetectionParams{0.8, 1e-5});
  MhtTracker mht(ekf, cfg, det);

  Measurement radar = positionMeas(10.0, 0.0, 1.0);
  radar.sensor = navtracker::SensorKind::ArpaTtm;
  Measurement lidar = positionMeas(11.0, 0.0, 2.0);
  lidar.sensor = navtracker::SensorKind::Lidar;
  mht.processBatch({radar});
  mht.processBatch({lidar});
  EXPECT_FALSE(mht.defaultDetectionModelWarning());
}

TEST(MhtTracker, NoWarningWithSingleSensorKindOnDefaultModel) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  for (int i = 1; i <= 5; ++i) {
    mht.processBatch({positionMeas(static_cast<double>(i) * 5.0, 0.0,
                                   static_cast<double>(i))});
  }
  EXPECT_FALSE(mht.defaultDetectionModelWarning());
}

// --- Scan-observation enrichment (backlog item 5) ----------------------

namespace {

// Recording model: captures every observe() bundle so tests can assert
// what evidence the tracker feeds spatial clutter estimators.
struct RecordingDetectionModel : navtracker::ISensorDetectionModel {
  using navtracker::ISensorDetectionModel::paramsFor;
  navtracker::DetectionParams paramsFor(
      navtracker::SensorKind, navtracker::MeasurementModel) const override {
    return navtracker::DetectionParams{0.9, 1e-4};
  }
  void observe(const std::vector<ScanObservation>& bundle) override {
    bundles.push_back(bundle);
  }
  std::vector<std::vector<ScanObservation>> bundles;
};

}  // namespace

// Clutter evidence is labeled from the GLOBAL HYPOTHESIS, not the
// birth gate (backlog item 5 residue): each return's clutter weight is
// 1 − r of the tree that claims it in the chosen assignment (or that it
// birthed this scan), 1.0 when nothing claims it. A brand-new target's
// first return therefore contributes 1 − r₀ = 0.5 instead of a full
// count (halving the birth self-poisoning), and returns claimed by
// high-existence tracks contribute ≈ 0.
TEST(MhtTracker, ObserveBundleWeightsBirthsAtInitExistence) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  auto rec = std::make_shared<RecordingDetectionModel>();
  MhtTracker mht(ekf, cfg, rec);

  // First scan, no prior trees: both position returns birth trees
  // (claimed at r₀ = 0.5); the camera bearing can't initiate and is
  // claimed by nothing → full clutter weight.
  Measurement p1 = positionMeas(0.0, 0.0, 1.0);
  Measurement p2 = positionMeas(1000.0, 1000.0, 1.0);
  Measurement b;
  b.time = Timestamp::fromSeconds(1.0);
  b.sensor = navtracker::SensorKind::EoIr;
  b.model = MeasurementModel::Bearing2D;
  b.value = Eigen::VectorXd::Constant(1, 0.25);
  b.covariance = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
  mht.processBatch({p1, p2, b});

  ASSERT_EQ(rec->bundles.size(), 1u);
  const auto& bundle = rec->bundles.front();
  ASSERT_EQ(bundle.size(), 2u);  // (Unknown, Position2D) + (EoIr, Bearing2D)

  const navtracker::ISensorDetectionModel::ScanObservation* pos = nullptr;
  const navtracker::ISensorDetectionModel::ScanObservation* brg = nullptr;
  for (const auto& o : bundle) {
    if (o.model == MeasurementModel::Position2D) pos = &o;
    if (o.model == MeasurementModel::Bearing2D) brg = &o;
  }
  ASSERT_NE(pos, nullptr);
  ASSERT_NE(brg, nullptr);

  EXPECT_DOUBLE_EQ(pos->time.seconds(), 1.0);
  ASSERT_EQ(pos->positions.size(), 2u);
  // Both returns birthed trees → clutter weight 1 − r₀ = 0.5 each, and
  // neither counts as fully unclaimed.
  ASSERT_EQ(pos->clutter_positions.size(), 2u);
  ASSERT_EQ(pos->clutter_position_weights.size(), 2u);
  EXPECT_DOUBLE_EQ(pos->clutter_position_weights[0], 0.5);
  EXPECT_DOUBLE_EQ(pos->clutter_position_weights[1], 0.5);
  EXPECT_EQ(pos->num_unassociated, 0);

  EXPECT_DOUBLE_EQ(brg->time.seconds(), 1.0);
  ASSERT_EQ(brg->bearings.size(), 1u);
  EXPECT_DOUBLE_EQ(brg->bearings.front(), 0.25);
  ASSERT_EQ(brg->clutter_bearings.size(), 1u);
  ASSERT_EQ(brg->clutter_bearing_weights.size(), 1u);
  EXPECT_DOUBLE_EQ(brg->clutter_bearings.front(), 0.25);
  EXPECT_DOUBLE_EQ(brg->clutter_bearing_weights.front(), 1.0);
  EXPECT_EQ(brg->num_unassociated, 1);
  EXPECT_TRUE(brg->positions.empty());
}

TEST(MhtTracker, ObserveBundleWeightsClaimedReturnsByExistence) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  auto rec = std::make_shared<RecordingDetectionModel>();
  MhtTracker mht(ekf, cfg, rec);

  // Establish a track at the origin, then offer one gating return and
  // one far return (births a fresh tree) in the same scan.
  mht.processBatch({positionMeas(0.0, 0.0, 1.0)});
  mht.processBatch({positionMeas(1.0, 0.0, 2.0),
                    positionMeas(5000.0, 5000.0, 2.0)});

  ASSERT_EQ(rec->bundles.size(), 2u);
  const auto& second = rec->bundles.back();
  ASSERT_EQ(second.size(), 1u);
  const auto& o = second.front();
  ASSERT_EQ(o.positions.size(), 2u);
  ASSERT_EQ(o.clutter_positions.size(), 2u);
  ASSERT_EQ(o.clutter_position_weights.size(), 2u);
  // Scan order is preserved: [0] = the claimed return, [1] = the birth.
  EXPECT_DOUBLE_EQ(o.clutter_positions[0].x(), 1.0);
  EXPECT_DOUBLE_EQ(o.clutter_positions[1].x(), 5000.0);
  // The gating return is claimed by the existing tree's chosen hit
  // leaf: its IPDA existence after the update is well above r₀, so the
  // clutter weight must sit clearly below the birth's 0.5 — but above
  // zero (the hypothesis is not yet certain).
  EXPECT_LT(o.clutter_position_weights[0], 0.3);
  EXPECT_GT(o.clutter_position_weights[0], 0.0);
  EXPECT_DOUBLE_EQ(o.clutter_position_weights[1], 0.5);
  // Neither return is fully unclaimed.
  EXPECT_EQ(o.num_unassociated, 0);
}

TEST(MhtTracker, ObserveBundleFullWeightWhenNoTreeClaims) {
  // With the IPDA lifecycle off, existence is the 1.0 sentinel: claimed
  // and birthed returns carry weight 0 (and are omitted), so only the
  // genuinely unexplained bearing contributes clutter evidence.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.use_ipda_lifecycle = false;
  cfg.use_visibility = false;
  auto rec = std::make_shared<RecordingDetectionModel>();
  MhtTracker mht(ekf, cfg, rec);

  Measurement p1 = positionMeas(0.0, 0.0, 1.0);
  Measurement b;
  b.time = Timestamp::fromSeconds(1.0);
  b.sensor = navtracker::SensorKind::EoIr;
  b.model = MeasurementModel::Bearing2D;
  b.value = Eigen::VectorXd::Constant(1, 0.25);
  b.covariance = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
  mht.processBatch({p1, b});

  ASSERT_EQ(rec->bundles.size(), 1u);
  const navtracker::ISensorDetectionModel::ScanObservation* pos = nullptr;
  const navtracker::ISensorDetectionModel::ScanObservation* brg = nullptr;
  for (const auto& o : rec->bundles.front()) {
    if (o.model == MeasurementModel::Position2D) pos = &o;
    if (o.model == MeasurementModel::Bearing2D) brg = &o;
  }
  ASSERT_NE(pos, nullptr);
  ASSERT_NE(brg, nullptr);
  EXPECT_TRUE(pos->clutter_positions.empty());
  EXPECT_EQ(pos->num_unassociated, 0);
  ASSERT_EQ(brg->clutter_bearing_weights.size(), 1u);
  EXPECT_DOUBLE_EQ(brg->clutter_bearing_weights.front(), 1.0);
  EXPECT_EQ(brg->num_unassociated, 1);
}
