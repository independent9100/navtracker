#include <algorithm>
#include <gtest/gtest.h>
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/tracking/TrackTree.hpp"

using navtracker::TrackId;
using navtracker::TrackTree;
using navtracker::TrackTreeNode;

namespace {

TrackTreeNode rootNode(double x, double y) {
  TrackTreeNode n;
  n.parent = TrackTreeNode::kNoParent;
  n.scan_idx = 0;
  n.state = Eigen::Vector4d(x, y, 0.0, 0.0);
  n.covariance = Eigen::Matrix4d::Identity();
  n.time = navtracker::Timestamp::fromSeconds(0.0);
  n.score = 0.0;
  n.is_leaf = true;
  return n;
}

}  // namespace

TEST(TrackTree, ConstructorMakesSingleLeafRoot) {
  TrackTree tt(TrackId{1}, rootNode(0.0, 0.0));
  ASSERT_EQ(tt.nodes().size(), 1u);
  EXPECT_EQ(tt.nodes()[0].parent, TrackTreeNode::kNoParent);
  EXPECT_TRUE(tt.nodes()[0].is_leaf);
  EXPECT_EQ(tt.externalId().value, 1u);
}

TEST(TrackTree, LeafIndicesReturnsLeavesOnly) {
  TrackTree tt(TrackId{2}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  TrackTreeNode c = rootNode(1.0, 0.0);
  c.parent = 0;
  c.scan_idx = 1;
  tt.mutableNodes().push_back(c);
  const auto leaves = tt.leafIndices();
  ASSERT_EQ(leaves.size(), 1u);
  EXPECT_EQ(leaves[0], 1u);
}

TEST(TrackTree, BestLeafIndexPicksHighestScore) {
  TrackTree tt(TrackId{3}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  TrackTreeNode a = rootNode(1.0, 0.0);
  a.parent = 0; a.scan_idx = 1; a.score = 1.0;
  TrackTreeNode b = rootNode(2.0, 0.0);
  b.parent = 0; b.scan_idx = 1; b.score = 3.5;
  tt.mutableNodes().push_back(a);
  tt.mutableNodes().push_back(b);
  EXPECT_EQ(tt.bestLeafIndex(), 2u);
}

#include <memory>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"

TEST(TrackTree, BranchProducesMissAndPerMeasurementChildren) {
  TrackTree tt(TrackId{4}, rootNode(0.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(0.5, 0.0);
  z.covariance = Eigen::Matrix2d::Identity() * 1.0;
  z.source_id = "t";

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {z}, navtracker::Timestamp::fromSeconds(1.0), p);

  EXPECT_FALSE(tt.nodes()[0].is_leaf);
  EXPECT_EQ(tt.leafIndices().size(), 2u);
  const std::size_t best = tt.bestLeafIndex();
  EXPECT_GT(tt.nodes()[best].score, 0.0);
}

TEST(TrackTree, BranchSkipsUngatedMeasurements) {
  TrackTree tt(TrackId{5}, rootNode(0.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(1000.0, 1000.0);
  z.covariance = Eigen::Matrix2d::Identity() * 1.0;
  z.source_id = "t";

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {z}, navtracker::Timestamp::fromSeconds(1.0), p);

  EXPECT_EQ(tt.leafIndices().size(), 1u);
}

TEST(TrackTree, PruneNScanKeepsBestTrunk) {
  TrackTree tt(TrackId{6}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_node = [&](std::size_t parent, int sc, double score) -> std::size_t {
    TrackTreeNode n;
    n.parent = parent; n.scan_idx = sc; n.score = score;
    n.state = Eigen::Vector4d::Zero(); n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(static_cast<double>(sc));
    n.is_leaf = false;
    tt.mutableNodes().push_back(n);
    return tt.mutableNodes().size() - 1;
  };
  const std::size_t a  = add_node(0, 1, 1.0);
  const std::size_t b  = add_node(0, 1, 0.5);
  const std::size_t a1 = add_node(a, 2, 5.0);
  const std::size_t b1 = add_node(b, 2, 2.0);
  tt.mutableNodes()[a1].is_leaf = true;
  tt.mutableNodes()[b1].is_leaf = true;

  // N=1: leaves at scan 2 look back 1 step. Ancestors at depth 1: a (score 5.0
  // via a1), b (score 2.0 via b1). Keep a's subtree.
  const std::size_t removed = tt.pruneNScan(1);
  EXPECT_GT(removed, 0u);
  const auto leaves = tt.leafIndices();
  ASSERT_EQ(leaves.size(), 1u);
  // After pruning + compaction, indices have changed. Verify the remaining
  // leaf has the highest score that previously belonged to a1.
  EXPECT_DOUBLE_EQ(tt.nodes()[leaves[0]].score, 5.0);
}

TEST(TrackTree, PruneKLocalDropsLowestScoringLeaves) {
  TrackTree tt(TrackId{7}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_leaf = [&](double score) {
    TrackTreeNode n;
    n.parent = 0; n.scan_idx = 1; n.score = score;
    n.state = Eigen::Vector4d::Zero(); n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(1.0);
    n.is_leaf = true;
    tt.mutableNodes().push_back(n);
  };
  add_leaf(5.0);
  add_leaf(3.0);
  add_leaf(7.0);
  add_leaf(1.0);
  const std::size_t dropped = tt.pruneKLocal(2);
  EXPECT_EQ(dropped, 2u);
  const auto leaves = tt.leafIndices();
  ASSERT_EQ(leaves.size(), 2u);
  std::vector<double> scores;
  for (std::size_t li : leaves) scores.push_back(tt.nodes()[li].score);
  std::sort(scores.begin(), scores.end());
  EXPECT_DOUBLE_EQ(scores[0], 5.0);
  EXPECT_DOUBLE_EQ(scores[1], 7.0);
}

TEST(TrackTree, CountHitsInWindowCountsRecentHits) {
  TrackTree tt(TrackId{8}, rootNode(0.0, 0.0));
  // Chain: root(hit) → miss → hit → miss → hit
  tt.mutableNodes()[0].is_leaf = false;
  auto add_child = [&](std::size_t parent, int scan_idx, bool is_hit) {
    TrackTreeNode n;
    n.parent = parent;
    n.scan_idx = scan_idx;
    n.score = 0.0;
    n.state = Eigen::Vector4d::Zero();
    n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(scan_idx);
    n.is_leaf = false;
    n.is_hit = is_hit;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  const std::size_t a = add_child(0, 1, false);  // miss
  const std::size_t b = add_child(a, 2, true);   // hit
  const std::size_t c = add_child(b, 3, false);  // miss
  const std::size_t d = add_child(c, 4, true);   // hit
  tt.mutableNodes()[d].is_leaf = true;

  // window=3 walks back from leaf d through {d, c, b}: hits = 2 (d, b)
  EXPECT_EQ(tt.countHitsInWindow(d, 3), 2);
  // window=5 covers everything: 3 hits (root, b, d)
  EXPECT_EQ(tt.countHitsInWindow(d, 5), 3);
  // window=1 sees only leaf d: 1 hit
  EXPECT_EQ(tt.countHitsInWindow(d, 1), 1);
}

TEST(TrackTree, MergeBranchesDropsBhattacharyyaDuplicates) {
  TrackTree tt(TrackId{9}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_leaf = [&](double px, double py, double score) {
    TrackTreeNode n;
    n.parent = 0;
    n.scan_idx = 1;
    n.score = score;
    n.state = Eigen::VectorXd(4);
    n.state << px, py, 0.0, 0.0;
    n.covariance = Eigen::Matrix4d::Identity();  // unit covariance
    n.time = navtracker::Timestamp::fromSeconds(1.0);
    n.is_leaf = true;
    n.is_hit = true;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  // Two near-duplicate leaves at (0,0) with different scores; a third
  // well-separated leaf at (100, 100).
  const std::size_t a = add_leaf(0.0, 0.0, 5.0);
  const std::size_t b = add_leaf(0.01, 0.01, 3.0);  // ~identical to a
  const std::size_t c = add_leaf(100.0, 100.0, 4.0);

  const std::size_t dropped = tt.mergeBranches(0.5);
  EXPECT_EQ(dropped, 1u);
  // The lower-scoring duplicate (b) was dropped.
  EXPECT_FALSE(tt.nodes()[b].is_leaf);
  EXPECT_TRUE(tt.nodes()[a].is_leaf);
  EXPECT_TRUE(tt.nodes()[c].is_leaf);
}

TEST(TrackTree, PruneKLocalKeepsProtectedLeavesBelowThreshold) {
  TrackTree tt(TrackId{20}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_leaf = [&](double score, bool prot) -> std::size_t {
    TrackTreeNode n;
    n.parent = 0; n.scan_idx = 1; n.score = score;
    n.state = Eigen::Vector4d::Zero();
    n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(1.0);
    n.is_leaf = true;
    n.is_protected = prot;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  const std::size_t hi  = add_leaf(10.0, false);
  const std::size_t mid = add_leaf(5.0,  false);
  const std::size_t lo  = add_leaf(1.0,  true);   // protected
  // pruneKLocal(1): top-1 by score is `hi`. Without protection, `mid`
  // and `lo` would both be demoted. With protection, `lo` survives;
  // only `mid` is demoted.
  const std::size_t dropped = tt.pruneKLocal(1);
  EXPECT_EQ(dropped, 1u);
  EXPECT_TRUE(tt.nodes()[hi].is_leaf);
  EXPECT_FALSE(tt.nodes()[mid].is_leaf);
  EXPECT_TRUE(tt.nodes()[lo].is_leaf);
}

TEST(TrackTree, PruneNScanPreservesProtectedAncestorChain) {
  TrackTree tt(TrackId{21}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_node = [&](std::size_t parent, int sc, double score) -> std::size_t {
    TrackTreeNode n;
    n.parent = parent; n.scan_idx = sc; n.score = score;
    n.state = Eigen::Vector4d::Zero(); n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(static_cast<double>(sc));
    n.is_leaf = false;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  const std::size_t a  = add_node(0, 1, 1.0);
  const std::size_t b  = add_node(0, 1, 0.5);
  const std::size_t a1 = add_node(a, 2, 5.0);
  const std::size_t b1 = add_node(b, 2, 2.0);
  tt.mutableNodes()[a1].is_leaf = true;
  tt.mutableNodes()[b1].is_leaf = true;
  // Mark the loser chain (b → b1) as protected — simulating what the
  // global solve would do for an alternative top-K leaf.
  tt.mutableNodes()[b].is_protected = true;
  tt.mutableNodes()[b1].is_protected = true;

  const std::size_t removed = tt.pruneNScan(1);
  EXPECT_EQ(removed, 0u);
  // Both leaves survive: a1 as the winner-chain leaf, b1 via protection.
  const auto leaves = tt.leafIndices();
  EXPECT_EQ(leaves.size(), 2u);
  std::vector<double> scores;
  for (std::size_t li : leaves) scores.push_back(tt.nodes()[li].score);
  std::sort(scores.begin(), scores.end());
  EXPECT_DOUBLE_EQ(scores[0], 2.0);
  EXPECT_DOUBLE_EQ(scores[1], 5.0);
  // Parent indices must still be valid after the (no-op here) compaction.
  for (const TrackTreeNode& n : tt.nodes()) {
    if (n.parent != TrackTreeNode::kNoParent) {
      EXPECT_LT(n.parent, tt.nodes().size());
    }
  }
}

TEST(TrackTree, MergeBranchesSpotsProtectedLeaf) {
  TrackTree tt(TrackId{22}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_leaf = [&](double px, double py, double score, bool prot) {
    TrackTreeNode n;
    n.parent = 0; n.scan_idx = 1; n.score = score;
    n.state = Eigen::VectorXd(4);
    n.state << px, py, 0.0, 0.0;
    n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(1.0);
    n.is_leaf = true;
    n.is_hit = true;
    n.is_protected = prot;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  // a and b are near-duplicates; b is the lower-scoring one but is
  // protected. mergeBranches must NOT eliminate b.
  const std::size_t a = add_leaf(0.0, 0.0,    5.0, false);
  const std::size_t b = add_leaf(0.01, 0.01,  3.0, true);
  const std::size_t dropped = tt.mergeBranches(0.5);
  EXPECT_EQ(dropped, 0u);
  EXPECT_TRUE(tt.nodes()[a].is_leaf);
  EXPECT_TRUE(tt.nodes()[b].is_leaf);
}

// W5.5: deferred-commitment leaf protection must survive branch().
//
// Real pipeline order (MhtTracker::processBatch): the PREVIOUS scan's global
// solve sets is_protected on the top-K alternative leaves; the NEXT scan calls
// branch() FIRST (MhtTracker.cpp:283), which demotes every leaf to internal and
// creates fresh children. If branch() does not propagate is_protected to those
// children, the very next pruneKLocal/mergeBranches/pruneNScan sees no protected
// leaf and drops the alternative — the protection is always one branch() behind
// and therefore inert. The three protection tests above set is_protected on
// pre-existing leaves and never branch() first, so they cannot catch this.
TEST(TrackTree, ProtectedAlternativeSurvivesPruneAcrossBranch) {
  TrackTree tt(TrackId{50}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  auto add_leaf = [&](double score, bool prot) -> std::size_t {
    TrackTreeNode n;
    n.parent = 0; n.scan_idx = 1; n.score = score;
    n.state = Eigen::Vector4d::Zero();
    n.covariance = Eigen::Matrix4d::Identity();
    n.time = navtracker::Timestamp::fromSeconds(1.0);
    n.is_leaf = true;
    n.is_protected = prot;
    tt.mutableNodes().push_back(n);
    return tt.nodes().size() - 1;
  };
  const std::size_t a = add_leaf(10.0, /*prot=*/false);  // K=1 best
  const std::size_t b = add_leaf(1.0, /*prot=*/true);    // protected alternative
  (void)a;

  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);
  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});

  // Scan 2: branch() FIRST (empty scan → one miss child per leaf, score
  // carried through), THEN pruneKLocal(1). a's child (score 10) outranks b's
  // child (score 1); without inherited protection b's child is demoted and b's
  // subtree is left leafless.
  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {}, navtracker::Timestamp::fromSeconds(2.0), p);
  tt.pruneKLocal(1);

  // TEETH: the protected alternative b must still carry a live leaf.
  bool b_has_live_leaf = false;
  for (std::size_t li : tt.leafIndices()) {
    std::size_t cur = li;
    while (cur != TrackTreeNode::kNoParent) {
      if (cur == b) { b_has_live_leaf = true; break; }
      cur = tt.nodes()[cur].parent;
    }
    if (b_has_live_leaf) break;
  }
  EXPECT_TRUE(b_has_live_leaf)
      << "protected alternative pruned across branch() — protection is one "
         "branch() behind";
}

TEST(TrackTree, MergeBranchesDisabledWhenThresholdZero) {
  TrackTree tt(TrackId{10}, rootNode(0.0, 0.0));
  tt.mutableNodes()[0].is_leaf = false;
  TrackTreeNode n1, n2;
  n1.parent = 0; n1.scan_idx = 1; n1.score = 1.0;
  n1.state = Eigen::Vector4d::Zero(); n1.covariance = Eigen::Matrix4d::Identity();
  n1.time = navtracker::Timestamp::fromSeconds(1.0); n1.is_leaf = true;
  n2 = n1; n2.score = 2.0;
  tt.mutableNodes().push_back(n1);
  tt.mutableNodes().push_back(n2);
  EXPECT_EQ(tt.mergeBranches(0.0), 0u);
  EXPECT_EQ(tt.mergeBranches(-1.0), 0u);
}

// --- Per-sensor miss-branch scoring ---------------------------------------
//
// The miss penalty must reflect which sensor(s) actually surveyed this
// scan instant: log Π_s (1 − P_D^s(track)), with per-sensor P_D
// conditioned on coverage. A global per-scan P_D charges every track
// log(1 − 0.9) ≈ −2.3 at the UNION of all sensors' event rates (~16 Hz
// on AutoFerry) and kills any track its fastest sensor can't see.

namespace {

// The miss leaf is the branch child carrying no measurement.
const TrackTreeNode* findMissLeaf(const TrackTree& tt) {
  for (const auto& n : tt.nodes()) {
    if (n.is_leaf && n.scan_meas_idx == TrackTreeNode::kNoMeasurement)
      return &n;
  }
  return nullptr;
}

navtracker::Measurement farMeasurement(navtracker::SensorKind s) {
  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.sensor = s;
  z.value = Eigen::Vector2d(1000.0, 1000.0);  // never gates
  z.covariance = Eigen::Matrix2d::Identity();
  z.source_id = "t";
  return z;
}

}  // namespace

TEST(TrackTree, MissBranchUsesPerSensorPd) {
  TrackTree tt(TrackId{20}, rootNode(0.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  det.set(navtracker::SensorKind::Lidar,
          navtracker::MeasurementModel::Position2D,
          navtracker::DetectionParams{0.5, 1e-4});

  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {farMeasurement(navtracker::SensorKind::Lidar)},
            navtracker::Timestamp::fromSeconds(1.0), p);

  const TrackTreeNode* miss = findMissLeaf(tt);
  ASSERT_NE(miss, nullptr);
  EXPECT_NEAR(miss->score, std::log(1.0 - 0.5), 1e-12);
}

TEST(TrackTree, MissBranchIgnoresOutOfRangeSensor) {
  // Track at (500, 0); lidar at the origin with 100 m coverage: it could
  // not have detected the track, so the miss costs nothing.
  TrackTree tt(TrackId{21}, rootNode(500.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  det.set(navtracker::SensorKind::Lidar,
          navtracker::MeasurementModel::Position2D,
          navtracker::DetectionParams{0.5, 1e-4, /*max_range_m=*/100.0});

  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {farMeasurement(navtracker::SensorKind::Lidar)},
            navtracker::Timestamp::fromSeconds(1.0), p);

  const TrackTreeNode* miss = findMissLeaf(tt);
  ASSERT_NE(miss, nullptr);
  EXPECT_NEAR(miss->score, 0.0, 1e-12);
}

TEST(TrackTree, MissBranchCombinesDistinctScanSensors) {
  // A scan carrying radar (P_D .8) and lidar (P_D .5) measurements:
  // missed by both → log(0.2) + log(0.5).
  TrackTree tt(TrackId{22}, rootNode(0.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  det.set(navtracker::SensorKind::ArpaTtm,
          navtracker::MeasurementModel::Position2D,
          navtracker::DetectionParams{0.8, 1e-4});
  det.set(navtracker::SensorKind::Lidar,
          navtracker::MeasurementModel::Position2D,
          navtracker::DetectionParams{0.5, 1e-4});

  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf,
            {farMeasurement(navtracker::SensorKind::ArpaTtm),
             farMeasurement(navtracker::SensorKind::Lidar)},
            navtracker::Timestamp::fromSeconds(1.0), p);

  const TrackTreeNode* miss = findMissLeaf(tt);
  ASSERT_NE(miss, nullptr);
  EXPECT_NEAR(miss->score, std::log(0.2) + std::log(0.5), 1e-12);
}

TEST(TrackTree, ExistencePredictIsDtScaled) {
  // IPDA persistence is a per-second rate: a miss after 0.1 s decays
  // existence less than a miss after 1.0 s; the 1.0 s case reproduces
  // the legacy per-scan recursion exactly.
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);
  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.5, 1e-4});

  auto runMiss = [&](double dt) {
    TrackTreeNode root = rootNode(0.0, 0.0);
    root.existence_probability = 0.8;
    TrackTree tt(TrackId{23}, root);
    TrackTree::BranchParams p{&det, 9.0};
    p.update_existence = true;
    p.existence_persistence = 0.99;
    p.gate_probability_mass = 0.99;
    auto z = farMeasurement(navtracker::SensorKind::Lidar);
    z.time = navtracker::Timestamp::fromSeconds(dt);
    tt.branch(ekf, {z}, navtracker::Timestamp::fromSeconds(dt), p);
    const TrackTreeNode* miss = findMissLeaf(tt);
    return miss ? miss->existence_probability : -1.0;
  };

  const double r_fast = runMiss(0.1);
  const double r_slow = runMiss(1.0);
  EXPECT_GT(r_fast, r_slow);

  // dt = 1 s endpoint == legacy closed form (Musicki 1994 miss update).
  const double r_pred = 0.99 * 0.8;
  const double l_miss = 1.0 - 0.5 * 0.99;
  const double r_expected = r_pred * l_miss / (1.0 - r_pred + r_pred * l_miss);
  EXPECT_NEAR(r_slow, r_expected, 1e-12);
}

TEST(TrackTree, MissBranchSplitsSameKindSensorsBySourceId) {
  // EO and IR cameras share SensorKind::EoIr but are distinct physical
  // sensors: a scan carrying a bearing from each must charge BOTH miss
  // penalties, each with its own source-keyed P_D.
  TrackTree tt(TrackId{30}, rootNode(0.0, 0.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  det.set(navtracker::SensorKind::EoIr,
          navtracker::MeasurementModel::Position2D, "eo",
          navtracker::DetectionParams{0.8, 1e-4});
  det.set(navtracker::SensorKind::EoIr,
          navtracker::MeasurementModel::Position2D, "ir",
          navtracker::DetectionParams{0.4, 1e-4});

  navtracker::Measurement eo = farMeasurement(navtracker::SensorKind::EoIr);
  eo.source_id = "eo";
  navtracker::Measurement ir = farMeasurement(navtracker::SensorKind::EoIr);
  ir.source_id = "ir";

  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {eo, ir}, navtracker::Timestamp::fromSeconds(1.0), p);

  const TrackTreeNode* miss = findMissLeaf(tt);
  ASSERT_NE(miss, nullptr);
  EXPECT_NEAR(miss->score, std::log(1.0 - 0.8) + std::log(1.0 - 0.4), 1e-12);
}

TEST(TrackTree, HitScoreUsesSourceKeyedClutter) {
  // Two identical gating measurements that differ only in source_id score
  // against their own λ_C: Δscore = log(λ_ir) − log(λ_eo) after the shared
  // P_D terms cancel (same P_D in both entries).
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  det.set(navtracker::SensorKind::EoIr,
          navtracker::MeasurementModel::Position2D, "eo",
          navtracker::DetectionParams{0.6, 0.9});
  det.set(navtracker::SensorKind::EoIr,
          navtracker::MeasurementModel::Position2D, "ir",
          navtracker::DetectionParams{0.6, 0.3});

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.sensor = navtracker::SensorKind::EoIr;
  z.value = Eigen::Vector2d(0.1, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();

  double hit_scores[2];
  const char* sources[2] = {"eo", "ir"};
  for (int i = 0; i < 2; ++i) {
    TrackTree tt(TrackId{31}, rootNode(0.0, 0.0));
    navtracker::Measurement m = z;
    m.source_id = sources[i];
    TrackTree::BranchParams p{&det, 9.0};
    tt.branch(ekf, {m}, navtracker::Timestamp::fromSeconds(1.0), p);
    double best = -1e300;
    for (const auto& n : tt.nodes())
      if (n.is_leaf && n.is_hit) best = std::max(best, n.score);
    hit_scores[i] = best;
  }
  EXPECT_NEAR(hit_scores[1] - hit_scores[0], std::log(0.9) - std::log(0.3),
              1e-9);
}

TEST(TrackTree, MissBranchIgnoresOutOfSectorSensor) {
  // Track due north of the sensor; camera sector looks east (±45°). The
  // camera could not have seen the track → its miss costs nothing.
  TrackTree tt(TrackId{32}, rootNode(0.0, 500.0));
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});
  navtracker::DetectionParams cam{0.8, 1e-4};
  cam.sector_center_rad = 0.0;
  cam.sector_width_rad = M_PI / 2.0;
  det.set(navtracker::SensorKind::EoIr,
          navtracker::MeasurementModel::Position2D, cam);

  TrackTree::BranchParams p{&det, 9.0};
  tt.branch(ekf, {farMeasurement(navtracker::SensorKind::EoIr)},
            navtracker::Timestamp::fromSeconds(1.0), p);

  const TrackTreeNode* miss = findMissLeaf(tt);
  ASSERT_NE(miss, nullptr);
  EXPECT_NEAR(miss->score, 0.0, 1e-12);
}

TEST(TrackTree, PerSensorGateOverrideWidensGate) {
  // Backlog item 11 (conveyor diagnosis): sparse position sensors must
  // be able to recapture a drifted track, so DetectionParams can widen
  // the gate per sensor. A return at Δ = 10 m (χ² ≈ 30 against the
  // predicted S) is outside the global gate 9 but inside the sensor's
  // declared 100 → the hit branch must exist with the override and must
  // NOT exist without it.
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.sensor = navtracker::SensorKind::ArpaTtm;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();

  for (const bool wide : {false, true}) {
    navtracker::FixedSensorDetectionModel det(
        navtracker::DetectionParams{0.9, 1e-4});
    navtracker::DetectionParams radar{0.8, 1e-5};
    if (wide) radar.gate_threshold = 100.0;
    det.set(navtracker::SensorKind::ArpaTtm,
            navtracker::MeasurementModel::Position2D, radar);

    TrackTree tt(TrackId{40}, rootNode(0.0, 0.0));
    TrackTree::BranchParams p{&det, 9.0};
    tt.branch(ekf, {z}, navtracker::Timestamp::fromSeconds(1.0), p);

    bool has_hit = false;
    for (const auto& n : tt.nodes())
      if (n.is_leaf && n.is_hit) has_hit = true;
    EXPECT_EQ(has_hit, wide) << "wide=" << wide;
  }
}

TEST(TrackTree, RecaptureGateScalesWithPositionAnchorAge) {
  // Conveyor fix, adaptive variant: the position-sensor gate widens
  // with the age of the leaf's last position anchor. A bearing-carried
  // leaf (anchor stale) accepts a 10 m-off radar return; a leaf that
  // was position-anchored moments ago keeps the tight gate and rejects
  // the same return (clutter protection where recapture isn't needed).
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);
  navtracker::FixedSensorDetectionModel det(
      navtracker::DetectionParams{0.9, 1e-4});

  navtracker::Measurement pos0;
  pos0.time = navtracker::Timestamp::fromSeconds(1.0);
  pos0.model = navtracker::MeasurementModel::Position2D;
  pos0.sensor = navtracker::SensorKind::ArpaTtm;
  pos0.value = Eigen::Vector2d(100.0, 0.0);
  pos0.covariance = Eigen::Matrix2d::Identity();

  navtracker::Measurement brg = pos0;
  brg.model = navtracker::MeasurementModel::Bearing2D;
  brg.sensor = navtracker::SensorKind::EoIr;
  brg.value = Eigen::VectorXd::Constant(1, 0.0);
  brg.covariance = Eigen::MatrixXd::Identity(1, 1) * 0.0025;

  navtracker::Measurement late = pos0;
  late.time = navtracker::Timestamp::fromSeconds(1.2);
  late.value = Eigen::Vector2d(110.0, 0.0);  // 10 m off the prediction

  for (const bool bearing_carried : {true, false}) {
    TrackTree tt(TrackId{41}, rootNode(100.0, 0.0));
    TrackTree::BranchParams p{&det, 9.0};
    p.recapture_tau_s = 0.25;
    p.recapture_max_scale = 8.0;

    // Scan 1 (t = 1.0): bearing-carried keeps the anchor at the root's
    // time (0.0); position-carried refreshes it to 1.0.
    tt.branch(ekf, {bearing_carried ? brg : pos0},
              navtracker::Timestamp::fromSeconds(1.0), p);
    // Keep only the hit branch to make scan 2 deterministic.
    for (auto& n : tt.mutableNodes())
      if (n.is_leaf && !n.is_hit) n.is_leaf = false;

    // Scan 2 (t = 1.2): radar return 10 m off. Bearing-carried leaf has
    // anchor age 1.2 → gate ≈ 9·(1 + 1.2/0.25) = 52 → recaptured.
    // Position-anchored leaf has age 0.2 → gate ≈ 16 → rejected.
    tt.branch(ekf, {late}, navtracker::Timestamp::fromSeconds(1.2), p);
    bool recaptured = false;
    for (const auto& n : tt.nodes())
      if (n.is_leaf && n.is_hit && n.scan_idx == 2) recaptured = true;
    EXPECT_EQ(recaptured, bearing_carried)
        << "bearing_carried=" << bearing_carried;
  }
}
