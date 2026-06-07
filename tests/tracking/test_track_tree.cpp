#include <algorithm>
#include <gtest/gtest.h>
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

  TrackTree::BranchParams p{0.9, 1e-4, 9.0};
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

  TrackTree::BranchParams p{0.9, 1e-4, 9.0};
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
