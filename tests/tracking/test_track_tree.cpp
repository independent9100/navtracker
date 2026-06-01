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
