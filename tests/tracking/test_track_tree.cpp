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
