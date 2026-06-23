#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/pmbm/PmbmTypes.hpp"

namespace nt = navtracker::pmbm;

namespace {

nt::PoissonComponent makePoissonComponent(double weight) {
  nt::PoissonComponent c;
  c.weight = weight;
  c.mean = Eigen::Vector4d::Zero();
  c.covariance = Eigen::Matrix4d::Identity();
  return c;
}

nt::Bernoulli makeBernoulli(nt::BernoulliId id, double r) {
  nt::Bernoulli b;
  b.id = id;
  b.existence_probability = r;
  b.mean = Eigen::Vector4d::Zero();
  b.covariance = Eigen::Matrix4d::Identity();
  return b;
}

}  // namespace

TEST(PmbmTypes, PoissonComponentMassReportsWeight) {
  const auto c = makePoissonComponent(0.42);
  EXPECT_DOUBLE_EQ(c.mass(), 0.42);
}

TEST(PmbmTypes, BernoulliIsAliveAboveThresholdInclusive) {
  EXPECT_TRUE(makeBernoulli(1, 0.5).isAlive(0.05));
  EXPECT_TRUE(makeBernoulli(1, 0.05).isAlive(0.05));
  EXPECT_FALSE(makeBernoulli(1, 0.049).isAlive(0.05));
  EXPECT_FALSE(makeBernoulli(1, 0.0).isAlive(0.05));
}

TEST(PmbmTypes, DefaultBernoulliIdIsInvalid) {
  const nt::Bernoulli b;
  EXPECT_EQ(b.id, nt::kInvalidBernoulliId);
  EXPECT_DOUBLE_EQ(b.existence_probability, 0.0);
}

TEST(PmbmTypes, GlobalHypothesisAliveCountReadsBernoulliExistence) {
  nt::GlobalHypothesis h;
  h.weight = 1.0;
  h.bernoullis.push_back(makeBernoulli(1, 0.9));
  h.bernoullis.push_back(makeBernoulli(2, 0.1));
  h.bernoullis.push_back(makeBernoulli(3, 0.001));
  EXPECT_EQ(h.aliveCount(0.05), 2u);
  EXPECT_EQ(h.aliveCount(0.5), 1u);
  EXPECT_EQ(h.aliveCount(0.95), 0u);
}

TEST(PmbmTypes, PmbmDensityEmptyAtConstruction) {
  const nt::PmbmDensity d;
  EXPECT_TRUE(d.empty());
  EXPECT_DOUBLE_EQ(d.totalMbmWeight(), 0.0);
}

TEST(PmbmTypes, PmbmDensityTotalMbmWeightSumsAcrossHypotheses) {
  nt::PmbmDensity d;
  nt::GlobalHypothesis h1;
  h1.weight = 0.6;
  nt::GlobalHypothesis h2;
  h2.weight = 0.3;
  nt::GlobalHypothesis h3;
  h3.weight = 0.1;
  d.mbm = {h1, h2, h3};
  EXPECT_DOUBLE_EQ(d.totalMbmWeight(), 1.0);
  EXPECT_FALSE(d.empty());
}

TEST(PerTrackView, EmptyMbmYieldsEmptyView) {
  nt::PmbmDensity d;
  nt::rebuildPerTrackViewFromFlat(d);
  EXPECT_TRUE(d.tracks.empty());
  EXPECT_TRUE(d.tracked_mbm.empty());
}

TEST(PerTrackView, SingleHypSingleBernoulliRebuildsOneTrackOneHyp) {
  nt::PmbmDensity d;
  nt::GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  auto b = makeBernoulli(42, 0.8);
  b.last_update = navtracker::Timestamp::fromSeconds(10.0);
  h.bernoullis.push_back(b);
  d.mbm.push_back(h);
  nt::rebuildPerTrackViewFromFlat(d);
  ASSERT_EQ(d.tracks.size(), 1u);
  EXPECT_EQ(d.tracks[0].id, 42u);
  EXPECT_DOUBLE_EQ(d.tracks[0].birth_time.seconds(), 10.0);
  ASSERT_EQ(d.tracks[0].hypotheses.size(), 1u);
  EXPECT_DOUBLE_EQ(d.tracks[0].hypotheses[0].existence_probability, 0.8);
  ASSERT_EQ(d.tracked_mbm.size(), 1u);
  EXPECT_DOUBLE_EQ(d.tracked_mbm[0].weight, 1.0);
  ASSERT_EQ(d.tracked_mbm[0].hyp_index.size(), 1u);
  EXPECT_EQ(d.tracked_mbm[0].hyp_index[0], 0);
}

TEST(PerTrackView, AbsentTrackInHypothesisFlaggedKAbsent) {
  // h1 has Bernoullis {1, 2}; h2 has Bernoulli {2} only. After view
  // rebuild, h2.hyp_index for track 1 (id=1) must be kAbsent.
  nt::PmbmDensity d;
  nt::GlobalHypothesis h1;
  h1.weight = 0.7;
  h1.bernoullis.push_back(makeBernoulli(1, 0.9));
  h1.bernoullis.push_back(makeBernoulli(2, 0.8));
  nt::GlobalHypothesis h2;
  h2.weight = 0.3;
  h2.bernoullis.push_back(makeBernoulli(2, 0.5));
  d.mbm = {h1, h2};
  nt::rebuildPerTrackViewFromFlat(d);
  ASSERT_EQ(d.tracks.size(), 2u);
  EXPECT_EQ(d.tracks[0].id, 1u);
  EXPECT_EQ(d.tracks[1].id, 2u);
  ASSERT_EQ(d.tracked_mbm.size(), 2u);
  EXPECT_EQ(d.tracked_mbm[0].hyp_index[0], 0);  // h1 sees track 1 → hyp 0
  EXPECT_EQ(d.tracked_mbm[0].hyp_index[1], 0);  // h1 sees track 2 → hyp 0
  EXPECT_EQ(d.tracked_mbm[1].hyp_index[0],
            nt::TrackedGlobalHypothesis::kAbsent);  // h2 lacks track 1
  EXPECT_EQ(d.tracked_mbm[1].hyp_index[1], 1);     // h2 sees track 2 → hyp 1
  EXPECT_EQ(d.tracks[1].hypotheses.size(), 2u);
  EXPECT_DOUBLE_EQ(d.tracks[1].hypotheses[0].existence_probability, 0.8);
  EXPECT_DOUBLE_EQ(d.tracks[1].hypotheses[1].existence_probability, 0.5);
}

TEST(PerTrackView, BirthTimeIsMinOfFlatLastUpdateForId) {
  // Same id appears in two hypotheses with different last_update; view
  // birth_time should be the minimum (lower bound on the true birth).
  nt::PmbmDensity d;
  nt::GlobalHypothesis h1;
  h1.weight = 0.5;
  auto b1 = makeBernoulli(7, 0.9);
  b1.last_update = navtracker::Timestamp::fromSeconds(20.0);
  h1.bernoullis.push_back(b1);
  nt::GlobalHypothesis h2;
  h2.weight = 0.5;
  auto b2 = makeBernoulli(7, 0.7);
  b2.last_update = navtracker::Timestamp::fromSeconds(5.0);
  h2.bernoullis.push_back(b2);
  d.mbm = {h1, h2};
  nt::rebuildPerTrackViewFromFlat(d);
  ASSERT_EQ(d.tracks.size(), 1u);
  EXPECT_DOUBLE_EQ(d.tracks[0].birth_time.seconds(), 5.0);
}

TEST(PerTrackView, RebuildIsIdempotent) {
  nt::PmbmDensity d;
  nt::GlobalHypothesis h;
  h.weight = 1.0;
  h.bernoullis.push_back(makeBernoulli(1, 0.9));
  d.mbm.push_back(h);
  nt::rebuildPerTrackViewFromFlat(d);
  const auto first_size = d.tracks.size();
  nt::rebuildPerTrackViewFromFlat(d);
  EXPECT_EQ(d.tracks.size(), first_size);
  EXPECT_EQ(d.tracks[0].hypotheses.size(), 1u);
}

TEST(PmbmTypes, PpmAndMbmEvolveIndependently) {
  // Smoke test: PPP and MBM are stored independently and have independent
  // empty()/size() semantics. PMBM math composes them only at output time.
  nt::PmbmDensity d;
  d.ppp.push_back(makePoissonComponent(0.5));
  EXPECT_FALSE(d.empty());
  EXPECT_DOUBLE_EQ(d.totalMbmWeight(), 0.0);

  nt::GlobalHypothesis h;
  h.weight = 1.0;
  h.bernoullis.push_back(makeBernoulli(1, 0.8));
  d.mbm.push_back(h);
  EXPECT_FALSE(d.empty());
  EXPECT_DOUBLE_EQ(d.totalMbmWeight(), 1.0);
  EXPECT_EQ(d.ppp.size(), 1u);
  EXPECT_EQ(d.mbm.size(), 1u);
}
