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
