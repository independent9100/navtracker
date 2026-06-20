#include <gtest/gtest.h>

#include <memory>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Timestamp;
using navtracker::pmbm::Bernoulli;
using navtracker::pmbm::GlobalHypothesis;
using navtracker::pmbm::PmbmTracker;
using navtracker::pmbm::PoissonComponent;

namespace {

// Linear-CV state: [px, py, vx, vy].
Eigen::Vector4d cvState(double px, double py, double vx, double vy) {
  return Eigen::Vector4d(px, py, vx, vy);
}

Eigen::Matrix4d posCov(double sigma_pos, double sigma_vel) {
  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = P(1, 1) = sigma_pos * sigma_pos;
  P(2, 2) = P(3, 3) = sigma_vel * sigma_vel;
  return P;
}

PoissonComponent makePoisson(double weight, double px, double py, double vx,
                             double vy) {
  PoissonComponent c;
  c.weight = weight;
  c.mean = cvState(px, py, vx, vy);
  c.covariance = posCov(5.0, 1.0);
  return c;
}

Bernoulli makeBernoulli(navtracker::pmbm::BernoulliId id, double r, double px,
                        double py, double vx, double vy) {
  Bernoulli b;
  b.id = id;
  b.existence_probability = r;
  b.mean = cvState(px, py, vx, vy);
  b.covariance = posCov(2.0, 0.5);
  b.last_update = Timestamp::fromSeconds(0.0);
  return b;
}

struct Fixture {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
};

}  // namespace

TEST(PmbmTrackerPredict, FirstPredictInitialisesFilterTime) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  EXPECT_FALSE(tracker.hasCurrentTime());

  tracker.predict(Timestamp::fromSeconds(1.0));
  EXPECT_TRUE(tracker.hasCurrentTime());
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 1.0);
  EXPECT_TRUE(tracker.density().empty());
}

TEST(PmbmTrackerPredict, EmptyDensityPredictIsNoOpButAdvancesTime) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.predict(Timestamp::fromSeconds(5.0));
  EXPECT_TRUE(tracker.density().empty());
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 5.0);
}

TEST(PmbmTrackerPredict, PpmComponentWeightDecaysByPS) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.survival_probability = 0.95;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(
      makePoisson(1.0, 0.0, 0.0, 0.0, 0.0));

  tracker.predict(Timestamp::fromSeconds(1.0));

  ASSERT_EQ(tracker.density().ppp.size(), 1u);
  EXPECT_DOUBLE_EQ(tracker.density().ppp[0].weight, 0.95);
}

TEST(PmbmTrackerPredict, PpmComponentMeanPropagatesViaEstimator) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(
      makePoisson(1.0, 10.0, 20.0, 3.0, -2.0));

  tracker.predict(Timestamp::fromSeconds(2.0));

  // CV motion: px += vx * dt, py += vy * dt; vx, vy unchanged.
  const auto& c = tracker.density().ppp[0];
  EXPECT_DOUBLE_EQ(c.mean(0), 10.0 + 3.0 * 2.0);
  EXPECT_DOUBLE_EQ(c.mean(1), 20.0 + (-2.0) * 2.0);
  EXPECT_DOUBLE_EQ(c.mean(2), 3.0);
  EXPECT_DOUBLE_EQ(c.mean(3), -2.0);
}

TEST(PmbmTrackerPredict, BernoulliExistenceDecaysByPS) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.survival_probability = 0.99;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  GlobalHypothesis h;
  h.weight = 1.0;
  h.bernoullis.push_back(makeBernoulli(1, 0.8, 0.0, 0.0, 0.0, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  tracker.predict(Timestamp::fromSeconds(1.0));

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_DOUBLE_EQ(tracker.density().mbm[0].bernoullis[0].existence_probability,
                   0.8 * 0.99);
  EXPECT_DOUBLE_EQ(tracker.density().mbm[0].weight, 1.0);
}

TEST(PmbmTrackerPredict, BernoulliMeanPropagatesViaEstimator) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(0.0));
  GlobalHypothesis h;
  h.weight = 1.0;
  h.bernoullis.push_back(makeBernoulli(42, 0.9, 100.0, 200.0, 5.0, 7.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  tracker.predict(Timestamp::fromSeconds(3.0));

  const auto& b = tracker.density().mbm[0].bernoullis[0];
  EXPECT_DOUBLE_EQ(b.mean(0), 100.0 + 5.0 * 3.0);
  EXPECT_DOUBLE_EQ(b.mean(1), 200.0 + 7.0 * 3.0);
  EXPECT_DOUBLE_EQ(b.mean(2), 5.0);
  EXPECT_DOUBLE_EQ(b.mean(3), 7.0);
  EXPECT_EQ(b.id, 42u);
  EXPECT_DOUBLE_EQ(b.last_update.seconds(), 3.0);
}

TEST(PmbmTrackerPredict, ZeroDtPredictIsNoOp) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(
      makePoisson(1.0, 0.0, 0.0, 0.0, 0.0));

  tracker.predict(Timestamp::fromSeconds(0.0));
  EXPECT_DOUBLE_EQ(tracker.density().ppp[0].weight, 1.0);
}

TEST(PmbmTrackerPredict, StaleScanAdvancesTimeOnly) {
  // Stale input convention (matches MhtTracker): a `to` ≤ currentTime()
  // advances the clock without propagating; the math is undefined under
  // negative dt.
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(5.0));
  tracker.mutableDensityForTesting().ppp.push_back(
      makePoisson(0.5, 0.0, 0.0, 1.0, 0.0));

  tracker.predict(Timestamp::fromSeconds(3.0));
  // Time advances to 3.0 (no rewind protection — caller's responsibility),
  // but density does not propagate.
  EXPECT_DOUBLE_EQ(tracker.density().ppp[0].weight, 0.5);
  EXPECT_DOUBLE_EQ(tracker.density().ppp[0].mean(0), 0.0);
  EXPECT_DOUBLE_EQ(tracker.currentTime().seconds(), 3.0);
}

TEST(PmbmTrackerPredict, BirthModelInjectsPpmComponentsEachPredict) {
  Fixture f;
  auto birth =
      [](Timestamp /*t*/, double /*dt*/) -> std::vector<PoissonComponent> {
    std::vector<PoissonComponent> v;
    PoissonComponent c;
    c.weight = 0.1;
    c.mean = Eigen::Vector4d::Zero();
    c.covariance = Eigen::Matrix4d::Identity() * 100.0;
    v.push_back(c);
    return v;
  };
  PmbmTracker tracker(f.ekf, {}, birth);

  tracker.predict(Timestamp::fromSeconds(0.0));
  // Birth fires on initialisation (dt = 0 is allowed for the seed).
  EXPECT_EQ(tracker.density().ppp.size(), 1u);

  tracker.predict(Timestamp::fromSeconds(1.0));
  // And again on each subsequent predict.
  EXPECT_EQ(tracker.density().ppp.size(), 2u);
}

TEST(PmbmTrackerPredict, MultipleHypothesesAndBernoullisAllPropagate) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.survival_probability = 0.9;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  auto& d = tracker.mutableDensityForTesting();

  GlobalHypothesis h1;
  h1.weight = 0.7;
  h1.bernoullis.push_back(makeBernoulli(1, 1.0, 0.0, 0.0, 2.0, 0.0));
  h1.bernoullis.push_back(makeBernoulli(2, 0.5, 10.0, 0.0, 0.0, 0.0));
  GlobalHypothesis h2;
  h2.weight = 0.3;
  h2.bernoullis.push_back(makeBernoulli(1, 0.9, 0.0, 0.0, 2.0, 0.0));
  d.mbm.push_back(std::move(h1));
  d.mbm.push_back(std::move(h2));

  tracker.predict(Timestamp::fromSeconds(1.0));

  // Mixture weights unchanged by predict (only update redistributes mass
  // across global hypotheses).
  EXPECT_DOUBLE_EQ(tracker.density().mbm[0].weight, 0.7);
  EXPECT_DOUBLE_EQ(tracker.density().mbm[1].weight, 0.3);

  // Each Bernoulli existence decayed by p_S; positions advanced.
  EXPECT_DOUBLE_EQ(
      tracker.density().mbm[0].bernoullis[0].existence_probability, 0.9);
  EXPECT_DOUBLE_EQ(tracker.density().mbm[0].bernoullis[0].mean(0), 2.0);
  EXPECT_DOUBLE_EQ(
      tracker.density().mbm[0].bernoullis[1].existence_probability, 0.45);
  EXPECT_DOUBLE_EQ(tracker.density().mbm[0].bernoullis[1].mean(0), 10.0);
  EXPECT_DOUBLE_EQ(
      tracker.density().mbm[1].bernoullis[0].existence_probability, 0.81);
}
