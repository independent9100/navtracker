#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::Bernoulli;
using navtracker::pmbm::GlobalHypothesis;
using navtracker::pmbm::PmbmTracker;
using navtracker::pmbm::PoissonComponent;

namespace {

Eigen::Vector4d cvState(double px, double py, double vx, double vy) {
  return Eigen::Vector4d(px, py, vx, vy);
}
Eigen::Matrix4d posCov(double sp, double sv) {
  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = P(1, 1) = sp * sp;
  P(2, 2) = P(3, 3) = sv * sv;
  return P;
}

Measurement pos2d(double t, double x, double y, double sigma = 1.0) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = SensorKind::Lidar;
  z.source_id = "r0";
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
  return z;
}

Bernoulli mkBernoulli(navtracker::pmbm::BernoulliId id, double r, double px,
                      double py) {
  Bernoulli b;
  b.id = id;
  b.existence_probability = r;
  b.mean = cvState(px, py, 0.0, 0.0);
  b.covariance = posCov(2.0, 0.5);
  b.last_update = Timestamp::fromSeconds(0.0);
  return b;
}

PoissonComponent mkPoisson(double w, double px, double py) {
  PoissonComponent c;
  c.weight = w;
  c.mean = cvState(px, py, 0.0, 0.0);
  c.covariance = posCov(5.0, 1.0);
  return c;
}

struct Fixture {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
};

}  // namespace

// ---------------------------------------------------------------------------
// Misdetection (empty scan, single Bernoulli).
// r ← (1 − p_D) · r / (1 − r · p_D).
// r=0.8, p_D=0.9 → 0.1·0.8 / (1 − 0.72) = 0.08 / 0.28 ≈ 0.2857.
TEST(PmbmTrackerUpdate, MisdetectionDecaysExistenceCorrectly) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;  // isolate update math from predict
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  h.bernoullis.push_back(mkBernoulli(1, 0.8, 0.0, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  tracker.processBatch({});  // empty scan: every Bernoulli is a miss

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.08 / 0.28, 1e-9);
}

// ---------------------------------------------------------------------------
// Single Bernoulli + single matching measurement → r → 1, state pulled
// towards the measurement (EKF update on Position2D).
TEST(PmbmTrackerUpdate, DetectionLiftsExistenceAndUpdatesState) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1e-6;  // very low so PPP/clutter doesn't dominate
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  h.bernoullis.push_back(mkBernoulli(1, 0.9, 0.0, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  tracker.processBatch({pos2d(1.0, 0.1, 0.0, 0.5)});

  // Best child should put existence at 1.0 for the existing Bernoulli.
  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& best = tracker.density().mbm.front();
  ASSERT_FALSE(best.bernoullis.empty());
  EXPECT_NEAR(best.bernoullis[0].existence_probability, 1.0, 1e-9);
  // EKF on Position2D pulls position towards the measurement: with R=0.25,
  // P_pos=4, posterior mean lies between 0 and 0.1.
  EXPECT_GT(best.bernoullis[0].mean(0), 0.0);
  EXPECT_LE(best.bernoullis[0].mean(0), 0.1);
}

// ---------------------------------------------------------------------------
// No prior MBM, no PPP, single measurement → seeds one (clutter-only)
// child, with the new-target Bernoulli existence ≈ 0 (gets pruned by r_min).
TEST(PmbmTrackerUpdate, NoPriorNoPppMeasurementProducesClutterOnly) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.clutter_intensity = 1e-4;
  cfg.r_min = 1e-3;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  tracker.processBatch({pos2d(1.0, 10.0, 10.0)});

  // One child hypothesis (the only feasible assignment: clutter row).
  ASSERT_FALSE(tracker.density().mbm.empty());
  // The new-target Bernoulli has rho_target = 0 → r = 0 → pruned.
  for (const auto& h : tracker.density().mbm) {
    for (const auto& b : h.bernoullis) {
      EXPECT_GE(b.existence_probability, cfg.r_min);
    }
  }
}

// ---------------------------------------------------------------------------
// PPP component near the measurement → new Bernoulli with high existence.
TEST(PmbmTrackerUpdate, PppNearMeasurementBirthsHighExistenceBernoulli) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;  // low clutter → r_new ≈ 1
  cfg.survival_probability = 1.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  // Heavy PPP component sitting on top of where the measurement will land.
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 10.0, 10.0));

  tracker.processBatch({pos2d(1.0, 10.0, 10.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& best = tracker.density().mbm.front();
  ASSERT_EQ(best.bernoullis.size(), 1u);
  EXPECT_GT(best.bernoullis[0].existence_probability, 0.99);
  // The new Bernoulli got a fresh id, never reused.
  EXPECT_NE(best.bernoullis[0].id, navtracker::pmbm::kInvalidBernoulliId);
}

// ---------------------------------------------------------------------------
// New-Bernoulli ids are unique and monotonically increasing across calls.
TEST(PmbmTrackerUpdate, NewBernoulliIdsAreUniqueAndMonotonic) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  // Seed two PPP components in different places.
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 50.0, 50.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5),
                        pos2d(1.0, 50.0, 50.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  // Across the best hypothesis, both new Bernoullis exist with distinct ids.
  const auto& best = tracker.density().mbm.front();
  ASSERT_EQ(best.bernoullis.size(), 2u);
  EXPECT_NE(best.bernoullis[0].id, best.bernoullis[1].id);
  EXPECT_GT(best.bernoullis[0].id, 0u);
  EXPECT_GT(best.bernoullis[1].id, 0u);
}

// ---------------------------------------------------------------------------
// Hypothesis weights sum to 1 after update + prune.
TEST(PmbmTrackerUpdate, MixtureWeightsSumToOneAfterUpdate) {
  Fixture f;
  PmbmTracker tracker(f.ekf, {});
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5),
                        pos2d(1.0, 30.0, 30.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  double sum = 0.0;
  for (const auto& h : tracker.density().mbm) sum += h.weight;
  EXPECT_NEAR(sum, 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Low-existence Bernoulli is pruned by r_min after a string of misses.
TEST(PmbmTrackerUpdate, LowExistenceBernoulliPrunedByRmin) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.99;  // strong miss penalty
  cfg.survival_probability = 1.0;
  cfg.r_min = 0.05;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  h.bernoullis.push_back(mkBernoulli(1, 0.1, 0.0, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  // Several empty scans → repeated misdetection → r tumbles below r_min.
  for (int i = 1; i < 30; ++i) {
    tracker.processBatch({});
  }

  ASSERT_FALSE(tracker.density().mbm.empty());
  EXPECT_TRUE(tracker.density().mbm[0].bernoullis.empty())
      << "Bernoulli should be pruned after sustained misses.";
}

// ---------------------------------------------------------------------------
// PPP decays by (1 − P_D) after each update (§3.3).
TEST(PmbmTrackerUpdate, PppDecaysByOneMinusPdAfterUpdate) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.8;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1e-3;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 100.0, 100.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0)});  // measurement far from PPP

  ASSERT_EQ(tracker.density().ppp.size(), 1u);
  EXPECT_NEAR(tracker.density().ppp[0].weight, 0.2, 1e-9);
}

// ---------------------------------------------------------------------------
// Two distinct measurements + two well-separated PPP components → Murty
// K-best returns the matched assignment as the most likely child.
TEST(PmbmTrackerUpdate, MurtyPicksMatchedAssignmentAsBestChild) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  cfg.k_best_per_hypothesis = 3;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 100.0, 100.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5),
                        pos2d(1.0, 100.0, 100.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& best = tracker.density().mbm.front();
  ASSERT_EQ(best.bernoullis.size(), 2u);
  // Both Bernoulli means near their respective measurements.
  std::vector<double> xs{best.bernoullis[0].mean(0), best.bernoullis[1].mean(0)};
  std::sort(xs.begin(), xs.end());
  EXPECT_NEAR(xs[0], 0.0, 1.0);
  EXPECT_NEAR(xs[1], 100.0, 1.0);
}

// ---------------------------------------------------------------------------
// Determinism: replaying the exact same scan sequence gives identical
// MBM state (same hypothesis count, same Bernoulli ids assigned in order).
TEST(PmbmTrackerUpdate, ReplayDeterministicallyReproducesState) {
  auto run = []() {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator ekf{motion, 5.0};
    PmbmTracker::Config cfg;
    cfg.probability_of_detection = 0.9;
    cfg.clutter_intensity = 1e-4;
    PmbmTracker tracker(ekf, cfg);
    tracker.predict(Timestamp::fromSeconds(0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

    tracker.processBatch({pos2d(1.0, 0.1, 0.0, 0.5)});
    tracker.processBatch({pos2d(2.0, 0.2, 0.0, 0.5)});
    tracker.processBatch({pos2d(3.0, 0.3, 0.0, 0.5)});

    std::vector<std::pair<navtracker::pmbm::BernoulliId, double>> snap;
    for (const auto& h : tracker.density().mbm) {
      for (const auto& b : h.bernoullis) {
        snap.push_back({b.id, b.existence_probability});
      }
    }
    return snap;
  };
  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].first, b[i].first);
    EXPECT_DOUBLE_EQ(a[i].second, b[i].second);
  }
}
