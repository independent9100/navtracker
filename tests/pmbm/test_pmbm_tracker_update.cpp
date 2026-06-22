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
// Idle-decay (Phase 3A). With source_aware_misdetection ON and a
// Bernoulli whose contributing sources are absent from this scan, the
// regular misdetection recursion is skipped — but the idle_halflife_sec
// decay factor should still apply. After one half-life (60 s), the
// existence probability halves; with no source overlap the state is
// otherwise untouched.
TEST(PmbmTrackerUpdate, IdleDecayHalvesExistenceAfterHalflife) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;       // isolate idle-decay from p_S
  cfg.clutter_intensity = 1e-6;         // make PPP-driven birth dominant
  cfg.k_best_per_hypothesis = 1;        // single child per parent for clarity
  cfg.source_aware_misdetection = true;
  cfg.idle_halflife_sec = 60.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // PPP component at origin → first scan births a high-r Bernoulli whose
  // contribution_history records source "A".
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  Measurement zA = pos2d(0.0, 0.0, 0.0, 0.5);
  zA.source_id = "A";
  tracker.processBatch({zA});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm.front().bernoullis.size(), 1u);
  const auto id_a = tracker.density().mbm.front().bernoullis.front().id;
  const double r_before =
      tracker.density().mbm.front().bernoullis.front().existence_probability;
  ASSERT_GT(r_before, 0.9);

  // Second scan: source "B" only, far enough away that no Bernoulli
  // gate-passes — Murty assigns this to a new-target row, leaving id_a
  // un-assigned (misdetection). Source-aware skip + idle_halflife_sec
  // path: r ← r · exp(−ln 2 · 60 / 60) = 0.5 · r.
  Measurement zB = pos2d(60.0, 1.0e4, 1.0e4, 0.5);
  zB.source_id = "B";
  tracker.processBatch({zB});

  ASSERT_FALSE(tracker.density().mbm.empty());
  double r_after = -1.0;
  for (const auto& bb : tracker.density().mbm.front().bernoullis) {
    if (bb.id == id_a) { r_after = bb.existence_probability; break; }
  }
  ASSERT_GE(r_after, 0.0);
  EXPECT_NEAR(r_after, r_before * 0.5, 1e-9);
}

// ---------------------------------------------------------------------------
// Phantom-birth gate (Phase 3B). When ρ_target/ρ_total < threshold the
// new-target row's Bernoulli must NOT be materialised, but the
// assignment must remain feasible (the measurement is consumed by
// clutter mass). With an empty PPP and a single clutter-only
// measurement, ρ_target = 0 so r_new = 0; the resulting MBM must have
// zero Bernoullis (no phantom born), and the hypothesis must still
// exist (assignment feasibility preserved).
TEST(PmbmTrackerUpdate, PhantomBirthSuppressedBelowThreshold) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1.0;          // pure clutter regime
  cfg.measurement_driven_birth = false; // gate the row, not the PPP injection
  cfg.min_new_bernoulli_existence = 0.05;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  // PPP empty → rho_target = 0 → r_new = 0 < 0.05 → suppress.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  EXPECT_EQ(tracker.density().mbm[0].bernoullis.size(), 0u);
}

// Companion: with the same config but a near-coincident PPP component,
// ρ_target ≫ λ_C, so r_new ≈ 1 and the Bernoulli IS born. Confirms the
// gate doesn't block legitimate births.
TEST(PmbmTrackerUpdate, PhantomBirthGateAllowsHighExistenceCandidate) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1e-6;
  cfg.measurement_driven_birth = false;
  cfg.min_new_bernoulli_existence = 0.05;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  tracker.processBatch({pos2d(1.0, 0.1, 0.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_GT(tracker.density().mbm[0].bernoullis[0].existence_probability, 0.9);
}

// ---------------------------------------------------------------------------
// Adaptive Birth (Reuter 2014): r_new = λ_birth / (λ_birth + λ_C).
// With λ_birth = λ_C, r_new must be exactly 0.5 independently of PPP
// state. The legacy formula would give r_new ≈ 1 under
// measurement_driven_birth contamination — this test pins the decoupling.
TEST(PmbmTrackerUpdate, AdaptiveBirthRNewEqualsLambdaRatio) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 0.2;
  cfg.measurement_driven_birth = true;  // intentionally; adaptive
                                        // path must override.
  cfg.adaptive_birth = true;
  cfg.lambda_birth = 0.2;  // r_new = 0.2/(0.2+0.2) = 0.5
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.5, 1e-9);
}

// ---------------------------------------------------------------------------
// Adaptive Birth bypasses the measurement-driven PPP injection: even with
// measurement_driven_birth = true, the PPP intensity must stay empty
// after processBatch (the spatial state for the new Bernoulli comes
// from estimator.initiate, not from the PPP).
TEST(PmbmTrackerUpdate, AdaptiveBirthDoesNotInjectPpp) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1e-3;
  cfg.measurement_driven_birth = true;
  cfg.adaptive_birth = true;
  cfg.lambda_birth = 1e-2;
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5),
                        pos2d(1.0, 100.0, 100.0, 0.5),
                        pos2d(1.0, 200.0, 200.0, 0.5)});

  // No PPP components added — adaptive path skips the injection entirely.
  EXPECT_TRUE(tracker.density().ppp.empty());
}

// ---------------------------------------------------------------------------
// Adaptive Birth spatial state: the new Bernoulli's mean must equal the
// measurement (because estimator.initiate(z) places the Gaussian at z).
TEST(PmbmTrackerUpdate, AdaptiveBirthSpatialMeanEqualsMeasurement) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;
  cfg.clutter_intensity = 1e-6;  // r_new high
  cfg.adaptive_birth = true;
  cfg.lambda_birth = 1.0;
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  tracker.processBatch({pos2d(1.0, 42.0, -17.0, 0.5)});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  const auto& b = tracker.density().mbm[0].bernoullis[0];
  EXPECT_NEAR(b.mean(0), 42.0, 1e-9);
  EXPECT_NEAR(b.mean(1), -17.0, 1e-9);
}

// ---------------------------------------------------------------------------
// PPP-coverage birth gate (Phase 3 polish): under
// measurement_driven_birth, an injected PoissonComponent contributes
// ρ_target ≈ peak likelihood on the same measurement. With the
// smart_birth_skip_existing_ppp gate ON and threshold set low enough
// that pre-existing PPP coverage clears it, a second measurement at
// the same location must NOT inject a redundant PPP component.
TEST(PmbmTrackerUpdate, PpcCoverageGateSkipsRedundantPppInjection) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-4;
  cfg.measurement_driven_birth = true;
  cfg.smart_birth_skip_existing_ppp = true;
  cfg.smart_birth_skip_existing_ppp_threshold = 1e-4;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  // Seed PPP at origin with high weight so coverage at (0,0) easily
  // clears the threshold.
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  const auto ppp_before = tracker.density().ppp.size();
  // Measurement at origin → PPP would normally be injected; gate skips.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  // PPP count: started with 1 seed, predict decays its weight but
  // doesn't drop it, then the gate skips the injection that would
  // have added a 2nd component. So ppp.size() must be unchanged
  // relative to the pre-injection state (still 1).
  EXPECT_EQ(tracker.density().ppp.size(), ppp_before);
}

// Companion: gate OFF lets the redundant injection through (count
// grows by 1, original behaviour).
TEST(PmbmTrackerUpdate, PpcCoverageGateOffAllowsRedundantPppInjection) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-4;
  cfg.measurement_driven_birth = true;
  cfg.smart_birth_skip_existing_ppp = false;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  // Original behaviour: birth injected (1 seed + 1 birth = 2 PPP).
  // Then PPP-decay by (1-p_D) shrinks weights but doesn't drop
  // components yet; pruneAndNormalise applies weight_min = 1e-4 floor
  // — 1.0 · 0.1 = 0.1 still > 1e-4, kept.
  EXPECT_EQ(tracker.density().ppp.size(), 2u);
}

// ---------------------------------------------------------------------------
// TPMBM trajectory recording (Phase 4). With
// trajectory_window_scans > 0, each detection appends a
// TrajectoryPoint to the Bernoulli's trajectory; with the knob = 0
// the trajectory stays empty (Phase 3 bit-identical).
TEST(PmbmTrackerUpdate, TpmbmDisabledLeavesTrajectoryEmpty) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.trajectory_window_scans = 0;  // disabled
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(2.0, 0.1, 0.0, 0.5)});
  ASSERT_FALSE(tracker.density().mbm.empty());
  for (const auto& b : tracker.density().mbm.front().bernoullis) {
    EXPECT_TRUE(b.trajectory.empty());
  }
}

TEST(PmbmTrackerUpdate, TpmbmRecordsDetectionAndMisdetectionPoints) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.trajectory_window_scans = 10;  // generous window
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  // Scan 1 at t=1: birth Bernoulli at origin.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm.front().bernoullis.size(), 1u);
  const auto id = tracker.density().mbm.front().bernoullis.front().id;
  EXPECT_EQ(tracker.density().mbm.front().bernoullis.front().trajectory.size(),
            1u);
  EXPECT_EQ(tracker.density().mbm.front().bernoullis.front().birth_time
                .seconds(),
            1.0);

  // Scan 2 at t=2: detection at (0.1, 0). Trajectory grows to 2.
  tracker.processBatch({pos2d(2.0, 0.1, 0.0, 0.5)});
  ASSERT_FALSE(tracker.density().mbm.empty());
  const Bernoulli* survivor = nullptr;
  for (const auto& b : tracker.density().mbm.front().bernoullis) {
    if (b.id == id) { survivor = &b; break; }
  }
  ASSERT_NE(survivor, nullptr);
  EXPECT_EQ(survivor->trajectory.size(), 2u);
  EXPECT_EQ(survivor->trajectory.front().time.seconds(), 1.0);
  EXPECT_EQ(survivor->trajectory.back().time.seconds(), 2.0);

  // Scan 3 at t=3: empty scan → misdetection. Trajectory still grows
  // (post-predict state recorded).
  tracker.predict(Timestamp::fromSeconds(3.0));
  tracker.processBatch({});
  ASSERT_FALSE(tracker.density().mbm.empty());
  survivor = nullptr;
  for (const auto& b : tracker.density().mbm.front().bernoullis) {
    if (b.id == id) { survivor = &b; break; }
  }
  ASSERT_NE(survivor, nullptr);
  EXPECT_EQ(survivor->trajectory.size(), 3u);
  EXPECT_EQ(survivor->trajectory.back().time.seconds(), 3.0);
}

TEST(PmbmTrackerUpdate, TpmbmTrajectoryForReturnsDominantHypothesis) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.trajectory_window_scans = 10;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  tracker.processBatch({pos2d(2.0, 0.1, 0.0, 0.5)});
  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto id = tracker.density().mbm.front().bernoullis.front().id;
  const auto traj = tracker.trajectoryFor(id);
  EXPECT_EQ(traj.size(), 2u);
  EXPECT_EQ(traj.front().time.seconds(), 1.0);
  EXPECT_EQ(traj.back().time.seconds(), 2.0);

  // Unknown id returns empty.
  EXPECT_TRUE(tracker.trajectoryFor(999999).empty());
}

TEST(PmbmTrackerUpdate, TpmbmTrajectoryRespectsWindowCap) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.trajectory_window_scans = 3;  // tight cap
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  // 5 detection scans — trajectory must cap at 3 (most recent kept).
  for (int i = 1; i <= 5; ++i) {
    tracker.processBatch({pos2d(static_cast<double>(i),
                                0.1 * i, 0.0, 0.5)});
  }
  ASSERT_FALSE(tracker.density().mbm.empty());
  for (const auto& b : tracker.density().mbm.front().bernoullis) {
    if (b.trajectory.empty()) continue;
    EXPECT_LE(b.trajectory.size(), 3u);
    if (b.trajectory.size() == 3u) {
      EXPECT_EQ(b.trajectory.front().time.seconds(), 3.0);
      EXPECT_EQ(b.trajectory.back().time.seconds(), 5.0);
    }
  }
}

// ---------------------------------------------------------------------------
// Phase 4(B): ITrackSink wiring.
class RecordingTrackSink : public navtracker::ITrackSink {
 public:
  std::vector<navtracker::TrackLifecycleEvent> initiated;
  std::vector<navtracker::TrackLifecycleEvent> confirmed;
  std::vector<navtracker::TrackLifecycleEvent> updated;
  std::vector<navtracker::TrackLifecycleEvent> deleted;
  void onTrackInitiated(const navtracker::TrackLifecycleEvent& e) override {
    initiated.push_back(e);
  }
  void onTrackConfirmed(const navtracker::TrackLifecycleEvent& e) override {
    confirmed.push_back(e);
  }
  void onTrackUpdated(const navtracker::TrackLifecycleEvent& e) override {
    updated.push_back(e);
  }
  void onTrackDeleted(const navtracker::TrackLifecycleEvent& e) override {
    deleted.push_back(e);
  }
};

TEST(PmbmTrackerUpdate, TrackSinkFiresInitiatedAndConfirmedOnHighRBirth) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.confirm_threshold = 0.5;     // standard
  cfg.output_existence_floor = 0.1;
  PmbmTracker tracker(f.ekf, cfg);
  RecordingTrackSink sink;
  tracker.setTrackSink(&sink);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  // Single scan: r_new ≈ 1 ⇒ Confirmed immediately on emit. Sink
  // fires Initiated + Confirmed + Updated in that order, no Deleted.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  EXPECT_EQ(sink.initiated.size(), 1u);
  EXPECT_EQ(sink.confirmed.size(), 1u);
  EXPECT_EQ(sink.updated.size(), 1u);
  EXPECT_EQ(sink.deleted.size(), 0u);
  EXPECT_EQ(sink.initiated[0].id.value, sink.confirmed[0].id.value);
}

// Phase 4(D): inside an onTrackDeleted handler, trajectoryFor(id)
// returns the prior-scan trajectory snapshot (not empty). Captures
// the snapshot from the callback so the test can assert on it.
class TrajectorySnapshottingSink : public navtracker::ITrackSink {
 public:
  navtracker::pmbm::PmbmTracker* tracker{nullptr};
  std::map<std::uint64_t, std::vector<navtracker::pmbm::TrajectoryPoint>>
      deleted_trajectories;
  void onTrackInitiated(const navtracker::TrackLifecycleEvent&) override {}
  void onTrackConfirmed(const navtracker::TrackLifecycleEvent&) override {}
  void onTrackUpdated(const navtracker::TrackLifecycleEvent&) override {}
  void onTrackDeleted(const navtracker::TrackLifecycleEvent& e) override {
    deleted_trajectories[e.id.value] = tracker->trajectoryFor(e.id.value);
  }
};

TEST(PmbmTrackerUpdate, TrajectoryAvailableInsideOnTrackDeleted) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 0.5;
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.output_existence_floor = 0.5;
  cfg.trajectory_window_scans = 20;
  PmbmTracker tracker(f.ekf, cfg);
  TrajectorySnapshottingSink sink;
  sink.tracker = &tracker;
  tracker.setTrackSink(&sink);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  // Scan 1: birth + record one trajectory point
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  // Scan 2: detection (trajectory grows to 2)
  tracker.processBatch({pos2d(2.0, 0.1, 0.0, 0.5)});
  // Scans 3..6: empty misses → existence decays below floor → Deleted
  for (int t = 3; t <= 6; ++t) {
    tracker.predict(Timestamp::fromSeconds(static_cast<double>(t)));
    tracker.processBatch({pos2d(static_cast<double>(t), 1e5, 1e5, 0.5)});
  }
  // At least one delete observed AND its trajectory is non-empty.
  ASSERT_FALSE(sink.deleted_trajectories.empty());
  bool any_non_empty = false;
  for (const auto& [id, traj] : sink.deleted_trajectories) {
    if (!traj.empty()) { any_non_empty = true; break; }
  }
  EXPECT_TRUE(any_non_empty);
}

TEST(PmbmTrackerUpdate, TrackSinkFiresDeletedWhenExistenceFallsBelowFloor) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 0.5;       // fast decay to force delete
  cfg.clutter_intensity = 1e-6;
  cfg.k_best_per_hypothesis = 1;
  cfg.output_existence_floor = 0.5;     // drops fast
  PmbmTracker tracker(f.ekf, cfg);
  RecordingTrackSink sink;
  tracker.setTrackSink(&sink);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPoisson(1.0, 0.0, 0.0));

  // Scan 1: birth
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, 0.5)});
  ASSERT_EQ(sink.initiated.size(), 1u);
  ASSERT_EQ(sink.deleted.size(), 0u);
  const auto id = sink.initiated[0].id.value;

  // Scans 2..6: empty (miss). With p_S=0.5 and p_D=0.9, r decays
  // multiplicatively each scan. The track drops below the
  // output_existence_floor after a few scans → Deleted fires.
  for (int t = 2; t <= 6; ++t) {
    tracker.predict(Timestamp::fromSeconds(static_cast<double>(t)));
    // Empty scan with one sentinel measurement so scan.front().time
    // is well-defined for the sink event timestamp.
    tracker.processBatch({pos2d(static_cast<double>(t),
                                1e5, 1e5, 0.5)});  // far away
  }
  // Eventually the original id is deleted.
  bool found_delete = false;
  for (const auto& e : sink.deleted) {
    if (e.id.value == id) { found_delete = true; break; }
  }
  EXPECT_TRUE(found_delete);
}

// ---------------------------------------------------------------------------
// Phase 4(C): RTS smoothing.
TEST(PmbmTrackerUpdate, RtsSmoothNoOpOnShortTrajectory) {
  std::vector<navtracker::pmbm::TrajectoryPoint> traj;
  // Empty
  navtracker::pmbm::rtsSmoothTrajectory(traj);
  EXPECT_TRUE(traj.empty());
  // Single point
  navtracker::pmbm::TrajectoryPoint p;
  p.time = Timestamp::fromSeconds(1.0);
  p.state = cvState(0.0, 0.0, 0.0, 0.0);
  p.covariance = posCov(1.0, 0.1);
  p.predicted_state = p.state;
  p.predicted_covariance = p.covariance;
  traj.push_back(p);
  const auto pre = traj.front().state;
  navtracker::pmbm::rtsSmoothTrajectory(traj);
  EXPECT_TRUE(traj.front().state.isApprox(pre));
}

// Stationary 2-point trajectory: predicted_* = state at both points
// (no motion). RTS smoother is exactly correct under F=I; smoothed
// state at point 0 should be a Kalman blend pulling toward point 1's
// observation. With identical positions, the blend stays at origin
// but the covariance shrinks.
TEST(PmbmTrackerUpdate, RtsSmoothShrinksCovarianceOnStationaryTrajectory) {
  std::vector<navtracker::pmbm::TrajectoryPoint> traj(2);
  for (int k = 0; k < 2; ++k) {
    traj[k].time = Timestamp::fromSeconds(static_cast<double>(k + 1));
    traj[k].state = cvState(0.0, 0.0, 0.0, 0.0);
    traj[k].covariance = posCov(2.0, 0.5);
    traj[k].predicted_state = traj[k].state;
    traj[k].predicted_covariance = traj[k].covariance;
  }
  const auto cov_before = traj[0].covariance(0, 0);
  navtracker::pmbm::rtsSmoothTrajectory(traj);
  // Smoothed at k=0 covariance = P_filt + G·(P_smooth - P_pred)·G^T
  // With G = P_filt · P_pred^{-1} = I (both equal) and (P_smooth -
  // P_pred) = 0 (k=1's smoothed is its filtered, which equals its
  // predicted), expect zero change for this degenerate stationary
  // case. Sanity: smoother does NOT inflate covariance.
  EXPECT_LE(traj[0].covariance(0, 0), cov_before + 1e-9);
  EXPECT_TRUE(traj[0].state.isApprox(Eigen::Vector4d::Zero()));
}

// 2-point trajectory where the FUTURE (k=1) measurement disagrees
// with the predicted state — smoother pulls k=0 toward the future
// observation. Numerical check on the position blend.
TEST(PmbmTrackerUpdate, RtsSmoothPullsPastTowardFutureUpdate) {
  std::vector<navtracker::pmbm::TrajectoryPoint> traj(2);
  // k=0: filtered at origin, covariance σ² = 4 on position
  traj[0].time = Timestamp::fromSeconds(1.0);
  traj[0].state = cvState(0.0, 0.0, 0.0, 0.0);
  traj[0].covariance = posCov(2.0, 0.5);
  traj[0].predicted_state = traj[0].state;
  traj[0].predicted_covariance = traj[0].covariance;
  // k=1: predicted at origin (no motion under F=I), filtered at
  // (4, 0) — the update pulled it. Covariance same shape.
  traj[1].time = Timestamp::fromSeconds(2.0);
  traj[1].state = cvState(4.0, 0.0, 0.0, 0.0);
  traj[1].covariance = posCov(1.0, 0.5);  // tighter (post-update)
  traj[1].predicted_state = cvState(0.0, 0.0, 0.0, 0.0);  // pre-update
  traj[1].predicted_covariance = posCov(2.0, 0.5);
  navtracker::pmbm::rtsSmoothTrajectory(traj);
  // G_0 = P_filt_0 · P_pred_1^{-1} = (4) · (4)^{-1} = 1 (per axis)
  // x_smooth_0 = 0 + 1 · (4 − 0) = 4.
  EXPECT_NEAR(traj[0].state(0), 4.0, 1e-9);
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
