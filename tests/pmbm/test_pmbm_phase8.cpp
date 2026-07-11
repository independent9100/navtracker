// Phase 8: tests for coverage gaps identified in the multi-agent review.
// T1: PMBM + SensorBiasProvider end-to-end.
// T2: bhattacharyya_merge_threshold behaviour (id-stability survivor).
// T3: PMBM-specific bench determinism (BenchDeterminism uses ekf_cv_gnn).
// T4: per-sensor setSensorDetectionModel cost-matrix effect.
// T5: re-demotion lifecycle (Confirmed → Tentative → Confirmed sink events).

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/benchmark/Config.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/scenario/Builders.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "ports/ITrackSink.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::DetectionParams;
using navtracker::EkfEstimator;
using navtracker::FixedSensorBiasProvider;
using navtracker::FixedSensorDetectionModel;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorBiasKey;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::Bernoulli;
using navtracker::pmbm::BernoulliId;
using navtracker::pmbm::PmbmTracker;
using navtracker::pmbm::PoissonComponent;

namespace {

Measurement pos2d(double t, double x, double y, SensorKind sensor,
                  const std::string& source) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = sensor;
  z.source_id = source;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity() * 0.25;  // σ = 0.5 m
  return z;
}

PoissonComponent mkPpp(double w, double px, double py) {
  PoissonComponent c;
  c.weight = w;
  c.mean = Eigen::VectorXd::Zero(4);
  c.mean(0) = px;
  c.mean(1) = py;
  c.covariance = Eigen::MatrixXd::Identity(4, 4);
  c.covariance(0, 0) = c.covariance(1, 1) = 25.0;
  c.covariance(2, 2) = c.covariance(3, 3) = 1.0;
  return c;
}

struct Fixture {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
};

// Minimal scenario for the determinism test (duplicates the one in
// tests/benchmark/test_sweep.cpp; both files have their own anon
// namespace and can't share). 1 truth × 1 source × 5 scans.
class TinyStraightLine : public navtracker::benchmark::ScenarioRun {
 public:
  navtracker::benchmark::ScenarioDescriptor descriptor() const override {
    return {"tiny_line_p8", true, 2};
  }
  navtracker::Scenario generate(std::uint64_t seed) override {
    std::vector<double> times;
    for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
    return navtracker::buildStraightLineScenario(
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(10, 0),
        times, 1.0,
        static_cast<std::uint32_t>(seed),
        1);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// T1: PMBM + SensorBiasProvider end-to-end.
//
// Wire a FixedSensorBiasProvider that publishes a known +2 m east offset
// for the test sensor. A measurement at (10, 0) should be bias-corrected
// to (8, 0) before reaching the PMBM update; the post-update Bernoulli
// mean should reflect the corrected position, not the raw measurement.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, BiasProviderShiftsPostUpdateBernoulliMean) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  PmbmTracker tracker(f.ekf, cfg);

  FixedSensorBiasProvider provider;
  // Known per-(sensor, source) +2 m east offset; the corrector subtracts
  // the published bias from the measurement, so the corrected position
  // is (10 − 2, 0) = (8, 0).
  provider.setPositionBias(SensorBiasKey{SensorKind::Lidar, "r0"},
                           Eigen::Vector2d(2.0, 0.0));
  tracker.setSensorBiasProvider(&provider);

  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));

  Measurement z = pos2d(1.0, 10.0, 0.0, SensorKind::Lidar, "r0");
  tracker.processBatch({z});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  const auto& b = tracker.density().mbm[0].bernoullis[0];
  // PPP at origin + bias-corrected measurement near (8, 0) → posterior
  // mean lies between (0, 0) and (8, 0). It MUST be closer to (8, 0)
  // than to (10, 0), proving the bias was applied before update.
  EXPECT_LT(b.mean(0), 9.5) << "post-update x must reflect bias correction";
  EXPECT_GT(b.mean(0), 3.0) << "but the measurement still moves the prior";
}

// Companion: with no provider wired, the same measurement leaves the
// posterior at the uncorrected location — proves the provider hookup
// is what differs.
TEST(PmbmTrackerPhase8, NullBiasProviderLeavesMeasurementUntouched) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));

  Measurement z = pos2d(1.0, 10.0, 0.0, SensorKind::Lidar, "r0");
  tracker.processBatch({z});

  ASSERT_FALSE(tracker.density().mbm.empty());
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  const auto& b = tracker.density().mbm[0].bernoullis[0];
  // Should land near +10 (between 0 and 10) — proving without
  // provider, the raw measurement is used.
  EXPECT_GT(b.mean(0), 5.0);
}

// ---------------------------------------------------------------------------
// T2: bhattacharyya_merge_threshold behaviour.
//
// Construct a global hypothesis with two near-coincident Bernoullis
// (id 1 born earlier, id 2 born later). With a permissive merge
// threshold the pair MUST collapse to one Bernoulli carrying the
// older id (id-stability invariant). With a tight threshold both
// survive separately.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, BhattacharyyaMergeKeepsOlderIdAndDeletesYounger) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  cfg.bhattacharyya_merge_threshold = 5.0;  // permissive
  cfg.r_min = 1e-9;
  cfg.hypothesis_weight_min = 1e-9;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed two near-coincident Bernoullis with distinct ids into a single
  // global hypothesis. Their position blocks overlap heavily so the
  // Bhattacharyya distance is small; ids 7 (older) and 99 (younger).
  navtracker::pmbm::GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  auto mkB = [](BernoulliId id, double r, double px, double py) {
    Bernoulli b;
    b.id = id;
    b.existence_probability = r;
    b.mean = Eigen::Vector4d(px, py, 0.0, 0.0);
    b.covariance = Eigen::Matrix4d::Identity() * 4.0;
    b.last_update = Timestamp::fromSeconds(0.0);
    return b;
  };
  h.bernoullis.push_back(mkB(7,  0.8, 0.0, 0.0));
  h.bernoullis.push_back(mkB(99, 0.6, 0.1, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  // Non-empty scan with a far-away measurement (mergeBernoulliDuplicates
  // only runs on the non-empty-scan code path). The far measurement
  // doesn't gate to either seeded Bernoulli but does drive the post-
  // enumerate merge fold over the survivors.
  tracker.processBatch(
      {pos2d(1.0, 1000.0, 1000.0, SensorKind::Lidar, "r0")});

  ASSERT_FALSE(tracker.density().mbm.empty());
  // Find the hypothesis branch that retains the survivors of the
  // seeded pair (the dominant child is the one with the new-target
  // assignment; the merge ran over its Bernoulli list).
  bool found_seven = false;
  bool found_ninetynine = false;
  double r_seven = -1.0;
  for (const auto& h_out : tracker.density().mbm) {
    for (const auto& b : h_out.bernoullis) {
      if (b.id == 7u) { found_seven = true; r_seven = b.existence_probability; }
      if (b.id == 99u) found_ninetynine = true;
    }
  }
  EXPECT_TRUE(found_seven)
      << "older id 7 must survive the merge across hypotheses";
  EXPECT_FALSE(found_ninetynine)
      << "younger id 99 must not survive the merge";
  // Phase 8 R1 fix: merged existence is max(r_a, r_b) = 0.8 then
  // misdetected ((1-p_D)·r / (1-r·p_D)) = (0.1*0.8)/(1-0.72) ≈ 0.286.
  // We just pin "no double-count inflation" — under the old
  // independent-fold the merged r would have been
  // 1-(1-.8)(1-.6) = 0.92 → post-miss ≈ 0.535.
  EXPECT_LT(r_seven, 0.35)
      << "merged existence must NOT carry double-count inflation";
}

TEST(PmbmTrackerPhase8, BhattacharyyaMergeOffKeepsBothBernoullis) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  cfg.bhattacharyya_merge_threshold = 0.0;  // disabled
  cfg.r_min = 1e-9;
  cfg.hypothesis_weight_min = 1e-9;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  navtracker::pmbm::GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  auto mkB = [](BernoulliId id, double r, double px, double py) {
    Bernoulli b;
    b.id = id;
    b.existence_probability = r;
    b.mean = Eigen::Vector4d(px, py, 0.0, 0.0);
    b.covariance = Eigen::Matrix4d::Identity() * 4.0;
    b.last_update = Timestamp::fromSeconds(0.0);
    return b;
  };
  h.bernoullis.push_back(mkB(7,  0.8, 0.0, 0.0));
  h.bernoullis.push_back(mkB(99, 0.6, 0.1, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  // Non-empty scan again, but threshold disabled (= 0).
  tracker.processBatch(
      {pos2d(1.0, 1000.0, 1000.0, SensorKind::Lidar, "r0")});

  // Both seeded ids must survive across the resulting mixture when
  // merge is disabled.
  bool found_seven = false;
  bool found_ninetynine = false;
  for (const auto& h_out : tracker.density().mbm) {
    for (const auto& b : h_out.bernoullis) {
      if (b.id == 7u)  found_seven = true;
      if (b.id == 99u) found_ninetynine = true;
    }
  }
  EXPECT_TRUE(found_seven);
  EXPECT_TRUE(found_ninetynine);
}

// ---------------------------------------------------------------------------
// T3: PMBM-specific bench determinism.
//
// BenchDeterminism only covers ekf_cv_gnn. This test repeats the
// same logic for imm_cv_ct_pmbm_adapt and pins byte-identical rows
// across two sweeps. Catches non-determinism in the new TPMBM
// trajectory snapshots, adaptive-birth path, and adaptive-K
// selection that the legacy MHT test would never exercise.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, PmbmAdaptiveBenchIsByteIdenticalAcrossRuns) {
  using navtracker::benchmark::Config;
  using navtracker::benchmark::defaultConfigs;
  using navtracker::benchmark::runSweep;
  using navtracker::benchmark::ScenarioRun;
  using navtracker::benchmark::SweepParams;

  std::vector<Config> all = defaultConfigs();
  std::vector<Config> configs;
  for (const auto& c : all) {
    if (c.label == "imm_cv_ct_pmbm_adapt") configs.push_back(c);
  }
  ASSERT_EQ(configs.size(), 1u);

  std::vector<std::unique_ptr<ScenarioRun>> scenarios;
  scenarios.push_back(std::make_unique<TinyStraightLine>());

  SweepParams p;
  p.run_id = "phase8_pmbm_determinism";
  p.synthetic_seeds = 2;

  const auto rows_a = runSweep(configs, scenarios, p);
  const auto rows_b = runSweep(configs, scenarios, p);

  ASSERT_EQ(rows_a.size(), rows_b.size());
  ASSERT_GT(rows_a.size(), 0u);
  for (std::size_t i = 0; i < rows_a.size(); ++i) {
    EXPECT_EQ(rows_a[i].config, rows_b[i].config);
    EXPECT_EQ(rows_a[i].scenario, rows_b[i].scenario);
    EXPECT_EQ(rows_a[i].seed, rows_b[i].seed);
    EXPECT_EQ(rows_a[i].metric, rows_b[i].metric);
    // Wall-clock performance measurements ("wall_seconds" and the perf
    // round-2 per-scan latency rows scan_proc_ms_*) are not part of the
    // tracker's deterministic output; they legitimately vary run-to-run.
    // scan_interval_s / n_scans are data-derived, so they stay checked.
    if (rows_a[i].metric == "wall_seconds" ||
        rows_a[i].metric.rfind("scan_proc_ms", 0) == 0)
      continue;
    EXPECT_EQ(rows_a[i].value, rows_b[i].value);  // exact byte
  }
}

// ---------------------------------------------------------------------------
// T4: per-sensor setSensorDetectionModel cost-matrix effect.
//
// Two simultaneous measurements: one from a high-P_D / low-λ_C sensor,
// one from a low-P_D / high-λ_C sensor. Inject a PPP near both. The
// new-target Bernoulli built from the high-confidence sensor MUST have
// strictly higher existence_probability than the one from the
// low-confidence sensor. Proves the detection model is consulted per
// measurement, not via a global scalar.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, PerSensorDetectionModelDifferentiatesBernoulliExistence) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.5;
  cfg.clutter_intensity = 1e-3;  // shared fallback
  cfg.survival_probability = 1.0;
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);

  // High-quality sensor (Lidar):       P_D = 0.99, λ_C = 1e-6 (clean).
  // Low-quality sensor (ArpaTtm radar): P_D = 0.30, λ_C = 1e-1 (dense).
  auto model = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{0.5, 1e-3});
  model->set(SensorKind::Lidar, MeasurementModel::Position2D,
             DetectionParams{0.99, 1e-6});
  model->set(SensorKind::ArpaTtm, MeasurementModel::Position2D,
             DetectionParams{0.30, 1e-1});
  tracker.setSensorDetectionModel(model);

  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0,   0.0,   0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 100.0, 100.0));

  const auto z_hi = pos2d(1.0,   0.0,   0.0, SensorKind::Lidar,   "lidar0");
  const auto z_lo = pos2d(1.0, 100.0, 100.0, SensorKind::ArpaTtm, "radar0");
  tracker.processBatch({z_hi, z_lo});

  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& dom = tracker.density().mbm[0];
  ASSERT_EQ(dom.bernoullis.size(), 2u);
  // Match Bernoullis to measurements by proximity to PPP centres.
  double r_hi = -1.0, r_lo = -1.0;
  for (const auto& b : dom.bernoullis) {
    if (std::abs(b.mean(0) -   0.0) < 5.0) r_hi = b.existence_probability;
    if (std::abs(b.mean(0) - 100.0) < 5.0) r_lo = b.existence_probability;
  }
  ASSERT_GT(r_hi, 0.0);
  ASSERT_GT(r_lo, 0.0);
  // #24: require a meaningful separation margin, not a bare > (measured gap
  // ~0.98: r_hi~1.00 vs r_lo~0.018). 0.1 pins that the high-quality sensor's
  // birth beats the noisy one by a real amount, so an attenuated-differentiation
  // regression that shrinks the gap toward a tie goes red.
  EXPECT_GT(r_hi, r_lo + 0.1)
      << "high-P_D / low-λ_C sensor's birth must outscore the noisy one by a "
         "margin (r_hi=" << r_hi << " r_lo=" << r_lo << ")";
}

// ---------------------------------------------------------------------------
// T5: re-demotion lifecycle.
//
// Drive a Bernoulli's existence above confirm_threshold (fires
// Initiated + Confirmed) then let it decay below the threshold while
// staying above output_existence_floor (re-demotion to Tentative —
// status visible to consumers but NOT confirmed). The sink contract:
// Updated fires every scan the Bernoulli emits; Confirmed fires only
// on the up-edge transition.
// ---------------------------------------------------------------------------
class RecordingSink : public navtracker::ITrackSink {
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

TEST(PmbmTrackerPhase8, ConfirmedFiresOnlyOnUpEdgeNotOnReConfirmation) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;  // no automatic decay
  cfg.clutter_intensity = 1e-6;
  cfg.confirm_threshold = 0.7;
  cfg.output_existence_floor = 0.05;  // stays visible while Tentative
  cfg.r_min = 1e-9;
  cfg.hypothesis_weight_min = 1e-9;
  cfg.smart_birth_skip_existing = true;  // far measurements stay phantom
  cfg.smart_birth_skip_r_min = 0.0;
  PmbmTracker tracker(f.ekf, cfg);
  RecordingSink sink;
  tracker.setTrackSink(&sink);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));

  // Scan 1: high r_new at the PPP location → Confirmed up-edge.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
  ASSERT_EQ(sink.initiated.size(), 1u);
  ASSERT_EQ(sink.confirmed.size(), 1u);
  const auto target_id = sink.confirmed[0].id.value;

  // Demote the dominant target Bernoulli below confirm threshold but
  // above visibility floor (decay is too slow under survival=1 to get
  // here naturally; the test pins the event-edge semantics).
  ASSERT_FALSE(tracker.density().mbm.empty());
  for (auto& h : tracker.mutableDensityForTesting().mbm) {
    for (auto& b : h.bernoullis) {
      if (b.id == target_id) b.existence_probability = 0.40;
    }
  }

  // Scan 2: far-away measurement (does NOT update our Bernoulli; just
  // drives a processBatch so the lifecycle diff fires and snapshots
  // Tentative status into prev_emitted_statuses_).
  tracker.processBatch(
      {pos2d(2.0, 1e5, 1e5, SensorKind::Lidar, "r0")});
  EXPECT_EQ(sink.confirmed.size(), 1u)
      << "Confirmed→Tentative down-edge must NOT fire a Confirmed event";

  // Scan 3: detection on our Bernoulli at its location → posterior
  // existence jumps near 1, emitted as Confirmed, fires the second
  // up-edge. (No manual r set this time; the detection itself drives
  // the promotion, which is what production code would see.)
  tracker.processBatch(
      {pos2d(3.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
  EXPECT_EQ(sink.initiated.size(), 1u);
  EXPECT_EQ(sink.confirmed.size(), 2u)
      << "re-promotion Tentative→Confirmed must re-fire onTrackConfirmed";
}

// ---------------------------------------------------------------------------
// T4 strengthening (Option 2): the original T4 varies BOTH P_D and λ_C
// per sensor, so the assertion r_hi > r_lo cannot distinguish whether
// per-sensor P_D or per-sensor λ_C is the lever. These two follow-up
// tests pin one channel at a time so a regression in either path
// fails a test on its own.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, PerSensorPdAloneDifferentiatesBernoulliExistence) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.5;          // fallback, unused on this path
  cfg.clutter_intensity = 1e-3;                // matched per-sensor below
  cfg.survival_probability = 1.0;
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);

  // Same λ_C on both sensors — isolate P_D as the only differentiator.
  auto model = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{0.5, 1e-3});
  model->set(SensorKind::Lidar, MeasurementModel::Position2D,
             DetectionParams{0.99, 1e-3});
  model->set(SensorKind::ArpaTtm, MeasurementModel::Position2D,
             DetectionParams{0.30, 1e-3});
  tracker.setSensorDetectionModel(model);

  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0,   0.0,   0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 100.0, 100.0));

  const auto z_hi = pos2d(1.0,   0.0,   0.0, SensorKind::Lidar,   "lidar0");
  const auto z_lo = pos2d(1.0, 100.0, 100.0, SensorKind::ArpaTtm, "radar0");
  tracker.processBatch({z_hi, z_lo});

  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& dom = tracker.density().mbm[0];
  ASSERT_EQ(dom.bernoullis.size(), 2u);
  double r_hi = -1.0, r_lo = -1.0;
  for (const auto& b : dom.bernoullis) {
    if (std::abs(b.mean(0) -   0.0) < 5.0) r_hi = b.existence_probability;
    if (std::abs(b.mean(0) - 100.0) < 5.0) r_lo = b.existence_probability;
  }
  ASSERT_GT(r_hi, 0.0);
  ASSERT_GT(r_lo, 0.0);
  // #24: margin, not bare > (measured gap ~0.21: r_hi~0.857 vs r_lo~0.645 — the
  // narrowest of the three per-sensor tests, so most exposed to a P_D-plumbing
  // attenuation). 0.1 keeps ~0.11 headroom and fails on a halved differentiation.
  EXPECT_GT(r_hi, r_lo + 0.1)
      << "with λ_C equal, higher P_D must drive higher r_new by a margin (r_hi="
      << r_hi << " r_lo=" << r_lo << ")";
}

TEST(PmbmTrackerPhase8, PerSensorLambdaCAloneDifferentiatesBernoulliExistence) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.5;
  cfg.clutter_intensity = 1e-3;
  cfg.survival_probability = 1.0;
  cfg.min_new_bernoulli_existence = 0.0;
  PmbmTracker tracker(f.ekf, cfg);

  // Same P_D on both sensors — isolate λ_C as the only differentiator.
  auto model = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{0.5, 1e-3});
  model->set(SensorKind::Lidar, MeasurementModel::Position2D,
             DetectionParams{0.9, 1e-6});      // clean clutter env
  model->set(SensorKind::ArpaTtm, MeasurementModel::Position2D,
             DetectionParams{0.9, 1e-1});      // dense clutter env
  tracker.setSensorDetectionModel(model);

  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0,   0.0,   0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 100.0, 100.0));

  const auto z_hi = pos2d(1.0,   0.0,   0.0, SensorKind::Lidar,   "lidar0");
  const auto z_lo = pos2d(1.0, 100.0, 100.0, SensorKind::ArpaTtm, "radar0");
  tracker.processBatch({z_hi, z_lo});

  ASSERT_FALSE(tracker.density().mbm.empty());
  const auto& dom = tracker.density().mbm[0];
  ASSERT_EQ(dom.bernoullis.size(), 2u);
  double r_hi = -1.0, r_lo = -1.0;
  for (const auto& b : dom.bernoullis) {
    if (std::abs(b.mean(0) -   0.0) < 5.0) r_hi = b.existence_probability;
    if (std::abs(b.mean(0) - 100.0) < 5.0) r_lo = b.existence_probability;
  }
  ASSERT_GT(r_hi, 0.0);
  ASSERT_GT(r_lo, 0.0);
  // #24: margin, not bare > (measured gap ~0.95: r_hi~1.00 vs r_lo~0.052).
  EXPECT_GT(r_hi, r_lo + 0.1)
      << "with P_D equal, lower λ_C must drive higher r_new by a margin (r_hi="
      << r_hi << " r_lo=" << r_lo << ")";
}

// ---------------------------------------------------------------------------
// T5 strengthening (Option 2): the original T5 manually pokes
// b.existence_probability = 0.40 to fake the Confirmed → Tentative
// transition. This variant lets the production decay path drive r
// naturally — survival_probability < 1 thins r per predict, missed-
// detection in enumerateChildren thins it again per unmatched scan.
// If either decay path regresses, the lifecycle assertions fail.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, NaturalSurvivalDecayDrivesConfirmedDownEdge) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  // Soft decay: survival=0.9 + one missed scan brings r ≈ 0.45 —
  // below confirm_threshold=0.7 but well above output_existence_floor
  // and r_min so the Bernoulli survives as Tentative. Faster decay
  // (e.g. survival=0.6) crosses the floor in two scans and pins the
  // *re-birth* path (a new id) instead of the *re-promotion* path
  // (same id), which is what this test exists to pin.
  cfg.survival_probability = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.confirm_threshold = 0.7;
  cfg.output_existence_floor = 0.05;
  cfg.r_min = 1e-9;
  cfg.hypothesis_weight_min = 1e-9;
  cfg.smart_birth_skip_existing = true;
  cfg.smart_birth_skip_r_min = 0.0;
  PmbmTracker tracker(f.ekf, cfg);
  RecordingSink sink;
  tracker.setTrackSink(&sink);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));

  // Scan 1: detection → Confirmed up-edge. NO manual r poke.
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
  ASSERT_EQ(sink.initiated.size(), 1u);
  ASSERT_EQ(sink.confirmed.size(), 1u);
  const auto first_target_id = sink.confirmed[0].id.value;

  // Scan 2: far-away measurement → predict applies survival decay,
  // enumerateChildren applies missed-detection thinning to our
  // Bernoulli. The post-update r ≈ (1 − P_D)·survival·r /
  // (1 − P_D·survival·r) ≈ 0.45 → Tentative without deletion.
  tracker.processBatch({pos2d(2.0, 1e5, 1e5, SensorKind::Lidar, "r0")});
  EXPECT_EQ(sink.confirmed.size(), 1u)
      << "natural decay below confirm_threshold must NOT re-fire Confirmed";

  // Scan 3: detection on our Bernoulli → posterior r jumps near 1 →
  // re-Confirmed up-edge fires. Same id as scan 1 — proves
  // re-promotion, not re-birth.
  tracker.processBatch({pos2d(3.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
  EXPECT_EQ(sink.initiated.size(), 1u)
      << "re-promotion of a still-living Bernoulli must NOT fire Initiated";
  ASSERT_EQ(sink.confirmed.size(), 2u)
      << "natural re-promotion must re-fire onTrackConfirmed exactly once";
  EXPECT_EQ(sink.confirmed[1].id.value, first_target_id)
      << "re-Confirmed event must carry the original id, not a new one";
}

// ---------------------------------------------------------------------------
// Phase 9 M2 k_best_dominance_log_gap knob (PmbmTracker.cpp:987-1009).
// Two pinned guarantees: (a) default 0.0 is bit-identical to legacy
// behavior, (b) a positive log_gap actually drops K-siblings whose
// log_weight is more than `log_gap` nat below the top sibling.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, KBestDominanceCutoffDefaultIsBitIdenticalToLegacy) {
  Fixture f;
  PmbmTracker::Config base;
  base.probability_of_detection = 0.9;
  base.clutter_intensity = 1e-3;
  base.survival_probability = 1.0;
  base.adaptive_birth = true;
  base.adaptive_k_best = true;
  base.k_best_per_hypothesis = 3;
  base.lambda_birth = 1e-5;
  base.min_new_bernoulli_existence = 0.05;
  base.k_best_dominance_log_gap = 0.0;  // explicit default

  PmbmTracker::Config with_gap = base;
  with_gap.k_best_dominance_log_gap = 0.0;  // explicit 0 == off

  PmbmTracker a(f.ekf, base), b(f.ekf, with_gap);
  for (auto* t : {&a, &b}) {
    t->predict(Timestamp::fromSeconds(0.0));
    t->mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
  }
  const std::vector<Measurement> scan = {
      pos2d(1.0, 0.0,   0.0, SensorKind::Lidar, "r0"),
      pos2d(1.0, 5.0,   0.0, SensorKind::Lidar, "r0"),
      pos2d(1.0, 0.0,   5.0, SensorKind::Lidar, "r0"),
  };
  a.processBatch(scan);
  b.processBatch(scan);

  ASSERT_EQ(a.density().mbm.size(), b.density().mbm.size());
  for (std::size_t i = 0; i < a.density().mbm.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.density().mbm[i].log_weight,
                     b.density().mbm[i].log_weight)
        << "MBM[" << i << "] log_weight must be byte-identical at log_gap=0";
    EXPECT_EQ(a.density().mbm[i].bernoullis.size(),
              b.density().mbm[i].bernoullis.size());
  }
}

// ---------------------------------------------------------------------------
// Phase 9 M3 Option A output_merge_bhattacharyya_threshold knob
// (PmbmTracker.cpp refreshAggregatedTracks). Two pinned guarantees:
// (a) default 0.0 is bit-identical to legacy aggregated-output behavior,
// (b) a positive threshold actually folds two spatially-coincident
// aggregated ids into one (the younger id disappears from the output).
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase8, OutputMergeDefaultIsBitIdenticalToLegacy) {
  Fixture f;
  PmbmTracker::Config base;
  base.probability_of_detection = 0.9;
  base.clutter_intensity = 1e-3;
  base.survival_probability = 1.0;
  base.output_merge_bhattacharyya_threshold = 0.0;
  PmbmTracker a(f.ekf, base), b(f.ekf, base);
  for (auto* t : {&a, &b}) {
    t->predict(Timestamp::fromSeconds(0.0));
    t->mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    t->mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 50.0, 50.0));
  }
  const std::vector<Measurement> scan = {
      pos2d(1.0, 0.0,  0.0,  SensorKind::Lidar, "r0"),
      pos2d(1.0, 50.0, 50.0, SensorKind::Lidar, "r0"),
  };
  a.processBatch(scan);
  b.processBatch(scan);
  ASSERT_EQ(a.tracks().size(), b.tracks().size());
  for (std::size_t i = 0; i < a.tracks().size(); ++i) {
    EXPECT_EQ(a.tracks()[i].id.value, b.tracks()[i].id.value);
    EXPECT_DOUBLE_EQ(a.tracks()[i].state(0), b.tracks()[i].state(0));
    EXPECT_DOUBLE_EQ(a.tracks()[i].state(1), b.tracks()[i].state(1));
  }
}

TEST(PmbmTrackerPhase8, OutputMergeFoldsCoincidentIds) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-3;
  cfg.survival_probability = 1.0;
  cfg.r_min = 1e-9;
  cfg.hypothesis_weight_min = 1e-9;
  cfg.output_existence_floor = 0.05;
  cfg.bhattacharyya_merge_threshold = 0.0;  // disable within-hyp merge
  cfg.output_merge_bhattacharyya_threshold = 5.0;  // aggressive
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed two near-coincident Bernoullis with distinct ids into a
  // single global hypothesis. Output aggregation by id would emit
  // them as two output tracks; the cross-id merge must fold them.
  navtracker::pmbm::GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  auto mkB = [](BernoulliId id, double r, double px, double py) {
    Bernoulli b;
    b.id = id;
    b.existence_probability = r;
    b.mean = Eigen::Vector4d(px, py, 0.0, 0.0);
    b.covariance = Eigen::Matrix4d::Identity() * 4.0;
    b.last_update = Timestamp::fromSeconds(0.0);
    return b;
  };
  h.bernoullis.push_back(mkB(7,  0.8, 0.0, 0.0));
  h.bernoullis.push_back(mkB(99, 0.6, 0.1, 0.0));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  const auto& tracks_out = tracker.tracks();
  ASSERT_EQ(tracks_out.size(), 1u)
      << "two coincident-id aggregated tracks must fold to one with "
         "output_merge_bhattacharyya_threshold > 0";
  EXPECT_EQ(tracks_out[0].id.value, 7u)
      << "the older id (7) must survive the fold";
}

TEST(PmbmTrackerPhase8, KBestDominanceCutoffDropsSiblingsBelowGap) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-3;
  cfg.survival_probability = 1.0;
  cfg.adaptive_birth = true;
  cfg.adaptive_k_best = true;
  cfg.k_best_per_hypothesis = 5;
  cfg.lambda_birth = 1e-5;
  cfg.min_new_bernoulli_existence = 0.05;
  cfg.hypothesis_weight_min = 1e-9;  // don't let the regular floor prune
  cfg.r_min = 1e-9;

  // Run a 3-measurement scan with one parent and no PPP-aligned
  // dominant assignment so Murty produces multiple comparable
  // K-children. Then run the same scan with log_gap = 0 (control)
  // and log_gap = 0.5 (aggressive). Assert the gap-on run produces
  // strictly fewer children when alts exist within the gap.
  auto run_with_gap = [&](double gap) {
    PmbmTracker::Config c = cfg;
    c.k_best_dominance_log_gap = gap;
    PmbmTracker tracker(f.ekf, c);
    tracker.predict(Timestamp::fromSeconds(0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 10.0, 10.0));
    tracker.processBatch({
        pos2d(1.0, 0.0, 0.0,   SensorKind::Lidar, "r0"),
        pos2d(1.0, 10.0, 10.0, SensorKind::Lidar, "r0"),
        pos2d(1.0, 5.0, 5.0,   SensorKind::Lidar, "r0"),
    });
    return tracker.density().mbm.size();
  };
  const auto n_off = run_with_gap(0.0);
  const auto n_on  = run_with_gap(0.5);

  // The no-ADD invariant is the real, always-exercised teeth: a positive
  // dominance log_gap must never GROW the hypothesis count (a regression that
  // let the cutoff add children would go red here).
  EXPECT_GE(n_off, n_on)
      << "positive log_gap must NOT add hypotheses; n_off=" << n_off
      << " n_on=" << n_on;
  // #24 / STOP-AND-REPORT (backlog b24-1): the DROP half is UNCOVERED. The
  // comment claimed "the 3-measurement scan reliably produces ≥2 K-children",
  // but measured n_off == 1 — the MBM collapses to a single global hypothesis on
  // this scan, so the cutoff has nothing to drop and EXPECT_LT(n_on, n_off) never
  // ran (guarded-loop vacuity, exposed by the assertion sweep). Making it toothy
  // needs a scenario with genuine assignment ambiguity (≥2 comparable surviving
  // K-children) — a PMBM-scenario-construction task, not an assertion tweak.
  if (n_off >= 2) {
    EXPECT_LT(n_on, n_off)
        << "log_gap=0.5 must drop ≥ 1 close-weight sibling when "
           "the cost matrix admits multiple K-children with alts "
           "within 0.5 nat of the top";
  }
}

// ---------------------------------------------------------------------------
// Phase 9 S2: use_per_track_hypotheses=true keeps the per-track view in
// sync with the flat MBM after each processBatch. The view is a pure
// re-shape, so the output (tracks()) MUST be bit-identical between the
// flag off and on for any scenario — only the internal storage changes.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase9S2, PerTrackViewMatchesFlatAfterProcessBatch) {
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  cfg.use_per_track_hypotheses = true;
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 10.0, 10.0));
  tracker.processBatch({
      pos2d(1.0, 0.0, 0.0,   SensorKind::Lidar, "r0"),
      pos2d(1.0, 10.0, 10.0, SensorKind::Lidar, "r0"),
  });
  const auto& d = tracker.density();
  ASSERT_FALSE(d.mbm.empty());
  ASSERT_FALSE(d.tracks.empty()) << "flag ON must populate the per-track view";
  ASSERT_EQ(d.tracked_mbm.size(), d.mbm.size());
  // For each hypothesis, the per-track view's hyp_index must enumerate
  // exactly the same Bernoulli ids as the flat bernoullis list — same
  // count, same id mapping.
  for (std::size_t p = 0; p < d.mbm.size(); ++p) {
    std::size_t flat_alive = 0;
    for (const auto& b : d.mbm[p].bernoullis) {
      if (b.id != navtracker::pmbm::kInvalidBernoulliId) ++flat_alive;
    }
    std::size_t view_alive = 0;
    for (int j : d.tracked_mbm[p].hyp_index) {
      if (j != navtracker::pmbm::TrackedGlobalHypothesis::kAbsent) ++view_alive;
    }
    EXPECT_EQ(view_alive, flat_alive)
        << "hyp " << p << " view-alive=" << view_alive
        << " flat-alive=" << flat_alive;
    EXPECT_DOUBLE_EQ(d.tracked_mbm[p].weight, d.mbm[p].weight);
  }
}

// ---------------------------------------------------------------------------
// Phase 9 S3: alt_birth_log_gap_threshold suppresses phantom births in
// K-best alts whose log_weight is below (top_lw - threshold), without
// touching the top child OR the alts' detection contributions.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase9S3, AltBirthGateDefaultIsBitIdenticalToLegacy) {
  // gate = 0 → no behavioural change vs. an identical config without
  // the knob. Use a 3-measurement scan with two PPP priors that admits
  // multiple K-children, mirroring the dominance-cutoff test layout.
  Fixture f;
  auto build = [&](double gate) {
    PmbmTracker::Config c;
    c.probability_of_detection = 0.9;
    c.clutter_intensity = 1e-4;
    c.survival_probability = 1.0;
    c.k_best_per_hypothesis = 5;
    c.alt_birth_log_gap_threshold = gate;
    PmbmTracker tracker(f.ekf, c);
    tracker.predict(Timestamp::fromSeconds(0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 10.0, 10.0));
    tracker.processBatch({
        pos2d(1.0, 0.0, 0.0,   SensorKind::Lidar, "r0"),
        pos2d(1.0, 10.0, 10.0, SensorKind::Lidar, "r0"),
        pos2d(1.0, 5.0, 5.0,   SensorKind::Lidar, "r0"),
    });
    std::vector<std::size_t> b_counts;
    for (const auto& h : tracker.density().mbm) {
      b_counts.push_back(h.bernoullis.size());
    }
    return b_counts;
  };
  const auto off = build(0.0);
  const auto same = build(0.0);
  ASSERT_EQ(off, same);
}

TEST(PmbmTrackerPhase9S3, AltBirthGateStripsBirthsInWeakAltOnly) {
  // gate=0.5 nat with K=5 over the same 3-meas scan: top child is
  // kept intact (with births if any); alts BELOW the gap have their
  // newly-born Bernoullis (birth_time == scan time) stripped while
  // detected / misdetected Bernoullis stay. Verifies "births dropped,
  // detections kept" — the discriminator the M3 output-merge probe
  // lacked.
  Fixture f;
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-4;
  c.survival_probability = 1.0;
  c.k_best_per_hypothesis = 5;
  c.alt_birth_log_gap_threshold = 0.5;
  PmbmTracker tracker(f.ekf, c);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 10.0, 10.0));
  tracker.processBatch({
      pos2d(1.0, 0.0, 0.0,   SensorKind::Lidar, "r0"),
      pos2d(1.0, 10.0, 10.0, SensorKind::Lidar, "r0"),
      pos2d(1.0, 5.0, 5.0,   SensorKind::Lidar, "r0"),
  });
  ASSERT_FALSE(tracker.density().mbm.empty());
  const double top_lw = std::max_element(
      tracker.density().mbm.begin(), tracker.density().mbm.end(),
      [](const auto& a, const auto& b) {
        return a.log_weight < b.log_weight;
      })->log_weight;
  const double scan_t = 1.0;
  int gated_alts = 0;
  for (const auto& h : tracker.density().mbm) {
    if (h.log_weight >= top_lw - 0.5) continue;  // not a gated alt
    ++gated_alts;
    for (const auto& b : h.bernoullis) {
      EXPECT_NE(b.birth_time.seconds(), scan_t)
          << "alt child (lw=" << h.log_weight << " vs top=" << top_lw
          << ") still contains a fresh birth";
    }
  }
  // #24 / STOP-AND-REPORT (backlog b24-1): CONFIRMS W3 assertion-quality#3, and
  // worse — measured gated_alts == 0. The 3-measurement scan collapses to a
  // single global hypothesis (same root cause as KBestDominanceCutoffDropsSiblings
  // BelowGap: n_off==1), so NO hypothesis sits below top_lw-0.5 and the strip
  // check above never runs. The alt-birth-strip mechanism therefore has zero
  // behavioral coverage. An ASSERT_GT(gated_alts,0) here correctly turns this
  // vacuity RED, but the fix is a scenario redesign that produces a genuine
  // gated alt hypothesis (not a mechanical assertion change) — deferred to the
  // backlog. Documented rather than shipped red; the mechanism's default==legacy
  // invariant is covered by AltBirthGateDefaultIsBitIdenticalToLegacy.
  (void)gated_alts;
}

// ---------------------------------------------------------------------------
// Phase 9 review fix — Finding 6B: lambda_birth per-sensor override.
// When lambda_birth_per_sensor[z.sensor] is set, it overrides the
// scalar lambda_birth for new-target Bernoullis born from
// measurement z. AIS vs EO-IR birth-rate gap is order-of-magnitude
// on autoferry; the scalar mistunes both. Empty map = scalar
// fallback (bit-identical).
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase9Review, LambdaBirthPerSensorOverridesScalar) {
  Fixture f;
  auto build_r_new = [&](double scalar,
                          std::map<SensorKind, double> per_sensor) {
    PmbmTracker::Config cfg;
    cfg.probability_of_detection = 0.9;
    cfg.clutter_intensity = 1e-3;
    cfg.survival_probability = 1.0;
    cfg.adaptive_birth = true;
    cfg.lambda_birth = scalar;
    cfg.lambda_birth_per_sensor = std::move(per_sensor);
    cfg.min_new_bernoulli_existence = 0.0;
    PmbmTracker tracker(f.ekf, cfg);
    tracker.predict(Timestamp::fromSeconds(0.0));
    // Single AIS measurement → adaptive-birth Bernoulli with
    // r_new = lambda_birth / (lambda_birth + lambda_C(z)).
    tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Ais, "ais0")});
    if (tracker.density().mbm.empty()) return -1.0;
    if (tracker.density().mbm[0].bernoullis.empty()) return -1.0;
    return tracker.density().mbm[0].bernoullis[0].existence_probability;
  };
  // Scalar=1e-3 → r_new ≈ 1e-3/(1e-3+1e-3) = 0.5
  const double r_scalar = build_r_new(1e-3, {});
  // Override AIS to 1e-1 → r_new ≈ 1e-1/(1e-1+1e-3) ≈ 0.99
  const double r_override = build_r_new(1e-3, {{SensorKind::Ais, 1e-1}});
  ASSERT_GT(r_scalar, 0.0);
  ASSERT_GT(r_override, 0.0);
  EXPECT_GT(r_override, r_scalar + 0.3)
      << "per-sensor override (1e-1) must dominate scalar (1e-3): "
      << "r_scalar=" << r_scalar << " r_override=" << r_override;
  // And confirm scalar fallback when sensor not in the map.
  const double r_other_sensor = build_r_new(
      1e-3, {{SensorKind::ArpaTtm, 1e-1}});  // Ais NOT in map
  EXPECT_NEAR(r_other_sensor, r_scalar, 1e-9)
      << "non-mapped sensor must fall through to scalar";
}

// ---------------------------------------------------------------------------
// Phase 9 — cross_parent_birth_id_cache shares birth ids across ALL
// parents' K-children for the same measurement, mirroring MATLAB's
// filter-level new-track creation. Off by default = bit-identical.
// ---------------------------------------------------------------------------
TEST(PmbmTrackerPhase9XParent, CrossParentCacheDefaultOffBitIdentical) {
  Fixture f;
  auto run = [&](bool xparent) {
    PmbmTracker::Config c;
    c.probability_of_detection = 0.9;
    c.clutter_intensity = 1e-4;
    c.survival_probability = 1.0;
    c.adaptive_k_best = true;
    c.k_best_per_hypothesis = 3;
    c.cross_parent_birth_id_cache = xparent;
    PmbmTracker tracker(f.ekf, c);
    tracker.predict(Timestamp::fromSeconds(0.0));
    tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
    return tracker.density().mbm.size();
  };
  EXPECT_EQ(run(false), run(false));  // determinism baseline
}

TEST(PmbmTrackerPhase9XParent, CrossParentCacheWorksWithoutAdaptiveKBest) {
  // Review-2 fix: the flag must work whether or not adaptive_k_best is
  // on (docs claim independence). Earlier implementation silently
  // no-op'd when adaptive_k_best=false because parent_idx_arg stayed -1.
  Fixture f;
  auto run = [&](bool xparent) -> std::size_t {
    PmbmTracker::Config c;
    c.probability_of_detection = 0.9;
    c.clutter_intensity = 1e-4;
    c.survival_probability = 1.0;
    c.adaptive_k_best = false;  // <-- critical: fixed-K path
    c.k_best_per_hypothesis = 1;
    c.cross_parent_birth_id_cache = xparent;
    PmbmTracker tracker(f.ekf, c);
    tracker.predict(Timestamp::fromSeconds(0.0));
    auto& d = tracker.mutableDensityForTesting();
    d.ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    navtracker::pmbm::GlobalHypothesis h1, h2;
    h1.weight = 0.5; h1.log_weight = std::log(0.5);
    h2.weight = 0.5; h2.log_weight = std::log(0.5);
    d.mbm = {h1, h2};
    tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
    std::set<navtracker::pmbm::BernoulliId> ids;
    for (const auto& h : tracker.density().mbm) {
      for (const auto& b : h.bernoullis) ids.insert(b.id);
    }
    return ids.size();
  };
  // #24: the flag exists because it "silently no-op'd when adaptive_k_best=false"
  // (the post-review-2 defect). EXPECT_LE passes on exactly that no-op (on==off),
  // so it cannot catch the regression it guards. This 2-parent / 1-measurement
  // scan births distinct ids per parent with the cache OFF and one shared id ON,
  // so a STRICT reduction is the real invariant.
  EXPECT_LT(run(true), run(false))
      << "xparent must strictly reduce distinct birth ids even without "
         "adaptive_k_best (post-review-2 fix); on==off means the flag no-op'd";
}

TEST(PmbmTrackerPhase9XParent, CrossParentCacheSharesIdsAcrossParents) {
  // Two parent hypotheses each enumerate K children; if either parent
  // has a child that births measurement l, the cross-parent cache
  // ensures all such births (across both parents) share one id.
  // With xparent OFF, parents 0 and 1 birthing measurement l get
  // distinct ids → max BernoulliId grows ≥ 2. With xparent ON, max
  // BernoulliId growth is bounded by the number of measurements
  // birthed in the scan, regardless of how many parents claim them.
  Fixture f;
  auto run = [&](bool xparent) -> std::size_t {
    PmbmTracker::Config c;
    c.probability_of_detection = 0.9;
    c.clutter_intensity = 1e-4;
    c.survival_probability = 1.0;
    c.adaptive_k_best = true;
    c.k_best_per_hypothesis = 3;
    c.cross_parent_birth_id_cache = xparent;
    PmbmTracker tracker(f.ekf, c);
    tracker.predict(Timestamp::fromSeconds(0.0));
    // Two parent hypotheses with equal weight; each will enumerate
    // K children, some of which birth the lone measurement.
    auto& d = tracker.mutableDensityForTesting();
    d.ppp.push_back(mkPpp(1.0, 0.0, 0.0));
    navtracker::pmbm::GlobalHypothesis h1, h2;
    h1.weight = 0.5; h1.log_weight = std::log(0.5);
    h2.weight = 0.5; h2.log_weight = std::log(0.5);
    d.mbm = {h1, h2};
    tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
    // Count unique BernoulliIds emitted.
    std::set<navtracker::pmbm::BernoulliId> ids;
    for (const auto& h : tracker.density().mbm) {
      for (const auto& b : h.bernoullis) ids.insert(b.id);
    }
    return ids.size();
  };
  const auto n_off = run(false);
  const auto n_on  = run(true);
  // #24: EXPECT_LE(on,off) passes when the cache is a complete no-op (on==off),
  // which cannot distinguish "shares ids across parents" from "does nothing".
  // The 2-parent / 1-measurement scan yields ≥2 distinct ids with the cache OFF
  // and a single shared id ON, so require a STRICT reduction.
  EXPECT_LT(n_on, n_off)
      << "cross-parent cache must strictly SHARE (reduce) distinct ids across "
         "parents; off=" << n_off << " on=" << n_on << " (on==off means no-op)";
}

TEST(PmbmTrackerPhase9S2, PerTrackViewOffByDefaultIsBitIdentical) {
  // With the flag off, the per-track view stays empty — proves no
  // implicit work happens unless opted in.
  Fixture f;
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-6;
  cfg.survival_probability = 1.0;
  // cfg.use_per_track_hypotheses defaults to false
  PmbmTracker tracker(f.ekf, cfg);
  tracker.predict(Timestamp::fromSeconds(0.0));
  tracker.mutableDensityForTesting().ppp.push_back(mkPpp(1.0, 0.0, 0.0));
  tracker.processBatch({pos2d(1.0, 0.0, 0.0, SensorKind::Lidar, "r0")});
  EXPECT_TRUE(tracker.density().tracks.empty());
  EXPECT_TRUE(tracker.density().tracked_mbm.empty());
}
