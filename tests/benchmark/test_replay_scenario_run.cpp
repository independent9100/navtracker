#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include <set>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker::benchmark;

TEST(ReplayScenarioRun, ReplaysWithExpectedLabels) {
  const auto scenarios = defaultReplayScenarios();
  ASSERT_EQ(scenarios.size(), 3u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    labels.insert(s->descriptor().label);
    EXPECT_FALSE(s->descriptor().is_multi_seed);
    EXPECT_EQ(s->descriptor().seed_count, 1u);
  }
  EXPECT_EQ(labels.count("philos"), 1u);
  EXPECT_EQ(labels.count("philos_radartruth"), 1u);
  EXPECT_EQ(labels.count("haxr"), 1u);
}

TEST(ReplayScenarioRun, GenerateReturnsNonEmpty) {
  // generate() returns an empty Scenario when the fixture CSVs aren't
  // reachable from cwd (under ctest, cwd is build/). Skip in that case;
  // the assertion below pins behaviour when fixtures are present (the
  // bench harness driver runs from project root and won't see the skip).
  bool any_real = false;
  for (auto& s : defaultReplayScenarios()) {
    const auto data = s->generate(0);
    if (data.measurements.empty()) continue;  // fixture unreachable from cwd
    any_real = true;
    // #24: the old EXPECT_FALSE(empty) was dead code — the guard above already
    // established non-empty. The regression that actually slips through is a
    // fixture whose CSV is present but loads to a PARTIAL/truncated set (still
    // "non-empty", so silently accepted). A real replay clip carries hundreds of
    // measurements, so a substantial floor traps a truncated/1-row load.
    EXPECT_GT(data.measurements.size(), 50u)
        << "replay " << s->descriptor().label << " loaded only "
        << data.measurements.size() << " measurements — partial/truncated load";
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any_real, "replay fixtures not reachable from cwd");
}

// The Stage-1b occupancy / land / static-obstacle models wire in the bench
// Sweep ONLY when scen.datum.has_value() (Sweep.cpp). HAXR is a local metre
// frame with no geodetic origin, so HaxrScenarioRun must supply a NOMINAL fixed
// anchor datum — otherwise imm_cv_ct_pmbm_occupancy is silently bit-identical to
// its base on HAXR (the occupancy layer never runs) and the increment-8 A/B is a
// no-op. Guard that the HAXR scenario carries a datum whenever its fixtures are
// reachable. (Skips under ctest cwd=build/, like the other replay tests.)
TEST(ReplayScenarioRun, HaxrScenarioCarriesDatumSoOccupancyWires) {
  std::unique_ptr<ScenarioRun> haxr;
  for (auto& s : defaultReplayScenarios())
    if (s->descriptor().label == "haxr") haxr = std::move(s);
  ASSERT_TRUE(haxr);
  const auto scen = haxr->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(scen.measurements.empty(),
                                     "HAXR fixtures not reachable from cwd");
  EXPECT_TRUE(scen.datum.has_value())
      << "HAXR scenario has no datum — the Stage-1b occupancy layer will not "
         "wire, making the occupancy A/B a silent no-op";
}

// End-to-end harness regression on real data: AutoFerry scenario2 carries
// two ground-truth targets whose per-target timestamps used to fragment
// every evaluation step to cardinality 1, pegging OSPA near the 500 m
// cutoff and counting thousands of phantom id switches for every config.
// Pin the fixed behaviour: 2-target steps, and a plain EKF+GNN baseline
// scoring far below the cutoff with sane identity counts.
TEST(ReplayScenarioRun, AutoferryScenario2GnnMetricsAreSane) {
  std::unique_ptr<ScenarioRun> run;
  for (auto& s : defaultAutoferryScenarios()) {
    if (s->descriptor().label == "autoferry_scenario2") run = std::move(s);
  }
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      scen.measurements.empty(), "AutoFerry scenario2 not reachable from cwd");

  const auto configs = defaultConfigs();
  const Config* gnn = nullptr;
  for (const auto& c : configs)
    if (c.label == "ekf_cv_gnn") gnn = &c;
  ASSERT_NE(gnn, nullptr);

  auto est = gnn->build_estimator();
  auto asc = gnn->build_associator();
  navtracker::TrackManager mgr(/*min_misses=*/2, /*max_misses=*/4);
  navtracker::Tracker tracker(*est, *asc, mgr, /*init_gate_m=*/30.0);
  BenchSink sink;
  const auto result = runBench(scen, tracker, mgr, sink);

  ASSERT_FALSE(result.steps.empty());
  // Every step carries both ground-truth targets.
  for (const auto& step : result.steps) {
    ASSERT_EQ(step.truth.size(), 2u);
  }
  // NB: ospa_mean is deliberately NOT pinned here. On this clutter-heavy
  // harbour data a GNN tracker with no existence model genuinely confirms
  // ~20 false tracks per step, so its OSPA sits high for a real reason —
  // that is the tracker-comparison signal the bench exists to expose,
  // not a harness property. The harness-regression signals are the
  // 2-target step cardinality above and the identity metrics below
  // (the bug produced ~3.2e3 phantom switches).
  const auto m = computeMetrics(result, {});
  EXPECT_LT(m.id_switches, 50.0) << "phantom id switches are back";
  EXPECT_GT(m.lifetime_ratio, 0.9);
  EXPECT_LT(m.track_breaks, 10.0);
  EXPECT_LT(m.pos_rmse_m, 30.0);
}

// Companion pin for the canonical IMM+TOMHT config. Before the
// multi-sensor miss model (per-sensor coverage-conditioned P_D, dt-scaled
// lifecycle), the per-scan global miss penalty deleted tracks 0.4 s after
// their last hit: scenario2 showed ~600 track breaks and lifetime ~0.8.
TEST(ReplayScenarioRun, AutoferryScenario2MhtLifecycleIsSane) {
  std::unique_ptr<ScenarioRun> run;
  for (auto& s : defaultAutoferryScenarios()) {
    if (s->descriptor().label == "autoferry_scenario2") run = std::move(s);
  }
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      scen.measurements.empty(), "AutoFerry scenario2 not reachable from cwd");

  const auto configs = defaultConfigs();
  const Config* mht = nullptr;
  for (const auto& c : configs)
    if (c.label == "imm_cv_ct_mht") mht = &c;
  ASSERT_NE(mht, nullptr);

  auto est = mht->build_estimator();
  navtracker::MhtTracker::Config cfg = mht->mht_config();
  auto det = detectionModelFor(run->descriptor(), cfg);
  ASSERT_TRUE(det) << "autoferry should declare a per-sensor table";
  navtracker::MhtTracker tracker(*est, cfg, det);
  const auto result = runBenchMht(scen, tracker);

  ASSERT_FALSE(result.steps.empty());
  // Regression pins with ~2x margin over the canonical (IPDA+VIMM
  // lifecycle + time-based cross-tree duplicate merge, 2026-06-11)
  // measurement: lifetime 0.954, breaks 1.5, switches 39.5. History:
  // M-of-N post-multi-sensor-fix measured 0.77 / 64.5 / 146.5;
  // pre-fix ~600 breaks; pre-merge VIMM had 59 switches.
  const auto m = computeMetrics(result, {});
  EXPECT_GT(m.lifetime_ratio, 0.9);
  EXPECT_LT(m.track_breaks, 10.0);
  EXPECT_LT(m.id_switches, 80.0);
}

// Backlog item 4: EO and IR cameras share SensorKind::EoIr but have very
// different measured detection performance (aggregate across the nine
// ground-truthed scenarios: EO P_D 0.73, IR 0.46). Every autoferry
// descriptor must declare source-keyed camera entries with IR strictly
// below EO, plus the kind-wide fallback for unknown camera sources.
//
// λ_C is pinned UNIFORM across the camera entries on purpose: the
// measured per-environment unmatched-bearing rate (urban channel up to
// ~5 rad⁻¹) is persistent structured shoreline returns, not uniform
// Poisson clutter, and feeding it into the uniform-λ score collapsed
// urban lifetime (see 2026-06-11 evaluation-log entry). Until the
// spatial clutter map (backlog item 5) exists, a camera-λ split is a
// modelling error, and this pin is the regression guard for it.
TEST(ReplayScenarioRun, AutoferryDeclaresSplitEoIrDetectionEntries) {
  for (const auto& s : defaultAutoferryScenarios()) {
    const auto d = s->descriptor();
    const SensorDetectionEntry* eo = nullptr;
    const SensorDetectionEntry* ir = nullptr;
    const SensorDetectionEntry* kind_wide = nullptr;
    for (const auto& e : d.detection_table) {
      if (e.sensor != navtracker::SensorKind::EoIr) continue;
      if (e.source_id == "autoferry_eo") eo = &e;
      if (e.source_id == "autoferry_ir") ir = &e;
      if (e.source_id.empty()) kind_wide = &e;
    }
    ASSERT_NE(eo, nullptr) << d.label;
    ASSERT_NE(ir, nullptr) << d.label;
    ASSERT_NE(kind_wide, nullptr) << d.label;
    EXPECT_GT(eo->params.probability_of_detection,
              ir->params.probability_of_detection)
        << d.label;
    EXPECT_DOUBLE_EQ(eo->params.clutter_intensity,
                     kind_wide->params.clutter_intensity)
        << d.label;
    EXPECT_DOUBLE_EQ(ir->params.clutter_intensity,
                     kind_wide->params.clutter_intensity)
        << d.label;
  }
}

// Backlog item 7: philos truth is AIS-as-truth — asynchronous per-vessel
// messages with no scan structure. Before truth resampling, BenchRunner's
// exact-time bucketing fragmented every evaluation step to cardinality 1
// and ALL MHT configs scored lifetime <= 0.015 (phantom misses dominated
// every metric). The descriptor now declares a calibrated per-sensor
// table (radar 0.07/2.7e-6 per sub-scan event; AIS as a broadcast, not a
// sweep) and generate() resamples truth onto a shared 1 Hz clock.
TEST(ReplayScenarioRun, PhilosResampledTruthAndMhtLifecycle) {
  std::unique_ptr<ScenarioRun> run;
  for (auto& s : defaultReplayScenarios()) {
    if (s->descriptor().label == "philos") run = std::move(s);
  }
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(scen.measurements.empty(),
                                     "philos fixtures not reachable from cwd");

  // Resampled truth: shared-clock steps with real multi-vessel
  // cardinality (the fixture carries ~23 AIS vessels over ~20 s).
  std::map<double, int> card;
  for (const auto& t : scen.truth) card[t.time.seconds()] += 1;
  ASSERT_GE(card.size(), 15u);
  int max_card = 0;
  for (const auto& [t, n] : card) max_card = std::max(max_card, n);
  EXPECT_GE(max_card, 10);

  const auto configs = defaultConfigs();
  const Config* mht = nullptr;
  for (const auto& c : configs)
    if (c.label == "imm_cv_ct_mht") mht = &c;
  ASSERT_NE(mht, nullptr);

  auto est = mht->build_estimator();
  navtracker::MhtTracker::Config cfg = mht->mht_config();
  auto det = detectionModelFor(run->descriptor(), cfg);
  ASSERT_TRUE(det) << "philos should declare a per-sensor table";
  navtracker::MhtTracker tracker(*est, cfg, det);
  const auto result = runBenchMht(scen, tracker);
  ASSERT_FALSE(result.steps.empty());

  // Regression pins with margin over the 2026-06-11 measurement
  // (lifetime 0.295, breaks 0.04, switches 0.17, ospa 430, rmse 38).
  // Pre-fix every MHT config scored lifetime <= 0.015 with OSPA pegged
  // at the 500 m cutoff. The remaining lifetime ceiling is honest: the
  // fixture is a ~20 s snippet in which most vessels report AIS only
  // twice ~10 s apart, so confirmation at the second fix already costs
  // half of such a vessel's presence window.
  const auto m = computeMetrics(result, {});
  EXPECT_GT(m.lifetime_ratio, 0.2);
  EXPECT_LT(m.track_breaks, 2.0);
  EXPECT_LT(m.id_switches, 5.0);
  EXPECT_LT(m.ospa_mean, 470.0);
  EXPECT_LT(m.pos_rmse_m, 60.0);
}
