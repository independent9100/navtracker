#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include <cstdio>
#include <memory>
#include <set>
#include <string>

#include "adapters/benchmark/RbadScenarioRun.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker::benchmark;

namespace {

std::unique_ptr<ScenarioRun> findRbad(const std::string& label) {
  for (auto& s : defaultRbadScenarios())
    if (s->descriptor().label == label) return std::move(s);
  return nullptr;
}

// Run a scenario through the canonical IMM+MHT default config end to end.
navtracker::benchmark::BenchResult runMht(ScenarioRun& run,
                                          const navtracker::Scenario& scen) {
  const auto configs = defaultConfigs();
  const Config* mht = nullptr;
  for (const auto& c : configs)
    if (c.label == "imm_cv_ct_mht") mht = &c;
  EXPECT_NE(mht, nullptr);
  auto est = mht->build_estimator();
  navtracker::MhtTracker::Config cfg = mht->mht_config();
  auto det = detectionModelFor(run.descriptor(), cfg);
  navtracker::MhtTracker t(*est, cfg, det);
  return runBenchMht(scen, t);
}

int distinctOurTracks(const BenchResult& r) {
  std::set<std::uint64_t> ids;
  for (const auto& step : r.steps)
    for (const auto& trk : step.tracks) ids.insert(trk.id.value);
  return static_cast<int>(ids.size());
}

int distinctReferenceIds(const navtracker::Scenario& s) {
  std::set<std::uint64_t> ids;
  for (const auto& t : s.truth) ids.insert(t.truth_id);
  return static_cast<int>(ids.size());
}

}  // namespace

// The battery is 6 single-seed arrival approaches across 2 ports, each
// declaring a radar detection table.
TEST(RbadScenarioRun, BatteryLabels) {
  const auto scenarios = defaultRbadScenarios();
  ASSERT_EQ(scenarios.size(), 6u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    labels.insert(s->descriptor().label);
    EXPECT_FALSE(s->descriptor().is_multi_seed);
    EXPECT_EQ(s->descriptor().seed_count, 1u);
    EXPECT_GE(s->descriptor().detection_table.size(), 1u) << s->descriptor().label;
  }
  for (const char* l : {"rbad_kalimnos_16", "rbad_kalimnos_17", "rbad_kalimnos_3",
                        "rbad_kos_11", "rbad_kos_16", "rbad_kos_5"})
    EXPECT_EQ(labels.count(l), 1u) << l;
}

// generate() must carry a nominal datum (so Sweep wires datum-aware models) and
// the reference-tracker trajectories as "truth"; plots live in a fixed body
// frame within short berthing range of the origin (no ego pose). Skips under
// ctest (cwd = build/, fixtures unreachable) unless RBAD_DIR is set.
TEST(RbadScenarioRun, GenerateCarriesDatumAndReferenceTracks) {
  bool any = false;
  for (auto& s : defaultRbadScenarios()) {
    const auto scen = s->generate(0);
    if (scen.measurements.empty()) continue;
    any = true;
    EXPECT_TRUE(scen.datum.has_value()) << s->descriptor().label;
    EXPECT_FALSE(scen.truth.empty()) << s->descriptor().label;
    // Fixed body frame: every plot is within the short berthing range guarded
    // by the extractor (<= 200 m), around the origin station.
    for (const auto& m : scen.measurements)
      EXPECT_LT(m.value.norm(), 200.0) << s->descriptor().label;
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any, "rbad fixtures unreachable (set RBAD_DIR)");
}

// CROSS-TRACKER CONSISTENCY report — NOT an accuracy gate. The "truth" here is
// the authors' own reference tracker (Tracking_ID), not independent ground
// truth, so localization/RMSE are meaningless (both trackers sit on the same
// centroids) and are deliberately NOT reported. We surface only the continuity/
// cardinality CONSISTENCY numbers vs_reference_tracker and assert only that the
// pipeline produces tracks and completes (a reality-check arm, not a tuning
// target). Nothing is asserted on any consistency number or on Dock_Label
// (its distribution is reported in each scenario's meta.txt).
TEST(RbadScenarioRun, MhtConsistencyReportVsReferenceTracker) {
  int total_our_tracks = 0;
  bool any = false;
  std::printf(
      "\n[R-BAD vs_reference_tracker | mmWave FMCW, NOT marine X-band | "
      "consistency, NOT accuracy]\n");
  std::printf("%-18s %8s %8s %10s %10s %10s %10s\n", "scenario", "ref_ids",
              "our_trks", "lifetime", "id_sw", "breaks", "card_err");
  for (auto& s : defaultRbadScenarios()) {
    const auto scen = s->generate(0);
    if (scen.measurements.empty()) continue;
    any = true;
    const auto result = runMht(*s, scen);
    const auto m = computeMetrics(result, {});
    const int our = distinctOurTracks(result);
    total_our_tracks += our;
    std::printf("%-18s %8d %8d %10.3f %10.2f %10.2f %10.2f\n",
                s->descriptor().label.c_str(), distinctReferenceIds(scen), our,
                m.lifetime_ratio, m.id_switches, m.track_breaks, m.card_err_mean);
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any, "rbad fixtures unreachable (set RBAD_DIR)");
  EXPECT_GT(total_our_tracks, 0)
      << "tracker produced no confirmed tracks on any R-BAD approach";
}

// Determinism: the same frozen fixtures replayed twice produce an identical
// scenario (guards the loader path). Pipeline-level determinism is covered by
// the core engine's determinism guarantee (test_bench_determinism).
TEST(RbadScenarioRun, DeterministicReplay) {
  auto run = findRbad("rbad_kos_5");
  ASSERT_TRUE(run);
  const auto a = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      a.measurements.empty(), "rbad fixtures unreachable (set RBAD_DIR)");
  auto run2 = findRbad("rbad_kos_5");
  const auto b = run2->generate(0);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  ASSERT_EQ(a.truth.size(), b.truth.size());
  for (size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].time.seconds(), b.measurements[i].time.seconds());
    EXPECT_EQ(a.measurements[i].value, b.measurements[i].value);
  }
  for (size_t i = 0; i < a.truth.size(); ++i) {
    EXPECT_EQ(a.truth[i].truth_id, b.truth[i].truth_id);
    EXPECT_EQ(a.truth[i].position, b.truth[i].position);
    EXPECT_EQ(a.truth[i].velocity, b.truth[i].velocity);
  }
}
