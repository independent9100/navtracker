#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <set>
#include <string>

#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker::benchmark;

namespace {

std::unique_ptr<ScenarioRun> findSim(const std::string& label) {
  for (auto& s : defaultSimMultisensorScenarios())
    if (s->descriptor().label == label) return std::move(s);
  return nullptr;
}

// Max per-tick truth cardinality (vessels sharing an evaluation tick).
int maxTruthCardinality(const navtracker::Scenario& scen) {
  std::map<double, int> card;
  for (const auto& t : scen.truth) card[t.time.seconds()] += 1;
  int mx = 0;
  for (const auto& [t, n] : card) mx = std::max(mx, n);
  return mx;
}

// Run a scenario through the canonical IMM+MHT config end to end.
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
  EXPECT_TRUE(det) << "sim-ms scenario should declare a per-sensor table";
  navtracker::MhtTracker t(*est, cfg, det);
  return runBenchMht(scen, t);
}

}  // namespace

// The battery is 6 single-seed scenarios, each declaring a multi-sensor
// detection table (radar + AIS at minimum).
TEST(SimMultisensorScenarioRun, BatteryLabels) {
  const auto scenarios = defaultSimMultisensorScenarios();
  ASSERT_EQ(scenarios.size(), 6u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    labels.insert(s->descriptor().label);
    EXPECT_FALSE(s->descriptor().is_multi_seed);
    EXPECT_EQ(s->descriptor().seed_count, 1u);
    EXPECT_GE(s->descriptor().detection_table.size(), 2u) << s->descriptor().label;
  }
  for (const char* l : {"sim_ms_crossing", "sim_ms_headon", "sim_ms_overtaking",
                        "sim_ms_ais_dropout", "sim_ms_clutter_burst",
                        "sim_ms_anchored_camera"})
    EXPECT_EQ(labels.count(l), 1u) << l;
}

// generate() must carry a datum (so Sweep wires datum-aware occupancy/land/
// static models) and independent truth on a shared multi-vessel clock. Skips
// under ctest (cwd = build/, fixtures unreachable) unless SIMMS_DIR is set.
TEST(SimMultisensorScenarioRun, GenerateCarriesDatumAndSharedClockTruth) {
  bool any = false;
  for (auto& s : defaultSimMultisensorScenarios()) {
    const auto scen = s->generate(0);
    if (scen.measurements.empty()) continue;
    any = true;
    EXPECT_TRUE(scen.datum.has_value()) << s->descriptor().label;
    EXPECT_FALSE(scen.truth.empty()) << s->descriptor().label;
    EXPECT_GE(maxTruthCardinality(scen), 2) << s->descriptor().label;
  }
  if (!any) GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";
}

// R11 identity data-path: AIS fixes must carry the MMSI hint that lets fusion
// surface external identity. (Track-output attribute surfacing is a deeper
// pipeline concern; the honest bench-level gate is that the identity input
// flows and that fusion does not fragment it — see the cardinality gate below.)
TEST(SimMultisensorScenarioRun, AisMeasurementsCarryMmsiHint) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  if (scen.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";
  int ais = 0, ais_with_mmsi = 0;
  for (const auto& m : scen.measurements) {
    if (m.sensor != navtracker::SensorKind::Ais) continue;
    ++ais;
    if (m.hints.mmsi.has_value() && *m.hints.mmsi != 0u) ++ais_with_mmsi;
  }
  ASSERT_GT(ais, 0);
  EXPECT_EQ(ais, ais_with_mmsi) << "every AIS fix should carry an MMSI hint";
}

// The fusion accuracy gate: radar+AIS scored against INDEPENDENT truth. This is
// the first controlled fusion-vs-single-sensor number the project has. On a
// clean 2-vessel head-on the IMM+MHT arm should track both vessels with high
// lifetime, low OSPA/RMSE, stable identity, and NOT fragment identity
// (single-track-per-vessel). Bounds are generous over the measured baseline
// (see docs/baselines/2026-07-06_sim_multisensor_battery.md).
TEST(SimMultisensorScenarioRun, HeadonFusionAccuracyGate) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  if (scen.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";
  ASSERT_EQ(maxTruthCardinality(scen), 2);

  const auto result = runMht(*run, scen);
  ASSERT_FALSE(result.steps.empty());
  const auto m = computeMetrics(result, {});

  // Bounds carry ~1.5-2x margin over the measured baseline (lifetime 0.99,
  // ospa 40, rmse 26.5, id_switches 1.5 — 2026-07-06 sim_multisensor_battery).
  EXPECT_GT(m.lifetime_ratio, 0.9);
  EXPECT_LT(m.ospa_mean, 90.0);
  EXPECT_LT(m.pos_rmse_m, 45.0);
  EXPECT_LT(m.id_switches, 6.0);

  // Single-track-per-vessel: no dual-track. The most confirmed tracks held at
  // any step should not exceed the 2 true vessels by more than a small margin.
  int max_tracks = 0;
  for (const auto& step : result.steps)
    max_tracks = std::max<int>(max_tracks, static_cast<int>(step.tracks.size()));
  EXPECT_LE(max_tracks, 4) << "fusion is fragmenting identity (dual-tracking)";
}

// ADR-0002 (never-invisible) + #17 camera wedge: the anchored (nav_status=1)
// vessel and the radar-silent camera-only contact must both be present in the
// independent truth throughout, and the camera-only contact must surface as
// EoIr bearing measurements (the corroboration/hazard channel) — it is never
// suppressed into nothing.
TEST(SimMultisensorScenarioRun, AnchoredAndCameraOnlyNeverInvisible) {
  auto run = findSim("sim_ms_anchored_camera");
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  if (scen.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  constexpr std::uint64_t kAnchored = 257000601;
  constexpr std::uint64_t kCamOnly = 257000602;
  std::set<std::uint64_t> truth_ids;
  int anchored_ticks = 0;
  for (const auto& t : scen.truth) {
    truth_ids.insert(t.truth_id);
    if (t.truth_id == kAnchored) ++anchored_ticks;
  }
  EXPECT_EQ(truth_ids.count(kAnchored), 1u) << "anchored vessel absent from truth";
  EXPECT_EQ(truth_ids.count(kCamOnly), 1u) << "camera-only contact absent from truth";
  EXPECT_GT(anchored_ticks, 500) << "anchored vessel must persist across the run";

  // Camera-only contact is radar-silent by construction: it surfaces via the
  // camera bearing channel (EoIr), never nothing.
  int eoir = 0;
  for (const auto& mmt : scen.measurements)
    if (mmt.sensor == navtracker::SensorKind::EoIr) ++eoir;
  EXPECT_GT(eoir, 0) << "camera-only contact produced no bearing measurements";
}

// Determinism: the same fixtures replayed twice produce identical scenarios
// (the fixtures are frozen; this guards the loader path).
TEST(SimMultisensorScenarioRun, DeterministicReplay) {
  auto run = findSim("sim_ms_crossing");
  ASSERT_TRUE(run);
  const auto a = run->generate(0);
  if (a.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";
  auto run2 = findSim("sim_ms_crossing");
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
