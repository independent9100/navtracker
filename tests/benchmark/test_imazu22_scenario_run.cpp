#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include <cstdio>
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

// COARSE tripwire, NOT a pin. Measured worst-case aggregate (mean-per-truth)
// id-switches over the 22 cases with imm_cv_ct_mht is 72.0 (imazu_17, the dense
// overtaking+crossing 3-target family) — see docs/baselines/2026-07-08_imazu22.md.
// The band is ~2x that: it catches a gross regression (e.g. the loader/geometry
// breaking) without pinning an association outcome, which the knife-edge lessons
// forbid. If a real config change moves the worst case past this, re-measure and
// move the band with a documented reason — do not tune to fit it.
constexpr double kMaxIdSwitchesPerCase = 150.0;

std::unique_ptr<ScenarioRun> findImazu(const std::string& label) {
  for (auto& s : defaultImazuScenarios())
    if (s->descriptor().label == label) return std::move(s);
  return nullptr;
}

int maxTruthCardinality(const navtracker::Scenario& scen) {
  std::map<double, int> card;
  for (const auto& t : scen.truth) card[t.time.seconds()] += 1;
  int mx = 0;
  for (const auto& [t, n] : card) mx = std::max(mx, n);
  return mx;
}

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
  EXPECT_TRUE(det) << "imazu scenario should declare a per-sensor table";
  navtracker::MhtTracker t(*est, cfg, det);
  return runBenchMht(scen, t);
}

}  // namespace

// The battery is 22 single-seed scenarios imazu_01..imazu_22, each declaring a
// RADAR+AIS detection table (exactly 2 sensors — no camera, unlike sim_ms).
TEST(Imazu22ScenarioRun, BatteryLabels) {
  const auto scenarios = defaultImazuScenarios();
  ASSERT_EQ(scenarios.size(), 22u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    const auto d = s->descriptor();
    labels.insert(d.label);
    EXPECT_FALSE(d.is_multi_seed) << d.label;
    EXPECT_EQ(d.seed_count, 1u) << d.label;
    // radar + AIS only; the Imazu family carries no camera, so a phantom EoIr
    // entry (which would model misdetections for a sensor that isn't there) must
    // be absent.
    EXPECT_EQ(d.detection_table.size(), 2u) << d.label;
    for (const auto& e : d.detection_table)
      EXPECT_NE(e.sensor, navtracker::SensorKind::EoIr) << d.label;
  }
  for (int n = 1; n <= 22; ++n) {
    char lbl[16];
    std::snprintf(lbl, sizeof(lbl), "imazu_%02d", n);
    EXPECT_EQ(labels.count(lbl), 1u) << lbl;
  }
}

// generate() must carry a datum (Sweep wires datum-aware components off it) and
// independent truth on a shared clock. Skips under ctest (cwd = build/, fixtures
// unreachable) unless SIMMS_DIR is set.
TEST(Imazu22ScenarioRun, GenerateCarriesDatumAndTruth) {
  bool any = false;
  for (auto& s : defaultImazuScenarios()) {
    const auto scen = s->generate(0);
    if (scen.measurements.empty()) continue;
    any = true;
    EXPECT_TRUE(scen.datum.has_value()) << s->descriptor().label;
    EXPECT_FALSE(scen.truth.empty()) << s->descriptor().label;
    EXPECT_GE(maxTruthCardinality(scen), 1) << s->descriptor().label;
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any, "imazu fixtures unreachable (set SIMMS_DIR)");
}

// R11 identity data-path: AIS fixes must carry the MMSI hint (a 3-target case).
TEST(Imazu22ScenarioRun, AisMeasurementsCarryMmsiHint) {
  auto run = findImazu("imazu_12");
  ASSERT_TRUE(run);
  const auto scen = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      scen.measurements.empty(), "imazu fixtures unreachable (set SIMMS_DIR)");
  int ais = 0, ais_with_mmsi = 0;
  for (const auto& m : scen.measurements) {
    if (m.sensor != navtracker::SensorKind::Ais) continue;
    ++ais;
    if (m.hints.mmsi.has_value() && *m.hints.mmsi != 0u) ++ais_with_mmsi;
  }
  ASSERT_GT(ais, 0);
  EXPECT_EQ(ais, ais_with_mmsi) << "every AIS fix should carry an MMSI hint";
}

// Determinism (sampled): the same fixtures replayed twice produce identical
// scenarios. Sampled across a single-, a two- and a three-target case.
TEST(Imazu22ScenarioRun, DeterministicReplaySampled) {
  bool any = false;
  for (const char* label : {"imazu_01", "imazu_12", "imazu_22"}) {
    auto ra = findImazu(label);
    ASSERT_TRUE(ra) << label;
    const auto a = ra->generate(0);
    if (a.measurements.empty()) continue;
    any = true;
    auto rb = findImazu(label);
    const auto b = rb->generate(0);
    ASSERT_EQ(a.measurements.size(), b.measurements.size()) << label;
    ASSERT_EQ(a.truth.size(), b.truth.size()) << label;
    for (size_t i = 0; i < a.measurements.size(); ++i) {
      EXPECT_EQ(a.measurements[i].time.seconds(),
                b.measurements[i].time.seconds()) << label;
      EXPECT_EQ(a.measurements[i].value, b.measurements[i].value) << label;
    }
    for (size_t i = 0; i < a.truth.size(); ++i) {
      EXPECT_EQ(a.truth[i].truth_id, b.truth[i].truth_id) << label;
      EXPECT_EQ(a.truth[i].position, b.truth[i].position) << label;
      EXPECT_EQ(a.truth[i].velocity, b.truth[i].velocity) << label;
    }
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any, "imazu fixtures unreachable (set SIMMS_DIR)");
}

// The headline: identity stability through the crossing geometry, as a COARSE
// per-case tripwire. Runs the default MHT arm over all 22 and asserts no case
// exceeds the band. This is a regression guard, not an accuracy pin — the Imazu
// churn itself (up to 72 switches on the dense 3-target cases) is a reported
// FINDING (backlog #11 evidence), not something this test tries to bound tightly.
TEST(Imazu22ScenarioRun, IdSwitchesCoarseBand) {
  bool any = false;
  for (auto& run : defaultImazuScenarios()) {
    const auto scen = run->generate(0);
    if (scen.measurements.empty()) continue;
    any = true;
    const auto result = runMht(*run, scen);
    ASSERT_FALSE(result.steps.empty()) << run->descriptor().label;
    const auto m = computeMetrics(result, {});
    EXPECT_LE(m.id_switches, kMaxIdSwitchesPerCase)
        << run->descriptor().label
        << " id-switches blew past the tripwire band — re-measure vs "
           "docs/baselines/2026-07-08_imazu22.md before moving the band";
  }
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !any, "imazu fixtures unreachable (set SIMMS_DIR)");
}
