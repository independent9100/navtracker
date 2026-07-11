// R3 gate scenarios (extent-is-interim, 2026-07-02). Two known-limitation
// recorders for the future Stage-1b-ii extent→corroboration discriminator:
//   harbor_large_anchored_ship — a REAL extended target that extent alone would
//     wrongly SUPPRESS (must be KEPT once 1b-ii lands).
//   harbor_compact_dolphin     — a compact fixed structure that extent alone
//     would wrongly KEEP (SUPPRESS target for 1b-ii).
// These are NOT pass/fail gates under 1b-i; this file asserts the contract
// (truth is closed + deterministic) and records the 1b-i "before" numbers.
#include <gtest/gtest.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
ScenarioRun* find(const std::vector<std::unique_ptr<ScenarioRun>>& v,
                  const std::string& label) {
  for (const auto& s : v)
    if (s->descriptor().label == label) return s.get();
  return nullptr;
}
std::set<std::uint64_t> truthIds(ScenarioRun* h) {
  std::set<std::uint64_t> ids;
  for (const auto& ts : h->generate(0).truth) ids.insert(ts.truth_id);
  return ids;
}
bool hasSource(ScenarioRun* h, const std::string& src) {
  for (const auto& m : h->generate(0).measurements)
    if (m.source_id == src) return true;
  return false;
}
double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& scen, const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == scen && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  return n ? sum / n : 0.0;
}
}  // namespace

// The large ship is a REAL target (truth id 6) with an extended hull signature.
TEST(HarborGateScenarios, LargeAnchoredShipIsRealAndExtended) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = find(scenarios, "harbor_large_anchored_ship");
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(truthIds(h),
            (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u, 6u}));  // ship = id 6
  EXPECT_TRUE(hasSource(h, "sim_large_ship"));  // extended hull returns present
}

// The compact dolphin adds NO truth (it is fixed structure, not a vessel).
TEST(HarborGateScenarios, CompactDolphinAddsNoTruth) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = find(scenarios, "harbor_compact_dolphin");
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(truthIds(h), (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u}));
  EXPECT_TRUE(hasSource(h, "sim_dolphin"));
}

// The churn variant keeps harbor_complete_truth's closed truth (ids 1-5: two
// movers + three anchored boats; pier + clutter carry no truth). Only the
// uncharted pier's per-scan detection probability drops (0.9 -> 0.4).
TEST(HarborGateScenarios, ChurnVariantHasCompleteTruthAndPier) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = find(scenarios, "harbor_complete_truth_churn");
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(truthIds(h), (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u}));
  EXPECT_TRUE(hasSource(h, "sim_pier"));
}

TEST(HarborGateScenarios, DeterministicForSameSeed) {
  for (const std::string label :
       {"harbor_large_anchored_ship", "harbor_compact_dolphin",
        "harbor_complete_truth_churn"}) {
    const auto scenarios = defaultSimScenarios();
    ScenarioRun* h = find(scenarios, label);
    ASSERT_NE(h, nullptr) << label;
    auto a = h->generate(0);
    auto b = h->generate(0);
    ASSERT_EQ(a.measurements.size(), b.measurements.size()) << label;
    for (std::size_t i = 0; i < a.measurements.size(); ++i)
      EXPECT_EQ(a.measurements[i].value, b.measurements[i].value) << label;
  }
}

// Record the 1b-i "before" numbers (imm_cv_ct_pmbm). NOT a gate — 1b-i has no
// extent discriminator, so these numbers are the baseline the future 1b-ii pass
// must improve (ship KEPT; dolphin phantom SUPPRESSED).
TEST(HarborGateScenarios, RecordOneBiBaseline) {
  const auto all_configs = defaultConfigs();  // hold — `base` aliases it
  const Config* base = nullptr;
  for (const auto& c : all_configs)
    if (c.label == "imm_cv_ct_pmbm") base = &c;
  ASSERT_NE(base, nullptr);

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios()) {
    const std::string l = s->descriptor().label;
    if (l == "harbor_large_anchored_ship" || l == "harbor_compact_dolphin")
      scen.push_back(std::move(s));
  }
  ASSERT_EQ(scen.size(), 2u);

  SweepParams params;
  params.run_id = "harbor_gate_1bi_baseline";
  params.synthetic_seeds = 5;
  const auto rows = runSweep({*base}, scen, params);
  std::cout << "\n=== R3 gate scenarios — 1b-i baseline (imm_cv_ct_pmbm) ===\n";
  for (const std::string l :
       {"harbor_large_anchored_ship", "harbor_compact_dolphin"})
    std::cout << "  " << l
              << ": card_err=" << meanMetric(rows, "imm_cv_ct_pmbm", l, "card_err_mean")
              << " gospa_false=" << meanMetric(rows, "imm_cv_ct_pmbm", l, "gospa_false")
              << " lifetime=" << meanMetric(rows, "imm_cv_ct_pmbm", l, "lifetime_ratio")
              << "\n";
  std::cout << std::flush;
  SUCCEED();  // recorder, not a gate
}

// The PERMANENT suppression gate (2026-07-03). harbor_complete_truth at P_D 0.9
// is structurally unpassable by a birth-channel suppressor (the pier cohort
// confirms in scans 1-2 and never dies -> suppress_hits ~ 0). The churn variant
// (P_D 0.4) is where phantoms decay + re-birth, so birth suppression is
// MEASURABLE at all — this is where any future suppression mechanism must be
// evaluated. Gate properties, on complete truth so they cannot be gamed:
//   (1) the mechanism FIRES here (occ_suppress_hits > 0) — the churn variant's
//       reason to exist, vs ~0 on the P_D-0.9 yardstick;
//   (2) it does NOT harm the three anchored boats (lifetime not reduced) —
//       the ADR-0002 safety invariant;
//   (3) it does not increase false mass (gospa_false not worse).
// Uses imm_cv_ct_pmbm_occupancy_sensitive: the default 25 m/bar-0.5 classifier
// does not fire at P_D 0.4 (bar > per-cell P_D), so the gate tracks the tuning
// that is actually measurable — see eval-log 2026-07-03.
TEST(HarborGateScenarios, ChurnSuppressionGateBaseline) {
  const auto all_configs = defaultConfigs();  // hold — pointers below alias it
  const Config* land = nullptr;
  const Config* occ = nullptr;
  for (const auto& c : all_configs) {
    if (c.label == "imm_cv_ct_pmbm_land") land = &c;
    if (c.label == "imm_cv_ct_pmbm_occupancy_sensitive") occ = &c;
  }
  ASSERT_NE(land, nullptr);
  ASSERT_NE(occ, nullptr);

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_complete_truth_churn")
      scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "harbor_churn_suppression_gate";
  params.synthetic_seeds = 8;
  const std::vector<Config> gate_configs = {*land, *occ};
  const auto rows = runSweep(gate_configs, scen, params);
  const char* S = "harbor_complete_truth_churn";
  const double life_land = meanMetric(rows, "imm_cv_ct_pmbm_land", S, "lifetime_ratio");
  const double life_occ = meanMetric(rows, "imm_cv_ct_pmbm_occupancy_sensitive", S, "lifetime_ratio");
  const double gf_land = meanMetric(rows, "imm_cv_ct_pmbm_land", S, "gospa_false");
  const double gf_occ = meanMetric(rows, "imm_cv_ct_pmbm_occupancy_sensitive", S, "gospa_false");
  const double hits = meanMetric(rows, "imm_cv_ct_pmbm_occupancy_sensitive", S, "occ_suppress_hits");
  std::cout << "\n=== churn suppression gate (P_D 0.4, complete truth) ===\n"
            << "  land   : gospa_false=" << gf_land << " lifetime=" << life_land << "\n"
            << "  +occ   : gospa_false=" << gf_occ << " lifetime=" << life_occ
            << " suppress_hits=" << hits << "\n"
            << std::flush;
  // #24: (1) banded floor, not >0 — measured ~28 suppress_hits on churn; a floor
  // of 10 catches a partial collapse of the mechanism, not only total death.
  EXPECT_GT(hits, 10.0)
      << "occupancy suppression barely fired on churn (measured ~28): " << hits;
  // (2) boats preserved (ADR 0002): the old 1e-9 slack was FP-equality on an
  // adaptive lifetime ratio; use a real no-regression band (both arms ~0.975, a
  // 1-of-N boat drop is ~0.16, so 0.05 sits well below a real drop).
  EXPECT_GE(life_occ, life_land - 0.05)
      << "occupancy suppression dropped a boat vs land: occ=" << life_occ
      << " land=" << life_land;
  // (3) false mass not worse: the old +1e-9 on an O(1e3) quantity demanded bit
  // equality; use a proportional no-worse margin (measured occ ~1614 ≤ land
  // ~1705, so suppression already helps; 2% guards against a real increase).
  EXPECT_LE(gf_occ, gf_land * 1.02)
      << "occupancy suppression increased false mass vs land: occ=" << gf_occ
      << " land=" << gf_land;
}
