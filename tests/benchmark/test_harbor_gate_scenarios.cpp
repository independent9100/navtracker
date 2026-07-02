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

TEST(HarborGateScenarios, DeterministicForSameSeed) {
  for (const std::string label :
       {"harbor_large_anchored_ship", "harbor_compact_dolphin"}) {
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
  const Config* base = nullptr;
  for (const auto& c : defaultConfigs())
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
