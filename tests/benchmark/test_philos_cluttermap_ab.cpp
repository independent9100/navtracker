// Stage 1b spike measurement: does the live clutter-map feed reduce the philos
// PMBM over-count? A/B on the real philos replay:
//   A = imm_cv_ct_pmbm_land (recommended coastal baseline)
//   B = A + use_clutter_map + feed_clutter_map (clutter map live under PMBM)
// Prints every metric for both configs. Not a pass/fail gate — a measurement.
#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& scen, const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == scen && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

std::vector<std::unique_ptr<ScenarioRun>> philosOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultReplayScenarios())
    if (s->descriptor().label == "philos") out.push_back(std::move(s));
  return out;
}

// philos (real over-count) + a few synthetics (regression: uniform clutter,
// dense multi-target, clean crossing, near-shore clutter).
std::vector<std::unique_ptr<ScenarioRun>> abScenarios() {
  const std::set<std::string> sims = {"dense_clutter", "parallel_lanes_dense",
                                      "crossing_90", "shore_clutter_nearshore"};
  auto out = philosOnly();
  for (auto& s : defaultSimScenarios())
    if (sims.count(s->descriptor().label)) out.push_back(std::move(s));
  return out;
}
}  // namespace

// R2 characterization (synthetic, no philos fixture needed).
//
// The plan doc + north-star hypothesised that the clutter-map feed's
// nearest-neighbour-at-timestamp reconstruction was the ROOT of the
// dense_clutter regression (lifetime 0.90 → 0.26). This A/B DISPROVES that: with
// the true-assignment labeling fix in place, dense_clutter is still ~0.26 —
// byte-for-byte the same as the buggy NN path. The regression is the `1 − r`
// co-located death spiral on UNIFORM clutter (a low-r real target's own returns
// raise λ_C at the target; eval-log 2026-07-01), which is orthogonal to WHICH
// Bernoulli claims a return. The real cure is the Stage 1b-ii persistence /
// spatial-concentration gate — uniform clutter never crosses that bar.
//
// This test locks in the finding: base healthy, feed still spirals. When Stage
// 1b-ii lands and restores dense_clutter, THIS TEST SHOULD FAIL — update it then
// to assert restoration (that failure is the reminder, not a regression).
TEST(PmbmClutterFeedR2, TrueAssignmentIsOrthogonalToDenseClutterSpiral) {
  const Config* base = nullptr;
  const auto all = defaultConfigs();
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm_land") base = &c;
  ASSERT_NE(base, nullptr);

  std::vector<Config> configs;
  configs.push_back(*base);  // A: no feed
  Config b = *base;          // B: + live clutter-map feed (true-assignment label)
  b.label = "imm_cv_ct_pmbm_land_cluttermap";
  b.use_clutter_map = true;
  auto orig = base->pmbm_config;
  ASSERT_TRUE(static_cast<bool>(orig));
  b.pmbm_config = [orig]() {
    auto c = orig();
    c.feed_clutter_map = true;
    return c;
  };
  configs.push_back(std::move(b));

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "dense_clutter") scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "pmbm_r2_dense_clutter_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const double a =
      meanMetric(rows, "imm_cv_ct_pmbm_land", "dense_clutter", "lifetime_ratio");
  const double bl = meanMetric(rows, "imm_cv_ct_pmbm_land_cluttermap",
                               "dense_clutter", "lifetime_ratio");
  std::cout << "R2 dense_clutter lifetime_ratio: base=" << a << "  +feed=" << bl
            << "  (feed still spirals; cure = Stage 1b-ii persistence gate)\n";
  EXPECT_GT(a, 0.80);   // baseline is healthy (no feed)
  EXPECT_LT(bl, 0.50);  // feed still spirals — R2 labeling is orthogonal
}

TEST(PhilosClutterMapAB, ClutterFeedVsBaseline) {
  auto probe = philosOnly();
  ASSERT_EQ(probe.size(), 1u);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(probe[0]->generate(0).measurements.empty(),
                                     "philos fixtures not reachable from cwd");

  const Config* land = nullptr;
  const auto all = defaultConfigs();
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm_land") land = &c;
  ASSERT_NE(land, nullptr);

  std::vector<Config> configs;
  configs.push_back(*land);  // A: baseline
  Config b = *land;          // B: + live clutter-map feed
  b.label = "imm_cv_ct_pmbm_land_cluttermap";
  b.use_clutter_map = true;
  auto orig = land->pmbm_config;
  ASSERT_TRUE(static_cast<bool>(orig));
  b.pmbm_config = [orig]() {
    auto c = orig();
    c.feed_clutter_map = true;
    return c;
  };
  configs.push_back(std::move(b));

  SweepParams params;
  params.run_id = "philos_cluttermap_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, abScenarios(), params);
  ASSERT_FALSE(rows.empty());

  std::set<std::string> scenarios;
  for (const auto& r : rows) scenarios.insert(r.scenario);
  const char* A = "imm_cv_ct_pmbm_land";
  const char* B = "imm_cv_ct_pmbm_land_cluttermap";
  const std::vector<std::string> keys = {"card_err_mean", "gospa_mean",
                                         "gospa_false", "gospa_missed",
                                         "lifetime_ratio"};
  std::cout << "\n=== clutter-map A/B  (baseline land  vs  +cluttermap_feed) ===\n";
  for (const auto& scen : scenarios) {
    std::cout << "\n-- " << scen << " --\n";
    for (const auto& m : keys) {
      const double a = meanMetric(rows, A, scen, m);
      const double c = meanMetric(rows, B, scen, m);
      std::cout << "  " << m;
      for (std::size_t k = m.size(); k < 18; ++k) std::cout << ' ';
      std::cout << "  base=" << a << "\t+cm=" << c << "\tdelta=" << (c - a)
                << "\n";
    }
  }
  std::cout << std::flush;
}
