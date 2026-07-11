#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
// Mean of a metric over seeds for (config, scenario).
double meanMetric(const std::vector<MetricRow>& rows, const std::string& config,
                  const std::string& scenario, const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows) {
    if (r.config == config && r.scenario == scenario && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  }
  return n == 0 ? 0.0 : sum / n;
}

std::vector<std::unique_ptr<ScenarioRun>> shoreScenariosOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultSimScenarios()) {
    const std::string label = s->descriptor().label;
    if (label == "shore_clutter_open" || label == "shore_clutter_nearshore")
      out.push_back(std::move(s));
  }
  return out;
}
}  // namespace

// R5: Stage-1a charted-obstacle A/B. On harbor_charted_pier (same measurements
// and truth as harbor_complete_truth, but the pier is charted via
// syntheticObstacles), imm_cv_ct_pmbm_static must hard-drop the pier phantom
// births — card_err and gospa_false collapse — while the three anchored boats
// (>= 650 m from the pier) keep their lifetime. Closes the "Stage 1a: no
// measurement" gap.
TEST(SyntheticClutterAB, ChartedPierSuppressesPierKeepsBoats) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs())
    if (c.label == "imm_cv_ct_pmbm" || c.label == "imm_cv_ct_pmbm_static")
      configs.push_back(c);
  ASSERT_EQ(configs.size(), 2u);

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_charted_pier") scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "charted_pier_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const char* base = "imm_cv_ct_pmbm";
  const char* stat = "imm_cv_ct_pmbm_static";
  const std::string sc = "harbor_charted_pier";
  const double card_base = meanMetric(rows, base, sc, "card_err_mean");
  const double card_stat = meanMetric(rows, stat, sc, "card_err_mean");
  const double false_base = meanMetric(rows, base, sc, "gospa_false");
  const double false_stat = meanMetric(rows, stat, sc, "gospa_false");
  const double life_stat = meanMetric(rows, stat, sc, "lifetime_ratio");
  std::cout << "\n=== R5 charted-pier A/B ===\n"
            << "  card_err:    base=" << card_base << "  static=" << card_stat << "\n"
            << "  gospa_false: base=" << false_base << "  static=" << false_stat << "\n"
            << "  lifetime(static)=" << life_stat << "\n" << std::flush;
  // Charting the pier removes phantom-track over-count. #24: a bare `<` between
  // two adaptive aggregates flips under drift; require a margin sized to the
  // measured collapse (card_err base ~11.6 → static ~7.4, gap ~4.2; gospa_false
  // base ~2362 → static ~1518, gap ~844).
  EXPECT_LT(card_stat, card_base - 2.0)
      << "charted pier did not materially reduce card_err over-count: static="
      << card_stat << " base=" << card_base;
  EXPECT_LT(false_stat, false_base - 300.0)
      << "charted pier did not materially reduce false mass: static=" << false_stat
      << " base=" << false_base;
  // Real targets (the anchored boats + movers) are still tracked well.
  EXPECT_GT(life_stat, 0.9);
}

// Land-aware PDA pool end-to-end validation. On shore_clutter_transit a vessel
// establishes offshore then transits into a near-shore dock-clutter field. The
// dock returns are unclaimed, gated to the vessel, and coastline-flagged. Plain
// imm_cv_ct_pmbm_land_pda pools them and is pulled ashore; the land-aware pool
// (imm_cv_ct_pmbm_land_pda_wateronly) excludes them and holds the track nearer
// truth. Births are land-suppressed identically under both, so the only A/B
// difference is the softening pool.
TEST(SyntheticClutterAB, LandAwarePoolResistsDockClutterPull) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs())
    if (c.label == "imm_cv_ct_pmbm_land_pda" ||
        c.label == "imm_cv_ct_pmbm_land_pda_wateronly")
      configs.push_back(c);
  ASSERT_EQ(configs.size(), 2u);

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "shore_clutter_transit")
      scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "landaware_transit_ab";
  params.synthetic_seeds = 8;
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const std::string sc = "shore_clutter_transit";
  const double rmse_plain =
      meanMetric(rows, "imm_cv_ct_pmbm_land_pda", sc, "pos_rmse_m");
  const double rmse_water =
      meanMetric(rows, "imm_cv_ct_pmbm_land_pda_wateronly", sc, "pos_rmse_m");
  const double life_plain =
      meanMetric(rows, "imm_cv_ct_pmbm_land_pda", sc, "lifetime_ratio");
  const double life_water =
      meanMetric(rows, "imm_cv_ct_pmbm_land_pda_wateronly", sc, "lifetime_ratio");
  std::cout << "\n=== land-aware transit A/B ===\n"
            << "  pos_rmse_m:    plain=" << rmse_plain
            << "  wateronly=" << rmse_water << "\n"
            << "  lifetime_ratio: plain=" << life_plain
            << "  wateronly=" << life_water << "\n"
            << std::flush;
  // Plain PDA pools the unclaimed quay returns and is dragged ashore (pos_rmse
  // roughly doubles); the land-aware pool excludes them and holds the track on
  // truth (near the ~8 m measurement-limited tracking error). Margin 2 m is far
  // inside the measured ~8 m gap (paired: 10/10 seeds better, mean +8.4 m).
  EXPECT_LT(rmse_water, rmse_plain - 2.0);
  EXPECT_LT(rmse_water, 12.0);
  // The fix costs no track lifetime. #24: the old 1e-9 slack was FP-equality on
  // an adaptive ratio (both arms measured 1.0); use a real no-regression band.
  EXPECT_GE(life_water, life_plain - 0.05)
      << "land-aware pool cost track lifetime: water=" << life_water
      << " plain=" << life_plain;
}

TEST(SyntheticClutterAB, LandModelRemovesShoreOverCountKeepsRealTargets) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs()) {
    if (c.label == "imm_cv_ct_pmbm_coverage" ||
        c.label == "imm_cv_ct_pmbm_coverage_land")
      configs.push_back(c);
  }
  ASSERT_EQ(configs.size(), 2u);

  SweepParams params;
  params.run_id = "synthetic_clutter_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, shoreScenariosOnly(), params);

  for (const std::string scenario :
       {"shore_clutter_open", "shore_clutter_nearshore"}) {
    const double card_off = meanMetric(rows, "imm_cv_ct_pmbm_coverage",
                                       scenario, "card_err_mean");
    const double card_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "card_err_mean");
    // Land ON must reduce the (positive) over-count toward 0.
    EXPECT_LT(card_on, card_off) << scenario;
    EXPECT_LE(card_on, 1.0) << scenario;
    // Real targets are not dropped: lifetime_ratio stays healthy with land ON.
    const double life_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "lifetime_ratio");
    EXPECT_GT(life_on, 0.5) << scenario;
  }
}
