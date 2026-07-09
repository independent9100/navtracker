// Corroboration-veto isolation A/B on the sim anchored-vessel scenario
// (2026-07-09 ticket, controlled arm). HAXR has no truth; sim_ms_anchored_camera
// carries the generator's INDEPENDENT ground truth (consumed by neither sensor
// arm), so the veto's protective effect — does holding an AIS/anchored vessel
// track-eligible keep it from being suppressed as structure? — is directly
// attributable here. AIS is present in BOTH arms (it is the scenario's own feed);
// the ONLY difference is `corroboration_veto_enabled`.
//
// Measurement test: prints the truth-scored ON/OFF deltas (the verdict numbers go
// in docs/baselines/2026-07-09_veto_isolation.md + the eval-log). The veto's
// conservation invariant is proven fixed-input in
// LiveOccupancyModel.CorroborationVetoToggleDefaultOnReproducesVetoOffFallsThrough.
// The asserts here are well-formedness only (both arms scored the truth) — #24:
// no cross-config pin on a marginal delta whose sign is the verdict itself.
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {
namespace {

double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  return n ? sum / n : 0.0;
}

const Config* byLabel(const std::vector<Config>& all, const std::string& l) {
  for (const auto& c : all)
    if (c.label == l) return &c;
  return nullptr;
}

}  // namespace

TEST(VetoIsolationSimAB, VetoProtectiveEffectOnAnchoredVesselWithPerfectTruth) {
  auto scenarios = defaultSimMultisensorScenarios();
  ScenarioRun* anchored = nullptr;
  for (auto& s : scenarios)
    if (s->descriptor().label == "sim_ms_anchored_camera") anchored = s.get();
  ASSERT_NE(anchored, nullptr);
  if (anchored->generate(0).measurements.empty())
    GTEST_SKIP()
        << "sim_ms_anchored_camera fixtures absent (set SIMMS_DIR + generate)";

  const auto all = defaultConfigs();
  const Config* occ = byLabel(all, "imm_cv_ct_pmbm_occupancy_detector");
  ASSERT_NE(occ, nullptr);
  ASSERT_TRUE(occ->live_occupancy_params.has_value());
  ASSERT_TRUE(occ->live_occupancy_params->corroboration_veto_enabled)
      << "the shipped config must have the veto ON (the default under test)";

  Config veto_on = *occ;
  veto_on.label = "veto_on";
  Config veto_off = *occ;
  veto_off.label = "veto_off";
  LiveOccupancyParams off_params = *veto_off.live_occupancy_params;
  off_params.corroboration_veto_enabled = false;  // the ONLY difference
  veto_off.live_occupancy_params = off_params;

  // Rebuild the scenario list (generate() consumed above) and keep the anchored one.
  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimMultisensorScenarios())
    if (s->descriptor().label == "sim_ms_anchored_camera")
      scen.push_back(std::move(s));
  ASSERT_FALSE(scen.empty());

  SweepParams params;
  params.run_id = "veto_iso_sim_anchored";
  params.synthetic_seeds = 1;
  std::vector<Config> configs = {veto_off, veto_on};
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  // Truth-scored aggregate metrics: with perfect truth, a PROTECTIVE veto shows
  // as fewer missed truth / higher lifetime and lower suppression in ON vs OFF.
  // The anchored (stationary, structure-like) vessel is the one at risk of being
  // suppressed as structure, so it dominates any missed-delta.
  const char* METRICS[] = {"gospa_mean",     "gospa_missed",  "gospa_false",
                           "card_err_mean",  "lifetime_ratio", "id_switches",
                           "occ_peak_structures", "occ_suppress_hits"};
  std::cout << "\n=== veto-isolation SIM A/B — sim_ms_anchored_camera (perfect "
               "truth; AIS in BOTH arms; veto toggled) ===\n"
            << "  metric                 veto_OFF        veto_ON       delta(ON-OFF)\n";
  for (const char* m : METRICS) {
    const double off = meanMetric(rows, "veto_off", m);
    const double on = meanMetric(rows, "veto_on", m);
    std::cout << "  " << m;
    for (std::size_t k = std::string(m).size(); k < 21; ++k) std::cout << ' ';
    std::cout << off << "\t" << on << "\t" << (on - off) << "\n";
  }
  std::cout << std::flush;

  // Well-formedness only (both arms scored the truth); the protective delta's
  // SIGN is the verdict and is reported, not frozen.
  EXPECT_GT(meanMetric(rows, "veto_off", "gospa_mean"), 0.0);
  EXPECT_GT(meanMetric(rows, "veto_on", "gospa_mean"), 0.0);
}

}  // namespace benchmark
}  // namespace navtracker
