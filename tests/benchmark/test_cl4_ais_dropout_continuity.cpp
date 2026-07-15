// Q(b) — F2 provenance cycle (docs/superpowers/plans/2026-07-12-f2-provenance-
// cycle-ticket.md). ADR-0002 presence guard on the DEPLOYED config.
//
// In sim_ms_ais_dropout the crossing vessel (MMSI 257000401) loses AIS from
// t=200 s to t=380 s but stays radar-detectable throughout; the head-on vessel
// (257000402) keeps AIS. ADR-0002 ("presence over classification") requires the
// dropout vessel to remain tracked on radar with identity retained — it must
// NOT vanish into nothing while another sensor still sees it.
//
// This is measured on the shipped Cl-4 config imm_cv_ct_pmbm_coverage_land_ivgate,
// which sets idle_halflife_sec=0 and source_aware_misdetection=false. On that
// config the F2 source-touch fix touches only the (inert-here) AIS-ARPA bias
// loop, so it is byte-identical fix ON vs OFF (verified: the PMBM per-scan diag
// stream is md5-identical across the two builds).
//
// The F2 cycle weighed the risk that a genuine AIS dropout could decay existence
// via idle_halflife. That specific risk turned out DOUBLY moot: idle=0 on the
// deployed config, and — teeth-proven below — even forcing idle_halflife_sec=3 s
// leaves the dropout lifetime at 0.993, because idle-decay fires on TOTAL
// misdetection, not partial sensor loss: radar keeps re-detecting the vessel
// through the AIS gap, so the track never goes idle. The real lever that breaks
// this presence property is source_aware_misdetection: forcing it true on this
// config drops the dropout vessel below the floor (RED) — the other vessel's AIS
// touch marks the region "covered" and the radar-only track is wrongly decayed.
// That is exactly why the deployed config keeps the source-aware gate OFF, and it
// is what this permanent guard defends: re-enabling the miss gate (or any birth/
// confirmation/gating regression that loses the radar-only track) here trips it.
//
// The thresholds are ADR-0002 presence floors, not pinned baselines: measured
// lifetime_ratio is 0.993 (dropout) / 0.992 (steady) with 0 id-switches; the
// 180 s dropout window is ~30% of the ~600 s run, so a full-window track loss
// would drop lifetime to ~0.70 — well under the 0.85 floor asserted here.
// (Teeth: source_aware_misdetection=true on this config -> RED; deployed
// default off -> GREEN. Verified 2026-07-15.)

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"
#include "tests/support/FixtureGuard.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
// Returns the metric value, or a negative sentinel when the row is absent.
double metricVal(const std::vector<MetricRow>& rows, const std::string& cfg,
                 const std::string& scen, const std::string& metric) {
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == scen && r.metric == metric)
      return r.value;
  return -1.0;
}
}  // namespace

TEST(Cl4AisDropoutContinuity, DropoutVesselSurvivesOnRadarWithIdentityRetained) {
  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimMultisensorScenarios())
    if (s->descriptor().label == "sim_ms_ais_dropout") scen.push_back(std::move(s));
  ASSERT_FALSE(scen.empty());
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      scen.front()->generate(0).measurements.empty(),
      "sim_ms_ais_dropout fixtures absent (set SIMMS_DIR + generate)");

  const auto all = defaultConfigs();
  const Config* keep = nullptr;
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm_coverage_land_ivgate") keep = &c;
  ASSERT_NE(keep, nullptr) << "deployed Cl-4 config missing from defaultConfigs()";

  SweepParams params;
  params.run_id = "cl4_ais_dropout_continuity";
  params.synthetic_seeds = 1;
  std::vector<Config> configs = {*keep};
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const std::string cfg = "imm_cv_ct_pmbm_coverage_land_ivgate";
  const std::string sc = "sim_ms_ais_dropout";
  const double life_dropout =
      metricVal(rows, cfg, sc, "lifetime_ratio:truth_257000401");
  const double life_steady =
      metricVal(rows, cfg, sc, "lifetime_ratio:truth_257000402");
  const double ids_dropout =
      metricVal(rows, cfg, sc, "id_switches:truth_257000401");

  ASSERT_GE(life_dropout, 0.0)
      << "per-truth lifetime row missing for the dropout vessel (257000401)";

  // ADR-0002 presence: the dropout vessel is tracked through the AIS-loss window
  // on radar (must not be suppressed into nothing while radar-detectable).
  EXPECT_GT(life_dropout, 0.85)
      << "dropout vessel lost through the AIS-dropout window — ADR-0002 presence "
         "violation (measured 0.993)";
  EXPECT_GT(life_steady, 0.85) << "steady (always-AIS) vessel not tracked";

  // R11: identity retained across the dropout and the AIS re-acquire (no churn).
  EXPECT_LE(ids_dropout, 1.0)
      << "identity churned on the dropout vessel across the window (measured 0)";
}
