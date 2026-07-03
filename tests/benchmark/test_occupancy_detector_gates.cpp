// Stage 1b-ii DETECTOR bench gates (presence over classification, ADR 0002
// amendment 2026-07-03). Three integration gates for imm_cv_ct_pmbm_occupancy_
// detector, each on COMPLETE synthetic truth (so lifetime/cardinality are
// honestly scored, ungameable):
//
//   * Increment 4 — death-spiral guard. dense_clutter_datum wires the live
//     layer on dense UNIFORM clutter; the clutter-adaptive bar must reject it
//     (no false hazards, no suppressed real births) so the two real targets
//     track as under the land baseline.
//   * Increment 5 — presence over classification (the three-way M2 split).
//     (1) PRESENCE hard gate: every anchored radar-only boat is a track OR an
//         emitted static hazard — never neither.
//     (2) MOVERS keep their lifetime gate: a moving vessel represented as a
//         hazard is a rule-3 violation, not degraded mode.
//     (3) classification quality is REPORTED (KEEP-as-hazard fraction), not
//         gated — it is how corroboration (increment 6) will be measured.
//   * Increment 7 — recovery (rule-3). A boat anchored long enough to be
//     suppressed-as-hazard gets underway and must recover to a confirmed
//     moving track (lifetime over the moving phase), no permanent static pin.
//
// The model-level invariants (conservation by construction, bounded-latency
// decay, adaptive-bar rejection) are unit-tested in tests/static/
// test_live_occupancy_model.cpp; these gates are the end-to-end confirmation.
#include <gtest/gtest.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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

const Config* byLabel(const std::vector<Config>& all, const std::string& label) {
  for (const auto& c : all)
    if (c.label == label) return &c;
  return nullptr;
}

std::vector<std::unique_ptr<ScenarioRun>> onlyScenario(const std::string& label) {
  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == label) scen.push_back(std::move(s));
  return scen;
}

const char* kLand = "imm_cv_ct_pmbm_land";
const char* kDetector = "imm_cv_ct_pmbm_occupancy_detector";

}  // namespace

// Increment 4: the layer wired on dense uniform clutter must NOT death-spiral.
// The clutter-adaptive bar rejects the transient clutter cells (each revisited
// at a sub-structure rate), so the detector classifies ~no structure and the
// two real crossing targets survive exactly as under the land baseline.
TEST(OccupancyDetectorGates, DenseClutterDatumNoDeathSpiral) {
  const auto all = defaultConfigs();
  const Config* land = byLabel(all, kLand);
  const Config* det = byLabel(all, kDetector);
  ASSERT_NE(land, nullptr);
  ASSERT_NE(det, nullptr);
  auto scen = onlyScenario("dense_clutter_datum");
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "occ_det_gate_dc";
  params.synthetic_seeds = 8;
  const std::vector<Config> cfgs = {*land, *det};
  const auto rows = runSweep(cfgs, scen, params);
  const char* S = "dense_clutter_datum";

  const double life_land = meanMetric(rows, kLand, S, "lifetime_ratio");
  const double life_det = meanMetric(rows, kDetector, S, "lifetime_ratio");
  const double gospa_land = meanMetric(rows, kLand, S, "gospa_mean");
  const double gospa_det = meanMetric(rows, kDetector, S, "gospa_mean");
  const double structs = meanMetric(rows, kDetector, S, "occ_peak_structures");
  const double hits = meanMetric(rows, kDetector, S, "occ_suppress_hits");
  std::cout << "\n=== dense_clutter_datum death-spiral guard ===\n"
            << "  land    : lifetime=" << life_land << " gospa=" << gospa_land << "\n"
            << "  DETECTOR: lifetime=" << life_det << " gospa=" << gospa_det
            << "  structures=" << structs << " suppress_hits=" << hits << "\n"
            << std::flush;
  // No death spiral: lifetime is not degraded vs the no-layer baseline, and
  // false/GOSPA does not blow up. (The λ_C spike regressed this 0.90→0.26.)
  EXPECT_GE(life_det, life_land - 0.05);
  EXPECT_LE(gospa_det, gospa_land * 1.15 + 1.0);
}

// Increment 5: presence over classification on the honest yardstick.
TEST(OccupancyDetectorGates, PresenceOverClassificationOnHarbor) {
  const auto all = defaultConfigs();
  const Config* land = byLabel(all, kLand);
  const Config* det = byLabel(all, kDetector);
  ASSERT_NE(land, nullptr);
  ASSERT_NE(det, nullptr);
  auto scen = onlyScenario("harbor_complete_truth");
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "occ_det_gate_harbor";
  params.synthetic_seeds = 8;
  const std::vector<Config> cfgs = {*land, *det};
  const auto rows = runSweep(cfgs, scen, params);
  const char* S = "harbor_complete_truth";

  auto life = [&](const char* cfg, std::uint64_t id) {
    return meanMetric(rows, cfg, S, "lifetime_ratio:truth_" + std::to_string(id));
  };
  auto haz = [&](std::uint64_t id) {
    return meanMetric(rows, kDetector, S,
                      "occ_truth_in_hazard:truth_" + std::to_string(id));
  };

  std::cout << "\n=== presence over classification (harbor_complete_truth) ===\n";
  std::cout << "  movers (must stay TRACKS):\n";
  for (std::uint64_t id : {1u, 2u})
    std::cout << "    truth_" << id << ": land_life=" << life(kLand, id)
              << " det_life=" << life(kDetector, id) << " in_hazard=" << haz(id)
              << "\n";
  std::cout << "  anchored boats (track OR hazard, never neither):\n";
  double keep_as_hazard = 0.0;
  for (std::uint64_t id : {3u, 4u, 5u}) {
    std::cout << "    truth_" << id << ": land_life=" << life(kLand, id)
              << " det_life=" << life(kDetector, id) << " in_hazard=" << haz(id)
              << "\n";
    keep_as_hazard += haz(id);
  }
  std::cout << "  [reported, not gated] KEEP-as-hazard fraction = "
            << (keep_as_hazard / 3.0) << "\n"
            << std::flush;

  // (1) PRESENCE hard gate: each anchored radar-only boat is predominantly a
  // track OR predominantly an emitted hazard — never vanished into nothing.
  for (std::uint64_t id : {3u, 4u, 5u}) {
    const double l = life(kDetector, id);
    const double h = haz(id);
    EXPECT_TRUE(l > 0.5 || h > 0.5)
        << "anchored boat truth_" << id << " is NEITHER a track (life=" << l
        << ") NOR a hazard (in_hazard=" << h << ") — presence violated";
  }
  // (2) MOVERS keep their lifetime gate (hard) and are never emitted as a
  // hazard (a moving vessel as static hazard is a rule-3 violation).
  for (std::uint64_t id : {1u, 2u}) {
    EXPECT_GE(life(kDetector, id), life(kLand, id) - 0.05)
        << "mover truth_" << id << " lost track lifetime under the detector";
    EXPECT_LT(haz(id), 0.5)
        << "mover truth_" << id << " was represented as a static hazard";
  }
}

// Increment 7: static→moving recovery (rule-3). The stop→go boat gets underway
// at t=11 and must recover to a confirmed moving track — a substantial
// lifetime that can only come from the moving phase (the anchored phase is at
// most 10/40 scans and is where the boat is suppressed-as-hazard). A permanent
// static pin would leave lifetime near zero.
TEST(OccupancyDetectorGates, AnchoredGetsUnderwayRecovers) {
  const auto all = defaultConfigs();
  const Config* land = byLabel(all, kLand);
  const Config* det = byLabel(all, kDetector);
  ASSERT_NE(land, nullptr);
  ASSERT_NE(det, nullptr);
  auto scen = onlyScenario("harbor_anchored_gets_underway");
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "occ_det_gate_recovery";
  params.synthetic_seeds = 8;
  const std::vector<Config> cfgs = {*land, *det};
  const auto rows = runSweep(cfgs, scen, params);
  const char* S = "harbor_anchored_gets_underway";

  const double life_land = meanMetric(rows, kLand, S, "lifetime_ratio:truth_6");
  const double life_det = meanMetric(rows, kDetector, S, "lifetime_ratio:truth_6");
  const double haz6 =
      meanMetric(rows, kDetector, S, "occ_truth_in_hazard:truth_6");
  const double structs = meanMetric(rows, kDetector, S, "occ_peak_structures");
  const double hits = meanMetric(rows, kDetector, S, "occ_suppress_hits");
  std::cout << "\n=== static->moving recovery (harbor_anchored_gets_underway) ===\n"
            << "  land    : truth_6 lifetime=" << life_land << "\n"
            << "  DETECTOR: truth_6 lifetime=" << life_det
            << " final_in_hazard=" << haz6 << " structures=" << structs
            << " suppress_hits=" << hits << "\n"
            << std::flush;
  // Recovered to a moving track: lifetime substantially above zero (a permanent
  // pin would be ~0). 0.4 can only be met by tracking through the moving phase.
  EXPECT_GE(life_det, 0.4)
      << "stop->go boat did not recover to a moving track (permanent static pin?)";
}
