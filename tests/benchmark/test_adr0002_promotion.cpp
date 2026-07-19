// W6.1 — ADR-0002 rule-3: static->moving promotion within bounded latency.
//
// ADR-0002's 2026-07-03 amendment (presence over classification) rule 3: "If an
// object represented as static starts moving, the system must promote it to a
// vessel track within bounded latency." That load-bearing half of
// presence-over-classification had NO implementing test (docs/adr/0002 line 188:
// "unverified today"). This is that test.
//
// The promotion mechanism is EMERGENT, not an explicit "promote" call (there is
// none — verified by a full-tree search; see the wave-6 write-up): a mover's
// returns leave the occupancy footprint, the vacated cells decay, and its fresh
// returns birth a normal track. So the honest assertion is end-to-end latency,
// measured then pinned as a banded bound with margin.
//
// TWO LEVELS here:
//   (1) A metric unit test on `promotion_latency` (core/benchmark) with a NON-ZERO
//       and a never-promoted case — the teeth: it proves the metric measures a
//       real latency, so the scenario gates below are not vacuous.
//   (2) Scenario gates on `harbor_anchored_gets_underway`: truth_6 sits anchored
//       at (-300, 50) for ~10 scans, then gets underway at t = 11 s at 8 m/s.
//         * imm_cv_ct_pmbm_coverage_land_ivgate — THE DEPLOYABLE. It wires no
//           occupancy model, so an open-water anchored vessel is never suppressed:
//           it is tracked as a low-SOG track and promotion is immediate. The gate
//           proves the mover is never lost/pinned and keeps a STABLE track_id
//           across stop->go (the second gap ADR-0002 flags — unasserted before).
//         * imm_cv_ct_pmbm_occupancy_detector — runs the live-occupancy layer.
//           The compact anchored boat is (correctly) kept as a vessel by the
//           extent discriminator, so it too tracks through; the diagnostics
//           confirm the occupancy layer is nonetheless active on the scene's pier.
//
// The suppress-then-rebirth latency (an object that IS suppressed as a hazard,
// then moves) is bounded <= 5 scans at the model level by
// tests/static/test_live_occupancy_model.cpp VacatedCellsRecoverWithinBoundedLatency.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/types/Ids.hpp"

using namespace navtracker::benchmark;

namespace {

constexpr char kDeployable[] = "imm_cv_ct_pmbm_coverage_land_ivgate";
constexpr char kDetector[] = "imm_cv_ct_pmbm_occupancy_detector";
constexpr char kScenario[] = "harbor_anchored_gets_underway";
constexpr std::uint64_t kMoverTruthId = 6;

double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == kScenario && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

// Worst seed — the honest input to a latency BOUND (not the mean).
double maxMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                 const std::string& metric) {
  double m = -std::numeric_limits<double>::infinity();
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == kScenario && r.metric == metric)
      m = std::max(m, r.value);
  return std::isfinite(m) ? m : 0.0;
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

std::vector<MetricRow> runScenario(const std::vector<Config>& cfgs) {
  auto scen = onlyScenario(kScenario);
  EXPECT_EQ(scen.size(), 1u);
  SweepParams params;
  params.run_id = "adr0002_promotion";
  params.synthetic_seeds = 8;
  return runSweep(cfgs, scen, params);
}

// Build a synthetic BenchResult: one truth id 1, `n` steps; it is stationary for
// the first `move_at` steps then moves east at 5 m/s; a track co-located with it
// is present exactly on the steps in `tracked` (all within the 100 m assoc gate).
BenchResult makeResult(int n, int move_at, const std::vector<bool>& tracked) {
  BenchResult r;
  for (int k = 0; k < n; ++k) {
    BenchStep s;
    s.time = navtracker::Timestamp::fromSeconds(k);
    const bool moving = k >= move_at;
    Eigen::Vector2d pos(moving ? (k - move_at) * 5.0 : 0.0, 0.0);
    Eigen::Vector2d vel(moving ? 5.0 : 0.0, 0.0);
    s.truth.push_back(TruthStateSnapshot{1, pos, vel});
    if (k < static_cast<int>(tracked.size()) && tracked[k]) {
      TrackStateSnapshot t;
      t.id = navtracker::TrackId{7};
      t.position = pos;  // exactly on the truth -> assigned within gate
      t.velocity = vel;
      s.tracks.push_back(t);
    }
    r.steps.push_back(std::move(s));
  }
  return r;
}

double promoLatency(const BenchResult& r) {
  const auto assigns = assignPerStep(r, 100.0);
  const auto cont = computeContinuity(r, assigns);
  auto it = cont.per_truth.find(1);
  return it == cont.per_truth.end() ? -1.0 : it->second.promotion_latency;
}

}  // namespace

// (1) TEETH: the metric measures a real latency. Anchored for 4 steps, moves at
// step 4 (1-based present index 5). Three regimes:
TEST(Adr0002PromotionMetric, MeasuresLatencyOnsetToFirstTrackedWhileMoving) {
  // (a) tracked the whole time -> promotion is immediate (0).
  EXPECT_DOUBLE_EQ(promoLatency(makeResult(10, 4, std::vector<bool>(10, true))),
                   0.0);

  // (b) suppressed while anchored AND for the first 3 moving steps, first tracked
  // at step 7 (present index 8): latency = 8 - 5 = 3.
  {
    std::vector<bool> tr(10, false);
    for (int k = 7; k < 10; ++k) tr[k] = true;  // tracked from step 7 on
    EXPECT_DOUBLE_EQ(promoLatency(makeResult(10, 4, tr)), 3.0);
  }

  // (c) NEVER tracked while moving -> the sentinel is the whole moving-phase
  // length (6 moving steps), a large value that fails any bounded-latency gate.
  {
    std::vector<bool> tr(10, false);
    for (int k = 0; k < 4; ++k) tr[k] = true;  // tracked only while anchored
    EXPECT_DOUBLE_EQ(promoLatency(makeResult(10, 4, tr)), 6.0);
  }

  // (d) a truth that never moves has no promotion to make -> 0.
  EXPECT_DOUBLE_EQ(promoLatency(makeResult(6, 6, std::vector<bool>(6, true))),
                   0.0);
}

// (2a) The deployable: an open-water anchored vessel is never suppressed, so
// promotion is immediate and its identity survives stop->go.
TEST(Adr0002Promotion, DeployableRecoversMoverWithBoundedLatencyAndStableId) {
  const auto all = defaultConfigs();
  const Config* dep = byLabel(all, kDeployable);
  ASSERT_NE(dep, nullptr) << "deployable config missing from defaultConfigs()";
  const auto rows = runScenario({*dep});

  const std::string sfx = ":truth_" + std::to_string(kMoverTruthId);
  const double latency = maxMetric(rows, kDeployable, "promotion_latency" + sfx);
  const double lifetime = meanMetric(rows, kDeployable, "lifetime_ratio" + sfx);
  const double id_sw = maxMetric(rows, kDeployable, "id_switches" + sfx);
  const double breaks = maxMetric(rows, kDeployable, "track_breaks" + sfx);
  std::cout << "\n=== ADR-0002 rule-3 (DEPLOYABLE " << kDeployable << ") ===\n"
            << "  truth_" << kMoverTruthId << ": worst promotion_latency="
            << latency << " scans  mean lifetime=" << lifetime
            << "  worst id_switches=" << id_sw << "  worst track_breaks="
            << breaks << "\n" << std::flush;

  // Banded bound, measured worst-seed = 0 scans; margin to 2. A mover pinned as
  // static or lost at stop->go would blow this (or drop lifetime / churn the id).
  EXPECT_LE(latency, 2.0)
      << "the anchored mover was not promoted to a track promptly";
  EXPECT_GE(lifetime, 0.9)
      << "the mover was not tracked through most of its life";
  EXPECT_EQ(id_sw, 0.0)
      << "track identity must survive the stop->go transition (ADR-0002 rule-3)";
  EXPECT_LE(breaks, 1.0) << "the mover's track fragmented across stop->go";
}

// (2b) The occupancy-detector config: the compact anchored boat is (correctly)
// kept as a vessel by the extent discriminator, so it tracks through too; the
// diagnostics confirm the occupancy layer is active on the scene (the pier).
TEST(Adr0002Promotion, OccupancyDetectorKeepsCompactMoverAndLayerIsActive) {
  const auto all = defaultConfigs();
  const Config* det = byLabel(all, kDetector);
  ASSERT_NE(det, nullptr) << "detector config missing from defaultConfigs()";
  const auto rows = runScenario({*det});

  const std::string sfx = ":truth_" + std::to_string(kMoverTruthId);
  const double latency = maxMetric(rows, kDetector, "promotion_latency" + sfx);
  const double lifetime = meanMetric(rows, kDetector, "lifetime_ratio" + sfx);
  const double structs = meanMetric(rows, kDetector, "occ_peak_structures");
  const double hits = meanMetric(rows, kDetector, "occ_suppress_hits");
  std::cout << "\n=== ADR-0002 rule-3 (DETECTOR " << kDetector << ") ===\n"
            << "  truth_" << kMoverTruthId << ": worst promotion_latency="
            << latency << " scans  mean lifetime=" << lifetime
            << "  occ_peak_structures=" << structs << "  occ_suppress_hits="
            << hits << "\n" << std::flush;

  EXPECT_LE(latency, 5.0) << "the mover was not promoted within a bounded latency";
  EXPECT_GE(lifetime, 0.9) << "the mover was not tracked through most of its life";
  EXPECT_GT(structs, 0.0)
      << "the occupancy layer formed no structure — it is inert here, so this "
         "config would not exercise suppression at all";
}
