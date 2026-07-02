// R6 gate: a real boat moored ~22 m off a charted pier — inside the keep-clear
// buffer, outside the hard footprint. Contract (truth is closed + time-sorted)
// plus the end-to-end gate: under imm_cv_ct_pmbm_static the boat must stay
// tracked (proving R1's obstacle-scoped pre-suppression floor keeps it birthable
// through the soft buffer) while the pier over-count is still suppressed.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/geo/Datum.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
ScenarioRun* find(const std::vector<std::unique_ptr<ScenarioRun>>& v,
                  const std::string& label) {
  for (const auto& s : v)
    if (s->descriptor().label == label) return s.get();
  return nullptr;
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

// Contract: six closed truth targets (2 movers, 3 open-water boats, 1 near-pier
// boat id 6, stationary), time-sorted into 40 complete {1..6} groups.
TEST(HarborBoatNearPier, TruthIsClosedSixTargetsTimeSorted) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = find(scenarios, "harbor_boat_near_pier");
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);

  std::set<std::uint64_t> ids;
  std::map<std::uint64_t, double> max_speed;
  for (const auto& ts : scen.truth) {
    ids.insert(ts.truth_id);
    max_speed[ts.truth_id] =
        std::max(max_speed[ts.truth_id], ts.velocity.norm());
  }
  EXPECT_EQ(ids, (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u, 6u}));
  EXPECT_EQ(max_speed[6u], 0.0);  // near-pier boat is anchored

  // Time-sorted into 40 complete groups (BenchRunner::groupTruth contract).
  for (std::size_t i = 1; i < scen.truth.size(); ++i)
    ASSERT_LE(scen.truth[i - 1].time.seconds(), scen.truth[i].time.seconds());
  std::vector<double> gtimes;
  std::vector<std::set<std::uint64_t>> gids;
  for (const auto& ts : scen.truth) {
    const double t = ts.time.seconds();
    if (gtimes.empty() || gtimes.back() != t) {
      gtimes.push_back(t);
      gids.emplace_back();
    }
    gids.back().insert(ts.truth_id);
  }
  EXPECT_EQ(gtimes.size(), 40u);
  for (const auto& g : gids)
    EXPECT_EQ(g, (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u, 6u}));
}

// The near-pier boat sits inside the keep-clear buffer (10 m < d <= 50 m from
// the charted pier), i.e. exactly the soft-suppression band R1 must not kill.
TEST(HarborBoatNearPier, NearPierBoatIsInsideKeepClearBuffer) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = find(scenarios, "harbor_boat_near_pier");
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);
  ASSERT_TRUE(scen.datum.has_value());
  const geo::Datum datum = *scen.datum;

  const auto obs = h->syntheticObstacles();
  ASSERT_TRUE(obs.has_value());
  ASSERT_FALSE(obs->empty());

  // Boat 6's truth position (ENU).
  Eigen::Vector2d boat(0, 0);
  bool found = false;
  for (const auto& ts : scen.truth)
    if (ts.truth_id == 6u) { boat = ts.position.head<2>(); found = true; break; }
  ASSERT_TRUE(found);

  double d_min = 1e18;
  for (const auto& o : *obs) {
    const Eigen::Vector3d e = datum.toEnu(o.position);
    d_min = std::min(d_min, (Eigen::Vector2d(e.x(), e.y()) - boat).norm());
  }
  EXPECT_GT(d_min, 10.0);   // outside the hard footprint
  EXPECT_LE(d_min, 50.0);   // inside the soft keep-clear buffer
}

// Integration gate: under imm_cv_ct_pmbm_static the near-pier boat is tracked
// (aggregate lifetime over 6 targets stays high — a dropped boat would pull it
// to ~0.81) while the pier over-count is still suppressed vs the no-obstacle
// baseline. Proves the charted keep-clear buffer is SOFT (a moored boat next to
// structure still initiates and holds), the ADR 0002 promise. Added to the M2
// gate. (R1 is not load-bearing at this config/distance — see the scenario
// comment; R1's clean proof is test_pmbm_birth_floor.cpp.)
TEST(HarborBoatNearPier, StaticTracksNearPierBoatAndSuppressesPier) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs())
    if (c.label == "imm_cv_ct_pmbm" || c.label == "imm_cv_ct_pmbm_static")
      configs.push_back(c);
  ASSERT_EQ(configs.size(), 2u);

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_boat_near_pier")
      scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), 1u);

  SweepParams params;
  params.run_id = "harbor_boat_near_pier_gate";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const std::string sc = "harbor_boat_near_pier";
  const double life_static =
      meanMetric(rows, "imm_cv_ct_pmbm_static", sc, "lifetime_ratio");
  const double card_base =
      meanMetric(rows, "imm_cv_ct_pmbm", sc, "card_err_mean");
  const double card_static =
      meanMetric(rows, "imm_cv_ct_pmbm_static", sc, "card_err_mean");
  std::cout << "\n=== R6 boat-near-pier gate ===\n"
            << "  lifetime(static)=" << life_static
            << "  card_err base=" << card_base << " static=" << card_static
            << "\n" << std::flush;
  EXPECT_GT(life_static, 0.9);       // near-pier boat (1 of 6) is tracked
  EXPECT_LT(card_static, card_base); // pier still suppressed
}
