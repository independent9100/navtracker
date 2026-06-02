#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runBatchedBus(const IEstimator& est, const IDataAssociator& assoc,
                       const Scenario& s, double cutoff, int confirm, int del,
                       double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, JpdaVsGnnClutterCrossing) {
  std::vector<RunStats> gnn_runs, jpda_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusClutterCrossing(seed, /*clutter_per_rotation=*/5);
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    GnnAssociator  gnn(20.0);
    JpdaAssociator jpda(20.0, 0.9, 1e-4);
    gnn_runs .push_back(runBatchedBus(ekf, gnn,  s, 50.0, 2, 4, 30.0));
    jpda_runs.push_back(runBatchedBus(ekf, jpda, s, 50.0, 2, 4, 30.0));
  }
  const AggStats g = aggregate(gnn_runs);
  const AggStats j = aggregate(jpda_runs);
  std::fprintf(stderr,
      "\n[Bus JPDA vs GNN on ClutterCrossing, %d seeds]\n"
      "  GNN  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  JPDA : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      g.mean_ospa, g.std_ospa, g.mean_id_sw,
      j.mean_ospa, j.std_ospa, j.mean_id_sw);

  // Soft assertion: JPDA wins on at least one of OSPA or ID-switches.
  EXPECT_TRUE(j.mean_ospa < g.mean_ospa || j.mean_id_sw < g.mean_id_sw)
      << "JPDA does not beat GNN on either metric through the bus.";
}
