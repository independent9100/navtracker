#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
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

RunStats runMhtBatchedBus(const IEstimator& est, const Scenario& s,
                          double cutoff, const MhtTracker::Config& cfg) {
  MhtTracker tracker(est, cfg);
  const ScenarioResult r = runScenarioBatchedMht(s, tracker, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, MhtVsJpdaClutterCrossing) {
  std::vector<RunStats> jpda_runs, mht_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusClutterCrossing(seed, /*clutter_per_rotation=*/5);

    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    JpdaAssociator jpda(20.0, 0.9, 1e-4);

    MhtTracker::Config mht_cfg;
    mht_cfg.probability_of_detection = 0.9;
    mht_cfg.clutter_density = 1e-4;
    mht_cfg.gate_threshold = 20.0;
    mht_cfg.n_scan = 3;
    mht_cfg.k_max_leaves = 5;
    mht_cfg.score_delete_threshold = -15.0;

    jpda_runs.push_back(runBatchedBus(ekf, jpda, s, 50.0, 2, 4, 30.0));
    mht_runs .push_back(runMhtBatchedBus(ekf, s, 50.0, mht_cfg));
  }
  const AggStats j = aggregate(jpda_runs);
  const AggStats m = aggregate(mht_runs);
  std::fprintf(stderr,
      "\n[Bus MHT vs JPDA on ClutterCrossing, %d seeds]\n"
      "  JPDA : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  MHT  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      j.mean_ospa, j.std_ospa, j.mean_id_sw,
      m.mean_ospa, m.std_ospa, m.mean_id_sw);

  // No directional assertion — the prior sweep retracted the MHT win. We
  // print aggregates so the eval-log update can record whatever ratio the
  // bus produces.
  EXPECT_GT(j.mean_ospa, 0.0);
  EXPECT_GT(m.mean_ospa, 0.0);
}
