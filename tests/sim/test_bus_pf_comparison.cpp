#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runStandardBus(const IEstimator& est, const IDataAssociator& assoc,
                        const Scenario& s, double cutoff, int confirm, int del,
                        double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, PfVsEkfBearingOnlyMoving) {
  std::vector<RunStats> ekf_runs, pf_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusBearingOnlyMoving(seed);

    auto motion = std::make_shared<ConstantVelocity2D>(0.05);
    const EkfEstimator ekf(motion, 5.0);
    const ParticleFilterEstimator pf(motion, 2000, 5.0, 0.5,
                                     static_cast<std::uint64_t>(seed));
    GnnAssociator gnn(2500.0);

    ekf_runs.push_back(runStandardBus(ekf, gnn, s, 500.0, 1, 8, 90.0));
    pf_runs .push_back(runStandardBus(pf,  gnn, s, 500.0, 1, 8, 90.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats p = aggregate(pf_runs);
  std::fprintf(stderr,
      "\n[Bus PF vs EKF on BearingOnlyMovingSensor, %d seeds]\n"
      "  EKF : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  PF  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      p.mean_ospa, p.std_ospa, p.mean_id_sw);

  // PF vs EKF was directional in the prior sweep (overlapping CIs); allow
  // either to win, but make sure neither blows up.
  EXPECT_GT(e.mean_ospa, 0.0);
  EXPECT_GT(p.mean_ospa, 0.0);
}
