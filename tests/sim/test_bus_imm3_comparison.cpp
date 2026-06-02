#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/estimation/PrescribedTurn.hpp"
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

TEST(BusComparison, Imm3VsCvManeuvering) {
  std::vector<RunStats> ekf_runs, imm3_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusManeuvering(seed);

    auto cv4 = std::make_shared<ConstantVelocity2D>(0.5);
    const EkfEstimator ekf(cv4, 10.0);
    GnnAssociator gnn(50.0);

    std::vector<std::shared_ptr<IMotionModel>> motions3 = {
        std::make_shared<ConstantVelocity5State>(0.5, 0.001),
        std::make_shared<PrescribedTurn>(+0.2, 0.5, 0.001),
        std::make_shared<PrescribedTurn>(-0.2, 0.5, 0.001)};
    Eigen::MatrixXd pi3(3, 3);
    pi3 << 0.90, 0.05, 0.05,
           0.10, 0.85, 0.05,
           0.10, 0.05, 0.85;
    Eigen::VectorXd mu3(3); mu3 << 0.34, 0.33, 0.33;
    const ImmEstimator imm3(motions3, pi3, mu3, 10.0, 0.01);

    ekf_runs .push_back(runStandardBus(ekf,  gnn, s, 100.0, 1, 5, 10.0));
    imm3_runs.push_back(runStandardBus(imm3, gnn, s, 100.0, 1, 5, 10.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats i = aggregate(imm3_runs);
  std::fprintf(stderr,
      "\n[Bus IMM-3 vs CV on Maneuvering, %d seeds]\n"
      "  EKF   : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  IMM-3 : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      i.mean_ospa, i.std_ospa, i.mean_id_sw);

  EXPECT_TRUE(i.mean_ospa < e.mean_ospa || i.mean_id_sw < e.mean_id_sw)
      << "IMM-3 does not beat single-mode CV on either metric through the bus.";
}
