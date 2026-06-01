#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

namespace {

struct RunOutput {
  double mean_ospa;
  int id_switches;
  std::size_t final_track_count;
};

RunOutput runBatched(const IEstimator& est,
                     const IDataAssociator& assoc,
                     const Scenario& s,
                     double cutoff,
                     int confirm,
                     int del,
                     double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), mgr.size()};
}

}  // namespace

TEST(JpdaComparison, ClutterCrossing) {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildClutterCrossingScenario(
      Eigen::Vector2d(-200.0, 5.0), Eigen::Vector2d(15.0, 0.0),
      Eigen::Vector2d( 200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
      times, 5.0,
      /*clutter*/ 4,
      Eigen::Vector2d(-300.0, -50.0), Eigen::Vector2d(300.0, 50.0),
      31);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);

  GnnAssociator gnn(20.0);
  JpdaAssociator jpda(20.0, /*P_D*/ 0.9, /*lambda_C*/ 1e-4);

  const RunOutput g = runBatched(ekf, gnn,  s, 50.0, 2, 4, 30.0);
  const RunOutput j = runBatched(ekf, jpda, s, 50.0, 2, 4, 30.0);

  std::fprintf(stderr,
               "\n[ClutterCrossing] GNN  mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ClutterCrossing] JPDA mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               g.mean_ospa, g.id_switches, g.final_track_count,
               j.mean_ospa, j.id_switches, j.final_track_count);
}
