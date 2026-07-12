#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
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

RunOutput runMht(const IEstimator& est,
                 const Scenario& s,
                 double cutoff,
                 const MhtTracker::Config& cfg) {
  MhtTracker tracker(est, cfg);
  const ScenarioResult r = runScenarioBatchedMht(s, tracker, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), tracker.tracks().size()};
}

}  // namespace

TEST(MhtComparison, CrossingWithDropout) {
  std::vector<double> times;
  for (int i = 0; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingDropoutScenario(
      /*vx*/ 4.0, /*y*/ 1.0, times,
      /*noise*/ 1.0, /*dropout_start*/ 13.0, /*dropout_end*/ 17.0,
      /*seed*/ 113);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(9.0);
  JpdaAssociator jpda(9.0, 0.9, 1e-4);

  const RunOutput g = runBatched(ekf, gnn,  s, 20.0, 2, 6, 10.0);
  const RunOutput j = runBatched(ekf, jpda, s, 20.0, 2, 6, 10.0);

  MhtTracker::Config mcfg;
  mcfg.probability_of_detection = 0.9;
  mcfg.clutter_density = 1e-4;
  mcfg.gate_threshold = 9.0;
  mcfg.n_scan = 3;
  mcfg.k_max_leaves = 5;
  mcfg.score_delete_threshold = -15.0;
  const RunOutput m = runMht(ekf, s, 20.0, mcfg);

  std::fprintf(stderr,
               "\n[CrossingDropout] GNN  mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[CrossingDropout] JPDA mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[CrossingDropout] MHT  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               g.mean_ospa, g.id_switches, g.final_track_count,
               j.mean_ospa, j.id_switches, j.final_track_count,
               m.mean_ospa, m.id_switches, m.final_track_count);

  // #24: falsifiable blow-up guard (was print-only; W3 assertion-quality#5).
  EXPECT_TRUE(std::isfinite(g.mean_ospa)) << "CrossingWithDropout GNN: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(j.mean_ospa)) << "CrossingWithDropout JPDA: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(m.mean_ospa)) << "CrossingWithDropout MHT: non-finite OSPA (filter diverged)";
  EXPECT_LT(g.mean_ospa, 20.0) << "CrossingWithDropout GNN: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(j.mean_ospa, 20.0) << "CrossingWithDropout JPDA: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(m.mean_ospa, 20.0) << "CrossingWithDropout MHT: OSPA saturated at cutoff (total mis-track)";
}
