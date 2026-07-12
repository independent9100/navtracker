#include <cmath>
#include <cstdio>
#include <memory>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"
#include "core/estimation/PrescribedTurn.hpp"
#include "core/estimation/UkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

namespace {

struct RunStats {
  double ospa;
  int id_switches;
  std::size_t tracks;
};

struct AggStats {
  double mean_ospa;
  double std_ospa;
  double mean_id_sw;
};

AggStats aggregate(const std::vector<RunStats>& runs) {
  const std::size_t N = runs.size();
  double sum_o = 0.0, sum_i = 0.0;
  for (const auto& r : runs) { sum_o += r.ospa; sum_i += r.id_switches; }
  const double m_o = sum_o / static_cast<double>(N);
  double sse = 0.0;
  for (const auto& r : runs) sse += (r.ospa - m_o) * (r.ospa - m_o);
  const double s_o = std::sqrt(sse / static_cast<double>(N - 1));
  return {m_o, s_o, sum_i / static_cast<double>(N)};
}

RunStats runStandard(const IEstimator& est, const IDataAssociator& assoc,
                     const Scenario& s, double cutoff, int confirm, int del,
                     double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), mgr.size()};
}

RunStats runBatched(const IEstimator& est, const IDataAssociator& assoc,
                    const Scenario& s, double cutoff, int confirm, int del,
                    double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), mgr.size()};
}

RunStats runMhtBatched(const IEstimator& est, const Scenario& s, double cutoff,
                       const MhtTracker::Config& cfg) {
  MhtTracker tracker(est, cfg);
  const ScenarioResult r = runScenarioBatchedMht(s, tracker, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), tracker.tracks().size()};
}

constexpr int kNumSeeds = 20;

}  // namespace

TEST(MultiSeedSweep, IMM3OnManeuvering) {
  std::vector<RunStats> ekf_runs, imm2_runs, imm3_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201 + k;
    const Scenario s = buildManeuveringTargetScenario(
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
        5.0, 5.0, 0.2, 1.0, 5.0, seed);
    auto cv4 = std::make_shared<ConstantVelocity2D>(0.5);
    const EkfEstimator ekf(cv4, 10.0);
    GnnAssociator gnn(50.0);

    std::vector<std::shared_ptr<IMotionModel>> motions2 = {
        std::make_shared<ConstantVelocity5State>(0.5, 0.01),
        std::make_shared<CoordinatedTurn>(0.5, 0.1)};
    Eigen::MatrixXd pi2(2, 2); pi2 << 0.95, 0.05, 0.10, 0.90;
    Eigen::VectorXd mu2(2); mu2 << 0.5, 0.5;
    const ImmEstimator imm2(motions2, pi2, mu2, 10.0, 0.1);

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

    ekf_runs .push_back(runStandard(ekf,  gnn, s, 100.0, 1, 5, 10.0));
    imm2_runs.push_back(runStandard(imm2, gnn, s, 100.0, 1, 5, 10.0));
    imm3_runs.push_back(runStandard(imm3, gnn, s, 100.0, 1, 5, 10.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats i2 = aggregate(imm2_runs);
  const AggStats i3 = aggregate(imm3_runs);
  std::fprintf(stderr,
      "\n[IMM3 on Maneuvering, %d seeds]\n"
      "  EKF   : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  IMM-2 : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  IMM-3 : %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      i2.mean_ospa, i2.std_ospa, i2.mean_id_sw,
      i3.mean_ospa, i3.std_ospa, i3.mean_id_sw);

  // #24: falsifiable blow-up guard (was print-only; W3 assertion-quality#5).
  // mean_ospa is the 20-seed aggregate of per-seed mean_ospa; per-seed OSPA
  // is clipped at the scenario cutoff 100.0.
  EXPECT_TRUE(std::isfinite(e.mean_ospa)) << "IMM3OnManeuvering EKF: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(i2.mean_ospa)) << "IMM3OnManeuvering IMM-2: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(i3.mean_ospa)) << "IMM3OnManeuvering IMM-3: non-finite OSPA (filter diverged)";
  EXPECT_LT(e.mean_ospa, 100.0) << "IMM3OnManeuvering EKF: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(i2.mean_ospa, 100.0) << "IMM3OnManeuvering IMM-2: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(i3.mean_ospa, 100.0) << "IMM3OnManeuvering IMM-3: OSPA saturated at cutoff (total mis-track)";
}

TEST(MultiSeedSweep, JpdaOnClutterCrossing) {
  std::vector<RunStats> gnn_runs, jpda_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201 + k;
    std::vector<double> times;
    for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
    const Scenario s = buildClutterCrossingScenario(
        Eigen::Vector2d(-200.0, 5.0), Eigen::Vector2d(15.0, 0.0),
        Eigen::Vector2d( 200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
        times, 5.0, 4,
        Eigen::Vector2d(-300.0, -50.0), Eigen::Vector2d(300.0, 50.0), seed);
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    GnnAssociator gnn(20.0);
    JpdaAssociator jpda(20.0, 0.9, 1e-4);
    gnn_runs .push_back(runBatched(ekf, gnn,  s, 50.0, 2, 4, 30.0));
    jpda_runs.push_back(runBatched(ekf, jpda, s, 50.0, 2, 4, 30.0));
  }
  const AggStats g = aggregate(gnn_runs);
  const AggStats j = aggregate(jpda_runs);
  std::fprintf(stderr,
      "\n[JPDA on ClutterCrossing, %d seeds]\n"
      "  GNN  : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  JPDA : %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      g.mean_ospa, g.std_ospa, g.mean_id_sw,
      j.mean_ospa, j.std_ospa, j.mean_id_sw);

  // #24: falsifiable blow-up guard (was print-only; W3 assertion-quality#5).
  // mean_ospa is the 20-seed aggregate of per-seed mean_ospa; per-seed OSPA
  // is clipped at the scenario cutoff 50.0.
  EXPECT_TRUE(std::isfinite(g.mean_ospa)) << "JpdaOnClutterCrossing GNN: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(j.mean_ospa)) << "JpdaOnClutterCrossing JPDA: non-finite OSPA (filter diverged)";
  EXPECT_LT(g.mean_ospa, 50.0) << "JpdaOnClutterCrossing GNN: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(j.mean_ospa, 50.0) << "JpdaOnClutterCrossing JPDA: OSPA saturated at cutoff (total mis-track)";
}

TEST(MultiSeedSweep, MhtOnCrossingDropout) {
  std::vector<RunStats> gnn_runs, jpda_runs, mht_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201 + k;
    std::vector<double> times;
    for (int i = 0; i <= 30; ++i) times.push_back(static_cast<double>(i));
    const Scenario s = buildCrossingDropoutScenario(
        4.0, 1.0, times, 1.0, 13.0, 17.0, seed);
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    GnnAssociator gnn(9.0);
    JpdaAssociator jpda(9.0, 0.9, 1e-4);
    MhtTracker::Config cfg;
    cfg.gate_threshold = 9.0; cfg.n_scan = 3; cfg.k_max_leaves = 5;
    cfg.score_delete_threshold = -15.0;
    gnn_runs .push_back(runBatched(ekf, gnn,  s, 20.0, 2, 6, 10.0));
    jpda_runs.push_back(runBatched(ekf, jpda, s, 20.0, 2, 6, 10.0));
    mht_runs .push_back(runMhtBatched(ekf, s, 20.0, cfg));
  }
  const AggStats g = aggregate(gnn_runs);
  const AggStats j = aggregate(jpda_runs);
  const AggStats m = aggregate(mht_runs);
  std::fprintf(stderr,
      "\n[MHT on CrossingDropout, %d seeds]\n"
      "  GNN  : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  JPDA : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  MHT  : %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      g.mean_ospa, g.std_ospa, g.mean_id_sw,
      j.mean_ospa, j.std_ospa, j.mean_id_sw,
      m.mean_ospa, m.std_ospa, m.mean_id_sw);

  // #24: falsifiable blow-up guard (was print-only; W3 assertion-quality#5).
  // mean_ospa is the 20-seed aggregate of per-seed mean_ospa; per-seed OSPA
  // is clipped at the scenario cutoff 20.0.
  EXPECT_TRUE(std::isfinite(g.mean_ospa)) << "MhtOnCrossingDropout GNN: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(j.mean_ospa)) << "MhtOnCrossingDropout JPDA: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(m.mean_ospa)) << "MhtOnCrossingDropout MHT: non-finite OSPA (filter diverged)";
  EXPECT_LT(g.mean_ospa, 20.0) << "MhtOnCrossingDropout GNN: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(j.mean_ospa, 20.0) << "MhtOnCrossingDropout JPDA: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(m.mean_ospa, 20.0) << "MhtOnCrossingDropout MHT: OSPA saturated at cutoff (total mis-track)";
}

TEST(MultiSeedSweep, PFOnBearingOnlyMovingSensor) {
  std::vector<RunStats> ekf_runs, ukf_runs, pf_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201 + k;
    std::vector<double> times;
    for (int i = 0; i <= 60; ++i) times.push_back(static_cast<double>(i));
    constexpr double kPi = 3.14159265358979323846;
    const double bearing_std = 1.5 * kPi / 180.0;
    const Scenario s = buildBearingOnlyMovingSensorScenario(
        Eigen::Vector2d(1500.0, 0.0),
        Eigen::Vector2d(0.0, -300.0),
        Eigen::Vector2d(0.0, 10.0),
        times, 300.0, bearing_std, seed);
    auto motion = std::make_shared<ConstantVelocity2D>(0.05);
    const EkfEstimator ekf(motion, 5.0);
    const UkfEstimator ukf(motion, 5.0);
    const ParticleFilterEstimator pf(motion, 2000, 5.0, 0.5,
                                     static_cast<std::uint64_t>(seed));
    GnnAssociator gnn(2500.0);
    ekf_runs.push_back(runStandard(ekf, gnn, s, 500.0, 1, 8, 90.0));
    ukf_runs.push_back(runStandard(ukf, gnn, s, 500.0, 1, 8, 90.0));
    pf_runs .push_back(runStandard(pf,  gnn, s, 500.0, 1, 8, 90.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats u = aggregate(ukf_runs);
  const AggStats p = aggregate(pf_runs);
  std::fprintf(stderr,
      "\n[PF on BearingOnlyMovingSensor, %d seeds]\n"
      "  EKF : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  UKF : %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  PF  : %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      u.mean_ospa, u.std_ospa, u.mean_id_sw,
      p.mean_ospa, p.std_ospa, p.mean_id_sw);

  // #24: falsifiable blow-up guard (was print-only; W3 assertion-quality#5).
  // mean_ospa is the 20-seed aggregate of per-seed mean_ospa; per-seed OSPA
  // is clipped at the scenario cutoff 500.0.
  EXPECT_TRUE(std::isfinite(e.mean_ospa)) << "PFOnBearingOnlyMovingSensor EKF: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(u.mean_ospa)) << "PFOnBearingOnlyMovingSensor UKF: non-finite OSPA (filter diverged)";
  EXPECT_TRUE(std::isfinite(p.mean_ospa)) << "PFOnBearingOnlyMovingSensor PF: non-finite OSPA (filter diverged)";
  EXPECT_LT(e.mean_ospa, 500.0) << "PFOnBearingOnlyMovingSensor EKF: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(u.mean_ospa, 500.0) << "PFOnBearingOnlyMovingSensor UKF: OSPA saturated at cutoff (total mis-track)";
  EXPECT_LT(p.mean_ospa, 500.0) << "PFOnBearingOnlyMovingSensor PF: OSPA saturated at cutoff (total mis-track)";
}
