// Runs the canonical scenarios through both EKF and UKF and reports the
// metrics so the comparison can be recorded in the evaluation log. Asserts
// only that both filters succeed at the baseline thresholds — the numerical
// comparison itself is documented, not asserted.

#include <cmath>
#include <cstdio>
#include <memory>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/UkfEstimator.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

namespace {

struct RunOutput {
  double mean_ospa;
  int id_switches;
  std::size_t final_track_count;
};

RunOutput run(const IEstimator& est,
              const Scenario& s,
              double gate,
              double cutoff,
              int confirm,
              int del,
              double miss_timeout) {
  GnnAssociator assoc(gate);
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  return {r.mean_ospa, countIdSwitches(r.steps, cutoff), mgr.size()};
}

}  // namespace

TEST(FilterComparison, SingleStraightLine) {
  std::vector<double> times;
  for (int i = 1; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(100.0, 0.0), Eigen::Vector2d(5.0, 0.0),
      times, 5.0, 13);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  const UkfEstimator ukf(motion, 5.0);

  const RunOutput e = run(ekf, s, 50.0, 50.0, 2, 3, 30.0);
  const RunOutput u = run(ukf, s, 50.0, 50.0, 2, 3, 30.0);

  std::fprintf(stderr,
               "\n[SingleStraightLine] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[SingleStraightLine] UKF mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count);

  EXPECT_LT(e.mean_ospa, 15.0);
  EXPECT_LT(u.mean_ospa, 15.0);
}

TEST(FilterComparison, ParallelTargets) {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 800.0),
      Eigen::Vector2d(5.0, 0.0),
      times, 5.0, 29);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  const UkfEstimator ukf(motion, 5.0);

  const RunOutput e = run(ekf, s, 50.0, 50.0, 2, 4, 30.0);
  const RunOutput u = run(ukf, s, 50.0, 50.0, 2, 4, 30.0);

  std::fprintf(stderr,
               "\n[ParallelTargets] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ParallelTargets] UKF mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count);

  EXPECT_EQ(e.final_track_count, 2u);
  EXPECT_EQ(u.final_track_count, 2u);
  EXPECT_LT(e.mean_ospa, 20.0);
  EXPECT_LT(u.mean_ospa, 20.0);
}

TEST(FilterComparison, Crossing) {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0), Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0), Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, 11);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  const UkfEstimator ukf(motion, 5.0);

  const RunOutput e = run(ekf, s, 50.0, 50.0, 2, 4, 30.0);
  const RunOutput u = run(ukf, s, 50.0, 50.0, 2, 4, 30.0);

  std::fprintf(stderr,
               "\n[Crossing] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[Crossing] UKF mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count);

  EXPECT_EQ(e.final_track_count, 2u);
  EXPECT_EQ(u.final_track_count, 2u);
  EXPECT_LE(e.id_switches, 2);
  EXPECT_LE(u.id_switches, 2);
}

TEST(FilterComparison, ShortRangePass) {
  // Target passes ~50 m abeam of the sensor; range/bearing nonlinearity is
  // sharpest near closest approach. Initial position seed; the rest are
  // RangeBearing2D.
  std::vector<double> times;
  for (int i = 0; i <= 40; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 5.0 * kPi / 180.0;
  const Scenario s = buildRangeBearingPassScenario(
      Eigen::Vector2d(500.0, 50.0), Eigen::Vector2d(-25.0, 0.0),
      times, 10.0, 10.0, bearing_std, 41);
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator ekf(motion, 10.0);
  const UkfEstimator ukf(motion, 10.0);
  const ParticleFilterEstimator pf(motion, 1000, 10.0, 0.5, 41);

  const RunOutput e = run(ekf, s, 1000.0, 200.0, 1, 5, 60.0);
  const RunOutput u = run(ukf, s, 1000.0, 200.0, 1, 5, 60.0);
  const RunOutput p = run(pf,  s, 1000.0, 200.0, 1, 5, 60.0);

  std::fprintf(stderr,
               "\n[ShortRangePass] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ShortRangePass] UKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ShortRangePass] PF  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count,
               p.mean_ospa, p.id_switches, p.final_track_count);
}

TEST(FilterComparison, VeryShortRangePass) {
  // Sharper pass: closest approach 20 m, range std 20 m, bearing std 10 deg.
  // Nonlinearity bites harder; UKF should pull ahead by more than the
  // ShortRangePass scenario.
  std::vector<double> times;
  for (int i = 0; i <= 40; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 10.0 * kPi / 180.0;
  const Scenario s = buildRangeBearingPassScenario(
      Eigen::Vector2d(400.0, 20.0), Eigen::Vector2d(-20.0, 0.0),
      times, 15.0, 20.0, bearing_std, 53);
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator ekf(motion, 10.0);
  const UkfEstimator ukf(motion, 10.0);
  const ParticleFilterEstimator pf(motion, 1000, 10.0, 0.5, 53);

  const RunOutput e = run(ekf, s, 1500.0, 300.0, 1, 5, 60.0);
  const RunOutput u = run(ukf, s, 1500.0, 300.0, 1, 5, 60.0);
  const RunOutput p = run(pf,  s, 1500.0, 300.0, 1, 5, 60.0);

  std::fprintf(stderr,
               "\n[VeryShortRangePass] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[VeryShortRangePass] UKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[VeryShortRangePass] PF  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count,
               p.mean_ospa, p.id_switches, p.final_track_count);
}

TEST(FilterComparison, AisDropout) {
  std::vector<double> times;
  for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
  for (int i = 12; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 5.0, 3);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  const UkfEstimator ukf(motion, 5.0);

  const RunOutput e = run(ekf, s, 80.0, 80.0, 2, 5, 15.0);
  const RunOutput u = run(ukf, s, 80.0, 80.0, 2, 5, 15.0);

  std::fprintf(stderr,
               "\n[AisDropout] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[AisDropout] UKF mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count);

  EXPECT_EQ(e.final_track_count, 1u);
  EXPECT_EQ(u.final_track_count, 1u);
}

TEST(FilterComparison, ShortRangeMultiSeedSweep) {
  // Repeat ShortRangePass over 20 seeds for each estimator config and report
  // mean ± stddev of mean_ospa. Confirms whether single-seed comparisons
  // survive Monte-Carlo averaging.
  std::vector<double> times;
  for (int i = 0; i <= 40; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 5.0 * kPi / 180.0;

  const std::vector<int> seeds = {
      41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
      51, 52, 53, 54, 55, 56, 57, 58, 59, 60};
  const std::vector<int> Ns = {200, 500, 1000, 2000};

  std::vector<double> ekf_ospa, ukf_ospa;
  std::vector<std::vector<double>> pf_ospa(Ns.size());

  for (int seed : seeds) {
    const Scenario s = buildRangeBearingPassScenario(
        Eigen::Vector2d(500.0, 50.0), Eigen::Vector2d(-25.0, 0.0),
        times, 10.0, 10.0, bearing_std, static_cast<std::uint32_t>(seed));
    auto motion = std::make_shared<ConstantVelocity2D>(0.5);

    const EkfEstimator ekf(motion, 10.0);
    const UkfEstimator ukf(motion, 10.0);
    ekf_ospa.push_back(run(ekf, s, 1000.0, 200.0, 1, 5, 60.0).mean_ospa);
    ukf_ospa.push_back(run(ukf, s, 1000.0, 200.0, 1, 5, 60.0).mean_ospa);

    for (std::size_t k = 0; k < Ns.size(); ++k) {
      const ParticleFilterEstimator pf(motion, Ns[k], 10.0, 0.5,
                                       static_cast<std::uint64_t>(seed));
      pf_ospa[k].push_back(run(pf, s, 1000.0, 200.0, 1, 5, 60.0).mean_ospa);
    }
  }

  auto stats = [](const std::vector<double>& v) {
    const double mean =
        std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
    double sse = 0.0;
    for (double x : v) sse += (x - mean) * (x - mean);
    const double sd =
        std::sqrt(sse / static_cast<double>(v.size() - 1));
    return std::make_pair(mean, sd);
  };

  const auto e = stats(ekf_ospa);
  const auto u = stats(ukf_ospa);
  std::fprintf(stderr, "\n[Sweep ShortRangePass, %zu seeds]\n", seeds.size());
  std::fprintf(stderr, "  EKF            : %.4f ± %.4f m\n", e.first, e.second);
  std::fprintf(stderr, "  UKF            : %.4f ± %.4f m\n", u.first, u.second);
  for (std::size_t k = 0; k < Ns.size(); ++k) {
    const auto p = stats(pf_ospa[k]);
    std::fprintf(stderr, "  PF  N=%-4d     : %.4f ± %.4f m\n",
                 Ns[k], p.first, p.second);
  }
}
