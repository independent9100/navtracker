#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/HarnessBatchedPmbm.hpp"

using namespace navtracker;

namespace {

// Phase 1 birth model. A single wide-cov Gaussian centered at the local
// ENU patch origin, weight tuned so the per-point intensity is
// comparable to the configured clutter intensity. Enough for two
// targets within ~150 m of the origin to be born on contact with the
// first matching measurement. Real deployments replace this with a
// measurement-adaptive birth (García-Fernández 2018 §IV-D) — Phase 1
// keeps the simplest possible model that runs end-to-end.
pmbm::PmbmTracker::BirthModelFn smokeBirthModel(double weight_per_scan,
                                                double pos_std_m) {
  return [weight_per_scan, pos_std_m](Timestamp /*t*/,
                                      double /*dt*/) {
    std::vector<pmbm::PoissonComponent> v;
    pmbm::PoissonComponent c;
    c.weight = weight_per_scan;
    c.mean = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
    P(0, 0) = P(1, 1) = pos_std_m * pos_std_m;
    P(2, 2) = P(3, 3) = 25.0;  // 5 m/s² velocity prior
    c.covariance = P;
    v.push_back(c);
    return v;
  };
}

struct RunOut {
  double mean_ospa;
  std::size_t final_track_count;
};

RunOut runPmbm(const Scenario& s, double cutoff) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  pmbm::PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_intensity = 1e-5;
  cfg.survival_probability = 0.99;
  cfg.k_best_per_hypothesis = 3;
  cfg.max_global_hypotheses = 20;
  cfg.r_min = 1e-3;
  cfg.hypothesis_weight_min = 1e-4;
  cfg.confirm_threshold = 0.5;
  cfg.output_existence_floor = 0.3;
  pmbm::PmbmTracker tracker(ekf, cfg, smokeBirthModel(1.0, 50.0));
  const ScenarioResult r = runScenarioBatchedPmbm(s, tracker, cutoff);
  return {r.mean_ospa, tracker.tracks().size()};
}

}  // namespace

// ---------------------------------------------------------------------------
// Two crossing targets, clean position-2D measurements. PMBM should
// pick up and track both within tolerance.
TEST(PmbmScenario, TwoTargetsCrossingTracked) {
  std::vector<double> times;
  for (int i = 0; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(4.0, 0.5),
      Eigen::Vector2d(80.0, 0.0), Eigen::Vector2d(-4.0, 0.5),
      times, /*noise*/ 1.0, /*seed*/ 113);

  const RunOut o = runPmbm(s, /*cutoff*/ 30.0);
  std::fprintf(stderr, "\n[PmbmScenario] crossing mean_ospa=%.3f tracks=%zu\n",
               o.mean_ospa, o.final_track_count);

  // OSPA cutoff is 30 m; perfect tracking gives ~position noise (1 m).
  // Allow a generous bound while the birth model is naive.
  EXPECT_LT(o.mean_ospa, 25.0);
  // #24: the scenario has TWO true crossing targets, so a >=1 floor passed even
  // when one target was lost or the two merged into one track. Pin BOTH tracked
  // (measured exactly 2; band [2,5] still tolerates ≤3 transient phantoms).
  EXPECT_GE(o.final_track_count, 2u)
      << "both crossing targets must be tracked (a lost/merged target reads <2): "
      << o.final_track_count;
  EXPECT_LE(o.final_track_count, 5u);
}

// ---------------------------------------------------------------------------
// Replay the same scenario twice with fresh tracker state — output must
// be bit-identical (per the determinism contract; PMBM uses std::map for
// id ordering and Murty is deterministic).
TEST(PmbmScenario, ReplayDeterminism) {
  std::vector<double> times;
  for (int i = 0; i <= 15; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(4.0, 0.5),
      Eigen::Vector2d(80.0, 0.0), Eigen::Vector2d(-4.0, 0.5),
      times, /*noise*/ 1.0, /*seed*/ 113);

  const RunOut a = runPmbm(s, 30.0);
  const RunOut b = runPmbm(s, 30.0);
  EXPECT_DOUBLE_EQ(a.mean_ospa, b.mean_ospa);
  EXPECT_EQ(a.final_track_count, b.final_track_count);
}
