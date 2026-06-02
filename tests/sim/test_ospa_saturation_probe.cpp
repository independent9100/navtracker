// One-shot diagnostic: split OSPA samples by empty-truth vs populated-truth
// to quantify the per-measurement-tick saturation effect.
// NOT a regression test. Always passes; prints stats to stderr.

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
#include "core/scenario/HarnessBatched.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

struct SaturationSplit {
  std::size_t total{0};
  std::size_t empty_truth{0};
  std::size_t populated{0};
  double sum_ospa_empty{0.0};
  double sum_ospa_pop{0.0};
};

SaturationSplit splitOspa(const ScenarioResult& r) {
  SaturationSplit s;
  for (std::size_t i = 0; i < r.steps.size(); ++i) {
    ++s.total;
    if (r.steps[i].truth.empty()) {
      ++s.empty_truth;
      s.sum_ospa_empty += r.ospa_per_step[i];
    } else {
      ++s.populated;
      s.sum_ospa_pop += r.ospa_per_step[i];
    }
  }
  return s;
}

}  // namespace

TEST(OspaSaturationProbe, JpdaClutterCrossing) {
  const std::uint32_t seed = 201u;
  const double cutoff = 50.0;
  const Scenario s = runBusClutterCrossing(seed, /*clutter_per_rotation=*/5);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  JpdaAssociator jpda(20.0, 0.9, 1e-4);
  TrackManager mgr(2, 4);
  Tracker tracker(ekf, jpda, mgr, 30.0);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);

  const SaturationSplit ss = splitOspa(r);
  const double mean_empty =
      ss.empty_truth ? ss.sum_ospa_empty / static_cast<double>(ss.empty_truth)
                     : 0.0;
  const double mean_pop =
      ss.populated ? ss.sum_ospa_pop / static_cast<double>(ss.populated)
                   : 0.0;
  const double frac_empty =
      ss.total ? static_cast<double>(ss.empty_truth) /
                     static_cast<double>(ss.total)
               : 0.0;

  std::fprintf(stderr,
      "\n[OSPA Saturation Probe: JPDA on ClutterCrossing, seed=%u, cutoff=%.1f]\n"
      "  total_steps     = %zu\n"
      "  empty_truth     = %zu  (%.1f%%)  mean OSPA = %.4f  (cutoff=%.1f)\n"
      "  populated_truth = %zu  (%.1f%%)  mean OSPA = %.4f\n"
      "  overall mean    = %.4f\n",
      seed, cutoff,
      ss.total,
      ss.empty_truth, 100.0 * frac_empty, mean_empty, cutoff,
      ss.populated, 100.0 * (1.0 - frac_empty), mean_pop,
      (ss.sum_ospa_empty + ss.sum_ospa_pop) /
          static_cast<double>(ss.total));
  SUCCEED();
}

TEST(OspaSaturationProbe, ImmManeuvering) {
  const std::uint32_t seed = 201u;
  const double cutoff = 100.0;
  const Scenario s = runBusManeuvering(seed);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 5);
  Tracker tracker(ekf, gnn, mgr, 10.0);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);

  const SaturationSplit ss = splitOspa(r);
  const double mean_empty =
      ss.empty_truth ? ss.sum_ospa_empty / static_cast<double>(ss.empty_truth)
                     : 0.0;
  const double mean_pop =
      ss.populated ? ss.sum_ospa_pop / static_cast<double>(ss.populated)
                   : 0.0;
  const double frac_empty =
      ss.total ? static_cast<double>(ss.empty_truth) /
                     static_cast<double>(ss.total)
               : 0.0;

  std::fprintf(stderr,
      "\n[OSPA Saturation Probe: EKF on Maneuvering, seed=%u, cutoff=%.1f]\n"
      "  total_steps     = %zu\n"
      "  empty_truth     = %zu  (%.1f%%)  mean OSPA = %.4f  (cutoff=%.1f)\n"
      "  populated_truth = %zu  (%.1f%%)  mean OSPA = %.4f\n"
      "  overall mean    = %.4f\n",
      seed, cutoff,
      ss.total,
      ss.empty_truth, 100.0 * frac_empty, mean_empty, cutoff,
      ss.populated, 100.0 * (1.0 - frac_empty), mean_pop,
      (ss.sum_ospa_empty + ss.sum_ospa_pop) /
          static_cast<double>(ss.total));
  SUCCEED();
}

TEST(OspaSaturationProbe, PfBearingOnly) {
  // PF scenario configures EO/IR at dt_s=1.0 (matches truth_sample_dt_s) so
  // we expect ~0% empty-truth ticks here, in contrast to the others.
  const std::uint32_t seed = 201u;
  const double cutoff = 500.0;
  const Scenario s = runBusBearingOnlyMoving(seed);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 8);
  Tracker tracker(ekf, gnn, mgr, 90.0);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);

  const SaturationSplit ss = splitOspa(r);
  const double mean_empty =
      ss.empty_truth ? ss.sum_ospa_empty / static_cast<double>(ss.empty_truth)
                     : 0.0;
  const double mean_pop =
      ss.populated ? ss.sum_ospa_pop / static_cast<double>(ss.populated)
                   : 0.0;
  const double frac_empty =
      ss.total ? static_cast<double>(ss.empty_truth) /
                     static_cast<double>(ss.total)
               : 0.0;

  std::fprintf(stderr,
      "\n[OSPA Saturation Probe: EKF on BearingOnlyMoving, seed=%u, cutoff=%.1f]\n"
      "  total_steps     = %zu\n"
      "  empty_truth     = %zu  (%.1f%%)  mean OSPA = %.4f  (cutoff=%.1f)\n"
      "  populated_truth = %zu  (%.1f%%)  mean OSPA = %.4f\n"
      "  overall mean    = %.4f\n",
      seed, cutoff,
      ss.total,
      ss.empty_truth, 100.0 * frac_empty, mean_empty, cutoff,
      ss.populated, 100.0 * (1.0 - frac_empty), mean_pop,
      (ss.sum_ospa_empty + ss.sum_ospa_pop) /
          static_cast<double>(ss.total));
  SUCCEED();
}
