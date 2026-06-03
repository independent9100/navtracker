#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

double runOneBearingOnly(std::uint32_t seed, const HeadingSweepKnob& knob) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 8);
  Tracker tracker(ekf, gnn, mgr, 90.0);
  const ScenarioResult r = runScenario(
      runBusBearingOnlyMovingWithHeading(seed, knob),
      tracker, mgr, /*cutoff=*/500.0);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return pw.mean;
}

}  // namespace

TEST(BusHeadingProbe, ConstantBiasShiftsOspa) {
  // Single seed, σ=0, R-inflation off. A constant bias on the HDT
  // should cause the projected bearing-only track to sit
  // ~range·sin(bias) to one side of truth at 1.5 km range. Pre-check
  // is just "OSPA goes up", which is the wired-up signal.
  // 1°/0.01°/s were too small to dominate baseline noise on this single
  // seed; bumped to 3°/0.03°/s per the task's fallback instructions.
  HeadingSweepKnob no_err;
  HeadingSweepKnob with_bias;
  with_bias.bias_deg = 3.0;
  const double ospa_clean = runOneBearingOnly(201u, no_err);
  const double ospa_biased = runOneBearingOnly(201u, with_bias);
  std::fprintf(stderr,
      "\n[Bus Heading Probe: BearingOnlyMoving, seed=201]\n"
      "  no error   : per-window OSPA mean = %.4f m\n"
      "  bias 3 deg : per-window OSPA mean = %.4f m\n",
      ospa_clean, ospa_biased);
  EXPECT_GT(ospa_biased, ospa_clean);
}

TEST(BusHeadingProbe, LinearDriftShiftsOspa) {
  // 0.03 deg/s drift over the 60 s scenario yields a 1.8° final heading
  // offset; expect the OSPA mean to rise relative to the no-error case.
  HeadingSweepKnob no_err;
  HeadingSweepKnob with_drift;
  with_drift.drift_deg_per_s = 0.03;
  const double ospa_clean = runOneBearingOnly(201u, no_err);
  const double ospa_drift = runOneBearingOnly(201u, with_drift);
  std::fprintf(stderr,
      "\n[Bus Heading Probe: BearingOnlyMoving, seed=201]\n"
      "  no error     : per-window OSPA mean = %.4f m\n"
      "  drift 0.03/s : per-window OSPA mean = %.4f m\n",
      ospa_clean, ospa_drift);
  EXPECT_GT(ospa_drift, ospa_clean);
}
