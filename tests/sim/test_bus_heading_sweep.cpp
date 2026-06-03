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
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

// Run a single (scenario, knob) cell with EKF+GNN, return one RunStats.
RunStats runStandardCell(
    navtracker::Scenario&& s, double cutoff, int confirm, int del,
    double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

RunStats runBatchedCell(
    navtracker::Scenario&& s, double cutoff, int confirm, int del,
    double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

struct Cell {
  const char* scenario_name;
  double sigma_heading_deg;
  bool r_inflation_on;
  AggStats agg;
};

void printCellsTable(const char* scenario_name,
                     const std::vector<Cell>& cells) {
  std::fprintf(stderr,
      "\n[Bus Heading Sweep on %s, %d seeds]\n"
      "  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean\n",
      scenario_name, kNumSeeds);
  for (const Cell& c : cells) {
    std::fprintf(stderr,
        "  %10.2f  | %-9s | %7.4f +/- %6.4f m | %.2f\n",
        c.sigma_heading_deg,
        c.r_inflation_on ? "on" : "off",
        c.agg.mean_ospa, c.agg.std_ospa, c.agg.mean_id_sw);
  }
}

}  // namespace

TEST(BusHeadingSweep, ClutterCrossing) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runBatchedCell(
            runBusClutterCrossingWithHeading(seed, /*clutter=*/5, knob),
            /*cutoff=*/50.0, /*confirm=*/2, /*del=*/4, /*miss=*/30.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"ClutterCrossing", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("ClutterCrossing", cells);
  SUCCEED();
}

TEST(BusHeadingSweep, BearingOnlyMoving) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runStandardCell(
            runBusBearingOnlyMovingWithHeading(seed, knob),
            /*cutoff=*/500.0, /*confirm=*/1, /*del=*/8, /*miss=*/90.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"BearingOnlyMoving", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("BearingOnlyMoving", cells);
  SUCCEED();
}

TEST(BusHeadingSweep, Maneuvering) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runStandardCell(
            runBusManeuveringWithHeading(seed, knob),
            /*cutoff=*/100.0, /*confirm=*/1, /*del=*/5, /*miss=*/10.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"Maneuvering", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("Maneuvering", cells);
  SUCCEED();
}
