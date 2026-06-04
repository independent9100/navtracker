// Bus-driven CPA-uncertainty sweep. For each (scenario, knob) cell run 20
// seeds through the SimulatedSensorBus + EKF + GNN, then at the end of each
// run synthesize own-ship via synthesizeOwnShipTrack and call
// computeCpaWithUncertainty against each confirmed target track. Aggregate
// mean(cpa) / mean(sigma_cpa) / mean(P(<200m)) across seeds and across
// confirmed tracks per cell. SUCCEED-only: this is reporting, not asserting.
//
// To keep the matrix manageable -- and per the plan's "simpler variant"
// allowance -- each cell uses ONE of the two knobs (heading sigma OR GPS
// sigma) at a time; a combined-knob cell is not built. The captured cells:
//   sigma_h=0, sigma_gps=0  (baseline)
//   sigma_h=2 deg, sigma_gps=0
//   sigma_h=0, sigma_gps=5 m
// per scenario in {ClutterCrossing, Maneuvering, BearingOnlyMoving}.

#include "tests/sim/BusComparisonHelpers.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/collision/Cpa.hpp"
#include "core/collision/CpaOwnShip.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

constexpr double kDThresholdM = 200.0;

struct CpaAgg {
  int n_pairs{0};
  double sum_cpa{0.0};
  double sum_sigma{0.0};
  double sum_p{0.0};
};

void addCpaPair(CpaAgg& agg, const CpaPrediction& p) {
  ++agg.n_pairs;
  agg.sum_cpa   += p.cpa_distance_m;
  agg.sum_sigma += p.sigma_cpa_m;
  agg.sum_p     += p.probability_below_threshold;
}

void printCpaRow(const char* scenario, double sigma_h_deg,
                 double sigma_gps_m, const CpaAgg& a) {
  if (a.n_pairs == 0) {
    std::fprintf(stderr,
        "  %-19s | %.1f | %.1f | (no confirmed tracks)\n",
        scenario, sigma_h_deg, sigma_gps_m);
    return;
  }
  const double inv = 1.0 / static_cast<double>(a.n_pairs);
  std::fprintf(stderr,
      "  %-19s | %.1f | %.1f | %9.3f | %9.4f | %.6f | n=%d\n",
      scenario, sigma_h_deg, sigma_gps_m,
      a.sum_cpa * inv, a.sum_sigma * inv, a.sum_p * inv, a.n_pairs);
}

// Drive a Scenario through EKF+GNN and CPA-evaluate every confirmed track
// against the own-ship synthesized from the last own-ship pose in the
// scenario. Aggregates into `agg`.
//
// Own-ship is treated as stationary at the ENU origin (which is true for
// ClutterCrossing / Maneuvering) for simplicity; for BearingOnlyMoving the
// own-ship is moving in y, so we use the *initial* pose at the origin with
// zero velocity as an approximation -- the test is documenting CPA uncertainty
// growth with sensor noise, not a realistic operational call.
void runCellAndAggregate(navtracker::Scenario&& s, double cutoff, int confirm,
                         int del, double miss_timeout, double q_proc,
                         const Eigen::Vector2d& own_velocity, CpaAgg& agg) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  (void)runScenario(s, tracker, mgr, cutoff);

  // Reference time: use the latest track's last_update so the own-ship
  // synthesized matches it.
  Timestamp t_ref = Timestamp::fromSeconds(0.0);
  for (const Track& tr : mgr.tracks()) {
    if (tr.status == TrackStatus::Confirmed && tr.state.size() >= 4) {
      if (Timestamp::fromSeconds(0.0) < tr.last_update &&
          t_ref < tr.last_update) {
        t_ref = tr.last_update;
      }
    }
  }
  if (!(Timestamp::fromSeconds(0.0) < t_ref)) {
    // No confirmed track with usable timestamp; nothing to aggregate.
    return;
  }

  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = own_velocity;
  pose.velocity_is_valid = false;
  const Track own_ship = synthesizeOwnShipTrack(pose, t_ref, provider);

  for (const Track& tr : mgr.tracks()) {
    if (tr.status != TrackStatus::Confirmed || tr.state.size() < 4) continue;
    const CpaPrediction p = computeCpaWithUncertainty(
        own_ship, tr, t_ref, kDThresholdM);
    addCpaPair(agg, p);
  }
}

struct Cell {
  const char* scenario;
  double sigma_h_deg;
  double sigma_gps_m;
  CpaAgg agg;
};

}  // namespace

TEST(BusCpaUncertainty, SweepAcrossScenarios) {
  std::fprintf(stderr,
      "\n[Bus CPA Uncertainty Sweep, %d seeds, EKF+GNN, R-on, "
      "d_threshold = %.0f m]\n"
      "  scenario            | s_h | s_gps |  mean cpa | sigma_cpa | P(<d)    | pairs\n",
      kNumSeeds, kDThresholdM);

  std::vector<Cell> cells;

  // ---- ClutterCrossing (stationary own-ship) ----
  {
    // Baseline: zero noise on both knobs.
    Cell c{"ClutterCrossing", 0.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 0.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusClutterCrossingWithHeading(seed, /*clutter=*/8,
                                                            knob),
                          50.0, 2, 4, 30.0, 0.1,
                          Eigen::Vector2d::Zero(), c.agg);
    }
    cells.push_back(c);
  }
  {
    Cell c{"ClutterCrossing", 2.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 2.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusClutterCrossingWithHeading(seed, /*clutter=*/8,
                                                            knob),
                          50.0, 2, 4, 30.0, 0.1,
                          Eigen::Vector2d::Zero(), c.agg);
    }
    cells.push_back(c);
  }
  {
    Cell c{"ClutterCrossing", 0.0, 5.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      GpsSweepKnob knob;
      knob.sigma_gps_m = 5.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusClutterCrossingWithGps(seed, knob),
                          50.0, 2, 4, 30.0, 0.1,
                          Eigen::Vector2d::Zero(), c.agg);
    }
    cells.push_back(c);
  }

  // ---- Maneuvering (stationary own-ship) ----
  {
    Cell c{"Maneuvering", 0.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 0.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusManeuveringWithHeading(seed, knob),
                          100.0, 1, 5, 10.0, 0.1,
                          Eigen::Vector2d::Zero(), c.agg);
    }
    cells.push_back(c);
  }
  {
    Cell c{"Maneuvering", 2.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 2.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusManeuveringWithHeading(seed, knob),
                          100.0, 1, 5, 10.0, 0.1,
                          Eigen::Vector2d::Zero(), c.agg);
    }
    cells.push_back(c);
  }
  // (Skipping the σ_GPS=5m cell for Maneuvering -- no Gps helper for that
  // scenario; the heading sweep is enough to show σ_cpa growth on it.)

  // ---- BearingOnlyMoving (own-ship moves north at 10 m/s) ----
  {
    Cell c{"BearingOnlyMoving", 0.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 0.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusBearingOnlyMovingWithHeading(seed, knob),
                          500.0, 1, 8, 90.0, 0.1,
                          Eigen::Vector2d(0.0, 10.0), c.agg);
    }
    cells.push_back(c);
  }
  {
    Cell c{"BearingOnlyMoving", 2.0, 0.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      HeadingSweepKnob knob;
      knob.sigma_heading_deg = 2.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusBearingOnlyMovingWithHeading(seed, knob),
                          500.0, 1, 8, 90.0, 0.1,
                          Eigen::Vector2d(0.0, 10.0), c.agg);
    }
    cells.push_back(c);
  }
  {
    Cell c{"BearingOnlyMoving", 0.0, 5.0, {}};
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
      GpsSweepKnob knob;
      knob.sigma_gps_m = 5.0;
      knob.r_inflation_on = true;
      runCellAndAggregate(runBusBearingOnlyMovingWithGps(seed, knob),
                          500.0, 1, 8, 90.0, 0.1,
                          Eigen::Vector2d(0.0, 10.0), c.agg);
    }
    cells.push_back(c);
  }

  for (const Cell& c : cells) {
    printCpaRow(c.scenario, c.sigma_h_deg, c.sigma_gps_m, c.agg);
  }
  SUCCEED();
}
