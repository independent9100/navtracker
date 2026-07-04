#pragma once

#include <vector>

#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

/**
 * Output of a scenario run: the OSPA score at every processing step, the
 * mean OSPA over the run, and the per-step observation log (`steps`) used
 * by the downstream metrics (id-switch counting, windowed OSPA).
 */
struct ScenarioResult {
  std::vector<double> ospa_per_step;
  double mean_ospa{0.0};
  std::vector<ScenarioStep> steps;
};

/**
 * Drive `tracker` through `scenario` one measurement at a time, scoring the
 * tracks against truth with OSPA (cutoff `ospa_cutoff`) at each step. This is
 * the sequential scenario/replay harness; `manager` supplies the current
 * track set for scoring. See `runScenarioBatched` for the joint-scan variant.
 */
ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff);

}  // namespace navtracker
