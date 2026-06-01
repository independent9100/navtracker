#pragma once

#include "core/scenario/Harness.hpp"

namespace navtracker {

// Same contract as runScenario, but groups consecutive same-timestamp
// measurements into a single batch and calls Tracker::processBatch. Used
// for JPDA / MHT scenarios where the joint scan matters.
ScenarioResult runScenarioBatched(const Scenario& scenario,
                                  Tracker& tracker,
                                  const TrackManager& manager,
                                  double ospa_cutoff);

}  // namespace navtracker
