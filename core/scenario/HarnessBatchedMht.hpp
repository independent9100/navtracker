#pragma once

#include "core/pipeline/MhtTracker.hpp"
#include "core/scenario/Harness.hpp"

namespace navtracker {

// Same contract as runScenarioBatched, but drives MhtTracker instead of
// the standard Tracker.
ScenarioResult runScenarioBatchedMht(const Scenario& scenario,
                                     MhtTracker& tracker,
                                     double ospa_cutoff);

}  // namespace navtracker
