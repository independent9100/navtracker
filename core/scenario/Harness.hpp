#pragma once

#include <vector>

#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

struct ScenarioResult {
  std::vector<double> ospa_per_step;
  double mean_ospa{0.0};
};

ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff);

}  // namespace navtracker
