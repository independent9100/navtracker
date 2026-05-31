#pragma once

#include <vector>

#include "core/scenario/Truth.hpp"

namespace navtracker {

// Count truth-target ID switches across a sequence of harness steps.
// For each truth index i, the metric finds the nearest track within `cutoff`
// at each step and counts transitions where the assigned TrackId changes
// from one non-zero value to a different non-zero value.
int countIdSwitches(const std::vector<ScenarioStep>& steps, double cutoff);

}  // namespace navtracker
