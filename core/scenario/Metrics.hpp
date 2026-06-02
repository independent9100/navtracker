#pragma once

#include <vector>

#include "core/scenario/Harness.hpp"
#include "core/scenario/Truth.hpp"

namespace navtracker {

// Count truth-target ID switches across a sequence of harness steps.
// For each truth index i, the metric finds the nearest track within `cutoff`
// at each step and counts transitions where the assigned TrackId changes
// from one non-zero value to a different non-zero value.
int countIdSwitches(const std::vector<ScenarioStep>& steps, double cutoff);

struct PerWindowOspa {
  double mean{0.0};                  // mean of per-window means
  double stddev{0.0};                // sample stddev (N-1) across windows; 0 if <2 windows
  std::vector<double> per_window;    // per-window mean OSPA (skipping empty windows)
};

// Group ScenarioResult.steps by floor((step.time - t0) / window_dt_s). For
// each window, average the OSPA values from result.ospa_per_step that fall
// in that window. Empty windows are skipped. The overall `mean` is the mean
// of per-window means (so each window contributes equally regardless of how
// many measurements fired during it).
PerWindowOspa computePerWindowOspa(const ScenarioResult& result,
                                   Timestamp t0,
                                   double window_dt_s);

}  // namespace navtracker
