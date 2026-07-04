#pragma once

#include "core/pmbm/PmbmTracker.hpp"
#include "core/scenario/Harness.hpp"

namespace navtracker {

/**
 * Same contract as runScenarioBatchedMht, but drives a PmbmTracker.
 * Aggregates the MBM into a stable single-Track view per Bernoulli id
 * each tick (PmbmTracker::tracks()) for OSPA scoring against truth.
 */
ScenarioResult runScenarioBatchedPmbm(const Scenario& scenario,
                                      pmbm::PmbmTracker& tracker,
                                      double ospa_cutoff);

}  // namespace navtracker
