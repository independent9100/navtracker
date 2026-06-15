#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

// Two replays, single-seed (file-driven): philos + haxr.
std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios();

// AutoFerry milliAmpere benchmark scenarios (single-seed, file-driven),
// one ScenarioRun per published scenario folder under data/autoferry/.
// Real RTK ground truth + heterogeneous active sensors (radar+lidar).
// See adapters/replay/AutoferryJsonReplay.hpp for format/frame details.
std::vector<std::unique_ptr<ScenarioRun>> defaultAutoferryScenarios();

// AutoFerry scenarios with synthetic truth-derived AIS anchor injected
// (item 9 option 1). Identical labels as defaultAutoferryScenarios but
// each scenario contains additional Position2D Ais measurements. Used
// by the imm_cv_ct_mht_biascal_anchored bench config.
std::vector<std::unique_ptr<ScenarioRun>> defaultAutoferryScenariosAnchored();

}  // namespace benchmark
}  // namespace navtracker
