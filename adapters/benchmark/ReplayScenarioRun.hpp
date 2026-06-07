#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

// Two replays, single-seed (file-driven).
std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios();

}  // namespace benchmark
}  // namespace navtracker
