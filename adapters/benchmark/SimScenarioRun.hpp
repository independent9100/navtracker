#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Returns the 7 synthetic baseline scenarios. Each is multi-seed with
 * seed_count = 10.
 */
std::vector<std::unique_ptr<ScenarioRun>> defaultSimScenarios();

}  // namespace benchmark
}  // namespace navtracker
