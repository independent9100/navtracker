#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Multi-sensor simulation scenario battery (file-driven, single-seed each).
 *
 * These consume the seeded Python-generated fixtures under
 * tests/fixtures/sim_multisensor/<label>_s<seed>/ (ownship / ais / radar_plots
 * / camera_bearings / truth CSVs). Unlike the real-data replays (philos, HAXR)
 * the truth here is INDEPENDENT of every sensor by construction — it is the
 * generator's ground truth, not an AIS projection — so scoring a radar+AIS
 * fusion arm against it yields an HONEST accuracy number, the first controlled
 * fusion-vs-single-sensor gate this project has (see the design ticket
 * docs/superpowers/plans/2026-07-06-multisensor-sim-gates-ticket.md).
 *
 * Mirrors PhilosScenarioRun (moving own-ship, body-frame radar + absolute AIS +
 * bearing-only camera). The fixture directory root is overridable via the
 * SIMMS_DIR env var (default tests/fixtures/sim_multisensor, resolved from the
 * process cwd = project root when the bench is launched as documented). Absent
 * fixtures => empty Scenario => callers skip.
 */
std::vector<std::unique_ptr<ScenarioRun>> defaultSimMultisensorScenarios();

}  // namespace benchmark
}  // namespace navtracker
