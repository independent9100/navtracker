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

/**
 * The Imazu 22 — the canonical ship-encounter benchmark (head-on / crossing /
 * overtaking singles up to 3-target combinations) as fixed-geometry sim
 * scenarios imazu_01..imazu_22. Same fixture root (SIMMS_DIR) and the SAME
 * scenario-run class as the sim_ms battery — only the labels differ and the
 * arm is radar+AIS (no camera). The geometry is published/explicit (not
 * trafficgen-sampled); it stresses identity stability through close crossing
 * passes. See docs/superpowers/plans/2026-07-08-imazu22-ticket.md.
 */
std::vector<std::unique_ptr<ScenarioRun>> defaultImazuScenarios();

}  // namespace benchmark
}  // namespace navtracker
