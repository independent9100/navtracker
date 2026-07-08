#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

/**
 * R-BAD berthing replay battery (file-driven, single-seed each).
 *
 * Consumes the fixtures produced by tests/fixtures/rbad/generator/extract_rbad.py
 * (radar_plots.csv + reference_tracks.csv per arrival approach). The dataset is
 * an automotive-band mmWave FMCW berthing-aid dataset collected on a ferry
 * (Zenodo 16936465, CC-BY-4.0) — a NEW SENSOR CLASS, not marine X-band; nothing
 * here is a marine-radar number and no config is tuned to it (reality-check
 * arm).
 *
 * TWO facts shape the wiring:
 *  - NO ego pose exists in the dataset, so the buffer is replayed as a FIXED
 *    body frame: a single station "rbad" at ENU origin (0,0), plots projected
 *    from it (E=X starboard, N=Y forward). A nominal datum anchors the frame for
 *    Sweep's datum-aware models; there is no real geo-reference.
 *  - The "truth" loaded here is the authors' own reference-TRACKER output
 *    (Tracking_ID), NOT independent ground truth. Every metric scored against it
 *    is a cross-tracker CONSISTENCY measure (report vs_reference_tracker), never
 *    an accuracy claim. See docs/algorithms/evaluation-log.md.
 *
 * Fixture root overridable via RBAD_DIR (default tests/fixtures/rbad, resolved
 * from the process cwd = project root). Absent fixtures => empty Scenario =>
 * callers skip (bench and gtest).
 */
std::vector<std::unique_ptr<ScenarioRun>> defaultRbadScenarios();

}  // namespace benchmark
}  // namespace navtracker
