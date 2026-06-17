#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/scenario/Truth.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {
namespace benchmark {

// One per-sensor detection-model entry a scenario can declare about its
// environment: P_D, λ_C in the sensor's measurement-space units, and an
// optional coverage radius / azimuth sector (inside DetectionParams).
// See ISensorDetectionModel for unit rules.
//
// `source_id` (optional, last so existing aggregate initialisers stay
// valid): non-empty entries calibrate one physical sensor unit by
// Measurement::source_id — two units sharing a SensorKind (EO and IR
// cameras are both SensorKind::EoIr) get independent (P_D, λ_C). Empty
// = kind-wide entry; sources without an exact match fall back to it.
struct SensorDetectionEntry {
  SensorKind sensor;
  MeasurementModel model;
  DetectionParams params;
  std::string source_id;
};

struct ScenarioDescriptor {
  std::string label;
  bool is_multi_seed{false};
  std::uint32_t seed_count{1};

  // Spatial clutter density (false alarms per m² per scan) for THIS
  // scenario's environment. Used by the MHT tracker's branch score
  // (log P_D + logLik − log λ_C) so confirmation/deletion match the
  // actual clutter. Synthetic scenarios are effectively clutter-free
  // (default 1e-4); real cluttered data declares realistic values.
  // This is a property of the sensor + environment, not a tuning knob —
  // a single global constant cannot serve both clean synthetic and
  // cluttered real scenes (verified: the value that suppresses real
  // false-tracks erases synthetic tracks).
  //
  // LEGACY single-scalar path: only consulted when `detection_table`
  // is empty. A single scalar cannot be dimensionally correct across
  // mixed sensors (m⁻² vs rad⁻¹) — multi-sensor scenarios should
  // declare the table instead.
  double clutter_density{1e-4};

  // Per-sensor detection table for multi-sensor scenarios. When
  // non-empty, the sweep injects a FixedSensorDetectionModel built from
  // these entries (tracker-config defaults for unlisted sensors) and
  // `clutter_density` above is ignored.
  std::vector<SensorDetectionEntry> detection_table;
};

// Port: produces a Scenario (measurements + truth) for a given seed.
// Replays ignore the seed; synthetics use it for noise realisation.
class ScenarioRun {
 public:
  virtual ~ScenarioRun() = default;
  virtual ScenarioDescriptor descriptor() const = 0;
  virtual Scenario generate(std::uint64_t seed) = 0;

  // Optional hook: seed per-scenario knowledge into the sensor-bias
  // estimator before any measurements are processed. Used by replay
  // scenarios with known offline calibration (e.g. AutoFerry env-2
  // cameras carry a documented 5-7° EO/IR bearing offset). Default is
  // no-op so synthetic and unprepared scenarios behave exactly as
  // before. Called once per (config × scenario × seed) cell, just
  // after the estimator is constructed.
  virtual void seedSensorBiasEstimator(SensorBiasEstimator& /*est*/) const {}
};

}  // namespace benchmark
}  // namespace navtracker
