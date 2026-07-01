#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/StaticObstacle.hpp"
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

  // Optional path to a GeoJSON coastline file for land-prior wiring.
  // Non-empty signals that this scenario has a known coastline fixture;
  // Sweep.cpp checks this (+ file existence) before building a
  // CoastlineModel. Relative to the process cwd (project root).
  std::string coastline_geojson_path;
  // Optional path to a GeoJSON file of charted static obstacles (ADR 0002
  // Stage 1). Non-empty signals a chart fixture; Sweep checks this + file
  // existence before building a StaticObstacleModel. Relative to the cwd.
  std::string static_obstacles_geojson_path;
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

  // Optional in-memory coastline for synthetic scenarios. Default = none, so
  // every existing scenario is untouched. When present AND config.use_land_model
  // AND Scenario.datum is set, Sweep builds a CoastlineModel from this geometry
  // (in preference to coastline_geojson_path) so the synthetic land mask that
  // seeds the shore clutter also drives the land model. Real-data replay
  // scenarios leave this null and keep using coastline_geojson_path.
  virtual std::optional<CoastlineGeometry> syntheticCoastline() const {
    return std::nullopt;
  }

  // Optional in-memory charted obstacles for synthetic scenarios. Default =
  // none (every existing scenario untouched). When present AND
  // config.use_static_obstacle_model AND Scenario.datum is set, Sweep builds a
  // StaticObstacleModel from these (in preference to
  // static_obstacles_geojson_path).
  virtual std::optional<std::vector<StaticObstacle>> syntheticObstacles() const {
    return std::nullopt;
  }
};

}  // namespace benchmark
}  // namespace navtracker
