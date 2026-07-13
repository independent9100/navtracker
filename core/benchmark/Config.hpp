#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/land/CoastlineGeometry.hpp"  // CoastlinePriorParams
#include "core/pipeline/MhtTracker.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Factory for a fresh estimator instance. Each call must return a new object
 * (do not share mutable state between runs in a sweep).
 */
using EstimatorFactory = std::function<std::shared_ptr<IEstimator>()>;

/** Factory for a fresh associator instance. Each call must return a new object. */
using AssociatorFactory = std::function<std::shared_ptr<IDataAssociator>()>;

/**
 * Per-sensor associator factory: the scenario's per-sensor detection
 * model is forwarded so the associator can use (P_D, λ_C) lookup per
 * measurement instead of a single scalar. Used for the per-sensor JPDA
 * ablation (backlog item 8). The shared_ptr's lifetime is owned by the
 * caller of the sweep (Sweep.cpp).
 */
using PerSensorAssociatorFactory =
    std::function<std::shared_ptr<IDataAssociator>(
        const std::shared_ptr<ISensorDetectionModel>&)>;

/**
 * Which tracker pipeline drives this config:
 *   - JpdaStyle: `Tracker` + `TrackManager` (GNN/JPDA/JIPDA — i.e. per-
 *     scan hard or soft association with an external M-of-N manager).
 *   - Mht:       `MhtTracker` (track-tree hypothesis MHT with internal
 *     branching, K=1 Hungarian global hypothesis, and M-of-N
 *     confirmation inside the tracker). Does not use TrackManager; the
 *     associator factory is ignored.
 */
enum class TrackerKind {
  JpdaStyle,
  Mht,
  Pmbm,
};

/**
 * A single labelled baseline configuration. The label is the canonical
 * identifier emitted by Sweep into result CSVs; the factories construct
 * the components on demand per (scenario × seed).
 */
struct Config {
  std::string label;
  EstimatorFactory build_estimator;
  AssociatorFactory build_associator;          // ignored if tracker_kind == Mht
  TrackerKind tracker_kind{TrackerKind::JpdaStyle};
  // Optional MHT configuration override. Used only when
  // tracker_kind == Mht; nullptr → default-constructed MhtTracker::Config.
  std::function<MhtTracker::Config()> mht_config{};
  // Backlog item 5 ablation: wrap the scenario's fixed detection table
  // in a ClutterMapSensorDetectionModel (spatially-varying λ_C learned
  // online from unassociated returns). Applies to Mht and Pmbm paths;
  // no effect when the scenario declares no detection table.
  bool use_clutter_map{false};
  // When set and the scenario declares a per-sensor detection table,
  // this takes precedence over `build_associator` — the scenario's
  // model is passed into the associator constructor (per-sensor JPDA,
  // backlog item 8). Falls back to `build_associator` when no table is
  // present. Single-hypothesis path only.
  PerSensorAssociatorFactory build_associator_per_sensor{};
  // Optional inter-sensor registration bias estimator factory (item 9).
  // When set, Sweep constructs a fresh estimator per cell, wires it as
  // the tracker's bias provider, and runs the post-scan pair extractor
  // to feed it observations. Null = no bias estimation (legacy).
  std::function<std::shared_ptr<SensorBiasEstimator>()>
      build_sensor_bias_estimator{};
  // Optional PMBM configuration override. Used only when
  // tracker_kind == Pmbm; nullptr → default-constructed
  // pmbm::PmbmTracker::Config (with measurement-driven birth ON).
  std::function<pmbm::PmbmTracker::Config()> pmbm_config{};
  // Optional explicit birth-model factory for the PMBM path. When null
  // and pmbm_config.measurement_driven_birth is true, no separate
  // predict-time birth fires (the measurement-driven path covers
  // birth). When pmbm_config.measurement_driven_birth is false, a
  // user-supplied factory is required for any new Bernoulli to spawn.
  std::function<pmbm::PmbmTracker::BirthModelFn()> pmbm_birth_model{};
  // Task 4: build a DeclaredSensorActivity coverage model from the
  // scenario's detection table + declared cadence and wire it into the
  // PMBM tracker. When true, Sweep.cpp constructs a DeclaredSensorActivity
  // from the per-sensor detection entries and calls
  // tracker.setSensorActivity(). Only meaningful when tracker_kind == Pmbm
  // and the scenario declares a detection_table.
  bool use_sensor_activity_model{false};
  // Task 6: build a CoastlineModel from the scenario's GeoJSON coastline
  // fixture and wire it into the PMBM tracker via setLandModel(). When
  // true, Sweep.cpp loads the GeoJSON (if the file exists), constructs
  // CoastlineModel with the scenario's ENU datum, and calls
  // tracker.setLandModel(). Only meaningful when tracker_kind == Pmbm,
  // cfg.use_land_model == true, and the scenario declares a
  // coastline_geojson_path. Scenarios without a coastline fixture silently
  // skip wiring (land model stays null → bit-identical to no-land behaviour).
  bool use_land_model{false};
  // Stage 1 static-obstacle branch (ADR 0002): when true, Sweep builds a
  // StaticObstacleModel from the scenario's synthetic obstacles (preferred)
  // or its static_obstacles_geojson_path, and calls
  // tracker.setStaticObstacleModel(). Only meaningful when tracker_kind ==
  // Pmbm and the PMBM config sets use_static_obstacle_model. Scenarios with no
  // obstacles silently skip wiring (model stays null → bit-identical).
  bool use_static_obstacle_model{false};
  // Stage 1b live occupancy layer: when true, Sweep builds a LiveOccupancyModel
  // anchored at the scenario datum and wires it BOTH as the birth-suppression
  // model (setStaticObstacleModel) and as the per-scan occupancy feed
  // (setLiveOccupancyFeed). Only meaningful for tracker_kind == Pmbm with the
  // PMBM config setting use_static_obstacle_model (to consult the seam). Mutually
  // exclusive with use_static_obstacle_model (both drive setStaticObstacleModel).
  // Scenarios with no datum silently skip wiring (model stays null → identical).
  bool use_live_occupancy_model{false};
  // Optional override of the LiveOccupancyModel grid/classifier parameters. When
  // unset, Sweep uses LiveOccupancyParams{} defaults. Used by sensitivity-probe
  // configs (e.g. imm_cv_ct_pmbm_occupancy_sensitive) to test whether coarser
  // cells / a lower persistence bar rescue structure classification on
  // realistic (churn / real-radar) detection rates.
  std::optional<LiveOccupancyParams> live_occupancy_params;
  // Detector mode: when true, Sweep sets the LiveOccupancyModel's
  // expected_clutter_per_m2 to the PMBM config's clutter_intensity, activating
  // the clutter-adaptive persistence bar (uniform clutter rejected relative to
  // its own density). Requires use_live_occupancy_model.
  bool occupancy_adaptive_clutter_bar{false};
  // Optional per-config shoreline clutter-ramp half-widths for the GeoJSON
  // CoastlineModel Sweep wires (Task 6). Unset ⇒ the shipped 50/50 default
  // (byte-identical to master). Cl-4 adoption (2026-07-12, ADR-0003): the
  // deployable config imm_cv_ct_pmbm_coverage_land_ivgate sets
  // offshore_halfwidth_m = 25 to revive near-shore births (6–42 m band) while
  // keeping the inland plateau at 50 m. The PMBM_OFFSHORE/INLAND_HALFWIDTH_M
  // env vars still override this as research levers. No other config sets it.
  std::optional<CoastlinePriorParams> coastline_prior_params;
};

/**
 * Returns the five baseline configurations in fixed order:
 *   1. ekf_cv_gnn
 *   2. ekf_cv_jpda
 *   3. ukf_cv_gnn
 *   4. ukf_ct_gnn
 *   5. imm_cv_ct_jpda
 *
 * Constructor arguments match those in tests/scenario/test_crossing.cpp,
 * test_jpda_comparison.cpp, and test_filter_comparison.cpp, so the
 * baseline reflects what the repo already considers reasonable defaults.
 */
std::vector<Config> defaultConfigs();

}  // namespace benchmark
}  // namespace navtracker
