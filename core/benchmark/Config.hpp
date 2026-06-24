#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {
namespace benchmark {

// Factory for a fresh estimator instance. Each call must return a new object
// (do not share mutable state between runs in a sweep).
using EstimatorFactory = std::function<std::shared_ptr<IEstimator>()>;

// Factory for a fresh associator instance. Each call must return a new object.
using AssociatorFactory = std::function<std::shared_ptr<IDataAssociator>()>;

// Per-sensor associator factory: the scenario's per-sensor detection
// model is forwarded so the associator can use (P_D, λ_C) lookup per
// measurement instead of a single scalar. Used for the per-sensor JPDA
// ablation (backlog item 8). The shared_ptr's lifetime is owned by the
// caller of the sweep (Sweep.cpp).
using PerSensorAssociatorFactory =
    std::function<std::shared_ptr<IDataAssociator>(
        const std::shared_ptr<ISensorDetectionModel>&)>;

// Which tracker pipeline drives this config:
//   - JpdaStyle: `Tracker` + `TrackManager` (GNN/JPDA/JIPDA — i.e. per-
//     scan hard or soft association with an external M-of-N manager).
//   - Mht:       `MhtTracker` (track-tree hypothesis MHT with internal
//     branching, K=1 Hungarian global hypothesis, and M-of-N
//     confirmation inside the tracker). Does not use TrackManager; the
//     associator factory is ignored.
enum class TrackerKind {
  JpdaStyle,
  Mht,
  Pmbm,
};

// A single labelled baseline configuration. The label is the canonical
// identifier emitted by Sweep into result CSVs; the factories construct
// the components on demand per (scenario × seed).
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
};

// Returns the five baseline configurations in fixed order:
//   1. ekf_cv_gnn
//   2. ekf_cv_jpda
//   3. ukf_cv_gnn
//   4. ukf_ct_gnn
//   5. imm_cv_ct_jpda
//
// Constructor arguments match those in tests/scenario/test_crossing.cpp,
// test_jpda_comparison.cpp, and test_filter_comparison.cpp, so the
// baseline reflects what the repo already considers reasonable defaults.
std::vector<Config> defaultConfigs();

}  // namespace benchmark
}  // namespace navtracker
