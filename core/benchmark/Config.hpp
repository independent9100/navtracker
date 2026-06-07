#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core/pipeline/MhtTracker.hpp"
#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {
namespace benchmark {

// Factory for a fresh estimator instance. Each call must return a new object
// (do not share mutable state between runs in a sweep).
using EstimatorFactory = std::function<std::shared_ptr<IEstimator>()>;

// Factory for a fresh associator instance. Each call must return a new object.
using AssociatorFactory = std::function<std::shared_ptr<IDataAssociator>()>;

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
