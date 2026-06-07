#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {
namespace benchmark {

// Factory for a fresh estimator instance. Each call must return a new object
// (do not share mutable state between runs in a sweep).
using EstimatorFactory = std::function<std::shared_ptr<IEstimator>()>;

// Factory for a fresh associator instance. Each call must return a new object.
using AssociatorFactory = std::function<std::shared_ptr<IDataAssociator>()>;

// A single labelled (estimator, associator) baseline configuration. The
// label is the canonical identifier emitted by Sweep into result CSVs;
// the factories construct the components on demand per (scenario × seed).
struct Config {
  std::string label;
  EstimatorFactory build_estimator;
  AssociatorFactory build_associator;
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
