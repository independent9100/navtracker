#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

struct TruthSample {
  Timestamp time;
  std::uint64_t truth_id{0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
};

struct Scenario {
  std::vector<Measurement> measurements;
  std::vector<TruthSample> truth;
};

// Snapshot of a single track at a particular processing step.
struct TrackSnapshot {
  TrackId id;
  Eigen::Vector2d position;
};

// What the harness observed at one processing step.
struct ScenarioStep {
  Timestamp time;
  std::vector<Eigen::Vector2d> truth;
  std::vector<TrackSnapshot> tracks;
};

}  // namespace navtracker
