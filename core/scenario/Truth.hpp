#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
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
  // ENU datum used when projecting measurements from geodetic to ENU.
  // Set by replay scenarios (e.g. PhilosScenarioRun) so that downstream
  // bench wiring (e.g. CoastlineModel) can query the same frame.
  // Empty for synthetic scenarios whose measurements are generated
  // directly in ENU without a geodetic origin.
  std::optional<geo::Datum> datum;
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
