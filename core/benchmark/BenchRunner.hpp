#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/benchmark/BenchSink.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {
namespace benchmark {

// Per-track snapshot at a single evaluation timestamp.
struct TrackStateSnapshot {
  TrackId id;
  Eigen::Vector2d position;  // ENU metres
  Eigen::Vector2d velocity;  // ENU m/s
};

// Per truth snapshot at a single evaluation timestamp.
struct TruthStateSnapshot {
  std::uint64_t truth_id;
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
};

// One evaluation slice: all truth + all confirmed tracks at the same time.
struct BenchStep {
  Timestamp time;
  std::vector<TruthStateSnapshot> truth;
  std::vector<TrackStateSnapshot> tracks;
};

struct BenchResult {
  std::vector<BenchStep> steps;
  std::vector<BenchSink::Event> sink_events;  // full lifecycle stream
};

// Drive the scenario through the tracker. Measurements are injected in
// timestamp order. After each truth timestamp is reached, snapshot truth
// and confirmed tracks. The supplied BenchSink is registered with the
// manager for the duration of the run.
BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink);

}  // namespace benchmark
}  // namespace navtracker
