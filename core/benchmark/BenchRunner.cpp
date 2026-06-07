#include "core/benchmark/BenchRunner.hpp"

#include <utility>

namespace navtracker {
namespace benchmark {
namespace {

struct TruthGroup {
  Timestamp time;
  std::vector<TruthStateSnapshot> snapshots;
};

std::vector<TruthGroup> groupTruth(const std::vector<TruthSample>& truth) {
  std::vector<TruthGroup> out;
  for (const auto& t : truth) {
    if (out.empty() || out.back().time != t.time) {
      out.push_back({t.time, {}});
    }
    out.back().snapshots.push_back({t.truth_id, t.position, t.velocity});
  }
  return out;
}

// Snapshot all currently Confirmed tracks. State layout assumed is
// [px, py, vx, vy] in ENU (constant-velocity 2D), matching the canonical
// EkfEstimator + ConstantVelocity2D stack. Tracks with fewer than 4 state
// entries are emitted with zero velocity rather than dropped.
BenchStep snapshotAt(const TrackManager& manager, const TruthGroup& g) {
  BenchStep step;
  step.time = g.time;
  step.truth = g.snapshots;
  for (const Track& tr : manager.tracks()) {
    if (tr.status != TrackStatus::Confirmed) continue;
    if (tr.state.size() < 2) continue;
    Eigen::Vector2d pos(tr.state(0), tr.state(1));
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    if (tr.state.size() >= 4) {
      vel = Eigen::Vector2d(tr.state(2), tr.state(3));
    }
    step.tracks.push_back(TrackStateSnapshot{tr.id, pos, vel});
  }
  return step;
}

}  // namespace

BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink) {
  manager.setTrackSink(&sink);

  BenchResult result;
  const auto truth_groups = groupTruth(scenario.truth);

  // Mirror runScenario: walk truth ticks in order; for each tick, process every
  // measurement whose timestamp is <= tick, then snapshot truth + Confirmed
  // tracks for that tick.
  std::size_t mi = 0;
  for (const auto& g : truth_groups) {
    while (mi < scenario.measurements.size() &&
           !(g.time < scenario.measurements[mi].time)) {
      tracker.process(scenario.measurements[mi]);
      ++mi;
    }
    result.steps.push_back(snapshotAt(manager, g));
  }

  // Drain any measurements after the last truth tick so the sink/lifecycle
  // stream remains complete, even though no further snapshot is taken.
  while (mi < scenario.measurements.size()) {
    tracker.process(scenario.measurements[mi]);
    ++mi;
  }

  manager.setTrackSink(nullptr);
  result.sink_events = sink.events();
  return result;
}

}  // namespace benchmark
}  // namespace navtracker
