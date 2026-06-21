#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>

#include "core/benchmark/BenchSink.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/pmbm/PmbmTracker.hpp"
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
  // 2x2 position-block covariance of the kinematic state at snapshot
  // time. Zero matrix when the underlying track has no kinematic
  // covariance (state.size() < 2 already filters those out). Used by
  // bench-side NEES (see core/benchmark/Consistency.hpp).
  Eigen::Matrix2d pos_covariance{Eigen::Matrix2d::Zero()};
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
//
// Only Confirmed tracks populate BenchStep::tracks; tentative tracks
// are filtered to keep downstream RMSE from being polluted by tracks
// that may later be deleted.
BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink);

// MHT-pipeline overload. Drives the scenario through MhtTracker (which
// owns its own track-tree forest and emits Confirmed/Tentative tracks
// directly via tracks()). No TrackManager, no BenchSink lifecycle stream
// — MhtTracker doesn't have sink wiring today. BenchResult::sink_events
// stays empty; the kinematic snapshots in BenchStep::tracks are still
// the load-bearing input to the metrics, so OSPA / RMSE / continuity
// all work for like-for-like comparison with the JpdaStyle pipeline.
// Optional post-scan hook. Invoked after every processBatch with the
// scan's timestamp; sees the tracker's updated `tracks()` snapshot. Use
// to feed per-cycle observers (e.g. SensorBiasEstimator pair extraction).
using PostScanHook = std::function<void(const MhtTracker&, Timestamp)>;

BenchResult runBenchMht(const Scenario& scenario, MhtTracker& tracker,
                        const PostScanHook& post_scan_hook = {});

// PMBM-pipeline overload. Mirrors runBenchMht: drives a PmbmTracker
// over the scenario, snapshotting one Confirmed track per Bernoulli id
// (aggregated by PmbmTracker::tracks()) at every truth timestamp.
// Optional post-scan hook (same contract as the MHT path) drives the
// SensorBiasEstimator's pair extraction + observe pipeline when wired.
using PmbmPostScanHook = std::function<void(const pmbm::PmbmTracker&, Timestamp)>;
BenchResult runBenchPmbm(const Scenario& scenario,
                         pmbm::PmbmTracker& tracker,
                         const PmbmPostScanHook& post_scan_hook = {});

}  // namespace benchmark
}  // namespace navtracker
