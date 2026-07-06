#include "core/benchmark/BenchRunner.hpp"

#include <chrono>

// BenchRunner — drives a Scenario through a Tracker, snapshotting full
// truth + track state at each truth timestamp.
//
// Math:
//   For each truth tick t_k, the runner first processes every
//   measurement m with m.time <= t_k via Tracker::processBatch(scan),
//   where `scan` = consecutive measurements sharing a timestamp. Then
//   snapshots:
//     - the truth samples at t_k (position + velocity from TruthSample);
//     - every Confirmed track from TrackManager::tracks(), pulling
//       (px, py) from state(0..1) and (vx, vy) from state(2..3).
//
// Assumptions:
//   - scenario.truth and scenario.measurements are both sorted by time.
//   - The estimator's state layout begins [px, py, vx, vy, ...]; this
//     holds for ConstantVelocity2D and any wrapper (UKF, EKF) that does
//     not reorder. For CoordinatedTurn / IMM with a different state
//     packing, velocity slicing falls back to zero when state.size() < 4
//     and is otherwise reported in the CV2D convention — known to be
//     unreliable for those configs; documented in
//     docs/baselines/README.md.
//   - Only Confirmed tracks contribute to BenchStep::tracks; tentative
//     tracks are excluded so downstream RMSE is not polluted by tracks
//     that may be deleted.
//
// Rationale:
//   The truth-tick-outer loop mirrors core/scenario/HarnessBatched.cpp::
//   runScenarioBatched so OSPA values from BenchResult are directly
//   comparable. Using processBatch (rather than process per-measurement)
//   is required for JPDA-style soft associators to function at all:
//   Tracker::process only consults AssociationResult::matches (hard
//   path), and JpdaAssociator only populates betas/beta_0 (soft path).
//   processBatch dispatches on which path is populated, so a single
//   loop here works for both GNN and JPDA configs. For single-
//   measurement scans (most synthetics), processBatch degenerates to
//   the same logic as process — no regression for hard associators.
//
// Improve next:
//   - Add a kinematic-projection accessor on Track (or per-estimator
//     extractor) so non-CV2D state layouts produce correct velocities.
//   - Optional flag to include tentative tracks for full parity with
//     runScenario when needed for like-for-like comparison.

namespace navtracker {
namespace benchmark {
namespace {

struct TruthGroup {
  Timestamp time;
  std::vector<TruthStateSnapshot> snapshots;
};

// Time one processBatch call and record it into the result's per-scan
// latency arrays. Templated over the tracker type (all three trackers
// share the `void processBatch(const std::vector<Measurement>&)` signature)
// so the timing site is identical for the JPDA, MHT and PMBM paths. Only
// processBatch is timed — the post-scan bias hook is deliberately outside
// the window (it is harness bookkeeping, not per-scan tracker cost).
template <typename TrackerT>
void timedProcessBatch(TrackerT& tracker,
                       const std::vector<Measurement>& scan,
                       const Timestamp& scan_t, BenchResult& result) {
  const auto t0 = std::chrono::steady_clock::now();
  tracker.processBatch(scan);
  const auto t1 = std::chrono::steady_clock::now();
  result.scan_process_seconds.push_back(
      std::chrono::duration<double>(t1 - t0).count());
  result.scan_time_sec.push_back(scan_t.seconds());
}

// RAII guard that ensures the manager's track sink is cleared even if a
// downstream call (Tracker::process, etc.) throws. Without this the
// manager would retain a dangling pointer to the stack-allocated
// BenchSink on the exception path.
struct SinkGuard {
  TrackManager& m;
  ~SinkGuard() { m.setTrackSink(nullptr); }
};

// precondition: `truth` is sorted by non-decreasing time. We only open a
// new bucket when `t.time` differs from the previous one, so out-of-order
// input (e.g. t=1, t=2, t=1) would silently produce duplicate groups.
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
    const Eigen::Vector2d vel = tr.state.size() >= 4
        ? Eigen::Vector2d(tr.state(2), tr.state(3))  // CV2D layout [px,py,vx,vy]
        : Eigen::Vector2d::Zero();
    TrackStateSnapshot snap{tr.id, pos, vel};
    if (tr.covariance.rows() >= 2 && tr.covariance.cols() >= 2) {
      snap.pos_covariance = tr.covariance.topLeftCorner<2, 2>();
    }
    step.tracks.push_back(std::move(snap));
  }
  return step;
}

}  // namespace

BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink) {
  manager.setTrackSink(&sink);
  SinkGuard guard{manager};

  BenchResult result;
  const auto truth_groups = groupTruth(scenario.truth);

  // Walk truth ticks in order. For each tick, process every measurement
  // whose timestamp is <= tick — grouped by shared timestamp into "scans"
  // — via Tracker::processBatch, then snapshot truth + Confirmed tracks.
  // Batching is required for JPDA; for single-measurement scans
  // (most synthetics) processBatch degenerates to the same logic as
  // process, so GNN configs are unaffected.
  const auto& meas = scenario.measurements;
  const auto flushScansUpTo = [&](const Timestamp& upto,
                                  std::size_t& mi) {
    while (mi < meas.size() && !(upto < meas[mi].time)) {
      const Timestamp scan_t = meas[mi].time;
      std::vector<Measurement> scan;
      while (mi < meas.size() && meas[mi].time == scan_t) {
        scan.push_back(meas[mi]);
        ++mi;
      }
      timedProcessBatch(tracker, scan, scan_t, result);
    }
  };

  std::size_t mi = 0;
  for (const auto& g : truth_groups) {
    flushScansUpTo(g.time, mi);
    result.steps.push_back(snapshotAt(manager, g));
  }

  // Drain any measurements after the last truth tick so the sink/lifecycle
  // stream remains complete, even though no further snapshot is taken.
  while (mi < meas.size()) {
    const Timestamp scan_t = meas[mi].time;
    std::vector<Measurement> scan;
    while (mi < meas.size() && meas[mi].time == scan_t) {
      scan.push_back(meas[mi]);
      ++mi;
    }
    timedProcessBatch(tracker, scan, scan_t, result);
  }

  result.sink_events = sink.events();
  return result;
}

namespace {

// Snapshot all Confirmed tracks emitted by MhtTracker. Identical layout
// rules to snapshotAt (CV2D state slice). MhtTracker reports one Track
// per surviving tree from its solveGlobalHypothesis-selected leaf, so
// tracks() is already the per-scan canonical view.
BenchStep snapshotAtMht(const MhtTracker& tracker, const TruthGroup& g) {
  BenchStep step;
  step.time = g.time;
  step.truth = g.snapshots;
  for (const Track& tr : tracker.tracks()) {
    if (tr.status != TrackStatus::Confirmed) continue;
    if (tr.state.size() < 2) continue;
    Eigen::Vector2d pos(tr.state(0), tr.state(1));
    const Eigen::Vector2d vel = tr.state.size() >= 4
        ? Eigen::Vector2d(tr.state(2), tr.state(3))
        : Eigen::Vector2d::Zero();
    TrackStateSnapshot snap{tr.id, pos, vel};
    if (tr.covariance.rows() >= 2 && tr.covariance.cols() >= 2) {
      snap.pos_covariance = tr.covariance.topLeftCorner<2, 2>();
    }
    step.tracks.push_back(std::move(snap));
  }
  return step;
}

}  // namespace

BenchResult runBenchMht(const Scenario& scenario, MhtTracker& tracker,
                        const PostScanHook& post_scan_hook) {
  BenchResult result;
  const auto truth_groups = groupTruth(scenario.truth);
  const auto& meas = scenario.measurements;
  const auto flushScansUpTo = [&](const Timestamp& upto, std::size_t& mi) {
    while (mi < meas.size() && !(upto < meas[mi].time)) {
      const Timestamp scan_t = meas[mi].time;
      std::vector<Measurement> scan;
      while (mi < meas.size() && meas[mi].time == scan_t) {
        scan.push_back(meas[mi]);
        ++mi;
      }
      timedProcessBatch(tracker, scan, scan_t, result);
      if (post_scan_hook) post_scan_hook(tracker, scan_t);
    }
  };

  std::size_t mi = 0;
  for (const auto& g : truth_groups) {
    flushScansUpTo(g.time, mi);
    result.steps.push_back(snapshotAtMht(tracker, g));
  }
  while (mi < meas.size()) {
    const Timestamp scan_t = meas[mi].time;
    std::vector<Measurement> scan;
    while (mi < meas.size() && meas[mi].time == scan_t) {
      scan.push_back(meas[mi]);
      ++mi;
    }
    timedProcessBatch(tracker, scan, scan_t, result);
    if (post_scan_hook) post_scan_hook(tracker, scan_t);
  }
  // sink_events intentionally empty: MhtTracker has no sink today.
  return result;
}

namespace {

// PMBM snapshot: identical layout to MHT, but PmbmTracker::tracks()
// already aggregates the MBM into one Track per Bernoulli id with
// status = Confirmed when P(exists) ≥ confirm_threshold.
BenchStep snapshotAtPmbm(const pmbm::PmbmTracker& tracker,
                         const TruthGroup& g) {
  BenchStep step;
  step.time = g.time;
  step.truth = g.snapshots;
  for (const Track& tr : tracker.tracks()) {
    if (tr.status != TrackStatus::Confirmed) continue;
    if (tr.state.size() < 2) continue;
    Eigen::Vector2d pos(tr.state(0), tr.state(1));
    const Eigen::Vector2d vel = tr.state.size() >= 4
        ? Eigen::Vector2d(tr.state(2), tr.state(3))
        : Eigen::Vector2d::Zero();
    TrackStateSnapshot snap{tr.id, pos, vel};
    if (tr.covariance.rows() >= 2 && tr.covariance.cols() >= 2) {
      snap.pos_covariance = tr.covariance.topLeftCorner<2, 2>();
    }
    step.tracks.push_back(std::move(snap));
  }
  return step;
}

}  // namespace

BenchResult runBenchPmbm(const Scenario& scenario,
                         pmbm::PmbmTracker& tracker,
                         const PmbmPostScanHook& post_scan_hook) {
  BenchResult result;
  const auto truth_groups = groupTruth(scenario.truth);
  const auto& meas = scenario.measurements;
  const auto flushScansUpTo = [&](const Timestamp& upto, std::size_t& mi) {
    while (mi < meas.size() && !(upto < meas[mi].time)) {
      const Timestamp scan_t = meas[mi].time;
      std::vector<Measurement> scan;
      while (mi < meas.size() && meas[mi].time == scan_t) {
        scan.push_back(meas[mi]);
        ++mi;
      }
      timedProcessBatch(tracker, scan, scan_t, result);
      if (post_scan_hook) post_scan_hook(tracker, scan_t);
    }
  };

  std::size_t mi = 0;
  for (const auto& g : truth_groups) {
    flushScansUpTo(g.time, mi);
    result.steps.push_back(snapshotAtPmbm(tracker, g));
  }
  while (mi < meas.size()) {
    const Timestamp scan_t = meas[mi].time;
    std::vector<Measurement> scan;
    while (mi < meas.size() && meas[mi].time == scan_t) {
      scan.push_back(meas[mi]);
      ++mi;
    }
    timedProcessBatch(tracker, scan, scan_t, result);
    if (post_scan_hook) post_scan_hook(tracker, scan_t);
  }
  return result;
}

}  // namespace benchmark
}  // namespace navtracker
