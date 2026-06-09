#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "core/tracking/TrackTree.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {

// Track-oriented MHT (TOMHT). Per scan:
//   1. Branch each tree's leaves on every gated measurement + missed-detection.
//   2. Prune K_local-worst leaves per tree.
//   3. Merge near-duplicate leaves within each tree by Bhattacharyya distance.
//   4. Prune N-scan-old branches (trunk merging).
//   5. Spawn new trees for measurements that gated to no tree.
//   6. Drop trees whose best-leaf score falls below score_delete_threshold.
//   7. Solve the global hypothesis (Hungarian K=1): pick one leaf per tree
//      so that no scan measurement is consumed by more than one tree.
//   8. Apply M-of-N confirmation gate: emit only trees with at least
//      `confirm_hits_needed` hits in the last `confirm_hits_window` nodes.
//   9. Materialize one Track per surviving tree (state from selected leaf,
//      id from tree's externalId) into `tracks_` for downstream consumers.
//
// Track identity is stable: each tree gets a fresh monotonic id at spawn
// time; ids are never reused across the MhtTracker's lifetime. The
// Tentative/Confirmed status of an emitted Track reflects the M-of-N gate.
//
// References:
//   - Blackman & Popoli (1999), Modern Tracking Systems ch. 16
//   - Blackman (2004), IEEE AES Mag 19(1) §III–§V
//   - Kuhn (1955) / Munkres (1957) for the assignment solver
class MhtTracker {
 public:
  struct Config {
    // Default per-sensor detection parameters. These populate the
    // "default" entry of the ISensorDetectionModel constructed when the
    // caller does not inject one — bit-identical to the previous global
    // (P_D, λ_C) behaviour. To get per-sensor tables (the textbook
    // multi-sensor formulation), inject a FixedSensorDetectionModel /
    // AdaptiveSensorDetectionModel with explicit per-sensor entries.
    //
    // P_D also serves as the miss-branch detection probability: the
    // missed-detection branch score uses log(1 - probability_of_detection)
    // because the miss is a per-track event, not per-measurement, so a
    // single number is used regardless of which sensors are in the scan.
    double probability_of_detection = 0.9;
    double clutter_density = 1e-4;
    double gate_threshold = 9.0;
    int n_scan = 3;
    std::size_t k_max_leaves = 5;
    double score_delete_threshold = -15.0;

    // Track confirmation mode.
    //
    // M-of-N (DEFAULT): confirm when the selected leaf has ≥
    // confirm_hits_needed hits among its last confirm_hits_window nodes.
    //
    // SPRT (use_sprt_confirm=true, Wald 1947): confirm when the leaf's
    // cumulative log-likelihood-ratio score crosses
    //   T_confirm = ln((1 − β) / α)
    // (deletion already uses the lower bound score_delete_threshold ≈
    // T_delete = ln(β / (1 − α))). In theory this calibrates confirmation
    // to a target false-track rate α and adapts to clutter via the −ln λ_C
    // term already in the score.
    //
    // *Measured*, however, SPRT is WORSE than M-of-N on the current score
    // scale (it degrades clean scenarios — crossing OSPA 18 → 88 — and is
    // neutral on real data), because (a) the cumulative-from-root score is
    // less responsive than a recent-window hit count — an early hit burst
    // keeps a quiet track confirmed — and (b) the per-hit increment is
    // dominated by the measurement-fit term log N, not a clean
    // target-vs-clutter ratio. SPRT becomes worthwhile only once the score
    // is a proper *existence* LLR with per-sensor clutter intensity — i.e.
    // the JIPDA direction. Kept implemented + tested behind the flag for
    // that work; default stays M-of-N until it actually wins.
    bool use_sprt_confirm = false;
    double sprt_alpha = 0.01;  // tolerable false-confirmation probability
    double sprt_beta = 0.01;   // tolerable missed-confirmation probability
    int confirm_hits_needed = 2;
    int confirm_hits_window = 3;

    // Bhattacharyya distance threshold for within-tree leaf merging.
    // Set <= 0 to disable. Typical 0.5–2.0 (smaller = stricter, fewer
    // merges).
    double merge_bhattacharyya_threshold = 1.0;

    // Murty K-best global hypothesis enumeration. The reported track
    // per tree always comes from the best (K=1) assignment — K>1 only
    // affects which alternative leaves are kept across scans for
    // deferred-commitment TOMHT. Set k_best=1 to disable Murty and use
    // the plain Hungarian K=1 path (bit-identical to behaviour before
    // Murty was wired in). Default 3 follows Blackman 2004 §V's
    // typical maritime/aerospace setting.
    int k_best = 3;

    // Score-Δ window for protected alternatives (Blackman 2004 §V).
    // After Murty returns the K best assignments in non-decreasing
    // cost order, only the K=1 best plus any alternative whose cost
    // exceeds the best by less than `score_delta_threshold` contribute
    // to `top_k_leaves` and therefore to protected-leaves carry-over.
    // Without this filter, K>1 protects every rank-K alternative
    // regardless of how arbitrarily worse it is — in cooperative
    // scenarios that's every miss-vs-hit pair, and trees grow ~2x per
    // scan. Set <= 0 to disable the filter (legacy behaviour).
    //
    // Units: same as branch score (cumulative log-likelihood-ratio).
    // A delta of 5.0 admits alternatives within ~e^5 ≈ 150x posterior
    // probability of the best — broad enough for genuine ambiguity,
    // tight enough to reject arbitrarily-worse rank-3 alternatives in
    // unambiguous scenarios.
    double score_delta_threshold = 5.0;

    // Per-tree adaptive N-scan: when a tree had more than one protected
    // alternative on the previous scan, extend its trunk-merge delay
    // by this many scans. Trees with one dominant leaf merge at the
    // base `n_scan`; trees with surviving deferred-commitment
    // alternatives wait `n_scan + n_scan_extension_when_protected`
    // scans before trunk merge, giving the alternative more time to
    // accumulate evidence (or be discarded as non-competitive). Set 0
    // to disable adaptation.
    int n_scan_extension_when_protected = 2;
  };

  // `detection_model` supplies per-sensor (P_D, λ_C) for branch scoring.
  // Null (default) installs a FixedSensorDetectionModel whose single
  // "default" entry is built from cfg.{probability_of_detection,
  // clutter_density} — bit-identical to the previous global behaviour.
  // Inject a FixedSensorDetectionModel with a per-(SensorKind,
  // MeasurementModel) table for the textbook multi-sensor case, or an
  // AdaptiveSensorDetectionModel for per-sensor online λ_C.
  MhtTracker(const IEstimator& estimator, Config cfg,
             std::shared_ptr<ISensorDetectionModel> detection_model = nullptr);

  void processBatch(const std::vector<Measurement>& scan);

  const std::vector<Track>& tracks() const { return tracks_; }
  std::size_t treeCount() const { return trees_.size(); }

  // Detection model in use. Exposed for diagnostics / tests (e.g. to
  // inspect per-sensor λ_C from an AdaptiveSensorDetectionModel).
  const ISensorDetectionModel& detectionModel() const {
    return *detection_model_;
  }

 private:
  const IEstimator& estimator_;
  Config cfg_;
  std::shared_ptr<ISensorDetectionModel> detection_model_;
  std::vector<TrackTree> trees_;
  std::vector<Track> tracks_;
  std::uint64_t next_external_id_{1};
};

}  // namespace navtracker
