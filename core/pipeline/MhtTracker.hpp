#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "core/tracking/TrackTree.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"
#include "ports/IInnovationSink.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {

namespace geo {
class Datum;
}

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

    // Stale-input guard, ON by default. A scan older than the
    // high-water mark of everything already processed would be applied
    // against newer leaf states and rewind node times, inflating the
    // next predict's dt. Such scans are dropped and counted
    // (staleDropped()). Use a ReorderBuffer upstream to *recover* late
    // data; opt out only if input is guaranteed time-ordered.
    bool reject_stale_measurements = true;

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

    // Cross-tree duplicate merge. The global hypothesis only enforces
    // per-scan measurement exclusivity, so two trees latched onto one
    // target both persist when each scan carries several detections of
    // it (multi-sensor): tree A takes one hit, tree B another, both
    // stay confirmed, and downstream consumers see a permanent
    // duplicate (+1 cardinality, id flapping). When two trees' best
    // leaves stay within this Bhattacharyya bound (position block) for
    // `duplicate_merge_seconds` of *sustained stream time*, the
    // younger tree is retired — the OLDER external id survives
    // (ID-stability invariant). The clock resets on any scan apart, so
    // crossing targets that merely brush past never accumulate it.
    // Time-based, NOT scan-counted, for the same reason confirmation
    // is not scan-counted: on a 16 Hz multi-sensor stream a 3-scan
    // streak is ~0.19 s and two real vessels passing close would merge
    // (measured: AutoFerry scenario6 breaks 2.5 → 11.5). Set the
    // threshold <= 0 to disable the pass.
    double duplicate_merge_bhattacharyya = 1.0;
    double duplicate_merge_seconds = 3.0;

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

    // IPDA lifecycle (Musicki-Evans-Stankovic 1994), ON by default
    // since 2026-06-11. Every leaf carries an existence_probability
    // r ∈ [0,1] updated by a Bayes recursion from per-sensor
    // (P_D, λ_C) — *the* calibrated quantity (unlike the raw LLR
    // score, which is dominated by measurement-fit). Confirm/delete
    // read r (with hysteresis) rather than score / M-of-N hit counts.
    //
    // Measured (2026-06-11 baseline, with honest per-sensor tables):
    // bit-identical to M-of-N on clean synthetics, decisively better
    // wherever misses/clutter exist (dense_clutter OSPA 421 → 245,
    // AutoFerry scenario2 breaks 64.5 → 1.5, lifetime 0.77 → 0.95).
    // NB: r is only calibrated if the detection table is honest —
    // see defaultDetectionModelWarning().
    //
    // Disabled: existence_probability stays at its 1.0 sentinel,
    // M-of-N / SPRT drive confirm/delete, tree-delete reads score —
    // the pre-2026-06-11 behaviour, kept as the comparison ablation.
    bool use_ipda_lifecycle = true;
    double ipda_init_existence = 0.5;     // prior r₀ at track birth
    double ipda_confirm_threshold = 0.9;  // r ≥ → Confirmed (first time)
    // Hysteresis: once confirmed, a track stays Confirmed while
    // r ≥ ipda_demote_threshold; it demotes to Tentative only below
    // that, and must re-cross ipda_confirm_threshold to confirm again.
    // Equal to ipda_confirm_threshold → no band (legacy readout).
    // Values above ipda_confirm_threshold are clamped down to it.
    double ipda_demote_threshold = 0.6;
    double ipda_delete_threshold = 0.05;  // r < → tree deleted
    double ipda_persistence = 0.99;       // P(eₖ=1 | eₖ₋₁=1)
    double ipda_gate_probability_mass = 0.99;  // P_G; 1 ≈ generous gate

    // VIMM visibility (Brekke & Wilthil 2019), ON by default since
    // 2026-06-11 (requires use_ipda_lifecycle). Each leaf also carries
    // a visibility_given_exists v ∈ [0,1]. A miss is then attributed
    // partly to "currently obscured" (v drops) rather than entirely
    // to "track is gone" (r drops). Designed for the AutoFerry
    // shadowing scenarios where the standard IPDA recursion would
    // kill a temporarily-hidden target; identical to plain IPDA on
    // miss-free runs (with v₀ = 1 the hit recursions coincide).
    bool use_visibility = true;
    double visibility_init = 1.0;         // v₀ at birth (just detected)
    double visibility_persistence = 0.95; // P(vₖ=1 | vₖ₋₁=1)
    double visibility_recovery = 0.3;     // P(vₖ=1 | vₖ₋₁=0)

    // Shared ambiguous bearings (backlog item 11). A Bearing2D
    // measurement whose hit branches exist in ≥ 2 trees this scan is
    // angularly unresolved — it cannot support an identity decision,
    // and at small angular separations the camera detection genuinely
    // merges both targets. With this flag the global solve exempts
    // such measurements from exclusivity (each tree's bearing hit
    // competes only against that tree's own alternatives), so both
    // trees use the bearing for kinematics while identity stays
    // anchored by the exclusive sensors (radar/lidar positions).
    // Diagnosed on AutoFerry scenario5: camera bearings gating into
    // two angularly-close tracks drove ~91 id switches as the solve
    // swapped them scan to scan (cameras 89% of scans, radar ~0.6 Hz
    // unable to re-anchor). Position measurements are never shared —
    // exclusivity is the right model for resolved sensors.
    bool share_ambiguous_bearings = false;

    // Adaptive recapture gate (backlog item 11, conveyor fix). For
    // POSITION-model measurements the effective gate is
    //   gate · min(gate_recapture_max_scale, 1 + anchor_age / τ)
    // where anchor_age is the hypothesis' time since its last
    // position-sensor update. Rationale (sc5 forensics): a track
    // carried by 16 Hz camera bearings drifts and turns overconfident
    // in range; the next sparse radar return then misses the fixed χ²
    // gate, births a duplicate alongside (45 of 48 near-truth
    // confirmations had a live track within 50 m), and identity hands
    // off every few seconds — a conveyor belt of short-lived ids. The
    // age-scaled gate widens exactly when recapture is needed and
    // keeps the tight clutter gate when the track is freshly range-
    // anchored. Bearings always use the base gate. τ = 0 disables.
    double gate_recapture_tau_s = 0.0;
    double gate_recapture_max_scale = 8.0;
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

  // Optional. When non-null, every surviving tree whose chosen leaf is
  // a HIT (took a measurement this scan) emits one InnovationEvent
  // computed from the parent leaf's state re-predicted to scan_time —
  // bit-exact reproduction of the pre-update state TrackTree::branch()
  // applied estimator.update against. Pruned alternative branches emit
  // nothing — we want the innovation of the filter the world saw.
  void setInnovationSink(IInnovationSink* sink) { innov_sink_ = sink; }

  // Optional. When non-null, every incoming measurement in a scan is
  // corrected by the provider's per-(sensor, source_id) published bias
  // before MHT processing. Null = pre-bias behavior; bit-identical to
  // legacy. See Tracker::setSensorBiasProvider for the same contract.
  void setSensorBiasProvider(const ISensorBiasProvider* provider) {
    bias_provider_ = provider;
  }

  // Re-express all internal hypothesis state from `old_datum`'s ENU frame
  // into `new_datum`'s. MhtTracker keeps authoritative kinematics in its
  // own per-tree nodes (not a TrackManager), so the TrackManager-based
  // shiftTracksOnDatumChange cannot reach it; wire an IDatumChangeSink
  // that calls this instead when running the MHT pipeline with
  // OwnShipProvider auto-recenter. Each node's (state, covariance, IMM
  // means/covariances) is shifted via shiftStateOnDatumChange; the
  // transient per-tree contribution history (≤2 s window, bias-extraction
  // only) is cleared rather than shifted. No-op when no consumer wires it.
  void onDatumRecentered(const geo::Datum& old_datum,
                         const geo::Datum& new_datum);

  const std::vector<Track>& tracks() const { return tracks_; }
  std::size_t treeCount() const { return trees_.size(); }

  // Detection model in use. Exposed for diagnostics / tests (e.g. to
  // inspect per-sensor λ_C from an AdaptiveSensorDetectionModel).
  const ISensorDetectionModel& detectionModel() const {
    return *detection_model_;
  }

  // Scans dropped by the stale-input guard (reject_stale_measurements).
  std::size_t staleDropped() const { return stale_dropped_; }

  // Cumulative count of chosen hit leaves that consumed a SHARED
  // (ambiguity-exempted) bearing — 0 unless share_ambiguous_bearings.
  // Diagnostic: how often the identity-free bearing path engages.
  std::size_t sharedBearingAssignments() const {
    return shared_bearing_assignments_;
  }

  // One-shot diagnostic, sticky once set: ≥2 distinct (SensorKind,
  // MeasurementModel) keys have been processed while running on the
  // auto-installed single-default detection model — i.e. every sensor
  // shares one (P_D, λ_C) despite different rates and λ_C *units*
  // (m⁻² vs rad⁻¹), which is dimensionally wrong and the exact
  // misconfiguration behind the pre-fix AutoFerry collapse. The
  // composition root should read this and inject a per-sensor
  // FixedSensorDetectionModel table. Never set when a model was
  // injected (even a single-default one — that is then a stated
  // choice, not an accident).
  bool defaultDetectionModelWarning() const {
    return default_detection_warning_;
  }

 private:
  const IEstimator& estimator_;
  Config cfg_;
  std::shared_ptr<ISensorDetectionModel> detection_model_;
  std::vector<TrackTree> trees_;
  std::vector<Track> tracks_;
  std::uint64_t next_external_id_{1};
  std::size_t stale_dropped_{0};
  std::size_t shared_bearing_assignments_{0};
  bool has_high_water_{false};
  Timestamp high_water_{};
  // Start of each pair's current uninterrupted closeness streak for
  // the cross-tree duplicate merge, keyed by (older external id,
  // younger external id). Erased when a pair separates and pruned when
  // either tree dies.
  std::map<std::pair<std::uint64_t, std::uint64_t>, Timestamp>
      duplicate_close_since_;
  bool using_default_detection_model_{false};
  bool default_detection_warning_{false};
  std::set<std::pair<SensorKind, MeasurementModel>> seen_sensor_keys_;
  IInnovationSink* innov_sink_{nullptr};
  const ISensorBiasProvider* bias_provider_{nullptr};
  // Per-tree contribution history keyed by externalId. Each scan we
  // append the SourceTouch for the chosen leaf's measurement (if it
  // was a hit) and prune entries older than kContributionWindowSec.
  // The Track views built each scan copy from this so consumers
  // (SensorBiasPairExtractor, AisArpaPairExtractor) see the same
  // contracts as the single-hypothesis Tracker pipeline.
  std::map<TrackId, std::vector<Track::SourceTouch>>
      contribution_history_;
  static constexpr double kContributionWindowSec = 2.0;
  // Per-tree fused identity, accumulated over the tree's lifetime and
  // copied into the emitted Track view each scan (the canonical pipeline
  // otherwise drops MMSI/provenance, since TrackTreeNode carries no
  // attributes). `tree_attributes_` takes the latest mmsi hint seen on a
  // committed (born or chosen-hit) measurement; `tree_sources_` is the
  // ordered set of distinct contributing source_ids. Both are pruned when
  // a tree dies, alongside contribution_history_.
  std::map<TrackId, TrackAttributes> tree_attributes_;
  std::map<TrackId, std::vector<std::string>> tree_sources_;
};

}  // namespace navtracker
