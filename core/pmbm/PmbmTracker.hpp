#pragma once

#include <functional>
#include <map>
#include <vector>

#include <memory>

#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "ports/ISensorDetectionModel.hpp"
#include "ports/ITrackSink.hpp"

// Poisson Multi-Bernoulli Mixture (PMBM) tracker. Sibling to MhtTracker,
// implementing the same multi-target tracking goal via the Random Finite
// Set (RFS) formulation rather than a hypothesis tree.
//
// See:
//   docs/algorithms/pmbm-design.md   — equation-level reference
//   docs/learning/23-pmbm.md         — plain-English introduction
//   docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md
//                                    — phased engineering plan
//
// Phase 1 (this file): GM-PMBM. Per-Bernoulli single-target density is a
// single Gaussian; estimator is whatever the caller injects (typically
// EKF or UKF). The standard point-target model is assumed (one
// measurement per target per scan, Poisson clutter, Poisson birth).
//
// Phase 2 swaps the per-Bernoulli density to an IMM mixture by
// extending the Bernoulli adapter to round-trip the imm_* fields on
// Track. Phase 3 (TPMBM) extends each Bernoulli with a trajectory
// (state history); structure unchanged otherwise.

namespace navtracker::pmbm {

class PmbmTracker {
 public:
  struct Config {
    // Per-scan target survival probability p_S. Applied multiplicatively
    // to every Bernoulli's existence probability and every PPP
    // component's weight during predict. 0.99 is the standard textbook
    // value; deployment may want sensor-conditional values for ships
    // entering/leaving radar coverage (future work).
    double survival_probability = 0.99;

    // Standard point-target detection parameters. Phase 1 uses one
    // (P_D, λ_C) pair across all sensors — equivalent to the default
    // single-entry ISensorDetectionModel — and the same caveat applies:
    // mixing sensors with different rates/units against a single λ_C is
    // dimensionally wrong (see MhtTracker::defaultDetectionModelWarning).
    // Per-sensor injection arrives with the M2 wiring; the math is
    // unchanged.
    double probability_of_detection = 0.9;
    double clutter_intensity = 1e-4;

    // χ² gate (squared Mahalanobis). Per-(Bernoulli, measurement)
    // pairs that fail this gate are excluded from the cost matrix
    // (assigned +∞), saving the O(N·M·d²) likelihood eval. Same scale as
    // MhtTracker::Config::gate_threshold; 9.0 ≈ 99 % χ²₂.
    double gate_threshold = 9.0;

    // Murty K-best per parent global hypothesis. Each prior produces up
    // to K child hypotheses; total children = sum across priors, capped
    // by `max_global_hypotheses` after weight pruning. K=3 follows the
    // MhtTracker default for the same Blackman 2004 reason — broad
    // enough for real ambiguity, tight enough to bound runtime.
    int k_best_per_hypothesis = 3;
    std::size_t max_global_hypotheses = 30;

    // Pruning thresholds. Applied at end of each update().
    // - r_min: Bernoulli existence floor. Below this, the Bernoulli is
    //   dropped from its hypothesis.
    // - weight_min: PPP component intensity floor.
    // - hypothesis_weight_min: global hypothesis posterior floor.
    double r_min = 1e-3;
    double weight_min = 1e-4;
    double hypothesis_weight_min = 1e-4;

    // Output emission threshold (§3.6 of pmbm-design.md). After
    // aggregating P(exists | id) across hypotheses, emit a Track with
    // status = Confirmed iff P(exists) ≥ confirm_threshold, else
    // Tentative. 0.5 matches the textbook midpoint.
    double confirm_threshold = 0.5;

    // Aggregated tracks() output also drops any Bernoulli id whose
    // P(exists) is below this floor — keeps phantom mass out of
    // downstream consumers without affecting the internal MBM.
    double output_existence_floor = 0.1;

    // Measurement-driven birth (default ON in real configs, OFF in unit
    // tests so the predict-step math stays interpretable). At the start
    // of every processBatch, lay down one PoissonComponent per
    // initiable measurement (skipping bearing-only) by calling
    // IEstimator::initiate — exactly the birth model the standard
    // PMBM literature (García-Fernández 2018 §IV-D, MTT toolbox
    // ``DBM_filter'') uses when no domain prior over ship arrival
    // regions is available. `birth_weight_per_measurement` controls
    // the per-component intensity; tune so it dominates the configured
    // clutter intensity in the local measurement-space volume.
    bool measurement_driven_birth = false;
    double birth_weight_per_measurement = 1.0;

    // Smart-birth gate. With measurement-driven birth, every gated
    // clutter return mints a fresh Bernoulli that competes with the
    // tracked one and creates an id_switch when it eventually
    // out-scores it. When smart_birth_skip_existing is true, a
    // measurement that's already explained by an existing Bernoulli
    // (any hypothesis: r ≥ smart_birth_skip_r_min AND χ² to that
    // Bernoulli ≤ smart_birth_skip_gate) skips birth — the existing
    // track absorbs the measurement and no phantom is born. Strongly
    // recommended ON; default OFF so unit tests of the bare predict /
    // update math remain interpretable. Adaptive Birth Distribution
    // (Reuter 2014).
    bool smart_birth_skip_existing = false;
    double smart_birth_skip_r_min = 0.5;
    double smart_birth_skip_gate = 9.0;  // χ² 2-DoF 99 %

    // Clutter-aware PPP-birth gate (Phase 3 polish). The
    // smart_birth_skip_existing check above covers only Bernoullis,
    // not the PPP intensity. Under measurement_driven_birth =
    // true, every clutter return injects a fresh PoissonComponent
    // centred on itself; on the next scan that component dominates
    // ρ_target locally and any nearby measurement births a high-r
    // Bernoulli regardless of the configured clutter density. This
    // accumulates phantom tracks in clutter-heavy scenarios
    // (dense_clutter, radar-clutter philos).
    //
    // Gate: before injecting a fresh PoissonComponent for
    // measurement z, sum Σ_{c ∈ existing PPP} w_c · ℓ(z | c). If
    // ≥ smart_birth_skip_existing_ppp_threshold, skip the injection
    // — the existing PPP already covers this area and another
    // component would just be redundant. Threshold is per-volume
    // mass; tune relative to the configured / per-sensor λ_C so
    // that "existing PPP density ≥ k · clutter density" is the
    // trigger (typical k = 1–5). Set ≤ 0 to disable.
    bool smart_birth_skip_existing_ppp = false;
    double smart_birth_skip_existing_ppp_threshold = 0.0;

    // Adaptive Birth (Reuter 2014). Decouples the new-target Bernoulli
    // **spatial** density (mean at z, covariance from estimator.initiate)
    // from the **existence** prior. Replaces the contaminated
    //   r_new_l = ρ_target_l / (ρ_target_l + λ_C(z_l))
    // — where ρ_target_l is dominated by the just-injected PPP component
    // centred on z_l under measurement_driven_birth, so r_new ≈ 1 for
    // every measurement including clutter — with the Reuter formulation:
    //   r_new_l = λ_birth / (λ_birth + λ_C(z_l))
    // where λ_birth is a configurable scalar (expected new-target rate
    // per scan per measurement-space volume, same units as λ_C). Skips
    // the measurement-driven PPP injection entirely; the spatial state
    // comes directly from estimator.initiate(z). Falls back on
    // birth_model_ (BirthModelFn) for explicit prior PPP if wired.
    //
    // When ON: phantom-birth gate (min_new_bernoulli_existence) should
    // be tuned lower (~0.05) because r_new is no longer artificially
    // pegged near 1 — real targets ramp up via posterior updates over
    // subsequent scans, not from a high initial r.
    //
    // Reference: Reuter, S. et al. (2014), "The Labeled Multi-Bernoulli
    // Filter", IEEE Trans. Signal Processing 62(12), §IV-B Adaptive
    // Birth Distribution.
    bool adaptive_birth = false;
    double lambda_birth = 1e-3;

    // Trajectory-PMBM (Phase 4, García-Fernández/Williams 2020). When
    // > 0, each Bernoulli records its forward-pass post-update state
    // history at every detection and post-predict state at every
    // misdetection, truncated to the most recent N points. Provides
    // operator-visible track history (consumed by ITrackSink on
    // delete) and the scaffold for future RTS smoothing.
    // 0 = disabled (Phase 3 bit-identical, zero overhead). Typical
    // bench value 50 (≈ 50 scans of history at scan rate 1 Hz).
    std::size_t trajectory_window_scans = 0;

    // Source-aware misdetection. AIS (and other source-specific
    // broadcast sensors) report per vessel — a scan with vessel A's
    // broadcast tells us nothing about vessel B's existence. With
    // source_aware_misdetection ON, a Bernoulli skips the misdetection
    // recursion when none of its contributing source_ids appears in
    // the current scan. Falls back to regular misdetection for fresh
    // Bernoullis with no contribution history (so brand-new births
    // still decay normally if they go unobserved). Critical for the
    // philos sparse-AIS scenario: without this, every other vessel's
    // broadcast misdetects ours and r dies in O(1) scan. Off by
    // default so unit-test math stays interpretable; on in the PMBM
    // bench config.
    bool source_aware_misdetection = false;

    // Within-hypothesis Bernoulli merging. Generalised from
    // MhtTracker::mergeBranches (cross-tree Bhattacharyya merge): after
    // enumerateChildren, fold pairs of Bernoullis within the same
    // global hypothesis whose 2-D position blocks are within
    // `bhattacharyya_merge_threshold`. Survivor keeps the older id
    // (id-stability invariant), gets r = 1 - (1-r_i)(1-r_j)
    // (independent-existence fold), and a r-weighted moment-matched
    // (mean, cov). Defends against the rare case where a smart-birth
    // gate misses (gate threshold below the true overlap) and an
    // existing Bernoulli plus a fresh new-Bernoulli end up on the
    // same target. Set ≤ 0 to disable; typical 0.5–2.0.
    double bhattacharyya_merge_threshold = 1.0;

    // PPP component count cap (applied after each prune). Without a
    // cap, every clutter return adds a PoissonComponent that takes
    // several scans to decay under (1-P_D)·p_S, growing unbounded on
    // long replays. Top-weighted are kept.
    std::size_t max_ppp_components = 200;

    // Idle-decay half-life (seconds). When source_aware_misdetection
    // is ON and a Bernoulli's contributing sources are absent from a
    // scan, the textbook recursion (1 − r·p_D)/… is skipped — correct,
    // because a different vessel's AIS broadcast carries no
    // information about this target's existence. The side-effect is
    // that ghost Bernoullis whose target has actually stopped
    // reporting never decay. This knob applies an explicit time-based
    // decay during those skipped-misdetection branches:
    //   r ← r · exp(−ln 2 · Δt / idle_halflife_sec)
    // where Δt is the elapsed time since the Bernoulli's last
    // detection (b.last_update → scan.front().time). Reflects a
    // prior expectation that a real maritime target reports at least
    // every few minutes; below the half-life it costs the
    // existence-stable real tracks essentially nothing, above it the
    // ghosts decay below r_min and prune. ≤ 0 disables. Default 0
    // for parity with prior behaviour; the bench config sets ~60 s.
    double idle_halflife_sec = 0.0;

    // Birth gate: new-target row Bernoulli birth threshold. The
    // standard PMBM new-target Bernoulli existence is
    //   r_new_l = ρ_target_l / (ρ_target_l + λ_C(z_l))
    // which is correctly small under dense clutter, but the Bernoulli
    // is still materialised, costs hypothesis cardinality, and shows
    // up as an id-flap candidate before the r_min pruner catches it
    // on the next scan. When this knob is > 0, the assignment cell is
    // still feasible (clutter mass consumed normally so Murty stays
    // balanced) but the Bernoulli is suppressed entirely when r_new
    // is below the threshold. Defends against dense_clutter where
    // every gated clutter return births a near-zero-r phantom.
    // Conservative default 0; bench config sets ~0.05.
    double min_new_bernoulli_existence = 0.0;
  };

  // Birth intensity callback. Called once per predict() (after the
  // survival decay), supplying the new filter time and the elapsed dt.
  // Returns PPP components to add to the current PPP intensity. Empty
  // (default-constructed std::function) installs a no-birth model — the
  // tracker still runs but no new targets can ever appear. Useful for
  // closed-set tests; real deployments must supply a model.
  using BirthModelFn =
      std::function<std::vector<PoissonComponent>(Timestamp, double)>;

  PmbmTracker(const IEstimator& estimator, Config cfg,
              BirthModelFn birth_model = {});

  // Advance the PPP intensity, every Bernoulli in every global
  // hypothesis, and the filter time to `to`. Birth intensity (per the
  // BirthModelFn) is added after the survival decay.
  //
  // The first call after construction initialises the filter time from
  // `to` (dt = 0; birth model is called with dt = 0 to allow seeding an
  // initial PPP); subsequent calls compute dt from the previous filter
  // time. Calls with `to` ≤ currentTime() advance the clock only (no
  // propagation, no birth) — matches the MhtTracker stale-input
  // convention without raising.
  void predict(Timestamp to);

  // Ingest one scan of measurements. Predicts to scan_time (the max
  // timestamp in `scan`, or current time for empty), then applies the
  // PMBM update step:
  //
  //   1. New-target candidates from PPP per measurement (§3.2)
  //   2. PPP decay by (1−P_D) (§3.3)
  //   3. Per hypothesis: per-Bernoulli per-measurement update +
  //      misdetection (§3.1); cost matrix (§3.4); Murty K-best
  //      enumeration → child global hypotheses
  //   4. Mixture pruning (§3.5)
  //
  // Empty `scan` is a no-op apart from the predict.
  void processBatch(const std::vector<Measurement>& scan);

  const PmbmDensity& density() const noexcept { return density_; }
  Timestamp currentTime() const noexcept { return current_time_; }
  bool hasCurrentTime() const noexcept { return has_current_time_; }
  const Config& config() const noexcept { return cfg_; }

  // Test / introspection hooks.
  PmbmDensity& mutableDensityForTesting() { return density_; }

  // Optional. When non-null, every incoming measurement in
  // processBatch is corrected by the provider's per-(sensor,
  // source_id) published bias before reaching the PMBM update (same
  // contract as MhtTracker::setSensorBiasProvider / Tracker::
  // setSensorBiasProvider). Null = bit-identical to legacy.
  void setSensorBiasProvider(const ISensorBiasProvider* provider) {
    bias_provider_ = provider;
  }

  // Optional. Push-based lifecycle observer (Phase 4(B)). After each
  // processBatch the tracker computes the diff between the prior-scan
  // emitted track set and the current refreshed aggregated_tracks_,
  // firing onTrackInitiated (new Tentative), onTrackConfirmed
  // (new Confirmed OR Tentative→Confirmed transition), onTrackUpdated
  // (every track present this scan), and onTrackDeleted (id present
  // prior, absent now — pruned below output_existence_floor). When
  // null = pull-only mode (legacy, bit-identical to Phase 4(A)).
  // Trajectory (TPMBM) is accessible via trajectoryFor(id) within
  // any of these callbacks — the call site holds the dominant
  // hypothesis until the next predict.
  void setTrackSink(ITrackSink* sink) { track_sink_ = sink; }

  // Per-sensor detection model. When set, the cost matrix, new-target
  // birth weight, and misdetection recursion use the per-(sensor,
  // model, source_id) (P_D, λ_C) and per-coverage missDetectionProb
  // instead of the Config-level scalars. The scenario's per-sensor
  // table is the textbook formulation for multi-sensor PMBM
  // (García-Fernández 2018 §IV-A); without it, λ_C and P_D are
  // dimensionally inconsistent across sensors of different MeasurementModels.
  // Null = fall back to Config::probability_of_detection /
  // ::clutter_intensity (single-sensor-equivalent behaviour).
  void setSensorDetectionModel(std::shared_ptr<ISensorDetectionModel> m) {
    detection_model_ = std::move(m);
  }

  // Next Bernoulli id that will be minted (introspection / tests).
  BernoulliId nextBernoulliId() const noexcept { return next_bernoulli_id_; }

  // TPMBM (Phase 4) — forward-pass trajectory for a given Bernoulli
  // id. Walks the MBM, finds the highest-weight global hypothesis
  // containing that id, returns its trajectory. Empty when (a) the
  // id is not present in any hypothesis, (b) trajectory_window_scans
  // is 0 (TPMBM disabled), or (c) the Bernoulli has not yet been
  // observed (e.g. just-created seed). One-stop accessor for
  // consumers that want trajectory output without subscribing to
  // every hypothesis.
  std::vector<TrajectoryPoint> trajectoryFor(BernoulliId id) const;

  // Drain + RTS-smooth all currently-live Bernoulli trajectories from
  // the dominant global hypothesis. For each Bernoulli still in the
  // MBM, copies its trajectory and applies rtsSmoothTrajectory in
  // place. Returns the smoothed trajectories keyed by Bernoulli id.
  //
  // Used by the bench to compute T-GOSPA on smoothed trajectories
  // (Phase 6 measurement). Does NOT include Bernoullis already pruned
  // mid-scenario; for those, the per-scan filtered state already lives
  // in BenchResult.steps so the raw T-GOSPA still picks them up.
  //
  // Empty when TPMBM is disabled (trajectory_window_scans == 0) or no
  // hypotheses exist.
  std::map<BernoulliId, std::vector<TrajectoryPoint>>
      collectSmoothedTrajectories() const;

  // Aggregated single-Track view of the MBM, one entry per unique
  // Bernoulli id (§3.6 of pmbm-design.md):
  //   P(exists | id) = Σ_{j: id ∈ j} w^j · r^{j,id}
  //   mean(id)       = (1 / P(exists)) · Σ w^j · r^{j,id} · μ^{j,id}
  //   cov(id)        = (1 / P(exists)) · Σ w^j · r^{j,id} · (P^{j,id} +
  //                                                          dd')
  // with d = μ^{j,id} − mean(id). Status = Confirmed iff
  // P(exists) ≥ confirm_threshold; tracks with P(exists) <
  // output_existence_floor are omitted entirely. TrackId.value reuses
  // the Bernoulli id so external consumers see a stable, non-reusable
  // identifier across scans.
  const std::vector<Track>& tracks() const;

 private:
  // One candidate "new Bernoulli" per measurement, fused from the PPP.
  // rho_target = Σ_i p_D · w_i · ℓ(z|c_i); rho_total = rho_target +
  // λ_FA(z). Existence of the new Bernoulli is rho_target / rho_total.
  //
  // For the spatial density we currently moment-match the multi-
  // component PPP-posterior to a single Gaussian (mean / covariance).
  // For the IMM ensemble fields we carry the DOMINANT PPP component's
  // post-update imm_* — exact when birth is driven by a single fresh
  // PoissonComponent (the measurement-driven birth case), approximate
  // otherwise. Sufficient for Phase 1; Phase 2 (full IMM-per-Bernoulli)
  // will fold the per-component IMM mixtures properly.
  struct NewTargetCandidate {
    double rho_target{0.0};
    double rho_total{0.0};
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd imm_means;
    std::vector<Eigen::MatrixXd> imm_covariances;
    Eigen::VectorXd imm_mode_probabilities;
  };

  std::vector<NewTargetCandidate> buildNewTargetCandidates(
      const std::vector<Measurement>& scan) const;

  // Reuter 2014 Adaptive Birth path (cfg_.adaptive_birth). One
  // candidate per measurement:
  //   r_new_l    = λ_birth / (λ_birth + λ_C(z_l))
  //   spatial f_l(x) ~ estimator.initiate(z_l) (mean = z, cov from
  //                    the estimator's birth covariance)
  // Decoupled from the PPP intensity — independent of any
  // measurement-driven PPP contamination. Non-initiable measurements
  // yield {rho_target=0, rho_total=λ_C(z)} — never a Bernoulli, just
  // clutter mass for assignment balance.
  std::vector<NewTargetCandidate> buildAdaptiveBirthCandidates(
      const std::vector<Measurement>& scan) const;

  // Per-parent enumeration: build the (n_j + m) × m cost matrix, solve
  // Murty K-best, materialise each assignment as a child global
  // hypothesis. `nts` is the per-measurement new-target cache from
  // buildNewTargetCandidates.
  void enumerateChildren(const GlobalHypothesis& parent,
                         const std::vector<Measurement>& scan,
                         const std::vector<NewTargetCandidate>& nts,
                         std::vector<GlobalHypothesis>& out);

  // After enumeration: renormalise mixture weights, drop hypotheses /
  // Bernoullis / PPP components below thresholds, cap mixture size at
  // cfg_.max_global_hypotheses.
  void pruneAndNormalise();

  // Merge near-duplicate Bernoullis within one global hypothesis by
  // Bhattacharyya distance on the position block (§3.5 of
  // docs/algorithms/pmbm-design.md, MhtTracker::mergeBranches analogue).
  void mergeBernoulliDuplicates(GlobalHypothesis& h) const;

  void refreshAggregatedTracks() const;

  const IEstimator& estimator_;
  Config cfg_;
  BirthModelFn birth_model_;
  PmbmDensity density_;
  Timestamp current_time_{};
  bool has_current_time_{false};
  BernoulliId next_bernoulli_id_{1};
  const ISensorBiasProvider* bias_provider_{nullptr};
  std::shared_ptr<ISensorDetectionModel> detection_model_;
  // Per-Bernoulli-id rolling source-touch history. Populated from the
  // dominant child after each scan; the same shape as
  // MhtTracker::contribution_history_. Folded into each emitted Track's
  // recent_contributions by refreshAggregatedTracks so the bias-pair
  // extractors (extractPositionPairs / extractBearingPairs /
  // extractCrossSensorPositionPairs) see actual contributions and the
  // PMBM path matches the MHT path on AIS-anchored scenarios.
  std::map<BernoulliId, std::vector<Track::SourceTouch>>
      contribution_history_;
  static constexpr double kContributionWindowSec = 2.0;
  mutable std::vector<Track> aggregated_tracks_;
  mutable bool aggregated_tracks_dirty_{true};

  // Phase 4(B). ITrackSink (optional). prev_emitted_statuses_ maps
  // emitted TrackId.value → status as of the prior scan; used to
  // detect Initiated / Tentative→Confirmed / Updated / Deleted
  // transitions on each processBatch.
  ITrackSink* track_sink_{nullptr};
  std::map<std::uint64_t, TrackStatus> prev_emitted_statuses_;
  // Phase 4(D). Snapshot trajectories from the prior scan's dominant
  // hypothesis, keyed by Bernoulli id. Updated at end of each
  // processBatch alongside prev_emitted_statuses_. Two purposes:
  // (a) inside an onTrackDeleted handler, trajectoryFor(id) falls
  //     back to this snapshot when the id is no longer in any live
  //     hypothesis — so the consumer gets the final trajectory.
  // (b) while a track is still alive, this is unused (live MBM is
  //     authoritative); the snapshot only "matters" when the live
  //     lookup fails. Cleared at end of each processBatch.
  std::map<std::uint64_t, std::vector<TrajectoryPoint>>
      prev_emitted_trajectories_;
  // Fire onTrack* events by diffing aggregated_tracks_ against the
  // prior-scan snapshot. Called at end of processBatch when
  // track_sink_ != nullptr. Uses scan-front time as the event
  // timestamp (matches MhtTracker convention).
  void firePmbmLifecycleEvents(Timestamp event_time);
};

}  // namespace navtracker::pmbm
