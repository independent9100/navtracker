#pragma once

#include <functional>
#include <map>
#include <set>
#include <vector>

#include <memory>

#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "ports/ISensorDetectionModel.hpp"
#include "ports/ILandModel.hpp"
#include "ports/ISensorActivity.hpp"
#include "ports/IStaleSignalSink.hpp"
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

    // Per-sensor λ_birth override (Phase 9 review Finding 6B).
    // When non-empty, `lambda_birth_per_sensor[z.sensor]` overrides
    // the scalar `lambda_birth` for measurement z. Mirrors the
    // existing per-sensor λ_C plumbing in ISensorDetectionModel.
    // AIS broadcasts have a much higher prior birth intensity than
    // EO-IR observations (orders of magnitude); the scalar
    // `lambda_birth` forces one number for all sensors, mistuning
    // the new-target Bernoulli existence on whichever sensor isn't
    // the scalar's reference. Empty map = fall back to scalar
    // (bit-identical to legacy).
    //
    // Reuter 2014 §IV-B treats λ_birth as a measurement-space
    // intensity; the formulation admits per-sensor stratification
    // (each sensor contributes its own birth-PHD).
    std::map<navtracker::SensorKind, double> lambda_birth_per_sensor;

    // Clutter-invariant birth existence (Task 1, 2026-06-24). When > 0,
    // the adaptive-birth path derives λ_birth from the per-measurement
    // λ_C so the new-target existence is exactly r_new = this value,
    // independent of the sensor/scenario clutter density:
    //   λ_birth = (r*/(1−r*))·λ_C  ⇒  r_new = λ_birth/(λ_birth+λ_C) = r*.
    // Fixes the philos over-confident-birth bug: a fixed *absolute*
    // λ_birth gives r_new ≈ 0.79 (radar) / ≈1.0 (AIS) on philos because
    // λ_C there is 2.7e-6 / 1e-9, not the 1e-4 the scalar was tuned for.
    //
    // Must be in (0, 1); values outside this range (0 or >= 1) fall back
    // to the lambda_birth / lambda_birth_per_sensor path (finite, predictable).
    // Values >= 1 would produce zero or negative denominator (→ Inf/NaN);
    // the guard in buildAdaptiveBirthCandidates enforces the contract.
    // Only effective when adaptive_birth = true.
    //
    // 0 = legacy (use lambda_birth / lambda_birth_per_sensor). Typical
    // 0.1–0.3 so real targets ramp via posterior over later detections.
    double birth_existence_target = 0.0;

    // Adaptive K-best per parent (MATLAB MTT-master convention).
    // When ON, each parent's K is derived from its weight share:
    //   K_p = max(1, ceil(max_global_hypotheses · w_p))
    // so high-weight parents get deeper exploration and low-weight
    // parents at most 1 child. Mirrors
    // `kbest = ceil(Nhyp_max * w_p)` in PoissonMBMtarget_update.m
    // and MBMtarget_update.m. The fixed `k_best_per_hypothesis` is
    // used as a per-parent ceiling so a single dominant parent
    // cannot run away with the entire mixture budget.
    // OFF by default for backward compatibility; bench config
    // `imm_cv_ct_pmbm_adapt` turns this ON (Phase 8 P2 fix).
    bool adaptive_k_best = false;

    // Per-parent K-best dominance cutoff (Phase 9 M2 diag). When
    // the top Murty child of a parent dominates by a wide margin,
    // the weaker K-children inject phantom-birth Bernoullis that
    // survive into the output aggregation via Σ w·r and emit as
    // spurious Confirmed tracks (Diagnostic A: this is the
    // mechanism behind the K=3 autoferry_scenario13_anchored
    // +44.97 % gospa regression).
    //
    // When > 0, drop K-children whose log_weight is below
    // (top_child_log_weight - k_best_dominance_log_gap). With
    // k_best_dominance_log_gap = 1.0, alts that are ≤ exp(-1) ≈
    // 37 % of the top in weight space are pruned at the per-parent
    // level — much cheaper than letting them propagate through
    // pruneAndNormalise.
    //
    // 0 = disabled (bit-identical to Phase 8/9 baseline). Tuned
    // bench value lives in the K=3 config.
    double k_best_dominance_log_gap = 0.0;

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

    // Per-vessel identity for the source-aware misdetection gate (2a).
    // AIS broadcasts all share source_id="ais", so the channel-keyed gate
    // cannot distinguish individual vessels: vessel A's broadcast causes
    // vessel B to be misdetected. When source_aware_identity=true AND a
    // SourceTouch entry has a vessel_id, the gate uses vessel_id instead
    // of source_id for that touch — so only a scan measurement with the
    // same vessel_id (hints.mmsi) counts as coverage. Channel fallback
    // applies when vessel_id is absent (bit-identical to legacy).
    // Off by default = bit-identical to source_aware_misdetection only.
    // DO NOT change AIS source_id="ais" — MHT's miss-dedup keys on it.
    bool source_aware_identity = false;

    // Per-scan dedup of compute_miss_pD (2b). Textbook: one detection
    // opportunity per distinct (sensor, model, source_id) channel per
    // scan, not one per return. With dedup_miss_pd=true, N returns from
    // one radar sweep count as ONE opportunity: effective miss-P_D =
    // 1−(1−P_D) rather than 1−(1−P_D)^N. Off by default = legacy
    // per-measurement product (bit-identical to Phase 8/9 baseline).
    // Requires a detection model to have any effect; falls back to the
    // Config::probability_of_detection scalar when no model is set.
    // INERT when use_sensor_activity=true: the surveillance-miss branch
    // computes the miss update via ISensorActivity and never calls
    // compute_miss_pD, so this flag is dead code under the coverage model
    // (measured 2026-06-30: dedup ON vs OFF byte-identical on philos + all
    // autoferry scenarios with imm_cv_ct_pmbm_coverage_land). It only
    // affects legacy-path configs (use_sensor_activity=false).
    bool dedup_miss_pd = false;

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

    // Output-side cross-id birth merge (Phase 9 M3 Option A).
    // refreshAggregatedTracks aggregates per-id mass/state across
    // global hypotheses. When two ids in the OUTPUT-LAYER
    // aggregation are spatially close (Bhattacharyya distance below
    // this threshold), fold the lighter (smaller mass) into the
    // heavier before emitting. Addresses the K=3 phantom-birth leak
    // from M2 Diagnostic A: alt hypotheses (sc13_anchored scan 3,
    // K-2/K-3 with w around 0.25 each) birth Bernoullis with fresh
    // ids that DON'T merge in the within-hypothesis pass (different
    // hypothesis, different id) but DO sit on top of the same
    // physical target. The output aggregation can see across ids;
    // the within-hypothesis merge can't.
    //
    // <= 0 disables (bit-identical to legacy). Same Bhattacharyya
    // semantics as bhattacharyya_merge_threshold; typical 0.5-2.0.
    double output_merge_bhattacharyya_threshold = 0.0;

    // M3 iter-6 discriminator gates (Phase 9 M3 Option A iter-2).
    // Iters 1-5 found the Bhattacharyya threshold alone can't
    // separate K=3 alt-hypothesis phantom births (sc13/16_anchored)
    // from legitimate close parallel tracks (philos). These gates
    // use signals already in the aggregation loop:
    //   - birth_time (per Bernoulli, populated at enumerateChildren)
    //   - hyp_count (how many global hypotheses an id appears in)
    // Phantoms are YOUNG (just born this scan) and WEAKLY supported
    // (in 1-2 alt hyps out of many); legitimate parallel tracks are
    // MATURE and WELL supported.
    //
    // max_age_sec: skip merge if max(age_a, age_b) > this value,
    // where age_i = current_time - earliest_birth_time(id_i).
    // 0 = no age gate.
    //
    // max_hyp_support: skip merge if min(hyp_count_a, hyp_count_b)
    // >= this value (i.e. BOTH ids are in N+ hypotheses → both
    // mature). 0 = no hyp-count gate.
    //
    // Both knobs default 0 = inactive. Threshold knob above must
    // also be > 0 for the merge block to run at all.
    double output_merge_max_age_sec = 0.0;
    int output_merge_max_hyp_support = 0;

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

    // Task 4: when true AND sensor_activity is wired, existence moves only
    // on a genuine per-duty-cycle surveillance miss; idle_halflife and the
    // per-blip miss path are bypassed (spec §4, §12). Default false ->
    // today's behaviour, bit-identical.
    bool use_sensor_activity = false;
    // Task 4 (spec §4 case 2): a cooperative-only track is retired only
    // after this many seconds with no own-identity report (0 = never by
    // this rule). Never a per-sweep existence penalty.
    double cooperative_stale_timeout_sec = 0.0;

    // Task A (land clutter prior): when true AND a land model is wired via
    // setLandModel(), scale the adaptive-birth intensity by
    //   (1 − clutterPrior(birth_pos))
    // so births near or on land are soft-suppressed. When the prior exceeds
    // land_birth_hard_gate (inland plateau) the birth is dropped entirely
    // (rho_target = 0). The hard gate fires only well inside land — the
    // ramp shape of a well-built prior ensures it never reaches the gate
    // threshold at the waterline, protecting anchored near-shore vessels.
    // Default false → bit-identical to today's behaviour.
    bool use_land_model = false;
    // χ clutter-prior threshold for a hard inland gate. When
    // clutterPrior(birth_pos) > land_birth_hard_gate, the birth candidate
    // is dropped entirely (rho_target set to 0). Must be in (0, 1].
    // 0.95 means "only fire the hard gate when the prior is very confidently
    // inland, never at the waterline". Ignored when use_land_model = false.
    double land_birth_hard_gate = 0.95;

    // Per-track-hypothesis structural path (Phase 9). When true,
    // `PmbmDensity::tracks` + `tracked_mbm` are populated alongside
    // the legacy flat `mbm` view. Phase 9 milestone S1 ships the
    // view-builder only (no behavioral change); subsequent milestones
    // re-route enumerateChildren, refreshAggregatedTracks, predict
    // and the smoother to read from the per-track view. Off by
    // default → bit-identical to Phase 8/9-M3 baseline.
    //
    // Background: see
    // docs/superpowers/specs/2026-06-23-pmbm-phase9-per-track-hypotheses.md
    // for the structural diff vs. flat `bernoullis` lists and the
    // mechanism by which per-track-hypothesis lineage discriminates
    // K-best alt-hypothesis phantom births from legitimate close
    // parallel tracks (sc13/16_anchored gospa +44/+38 % regression
    // at K=3 — the only candidate fix left after M1/M2/M3).
    bool use_per_track_hypotheses = false;

    // Phase 9 probe — cross-parent birth-id cache.
    //
    // Existing scan_birth_id_cache_ (Phase 8 iter 5) shares Bernoulli
    // ids across K siblings of one parent that birth the SAME
    // measurement. But across parents, the cache key is
    // (parent_idx, measurement_idx) — so two different parents' K-
    // children birthing the same measurement get DIFFERENT ids. Under
    // K=3 with a multi-parent mixture (sparse-AIS philos: many
    // surviving hypotheses every scan), every scan some alt-parent's
    // child mints a fresh id for an already-tracked target. When that
    // alt becomes the next-scan top, the OUTPUT id flips →
    // id_switch.
    //
    // When this flag is ON, the cache key drops parent_idx — all
    // K-children of any parent that birth measurement l share the
    // same BernoulliId. Mirrors MATLAB MTT-master's filter-level
    // new-track creation (PoissonMBMtarget_update.m: new tracks
    // belong to the FILTER, not to a specific globHyp). Probe target:
    // dent K=3 philos id_switches +50 % regression without the Phase
    // 9 S3 alt-birth gate's mass-accounting hack.
    //
    // OFF by default (bit-identical to S2/S3 baseline). Independent
    // of alt_birth_log_gap_threshold AND of adaptive_k_best — when this
    // flag is on, the cross-parent cache fires whether K-best per parent
    // is fixed or adaptive. (Review-2 fix 2026-06-23: earlier
    // implementation silently no-op'd when adaptive_k_best=false.)
    bool cross_parent_birth_id_cache = false;

    // Phase 9 S3 — alt-birth-gate. Per-parent K-best lineage-aware
    // suppression of new-target Bernoullis in non-top alt children.
    //
    // Mechanism (M2 Diagnostic A, M3 Option B reframing): under K>1
    // with a very confident top assignment (truth-anchor σ=5 m
    // measurements feed cleanly into the cost matrix; the autoferry-
    // anchored scenarios sc13/16/22), alt K-children must claim
    // measurements for spurious new-target rows in order to differ
    // from the top — Murty has no other room to manoeuvre. These
    // phantom births survive output aggregation Σ w·r above the
    // output_existence_floor and emit as Confirmed tracks → +44.97 %
    // gospa on sc13_anchored, +38.56 % on sc16_anchored.
    //
    // The Phase 9 spec proposed a full per-track-hypothesis structural
    // refactor (~1000 LOC, 3-5 days) to carry per-Bernoulli "born in
    // a weak alt of which parent" lineage. This knob achieves the
    // same effect using information already on hand at enumeration
    // time: Murty's `cost_k - cost_0` IS the log-weight gap from the
    // top sibling. When that gap exceeds `alt_birth_log_gap_threshold`
    // for K-child `k >= 1`, the new-target row Bernoullis in that
    // child are SUPPRESSED — the assignment cell is still feasible
    // and consumes ρ_total mass through the log_weight (so the K-best
    // mixture stays balanced), but no fresh Bernoulli is materialised
    // for the suppressed birth. Detection-row and misdetection-row
    // contributions of the alt child are kept (those represent
    // genuine assignment ambiguity on existing tracks, not phantom
    // mass).
    //
    // Different from `k_best_dominance_log_gap` (M2): that knob drops
    // the WHOLE child past the threshold; this knob keeps the child
    // but filters its phantom-birth contribution. M2 probe showed
    // log_gap=1.0 doesn't separate phantoms (gap 0.69 nat) from
    // legitimate close alts; the lineage gate solves the same problem
    // at a tighter level.
    //
    // <= 0 disables (bit-identical to Phase 9 S1/S2 baseline). Typical
    // bench value 0.3-1.0; tighter than k_best_dominance_log_gap
    // because only births are gated, not whole hypotheses. Active
    // only when k_override >= 2 (single-child enumeration has no
    // top-vs-alt comparison).
    double alt_birth_log_gap_threshold = 0.0;

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

  // Task 6: optional cooperative stale-signal sink. When non-null, fires
  // onTrackStale(id, now) once per scan per track whose cooperative own-
  // identity report is overdue (spec §9c: "we lost comms"). MUST NOT be
  // wired to anything that reduces existence — pure notification only.
  void setStaleSignalSink(IStaleSignalSink* s) { stale_sink_ = s; }

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

  // Task 4: optional sensor-activity port. When null (default) the tracker
  // behaves exactly as before Task 4 (bit-identical). When set and
  // cfg_.use_sensor_activity == true, the misdetection step consults the
  // port to distinguish genuine surveillance misses from idle intervals.
  void setSensorActivity(const ISensorActivity* a) { sensor_activity_ = a; }

  // Task A: optional land/coastline clutter prior. When null (default) or
  // cfg_.use_land_model == false, behaviour is bit-identical to today's.
  // When set and use_land_model = true, the adaptive-birth intensity is
  // scaled by (1 − clutterPrior(birth_pos)) for each candidate.
  void setLandModel(const ILandModel* m) { land_model_ = m; }

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
  // `k_override` < 0 → use cfg_.k_best_per_hypothesis (legacy).
  // `k_override` ≥ 1 → use that K (Phase 8 P2 adaptive K path).
  //
  // `parent_idx` ≥ 0 enables the Phase 8 iter 5 birth-id cache: all
  // K children of one parent that birth a new-target Bernoulli for
  // the same measurement reuse the same BernoulliId, so the
  // `mergeBernoulliDuplicates` step can fold them by id and the
  // hypothesis-weight prune sees coherent existence mass per id.
  // < 0 disables caching (bit-identical to legacy allocation).
  void enumerateChildren(const GlobalHypothesis& parent,
                         const std::vector<Measurement>& scan,
                         const std::vector<NewTargetCandidate>& nts,
                         std::vector<GlobalHypothesis>& out,
                         int k_override = -1,
                         int parent_idx = -1);

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
  // Phase 8 iter 5 birth-id cache: (parent_idx, measurement_idx) → id.
  // Cleared at the start of each processBatch. Populated when adaptive
  // K is enabled so siblings of the same parent for the same
  // measurement share a BernoulliId — see enumerateChildren docstring.
  std::map<std::pair<int, int>, BernoulliId> scan_birth_id_cache_;
  const ISensorBiasProvider* bias_provider_{nullptr};
  std::shared_ptr<ISensorDetectionModel> detection_model_;
  const ISensorActivity* sensor_activity_{nullptr};
  const ILandModel* land_model_{nullptr};
  // Task 5: per-Bernoulli "last activity check" timestamp for the sensor-
  // activity path. Keyed by BernoulliId; absent = use b.birth_time as the
  // window start.  Updated to current_time_ each time a surveillance miss
  // fires, so only ONE miss per duty_cycle is charged regardless of scan rate.
  // Cleaned up alongside contribution_history_ when Bernoullis are pruned.
  std::map<BernoulliId, Timestamp> last_activity_check_;
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
  // Task 6: optional cooperative stale-signal sink.
  IStaleSignalSink* stale_sink_{nullptr};
  // Task 6: set of Bernoulli ids whose cooperative own-identity report was
  // overdue in the current scan (populated by enumerateChildren, cleared at
  // the start of each processBatch, read by refreshAggregatedTracks to set
  // TrackStatus::Coasting and by processBatch to fire onTrackStale).
  mutable std::set<BernoulliId> cooperative_overdue_ids_;
  // Task 6: last time a cooperative source's measurement was DETECTED for
  // each Bernoulli. Used as the "last own-identity report time" for the
  // cooperative-only retirement timeout. Falls back to b.birth_time when
  // absent (freshly born / never detected by a cooperative source).
  // Cleaned up alongside contribution_history_ when Bernoullis are pruned.
  std::map<BernoulliId, Timestamp> last_cooperative_touch_;
  // Task 5 Step-4 fix: per-scan staging maps for hypothesis-consistent
  // surveillance-miss / cooperative-touch windows. enumerateChildren runs
  // once PER PARENT global hypothesis; the same BernoulliId appears across
  // many parents. Mutating last_activity_check_ / last_cooperative_touch_
  // mid-enumeration makes a later parent read a window a sibling parent
  // just advanced, so the result depends on parent order. Instead every
  // parent READS the persistent map as a read-only snapshot for the whole
  // scan, and WRITES the resolved-this-scan time into these staging maps
  // (max-merged so order-independent). processBatch merges the staging
  // maps into the persistent maps ONCE, after all parents are enumerated.
  // Both stay empty (inert, bit-identical) when use_sensor_activity is off.
  std::map<BernoulliId, Timestamp> staged_activity_check_;
  std::map<BernoulliId, Timestamp> staged_cooperative_touch_;
  // Stage a resolved-this-scan window time, keeping the latest (max) value
  // so the result is independent of which parent/assignment wrote first.
  void stageActivityCheck(BernoulliId id, Timestamp t);
  void stageCooperativeTouch(BernoulliId id, Timestamp t);
  // Returns the birth-intensity scale in [0, 1] for a birth at ENU position
  // `mean.head<2>()`, or a negative value meaning "hard-drop this birth":
  //   1.0   — no land model wired or use_land_model=false (bit-identical)
  //   [0,1) — soft suppression: scale lambda_birth by this factor
  //   < 0   — inland hard gate: set lambda_birth to 0 (no Bernoulli)
  double landBirthScale(const Eigen::VectorXd& mean) const {
    if (!cfg_.use_land_model || land_model_ == nullptr || mean.size() < 2)
      return 1.0;
    const double c = land_model_->clutterPrior(mean.head<2>());
    if (c > cfg_.land_birth_hard_gate) return -1.0;  // hard-drop
    return 1.0 - c;                                   // soft scale
  }

  // Returns true when a measurement from sensor `s` should update the
  // cooperative-touch timer. When an activity provider is wired and has a
  // profile for `s`, the profile's ChannelKind decides. Falls back to the
  // legacy `s == SensorKind::Cooperative` check when no provider is wired
  // or the provider has no profile for `s` — bit-identical to legacy in
  // both cases.
  bool isCooperativeSource(SensorKind s) const {
    if (sensor_activity_) {
      const auto k = sensor_activity_->channelKindFor(s);
      if (k.has_value()) return *k == ChannelKind::Cooperative;
    }
    return s == SensorKind::Cooperative;
  }
  // Apply the staging maps to the persistent maps (called once per scan,
  // after every parent has been enumerated). No-op when both are empty.
  void mergeStagedActivityMaps();
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
