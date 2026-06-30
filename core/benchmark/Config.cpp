#include "core/benchmark/Config.hpp"

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "ports/ISensorDetectionModel.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/estimation/NoiseModels.hpp"
#include "core/estimation/UkfEstimator.hpp"

namespace navtracker {
namespace benchmark {

namespace {

// Canonical constants pulled from existing scenario tests so the baselines
// reflect "what the repo currently considers reasonable defaults":
//
// EKF / UKF + CV2D: q_a = 0.1, init_speed_std = 5.0
//   (tests/scenario/test_crossing.cpp:26-28,
//    tests/scenario/test_filter_comparison.cpp:60-62)
// UKF + CT (5-state): q_a = 0.5, q_omega = 0.1, init_speed_std = 10.0
//   (tests/scenario/test_filter_comparison.cpp:249-262)
// IMM (CV5State + CT): pi = [[0.95, 0.05], [0.10, 0.90]],
//                      mu0 = [0.5, 0.5], q_a = 0.5, q_omega = 0.1 (CT),
//                      q_a_cv5 = 0.5, q_omega_cv5 = 0.01,
//                      init_speed_std = 10.0, init_omega_std = 0.1
//   (tests/scenario/test_filter_comparison.cpp:254-262)
// GNN gate threshold: 50.0 (chi-square on squared Mahalanobis)
//   (tests/scenario/test_crossing.cpp:28)
// JPDA: gate = 20.0, P_D = 0.9, lambda_c = 1e-4
//   (tests/scenario/test_jpda_comparison.cpp:57)

constexpr double kCv2dAccelPsd = 0.1;
constexpr double kCv2dInitSpeedStd = 5.0;

constexpr double kCtAccelPsd = 0.5;
constexpr double kCtOmegaPsd = 0.1;
constexpr double kCtInitSpeedStd = 10.0;

constexpr double kImmCv5AccelPsd = 0.5;
constexpr double kImmCv5OmegaPsd = 0.01;
constexpr double kImmCtAccelPsd = 0.5;
constexpr double kImmCtOmegaPsd = 0.1;
constexpr double kImmInitSpeedStd = 10.0;
constexpr double kImmInitOmegaStd = 0.1;
// Cl-2 #2 (b) — init-cov widening — measured against the gated UKF
// canonical 2026-06-20 (bench cl25_life_20260620.csv) and REJECTED.
// 10.0/0.1 → 15.0/0.2 was meant to fix env-1 sc3 unanchored median
// NEES (15 in canonical, "should be ~1.4"). Result: sc3 median went
// 15.0 → 17.6 (worse), autoferry-unanchored mean GOSPA +4.3%, env-2
// anchored mean GOSPA +17.1%. Wider gates pulled extra measurements
// into competing branches → hypothesis-tree cardinality bloat. Do
// not retry without first addressing the cardinality side of the
// problem (e.g. tighter MHT pruning, JIPDA on the sibling pipeline
// per Cl-1). See eval-log "Cl-2 #2 (a)+(b) close-out".

// Noisy-CV third mode for the 3-mode IMM (CV + CT + noisy-CV). Same
// CV5State motion model with the accel PSD inflated 50× — the
// standard Mazor 1998 maritime recipe for "the target is doing
// something neither CV nor CT can explain" (e.g. sudden speed change
// without a heading turn, drifting under thrust loss). 10×–100× is
// the conventional range; 50× is a midpoint we'll tune from data.
constexpr double kImmCv5NoisyAccelPsd = 25.0;
constexpr double kImmCv5NoisyOmegaPsd = 0.01;

constexpr double kGnnGate = 50.0;
constexpr double kJpdaGate = 20.0;
constexpr double kJpdaPd = 0.9;
constexpr double kJpdaClutterDensity = 1e-4;

std::shared_ptr<IEstimator> makeEkfCv() {
  auto motion = std::make_shared<ConstantVelocity2D>(kCv2dAccelPsd);
  return std::make_shared<EkfEstimator>(motion, kCv2dInitSpeedStd);
}

std::shared_ptr<IEstimator> makeUkfCv() {
  auto motion = std::make_shared<ConstantVelocity2D>(kCv2dAccelPsd);
  return std::make_shared<UkfEstimator>(motion, kCv2dInitSpeedStd);
}

std::shared_ptr<IEstimator> makeUkfCt() {
  auto motion = std::make_shared<CoordinatedTurn>(kCtAccelPsd, kCtOmegaPsd);
  return std::make_shared<UkfEstimator>(motion, kCtInitSpeedStd);
}

// CANONICAL IMM (CV5 + CT). Inner filter is UKF (sigma-point) per
// mode, promoted from EKF on 2026-06-20 (Cl-2 #3 close-out). See
// eval-log entry; pinned bench cl23_ukf_full_20260619.csv. The EKF
// inner filter is preserved as the imm_cv_ct_mht_ekf ablation.
std::shared_ptr<IEstimator> makeImmCvCt() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(kImmCv5AccelPsd,
                                               kImmCv5OmegaPsd),
      std::make_shared<CoordinatedTurn>(kImmCtAccelPsd, kImmCtOmegaPsd)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  return std::make_shared<ImmEstimator>(motions, pi, mu0,
                                        kImmInitSpeedStd, kImmInitOmegaStd,
                                        /*noise=*/nullptr,
                                        /*bearing_range_guard=*/false,
                                        /*use_ukf=*/true);
}

// EKF inner filter for the canonical IMM. The 2026-06-20 pre-UKF
// canonical, preserved as an ablation so the "did UKF land cleanly?"
// question stays measurable.
std::shared_ptr<IEstimator> makeImmCvCtEkf() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(kImmCv5AccelPsd,
                                               kImmCv5OmegaPsd),
      std::make_shared<CoordinatedTurn>(kImmCtAccelPsd, kImmCtOmegaPsd)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  return std::make_shared<ImmEstimator>(motions, pi, mu0,
                                        kImmInitSpeedStd, kImmInitOmegaStd);
}

// Canonical IMM + bearing range-variance guard (backlog item 12
// suspect b). The Joseph-form bearing update can drive along-LOS
// position variance below its predicted value through cross-coupling,
// leaving the filter overconfident in range it never measured. The
// guard restores the predicted LOS-direction variance post-update
// while preserving the legitimate cross-LOS reduction. Inherits the
// UKF inner filter from canonical post-2026-06-20.
std::shared_ptr<IEstimator> makeImmCvCtBearGuard() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(kImmCv5AccelPsd,
                                               kImmCv5OmegaPsd),
      std::make_shared<CoordinatedTurn>(kImmCtAccelPsd, kImmCtOmegaPsd)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  return std::make_shared<ImmEstimator>(motions, pi, mu0,
                                        kImmInitSpeedStd, kImmInitOmegaStd,
                                        /*noise=*/nullptr,
                                        /*bearing_range_guard=*/true,
                                        /*use_ukf=*/true);
}

// 3-mode IMM: CV5State (cruising), CoordinatedTurn (turning),
// noisy CV5State (unmodelled motion). The standard Mazor 1998
// maritime recipe — see docs/algorithms/algorithm-review-2026-06-07.md
// and the "Planned: noisy-CV third mode" note in estimation.md.
// Transition matrix is a symmetric "escape-hatch" topology: each
// mode is dominant (0.90 self-stay) and equally likely to switch to
// either of the other two.
std::shared_ptr<IEstimator> makeImmCvCtNoisy() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(kImmCv5AccelPsd,
                                               kImmCv5OmegaPsd),
      std::make_shared<CoordinatedTurn>(kImmCtAccelPsd, kImmCtOmegaPsd),
      std::make_shared<ConstantVelocity5State>(kImmCv5NoisyAccelPsd,
                                               kImmCv5NoisyOmegaPsd)};
  Eigen::MatrixXd pi(3, 3);
  pi << 0.90, 0.05, 0.05,
        0.05, 0.90, 0.05,
        0.05, 0.05, 0.90;
  Eigen::VectorXd mu0(3);
  mu0 << 0.45, 0.45, 0.10;  // bias prior toward CV/CT; noisy is the
                            // fallback, not the default expectation.
  return std::make_shared<ImmEstimator>(motions, pi, mu0,
                                        kImmInitSpeedStd, kImmInitOmegaStd,
                                        /*noise=*/nullptr,
                                        /*bearing_range_guard=*/false,
                                        /*use_ukf=*/true);
}

// Same IMM as makeImmCvCt but with a Student-t robust measurement model
// (ν=4) injected. Down-weights outlier measurements (EO/IR bearing clutter
// that slips the gate) instead of letting them pull the state at full
// weight. ν=4 is the roadmap's clutter-prone-sensor default.
std::shared_ptr<IEstimator> makeImmCvCtRobust() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(kImmCv5AccelPsd,
                                               kImmCv5OmegaPsd),
      std::make_shared<CoordinatedTurn>(kImmCtAccelPsd, kImmCtOmegaPsd)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  auto noise = std::make_shared<StudentTNoiseModel>(4.0);
  return std::make_shared<ImmEstimator>(motions, pi, mu0, kImmInitSpeedStd,
                                        kImmInitOmegaStd, noise,
                                        /*bearing_range_guard=*/false,
                                        /*use_ukf=*/true);
}

std::shared_ptr<IDataAssociator> makeGnn() {
  return std::make_shared<GnnAssociator>(kGnnGate);
}

std::shared_ptr<IDataAssociator> makeJpda() {
  return std::make_shared<JpdaAssociator>(kJpdaGate, kJpdaPd,
                                          kJpdaClutterDensity);
}

// Per-sensor JPDA: the model is the scenario's per-sensor (P_D, λ_C)
// table, looked up per measurement instead of one scalar pair. Brings
// JPDA to parity with MHT on per-sensor units (backlog item 8). Reuses
// the same gate threshold kJpdaGate.
std::shared_ptr<IDataAssociator> makeJpdaPerSensor(
    const std::shared_ptr<ISensorDetectionModel>& model) {
  return std::make_shared<JpdaAssociator>(kJpdaGate, model.get());
}

}  // namespace

namespace {

// MHT tracker canonical config. gate_threshold matches kJpdaGate
// (20.0) so MHT branches see the same gate volume the soft associator
// uses; the rest are the MhtTracker::Config defaults — which since
// 2026-06-11 means the IPDA + VIMM existence/visibility lifecycle
// with confirm 0.9 / demote 0.6 hysteresis (measured bit-identical to
// M-of-N on clean synthetics, decisively better under misses/clutter;
// see evaluation-log 2026-06-11). Bhattacharyya merging at 1.0,
// N-scan = 3.
MhtTracker::Config makeMhtConfig() {
  MhtTracker::Config cfg;
  cfg.gate_threshold = kJpdaGate;
  cfg.probability_of_detection = kJpdaPd;
  cfg.clutter_density = kJpdaClutterDensity;
  // Cl-2 #2 (a) — lifecycle re-tune (ipda_persistence 0.99 → 0.995,
  // ipda_delete_threshold 0.05 → 0.02) measured against the gated
  // UKF canonical 2026-06-20 (bench cl25_life_20260620.csv) and
  // REJECTED. Looser lifecycle kept *false-track* tentatives alive
  // alongside real ones → cardinality bloat → anchored GOSPA +17%
  // (sc3 +56%, sc4 +26%). Unanchored NEES median on the target
  // scenario (sc3) went 15.0 → 17.6 — the wrong direction. Header
  // defaults retained. See eval-log "Cl-2 #2 (a)+(b) close-out".
  return cfg;
}

// Adaptive recapture-gate ablation (backlog item 11). Position gate
// scales with the hypothesis' position-anchor age: gate = base ·
// min(max_scale, 1 + age/τ). Targets the bearing-carried-drift +
// radar-miss + duplicate-birth conveyor. Measured June 12 to drop
// sc5 ID switches 91 → 43 but at heavy lifetime cost (sc3 0.87 →
// 0.63) — pre item-12(a). Re-measuring with honest per-env R.
MhtTracker::Config makeMhtRecaptureConfig() {
  MhtTracker::Config cfg = makeMhtConfig();
  cfg.gate_recapture_tau_s = 2.0;       // June-12 value
  cfg.gate_recapture_max_scale = 8.0;   // default
  return cfg;
}

// IPDA-only ablation: existence lifecycle without the visibility
// channel — isolates what VIMM's obscuration handling adds on top of
// plain Musicki 1994.
MhtTracker::Config makeMhtIpdaConfig() {
  MhtTracker::Config cfg = makeMhtConfig();
  cfg.use_visibility = false;
  return cfg;
}

// M-of-N ablation: the pre-2026-06-11 lifecycle (2-of-3 hit-count
// confirmation, score-threshold deletion). Kept to measure what the
// existence lifecycle buys.
MhtTracker::Config makeMhtMofnConfig() {
  MhtTracker::Config cfg = makeMhtConfig();
  cfg.use_ipda_lifecycle = false;
  cfg.use_visibility = false;
  return cfg;
}

// PMBM Phase 1 config. Same (P_D, λ_C, gate) as the MHT canonical so
// the A/B isolates the association/lifecycle algorithm only.
// Measurement-driven birth (every initiable measurement seeds a fresh
// PoissonComponent — see PmbmTracker.cpp) is the de-facto standard
// Phase 1 birth model (García-Fernández 2018 §IV-D, MTT toolbox
// `DBM_filter'). Murty K=3 to match MhtTracker; total mixture capped
// at 30 globals. r_min / weight_min / hypothesis_weight_min at the
// defaults. confirm_threshold=0.5 emits a track once it carries half
// the aggregated existence mass.
pmbm::PmbmTracker::Config makePmbmConfig() {
  pmbm::PmbmTracker::Config cfg;
  cfg.gate_threshold = kJpdaGate;
  cfg.probability_of_detection = kJpdaPd;
  cfg.clutter_intensity = kJpdaClutterDensity;
  cfg.measurement_driven_birth = true;
  // Per-measurement birth intensity. The Phase 1 sweet spot tradeoff:
  // higher → faster track confirmation but id flapping when an
  // existing high-r Bernoulli sits on the same location; lower →
  // stable ids but slow / missed initiation in noisy scenarios. 0.3
  // is the middle-ground starting point for the first A/B; replace
  // with proper measurement-conditioned birth (only when no existing
  // Bernoulli gates) in Phase 1.5.
  cfg.birth_weight_per_measurement = 0.3;
  // Smart birth (Reuter 2014 Adaptive Birth Distribution): skip
  // birth at measurements already explained by an existing high-r
  // Bernoulli. Phase 1 measurement-driven birth without this gate
  // produced 100-170 id_switches per autoferry scenario (the
  // dominant Phase 1 regression in pmbm_phase1_first_ab_20260620);
  // turning it on cuts those flaps to MHT-comparable levels without
  // touching the Cl-2 #2 wins.
  cfg.smart_birth_skip_existing = true;
  cfg.smart_birth_skip_r_min = 0.5;
  cfg.smart_birth_skip_gate = kJpdaGate;
  // Phase 3 polish: PPP-coverage birth gate is available as a knob
  // (Config::smart_birth_skip_existing_ppp / *_threshold) but left
  // OFF in the bench config. Measured effect across thresholds
  // 1e-4 / 1e-5 / 1e-6 / 3e-6: marginal philos improvement (-2 to
  // -4 GOSPA) but consistent autoferry_scenario4_anchored
  // regression (+2.3) because the gate also suppresses real target
  // re-birth in tight, bias-corrected anchored cases. Closing the
  // structural philos/dense_clutter gaps requires Reuter (2014)
  // Adaptive Birth (decouple spatial birth from existence prior) —
  // parked in docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md.

  // Phase 4 (TPMBM): forward-pass trajectory recording per Bernoulli.
  // Window = 50 scans gives ~50 s of history at 1 Hz (typical bench
  // scenario), enough to expose the full ferry pass via
  // PmbmTracker::trajectoryFor(id). Zero algorithmic effect on
  // per-scan tracking; bench MUST stay bit-identical vs Phase 3.
  cfg.trajectory_window_scans = 50;
  // Source-aware misdetection: skip the misdetection recursion for
  // Bernoullis whose contributing source_ids don't appear in this
  // scan. Critical for sparse-broadcast sensors (AIS philos) where
  // vessel A's broadcast tells us nothing about vessel B's
  // existence; without this every other vessel's broadcast kills
  // our Bernoulli in O(1) scan and philos lifetime collapses to ~0.
  cfg.source_aware_misdetection = true;
  // K=1 (single best assignment per parent) for Phase 1: keeps the
  // existence mass concentrated on the dominant interpretation and
  // matches MhtTracker's K=1 global-hypothesis emission mode. K>1
  // re-introduces deferred-decision alternatives — measured to
  // *hurt* on the first A/B before within-id Bernoulli merging
  // ships (§3.5 of pmbm-design.md), so left at 1 for now.
  cfg.k_best_per_hypothesis = 1;
  cfg.max_global_hypotheses = 10;
  cfg.max_ppp_components = 200;
  cfg.confirm_threshold = 0.5;
  cfg.output_existence_floor = 0.1;
  // r_min: MATLAB MTT-master uses `existence_threshold = 1e-5`
  // (TPMBM_alive_filter.m); our PmbmTracker.hpp default 1e-3 was
  // 100× tighter and dropped legitimate low-r Bernoullis before
  // posterior could ramp them on a late detection. Loosen the
  // bench config (not the global default) to MATLAB parity
  // (Phase 8 R5 fix).
  cfg.r_min = 1e-5;
  // Phase 3 polish (closes the two named Phase 2 gaps):
  //
  // (A) Idle-decay half-life. Source-aware misdetection (above) skips
  //     the textbook recursion for Bernoullis whose contributing
  //     sources are absent from a scan, which is correct but leaves
  //     ghost tracks alive indefinitely on philos-style sparse-AIS
  //     scans. 60 s = a real maritime target is expected to report
  //     within ~1 minute; below this it costs real tracks essentially
  //     nothing (each AIS broadcast resets b.last_update), above it a
  //     ghost decays r → 0.5 r per minute and crosses r_min in
  //     ~10 minutes — empirically the right shape for the philos +43 %
  //     gap left after Phase 2.
  cfg.idle_halflife_sec = 10.0;
  // (B) Phantom-birth gate. Under dense_clutter the per-measurement
  //     birth row produces r_new ≈ 0 Bernoullis that cost cardinality
  //     for one scan before r_min prunes them — and within that scan
  //     they show up as id-flap candidates. Gating at r_new ≥ 0.05
  //     suppresses the phantom entirely while still consuming the
  //     clutter mass (assignment stays balanced); real targets have
  //     r_new ≥ 0.5 under any reasonable (P_D, λ_C) so the threshold
  //     never blocks a true initiation.
  cfg.min_new_bernoulli_existence = 0.5;
  return cfg;
}

}  // namespace

std::vector<Config> defaultConfigs() {
  std::vector<Config> configs;
  configs.reserve(10);
  // CANONICAL config, listed first. IMM (CV + CT) inside TOMHT with
  // the IPDA + VIMM existence/visibility lifecycle (the MhtTracker
  // defaults) is the tracker navtracker treats as its reference — the
  // baseline PMBM will be measured against. The remaining configs are
  // ablations that isolate one axis (estimator / motion model /
  // associator / lifecycle) away from it.
  //
  // The canonical wires the per-(sensor, source_id) registration
  // bias estimator (item 9: position pairs via AIS anchor; bearing
  // pairs via the bearing-bias variant). On scenarios with no AIS
  // source the estimator stays at its prior and `is_published` is
  // false — Tracker / MhtTracker's bias-correction call site
  // returns measurements unchanged. The cost is one extra factory
  // construction + a PostScanHook closure per cell; bit-identical
  // to the legacy null-provider path until an anchor appears.
  {
    Config c{"imm_cv_ct_mht", &makeImmCvCt, &makeJpda,
             TrackerKind::Mht, &makeMhtConfig};
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
  // Same as canonical but with Student-t robust updates — the ablation
  // that isolates the heavy-tailed-measurement (EO/IR clutter) axis.
  configs.push_back({"imm_cv_ct_mht_robust", &makeImmCvCtRobust, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  // Canonical minus the visibility channel — isolates what VIMM's
  // obscuration handling adds over plain IPDA. Also functionally the
  // canonical minus *both* the bias estimator and visibility (no
  // build_sensor_bias_estimator set), so it doubles as the
  // "_nobias_novis" ablation in the step-0 step-0 disambiguation.
  configs.push_back({"imm_cv_ct_mht_ipda", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtIpdaConfig});
  // Canonical minus the bias estimator only (visibility ON). With
  // imm_cv_ct_mht_novis below, this triple (canonical / _nobias /
  // _novis) separates the two axes that differentiate the canonical
  // from the otherwise-strongest sc13_anchored ablations. Measured
  // 2026-06-18: canonical NEES 73.3 vs ipda/recapture/bearguard ~25
  // on sc13_anchored, but those drop *both* bias and visibility.
  // _nobias / _novis isolate which knob is responsible.
  {
    Config c{"imm_cv_ct_mht_nobias", &makeImmCvCt, &makeJpda,
             TrackerKind::Mht, &makeMhtConfig};
    configs.push_back(std::move(c));
  }
  // Canonical minus visibility only (bias estimator wired).
  {
    Config c{"imm_cv_ct_mht_novis", &makeImmCvCt, &makeJpda,
             TrackerKind::Mht, &makeMhtIpdaConfig};
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
  // Canonical minus the existence lifecycle (pre-2026-06-11 M-of-N +
  // score-delete) — isolates what calibrated existence buys.
  configs.push_back({"imm_cv_ct_mht_mofn", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtMofnConfig});
  // Canonical plus the spatial clutter map (backlog item 5) — isolates
  // what spatially-resolved λ_C buys over the per-sensor scalar table
  // on structured (shoreline) clutter.
  configs.push_back({"imm_cv_ct_mht_cmap", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig,
                     /*use_clutter_map=*/true});
  // Cl-2 #3 close-out (2026-06-20): the pre-2026-06-20 canonical
  // (EKF inner filter) preserved as an ablation. Use this to ask
  // "what does the new UKF inner filter buy us?" on any scenario.
  // Sensor bias estimator wired (matches canonical).
  {
    Config c{"imm_cv_ct_mht_ekf", &makeImmCvCtEkf, &makeJpda,
             TrackerKind::Mht, &makeMhtConfig};
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
  // Canonical plus the bearing range-variance guard (backlog item 12
  // suspect b). Isolates the BOT pathology fix without changing any
  // other tracker mechanism. Expected effect on AutoFerry sc5:
  // position NIS (radar / lidar) drops toward consistent; nees_mean
  // collapses from ~80 toward O(few).
  configs.push_back({"imm_cv_ct_mht_bearguard", &makeImmCvCtBearGuard,
                     &makeJpda, TrackerKind::Mht, &makeMhtConfig});
  // Canonical plus the adaptive recapture-gate (backlog item 11).
  // Position gate widens with the hypothesis' position-anchor age so a
  // bearing-carried-drift track can still gate the next radar return.
  // June-12 measurement showed strong sc5 ID-switch reduction but heavy
  // lifetime cost; the latter was traced to overconfident covariance
  // (item 12). Re-measured on top of per-env R defaults.
  configs.push_back({"imm_cv_ct_mht_recapture", &makeImmCvCt,
                     &makeJpda, TrackerKind::Mht, &makeMhtRecaptureConfig});
  // JPDA/GNN-style ablations (single-hypothesis Tracker pipeline).
  configs.push_back({"ekf_cv_gnn", &makeEkfCv, &makeGnn});
  configs.push_back({"ekf_cv_jpda", &makeEkfCv, &makeJpda});
  // Per-sensor JPDA (backlog item 8): same EKF/CV pipeline, but JPDA
  // looks up (P_D, λ_C) per measurement from the scenario's table.
  // Bit-identical to ekf_cv_jpda on scenarios with no detection table
  // (the bench falls back to the scalar factory in that case).
  {
    Config c{"ekf_cv_jpda_persensor", &makeEkfCv, &makeJpda};
    c.build_associator_per_sensor = &makeJpdaPerSensor;
    configs.push_back(std::move(c));
  }
  configs.push_back({"ukf_cv_gnn", &makeUkfCv, &makeGnn});
  configs.push_back({"ukf_ct_gnn", &makeUkfCt, &makeGnn});
  configs.push_back({"imm_cv_ct_jpda", &makeImmCvCt, &makeJpda});
  {
    Config c{"imm_cv_ct_jpda_persensor", &makeImmCvCt, &makeJpda};
    c.build_associator_per_sensor = &makeJpdaPerSensor;
    configs.push_back(std::move(c));
  }
  configs.push_back({"imm_cv_ct_noisy_jpda", &makeImmCvCtNoisy, &makeJpda});
  // Other MHT ablations (MhtTracker pipeline). associator factory unused;
  // we still pass it because the Config struct ergonomics expect it
  // populated. The MhtTracker constructs its own gating + association
  // via TrackTree::branch internally.
  configs.push_back({"ekf_cv_mht", &makeEkfCv, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  configs.push_back({"imm_cv_ct_noisy_mht", &makeImmCvCtNoisy, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});

  // Cl-3 Phase 1 PMBM (GM, no IMM-per-Bernoulli yet — the IMM is the
  // *inner* estimator inside each Bernoulli's single Gaussian, but the
  // Bernoulli density itself is single-Gaussian; Phase 2 upgrades it to
  // a full IMM mixture). Sibling to imm_cv_ct_mht; same (P_D, λ_C,
  // gate, estimator), different association/lifecycle algorithm.
  // Headline A/B comparison.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;  // unused
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = &makePmbmConfig;
    // Same SensorBiasEstimator wiring as the MHT canonical so the
    // PMBM A/B is apples-to-apples on the AIS-anchored variants
    // (Schmidt-KF measurement correction is unchanged composing in
    // front of either tracker).
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Cl-3 Phase 7 — Adaptive Birth (Reuter 2014). Decouples spatial
  // birth from existence prior to fix the philos / dense_clutter
  // measurement-driven-birth contamination (see parking lot in
  // 2026-06-07-pmbm-integration-plan.md). A/B against imm_cv_ct_pmbm
  // until tuned in.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_adapt";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      // Phase 8 P2 adaptive K-best — parked OFF in the bench config.
      // Iter 1: K=5 measured big philos/dense_clutter/sc4 wins
      // (-15..-17 %) AND structural sc13/sc16/sc22 anchored
      // regressions (+25..+33 %). Tighter merge threshold (iter 3)
      // did not recover. Iter 5 added a birth-id cache so siblings
      // of one parent share a BernoulliId for the same measurement
      // — re-measured at K=5: regressions UNCHANGED. The real cause
      // is structural (our flat per-hypothesis Bernoulli list vs
      // MATLAB's per-track list of single-target hypotheses); the
      // adaptive_k_best switch + scan_birth_id_cache_ remain shipped
      // and correct, ready for the structural refactor.
      cfg.adaptive_k_best = false;
      cfg.k_best_per_hypothesis = 1;
      // λ_birth: expected new-target rate per scan per unit
      // measurement-space volume (same units as λ_C). Tuned across
      // {1e-3, 1e-4, 1e-5} in Phase 7 iter 3-4 against λ_C ≈ 1e-4:
      //   1e-3 (r_new ≈ 0.91): philos +15 % (too aggressive)
      //   1e-4 (r_new = 0.5):  philos +7 %
      //   1e-5 (r_new ≈ 0.09): philos -16 %, dense_clutter -52 %,
      //                         autoferry sc2-6 unanchored -5..-32 %,
      //                         all anchored T-GOSPA-raw -8..-60 %
      // The low value is the right shape: new Bernoullis are born
      // with small r and ramp up via posterior over subsequent
      // detections rather than being pegged near 1 by ρ_target
      // contamination — exactly the textbook PMBM behavior.
      cfg.lambda_birth = 1e-5;
      // Adaptive r_new is small at birth → real targets ramp via
      // posterior, so the phantom gate drops below the initial r.
      cfg.min_new_bernoulli_existence = 0.05;
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Task 1 probe: clutter-invariant birth existence. Same as
  // imm_cv_ct_pmbm_adapt but r_new is pinned to a target instead of a
  // fixed absolute λ_birth — fixes the philos over-confident-birth bug.
  // With birth_existence_target=0.1, λ_birth = (0.1/0.9)·λ_C so r_new=0.1
  // exactly regardless of sensor/scenario λ_C (autoferry 1e-4, philos
  // radar 2.7e-6, AIS 1e-9 all yield r_new=0.1 without manual retuning).
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_birthtarget";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.adaptive_k_best = false;
      cfg.k_best_per_hypothesis = 1;
      cfg.lambda_birth = 1e-5;          // ignored when target > 0
      cfg.birth_existence_target = 0.1; // <-- the knob under test
      cfg.min_new_bernoulli_existence = 0.05;
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Task 3: PMBM with the radar spatial clutter map. Same as
  // imm_cv_ct_pmbm_adapt but the ClutterMapSensorDetectionModel wraps
  // the scenario's fixed detection table, learning a higher λ_C on cells
  // with persistent shore / moored-structure returns. This lowers
  // r_new = λ_birth/(λ_birth+λ_C) on those cells, suppressing phantom
  // births at the birth rate rather than post-hoc via lifecycle pruning.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_cmap";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.k_best_per_hypothesis = 1;
      cfg.adaptive_k_best = false;  // explicit: mirrors imm_cv_ct_pmbm_adapt; safe if default changes
      cfg.lambda_birth = 1e-5;
      cfg.min_new_bernoulli_existence = 0.05;
      return cfg;
    };
    c.use_clutter_map = true;  // radar position map suppresses shore births
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Task 2c — principled PMBM bundle: correct misdetection math (dedup_miss_pd)
  // + clutter-invariant births (birth_existence_target) + per-vessel identity
  // gate (source_aware_identity) + raised cardinality-control floors.
  // dedup_miss_pd in isolation gave +92% philos GOSPA historically
  // (eval-log PHILOS-T1); this bundle (with controlled births + floors)
  // is ~+36% vs adapt — still a philos regression, hence ablation-only.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_bundle";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.k_best_per_hypothesis = 1;
      cfg.adaptive_k_best = false;
      // Task 2c sweep winner: target=0.1, floor=0.1 (lowest gospa 112.0,
      // card_err +46.25 — least overcounting in the sweep). All 6 combinations
      // (target∈{0.1,0.15,0.2} × floor∈{0.1,0.3}) yield gospa 112–119,
      // card_err +46..+57; none beats imm_cv_ct_pmbm_adapt (82.63). The root
      // cause is dedup_miss_pd=true reducing miss penalty → phantom Bernoullis
      // from clutter accumulate to r>0.3 before pruning. output_existence_floor
      // has negligible effect (phantoms' r>0.3). This config ships as ablation
      // documenting the dedup_miss_pd bottleneck.
      cfg.birth_existence_target = 0.1;    // Task 2c sweep winner
      cfg.source_aware_identity = true;    // Task 2a: per-vessel sensor gate
      cfg.dedup_miss_pd = true;            // Task 2b: correct misdetection math
      cfg.min_new_bernoulli_existence = 0.1;  // raised cardinality control (was 0.05)
      cfg.output_existence_floor = 0.1;    // Task 2c sweep: floor has no effect on phantoms (r>0.3)
      cfg.lambda_birth = 1e-5;             // ignored when birth_existence_target > 0
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Task 4 coverage model: honest per-duty-cycle surveillance miss +
  // cooperative stale signal, replacing idle_halflife + wrong-math
  // per-blip miss. Cadence/coverage declared in Sweep.cpp.
  // Clone of imm_cv_ct_pmbm_bundle with use_sensor_activity_model=true,
  // use_sensor_activity=true, idle_halflife_sec=0 (retired), dedup_miss_pd
  // disabled (coverage path bypasses compute_miss_pD wrong-math), and
  // cooperative_stale_timeout_sec for AIS-only track retirement.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_coverage";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;  // unused for Pmbm
    c.tracker_kind = TrackerKind::Pmbm;
    c.use_sensor_activity_model = true;  // Task 4: wire DeclaredSensorActivity
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.k_best_per_hypothesis = 1;
      cfg.adaptive_k_best = false;
      // Same sweep winner as the bundle (target=0.1, floor=0.1).
      cfg.birth_existence_target = 0.1;
      cfg.source_aware_identity = true;
      cfg.min_new_bernoulli_existence = 0.1;
      cfg.output_existence_floor = 0.1;
      cfg.lambda_birth = 1e-5;             // ignored when birth_existence_target > 0
      // Coverage path: honest surveillance miss via ISensorActivity.
      cfg.use_sensor_activity = true;
      // Retire the idle-halflife hack: the honest coverage model owns the
      // surveillance-absence signal; idle_halflife would double-count.
      cfg.idle_halflife_sec = 0.0;
      // The coverage path bypasses compute_miss_pD wrong-math — dedup is
      // irrelevant (and was the load-bearing brake in the bundle).
      cfg.dedup_miss_pd = false;
      // AIS is cooperative here → it never lowers existence via miss math.
      // Cooperative-only/AIS-only tracks must be retired by stale timeout or
      // cardinality grows. 120 s ≈ several missed AIS reports (declared/tunable).
      cfg.cooperative_stale_timeout_sec = 120.0;
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Task 6 — land-prior wiring: same as imm_cv_ct_pmbm_coverage but
  // CoastlineModel (Boston Harbor GeoJSON) suppresses adaptive-birth
  // intensity at land positions. Wiring happens in Sweep.cpp when the
  // scenario declares a coastline_geojson_path (philos only); autoferry
  // and synthetic scenarios have no coastline fixture, so the land model
  // is inert there — the run is bit-identical to imm_cv_ct_pmbm_coverage.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_coverage_land";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;  // unused for Pmbm
    c.tracker_kind = TrackerKind::Pmbm;
    c.use_sensor_activity_model = true;  // coverage model (same as parent)
    c.use_land_model = true;             // Task 6: wire CoastlineModel
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.k_best_per_hypothesis = 1;
      cfg.adaptive_k_best = false;
      cfg.birth_existence_target = 0.1;
      cfg.source_aware_identity = true;
      // Equal to birth_existence_target on purpose. Known consequence (the
      // (E) shore_clutter_nearshore validator measured it): the land soft-ramp
      // (r_new *= 1−c) and this phantom-birth floor are independent
      // multiplicative gates, so when floor == target ANY soft suppression
      // (c>0) drops r_new below the floor — the entire offshore soft band
      // (offshore_halfwidth_m, 50 m) becomes a no-birth zone, and a real
      // vessel within 50 m of shore will not initiate under this config.
      // We accept this: near-land operation is rare, and the alternative
      // (lowering the floor to 0.05 to revive near-shore births) re-admits
      // philos near-shore WATER clutter and regresses the real-data win
      // gospa 73.1→100.0, card_err +6.9→+36.2, gospa_false 3550→9000 — so
      // the 0.1 floor is retained. See docs/algorithms/synthetic-clutter-bench.md.
      cfg.min_new_bernoulli_existence = 0.1;
      cfg.output_existence_floor = 0.1;
      cfg.lambda_birth = 1e-5;
      cfg.use_sensor_activity = true;
      cfg.idle_halflife_sec = 0.0;
      cfg.dedup_miss_pd = false;
      cfg.cooperative_stale_timeout_sec = 120.0;
      cfg.use_land_model = true;   // activate land-prior birth gate
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }

  // Cl-3 Phase 9 — adaptive K with cap=3, shipped alongside K=1
  // imm_cv_ct_pmbm_adapt. Same-run 10-seed pinned baseline at
  // docs/baselines/pmbm_adapt_k3_phase9_20260623.csv.
  //
  // K=3 vs K=1 is a STRICTLY PER-SCENARIO, PER-METRIC tradeoff.
  // M1 iter-2 review found 28 exceptions to the natural rubrics
  // ("K=3 for id-stability" — wrong on 10 scenarios; "K=1 for
  // velocity-RMSE" — wrong on 18 scenarios). There is no scenario-
  // class shortcut; some scenarios are strict K=3 losses on every
  // headline metric (gospa + id_switches + pos_rmse all worse —
  // sc13, sc2_anc, sc5_anc, sc6_anc), some are strict K=3 wins,
  // most are mixed.
  //
  // Consumers must compute their own (config, scenario, metric)
  // table against their target workload:
  //
  //   python3 tools/bench_diff.py \\
  //     docs/baselines/pmbm_adapt_k3_phase9_20260623.csv \\
  //     docs/baselines/pmbm_adapt_k3_phase9_20260623.csv \\
  //     --metric <metric> --all-metrics
  //
  // (point both sides at the baseline — bench_diff joins on
  //  (config, scenario, metric) within the file). Or split the
  //  CSV by config and diff externally.
  //
  // The strongest single result: philos (real-world replay)
  // gospa_mean -17.23 % at K=3, but id_switches +50 % and
  // sog_rmse +16.75 %. Decide by whether the consumer is graded
  // on cardinality+position (pick K=3) or operator-facing track
  // continuity+kinematics (stay on K=1).
  //
  // Big known regressions (K=3 worse): autoferry_scenario13_
  // anchored (gospa +44.97 %, NEES +553 %); autoferry_scenario16_
  // anchored (gospa +38.56 %, sog_rmse +249 %, NEES +167 %);
  // non_cooperative (gospa +17.04 %). Phase 9 design doc lists
  // the open hypotheses for the future fix.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_adapt_k3";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.adaptive_k_best = true;
      cfg.k_best_per_hypothesis = 3;
      cfg.lambda_birth = 1e-5;
      cfg.min_new_bernoulli_existence = 0.05;
      // Phase 9 S4: cross-parent birth-id cache. Folded in 2026-06-23
      // after the _xparent probe recovered the autoferry-anchored
      // regressions (sc13_anc +44.97 % → +1.30 % vs K=1, sc16_anc
      // +38.56 % → +7.49 %, sc22_anc +14.24 % → +2.16 %). Tradeoff:
      // loses the prior K=3 philos GOSPA -17.23 % win (back to K=1
      // level), but gains philos pos_rmse -25.93 %, sog_rmse
      // -37.90 %, nees -10.46 % — the GOSPA win was an id-
      // fragmentation counting artifact; per-track state under
      // xparent is strictly better. See
      // docs/superpowers/specs/2026-06-23-pmbm-phase9-per-track-hypotheses.md
      // for the full mechanism explanation and bench tables.
      cfg.cross_parent_birth_id_cache = true;
      // Off-by-default probe knobs intentionally NOT enabled here.
      // Each was a probe that taught us about the mechanism without
      // becoming the right default:
      //   - alt_birth_log_gap_threshold (S3 lineage gate): partial
      //     fix with hacky mass accounting; superseded by xparent.
      //   - output_merge_bhattacharyya_threshold (M3 Option A): can't
      //     discriminate phantom from legitimate close-parallel at
      //     output layer.
      //   - k_best_dominance_log_gap (M2): drops whole alts;
      //     sacrifices philos.
      // All three remain wired + unit-tested for per-consumer
      // ablation but the bench K=3 config does NOT enable them.
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
  return configs;
}

}  // namespace benchmark
}  // namespace navtracker
