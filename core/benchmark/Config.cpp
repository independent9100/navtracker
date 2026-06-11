#include "core/benchmark/Config.hpp"

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
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
                                        kImmInitSpeedStd, kImmInitOmegaStd);
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
                                        kImmInitSpeedStd, kImmInitOmegaStd);
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
                                        kImmInitOmegaStd, noise);
}

std::shared_ptr<IDataAssociator> makeGnn() {
  return std::make_shared<GnnAssociator>(kGnnGate);
}

std::shared_ptr<IDataAssociator> makeJpda() {
  return std::make_shared<JpdaAssociator>(kJpdaGate, kJpdaPd,
                                          kJpdaClutterDensity);
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
  configs.push_back({"imm_cv_ct_mht", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  // Same as canonical but with Student-t robust updates — the ablation
  // that isolates the heavy-tailed-measurement (EO/IR clutter) axis.
  configs.push_back({"imm_cv_ct_mht_robust", &makeImmCvCtRobust, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  // Canonical minus the visibility channel — isolates what VIMM's
  // obscuration handling adds over plain IPDA.
  configs.push_back({"imm_cv_ct_mht_ipda", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtIpdaConfig});
  // Canonical minus the existence lifecycle (pre-2026-06-11 M-of-N +
  // score-delete) — isolates what calibrated existence buys.
  configs.push_back({"imm_cv_ct_mht_mofn", &makeImmCvCt, &makeJpda,
                     TrackerKind::Mht, &makeMhtMofnConfig});
  // JPDA/GNN-style ablations (single-hypothesis Tracker pipeline).
  configs.push_back({"ekf_cv_gnn", &makeEkfCv, &makeGnn});
  configs.push_back({"ekf_cv_jpda", &makeEkfCv, &makeJpda});
  configs.push_back({"ukf_cv_gnn", &makeUkfCv, &makeGnn});
  configs.push_back({"ukf_ct_gnn", &makeUkfCt, &makeGnn});
  configs.push_back({"imm_cv_ct_jpda", &makeImmCvCt, &makeJpda});
  configs.push_back({"imm_cv_ct_noisy_jpda", &makeImmCvCtNoisy, &makeJpda});
  // Other MHT ablations (MhtTracker pipeline). associator factory unused;
  // we still pass it because the Config struct ergonomics expect it
  // populated. The MhtTracker constructs its own gating + association
  // via TrackTree::branch internally.
  configs.push_back({"ekf_cv_mht", &makeEkfCv, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  configs.push_back({"imm_cv_ct_noisy_mht", &makeImmCvCtNoisy, &makeJpda,
                     TrackerKind::Mht, &makeMhtConfig});
  return configs;
}

}  // namespace benchmark
}  // namespace navtracker
