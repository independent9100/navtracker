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

std::shared_ptr<IDataAssociator> makeGnn() {
  return std::make_shared<GnnAssociator>(kGnnGate);
}

std::shared_ptr<IDataAssociator> makeJpda() {
  return std::make_shared<JpdaAssociator>(kJpdaGate, kJpdaPd,
                                          kJpdaClutterDensity);
}

}  // namespace

std::vector<Config> defaultConfigs() {
  std::vector<Config> configs;
  configs.reserve(5);
  configs.push_back({"ekf_cv_gnn", &makeEkfCv, &makeGnn});
  configs.push_back({"ekf_cv_jpda", &makeEkfCv, &makeJpda});
  configs.push_back({"ukf_cv_gnn", &makeUkfCv, &makeGnn});
  configs.push_back({"ukf_ct_gnn", &makeUkfCt, &makeGnn});
  configs.push_back({"imm_cv_ct_jpda", &makeImmCvCt, &makeJpda});
  return configs;
}

}  // namespace benchmark
}  // namespace navtracker
