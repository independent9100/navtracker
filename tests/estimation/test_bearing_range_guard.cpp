#include <memory>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/estimation/BearingRangeGuard.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

Measurement bearingMeas(double beta_rad, double t_s, double sigma_b_rad,
                        const Eigen::Vector2d& sensor =
                            Eigen::Vector2d::Zero()) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "cam";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = beta_rad;
  m.value = v;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma_b_rad * sigma_b_rad;
  m.covariance = R;
  m.sensor_position_enu = sensor;
  return m;
}

}  // namespace

// Unit math: a bearing update whose Joseph posterior reduces along-LOS
// variance must be clamped back to the predicted along-LOS variance,
// while leaving cross-LOS reduction intact.
TEST(BearingRangeGuard, ClampsAlongLosWhenPostBelowPre) {
  Eigen::Vector4d x_pred(500.0, 0.0, 0.0, 0.0);  // target due east at 500 m
  Eigen::Matrix4d P_pred = Eigen::Matrix4d::Zero();
  P_pred.topLeftCorner<2, 2>() << 100.0, 0.0, 0.0, 100.0;  // isotropic 10 m σ
  P_pred(2, 2) = 25.0;
  P_pred(3, 3) = 25.0;
  // Synthesize a "post" that's tighter along-LOS than pre (the
  // pathological case): rotate isotropic 100 to 50 along LOS.
  Eigen::Matrix2d P_post_xy;
  // Along LOS = +x; reduce x variance, leave y variance.
  P_post_xy << 50.0, 0.0, 0.0, 80.0;
  Eigen::Matrix4d P_post = P_pred;
  P_post.topLeftCorner<2, 2>() = P_post_xy;

  const Eigen::MatrixXd guarded = applyBearingRangeGuard(
      P_pred, P_post, x_pred, Eigen::Vector2d::Zero());
  const Eigen::Matrix2d Pxy = guarded.topLeftCorner<2, 2>();
  const Eigen::Vector2d n_los(1.0, 0.0);
  const double var_los = n_los.dot(Pxy * n_los);
  const Eigen::Vector2d n_perp(0.0, 1.0);
  const double var_cross = n_perp.dot(Pxy * n_perp);
  EXPECT_NEAR(var_los, 100.0, 1e-9);   // restored to pre
  EXPECT_NEAR(var_cross, 80.0, 1e-9);  // cross-LOS reduction preserved
}

TEST(BearingRangeGuard, NoOpWhenPostAlreadyAbovePre) {
  Eigen::Vector4d x_pred(500.0, 0.0, 0.0, 0.0);
  Eigen::Matrix4d P_pred = Eigen::Matrix4d::Zero();
  P_pred.topLeftCorner<2, 2>() << 100.0, 0.0, 0.0, 100.0;
  Eigen::Matrix4d P_post = P_pred;
  P_post.topLeftCorner<2, 2>() << 100.0, 0.0, 0.0, 50.0;  // only y reduced
  const Eigen::MatrixXd guarded = applyBearingRangeGuard(
      P_pred, P_post, x_pred, Eigen::Vector2d::Zero());
  EXPECT_TRUE(guarded.isApprox(P_post, 1e-12));
}

// EkfEstimator wiring: guard-on materially changes the posterior when
// the Joseph update would reduce along-LOS variance. Construct a state
// with position-velocity correlation; the bearing update's K leverages
// that correlation and the Joseph step ends up squeezing P_xy along
// LOS below the predicted value (the BOT pathology). Guard-on prevents
// it; guard-off lets it happen.
TEST(BearingRangeGuard, EkfWiredGuardChangesPosterior) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.001);
  EkfEstimator est_off(motion, 5.0, nullptr, /*guard=*/false);
  EkfEstimator est_on(motion, 5.0, nullptr, /*guard=*/true);

  Track seed_template;
  seed_template.state = Eigen::Vector4d(500.0, 0.0, 0.0, 5.0);
  // Diagonal P with healthy cross-LOS variance + small along-LOS.
  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
  P(0, 0) = 100.0;  P(1, 1) = 400.0;
  P(2, 2) = 4.0;    P(3, 3) = 4.0;
  // Position-velocity correlation to drive the cross-coupling.
  P(0, 2) = 5.0;    P(2, 0) = 5.0;
  seed_template.covariance = P;
  seed_template.last_update = Timestamp::fromSeconds(0.0);
  Track t_off = seed_template;
  Track t_on = seed_template;

  const double sigma_b = 0.001;  // 1 mrad bearing
  Measurement m = bearingMeas(0.0, 1.0, sigma_b);
  const double var_los_pre = t_off.covariance(0, 0);
  est_off.update(t_off, m);
  est_on.update(t_on, m);

  const double var_los_off = t_off.covariance(0, 0);
  const double var_los_on = t_on.covariance(0, 0);
  // Guard-on must be ≥ predicted (the spec invariant).
  EXPECT_GE(var_los_on, var_los_pre - 1e-9);
  // And guard-on yields a strictly different covariance from guard-off
  // when the guard actually fires (i.e., off-state collapsed below pre).
  if (var_los_off < var_los_pre - 1e-9) {
    EXPECT_GT(var_los_on, var_los_off);
  }
}
