// W3.4 — the load-bearing sign-convention fix.
//
// `HeadingBiasEstimator` fuses five observation kinds into ONE scalar bias
// state `b`, which the ARPA/EO-IR adapters consume as
//   corrected_compass_bearing = measured_compass_bearing - b.
// So the canonical convention is: b is the COMPASS-frame heading bias the
// gyro ADDS to truth (0 = true north, clockwise-positive, marine). Every
// observation kind must therefore drive b toward +b_compass for a physical
// gyro that reads high by b_compass.
//
// These tests build each kind THROUGH THE REAL geometry (projectRangeBearing
// ToEnu for the AIS/ARPA path, the ENU-math atan2 bearing convention for the
// bearing-innovation path) so the frame is measured, not assumed. Pre-fix,
// the v1 (AIS/ARPA) and v2 (bearing-innovation) paths — which live in the
// ENU-math frame (E=0, CCW) — drive b the WRONG way (toward -b_compass),
// fighting the v3 gyro-vs-reference kinds and the adapter correction. These
// are the teeth: they FAIL on the pre-fix estimator.

#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/projection/Projection.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

// Build one AIS+ARPA pair reproducing the real projection pipeline for a
// target at true compass bearing `theta_true_rad` (from north, CW), range R,
// own-ship at the origin, with the gyro reading high by `b_compass_rad`.
// The ARPA target is projected using the biased compass bearing; the AIS
// anchor is projected at the true compass bearing. This is exactly what
// ArpaAdapter (biased heading) and the AIS position feed produce.
AisArpaPairObservation makeCompassBiasedPair(Timestamp t,
                                             double theta_true_rad,
                                             double b_compass_rad,
                                             double R = 1500.0) {
  const Eigen::Vector2d own = Eigen::Vector2d::Zero();
  const PointAndCov2D ais =
      projectRangeBearingToEnu(R, theta_true_rad, 5.0, 0.5 * kDeg2Rad,
                               0.0, 0.0, own);
  const PointAndCov2D arpa =
      projectRangeBearingToEnu(R, theta_true_rad + b_compass_rad, 5.0,
                               0.5 * kDeg2Rad, 0.0, 0.0, own);
  AisArpaPairObservation o;
  o.time = t;
  o.own_position_enu = own;
  o.ais_target_position_enu = ais.pos_enu;
  o.arpa_target_position_enu = arpa.pos_enu;
  o.arpa_bearing_std_rad = 0.5 * kDeg2Rad;
  o.ais_position_std_m = 5.0;
  return o;
}

// -------------------------------------------------------------------------
// v1 — AIS/ARPA pairs (the primary open-water calibration source).
// -------------------------------------------------------------------------
TEST(HeadingBiasSignConvention, V1AisArpaConvergesToPlusCompassBias) {
  HeadingBiasEstimator est{};
  const double b = 2.0 * kDeg2Rad;  // gyro reads 2 deg high (compass, CW)
  for (int i = 0; i < 40; ++i) {
    const double theta = (i * 23.0) * kDeg2Rad;  // vary geometry
    est.observe(makeCompassBiasedPair(tAt(i * 1.0), theta, b));
  }
  const auto e = est.current();
  EXPECT_TRUE(e.is_published);
  // Must converge to +b (the value the adapter subtracts), NOT -b.
  EXPECT_NEAR(est.biasRad(), b, 0.2 * kDeg2Rad);
}

TEST(HeadingBiasSignConvention, V1SinglePairMovesTowardPlusBias) {
  HeadingBiasEstimator est{};
  const double b = 1.5 * kDeg2Rad;
  est.observe(makeCompassBiasedPair(tAt(0.0), 30.0 * kDeg2Rad, b));
  EXPECT_GT(est.biasRad(), 0.0);  // pre-fix this is < 0
}

// -------------------------------------------------------------------------
// v2 — bearing-domain innovations. The Tracker emits r = wrap(beta_obs -
// beta_pred) in the ENU-math frame; a physical compass gyro bias +b makes
// beta_obs = beta_true - b, so the emitted innovation is -b. The estimator
// must convert this to +b at its boundary.
// -------------------------------------------------------------------------
BearingInnovation makeInnovationFromCompassBias(double t_s,
                                                double b_compass_rad,
                                                double range_m = 800.0) {
  // Emitted innovation in the ENU-math frame for a +b compass gyro bias.
  BearingInnovation obs;
  obs.time = Timestamp::fromSeconds(t_s);
  obs.track_id = TrackId{1};
  obs.innovation_rad = -b_compass_rad;  // beta_obs - beta_pred = -b
  obs.predicted_state_var_rad2 = 1e-5;
  obs.variance_rad2 = 1e-5 + 1e-4;
  obs.range_m = range_m;
  return obs;
}

TEST(HeadingBiasSignConvention, V2BearingInnovationConvergesToPlusCompassBias) {
  HeadingBiasEstimator est{};
  const double b = 2.0 * kDeg2Rad;
  for (int i = 0; i < 60; ++i) {
    est.observe(makeInnovationFromCompassBias(i * 0.1, b));
  }
  // Must converge to +b_compass (consistent with the adapter correction),
  // not to the raw ENU-math innovation -b.
  EXPECT_GT(est.biasRad(), 0.0);
  EXPECT_NEAR(est.biasRad(), b, 0.3 * kDeg2Rad);
}

// -------------------------------------------------------------------------
// v3 — gyro vs GPS/COG/magnetic. gyro_reported = true + b (compass), so
// measurement = gyro - reference = +b. These are the CANONICAL kinds and
// already correct; this test pins that the convention direction we chose
// matches them (they must stay green).
// -------------------------------------------------------------------------
TEST(HeadingBiasSignConvention, V3GyroVsGpsHeadingConvergesToPlusCompassBias) {
  HeadingBiasEstimator est{};
  const double b = 2.0 * kDeg2Rad;
  const double sigma = 0.001;
  for (int i = 0; i < 60; ++i) {
    GyroVsGpsHeadingObservation o;
    o.time = Timestamp::fromSeconds(i * 0.1);
    o.gyro_rad = 0.7 + b;               // gyro reads high by b
    o.gps_true_heading_rad = 0.7;       // truth
    o.gps_true_heading_std_rad = sigma;
    est.observe(o);
  }
  EXPECT_NEAR(est.biasRad(), b, 0.05 * kDeg2Rad);
}

// -------------------------------------------------------------------------
// The whole point of the fix: all kinds AGREE. v1 and v3 fed the same
// physical +b must not fight — a mixed stream converges to +b, not toward 0.
// -------------------------------------------------------------------------
TEST(HeadingBiasSignConvention, MixedV1AndV3AgreeOnSign) {
  HeadingBiasEstimator est{};
  const double b = 2.0 * kDeg2Rad;
  const double sigma = 0.01;
  for (int i = 0; i < 40; ++i) {
    est.observe(makeCompassBiasedPair(tAt(i * 1.0), (i * 23.0) * kDeg2Rad, b));
    GyroVsGpsHeadingObservation o;
    o.time = Timestamp::fromSeconds(i * 1.0 + 0.5);
    o.gyro_rad = 0.7 + b;
    o.gps_true_heading_rad = 0.7;
    o.gps_true_heading_std_rad = sigma;
    est.observe(o);
  }
  EXPECT_NEAR(est.biasRad(), b, 0.2 * kDeg2Rad);
}

}  // namespace
}  // namespace navtracker
