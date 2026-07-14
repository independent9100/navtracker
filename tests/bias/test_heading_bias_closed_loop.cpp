// W3.1 — closed-loop heading-bias double-subtraction.
//
// In the documented wiring the ARPA/EO-IR adapter subtracts the published b̂
// from the bearing BEFORE projecting to ENU, so the AIS↔ARPA pair the
// extractor forms shows only the RESIDUAL bias (b_true − b_pub). The pre-fix
// estimator subtracted b̂ again (y = z − b̂), whose fixed point is b̂ =
// b_true/2 — the published bias decays to half and every corrected bearing
// keeps half the error. The fix carries the applied correction on the touch
// (Track::SourceTouch::applied_heading_bias_rad, set by the adapter) and adds
// it back so the estimator measures the full b_true.
//
// This test drives the real closed loop at the touch→extractor→estimator
// level, projecting positions through the real projectRangeBearingToEnu. The
// teeth: with reconstruction it converges to b_true; drop the reconstruction
// (applied = 0) and the SAME loop converges to b_true/2.

#include "core/bias/AisArpaPairExtractor.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/projection/Projection.hpp"
#include "core/types/Track.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

Track::SourceTouch touchAt(SensorKind k, Timestamp t, Eigen::Vector2d v,
                           double applied_heading_bias_rad = 0.0) {
  Track::SourceTouch s;
  s.sensor = k;
  s.source_id = "arpa";
  s.time = t;
  s.value_enu = v;
  s.sensor_position_enu = Eigen::Vector2d::Zero();
  s.covariance = Eigen::Matrix2d::Identity() * 25.0;
  s.applied_heading_bias_rad = applied_heading_bias_rad;
  return s;
}

// Run the closed loop for `carry_reconstruction` on/off and return the
// converged b̂. Each cycle: the adapter subtracts the currently-published b_pub
// from the compass bearing; the residual bias (b_true − b_pub) shows up in the
// projected ARPA return; the touch carries b_pub (or 0 if reconstruction is
// disabled); extract + observe; re-read the published estimate.
double runClosedLoop(double b_true, bool carry_reconstruction) {
  HeadingBiasEstimator est{};
  const Eigen::Vector2d own = Eigen::Vector2d::Zero();
  const double R = 1500.0;
  double b_pub = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double theta_true = (i * 23.0) * kDeg2Rad;         // compass, vary
    const double theta_arpa = theta_true + (b_true - b_pub); // residual bias
    const PointAndCov2D ais =
        projectRangeBearingToEnu(R, theta_true, 5.0, 0.5 * kDeg2Rad, 0, 0, own);
    const PointAndCov2D arpa =
        projectRangeBearingToEnu(R, theta_arpa, 5.0, 0.5 * kDeg2Rad, 0, 0, own);
    Track tr;
    tr.recent_contributions.push_back(
        touchAt(SensorKind::Ais, tAt(i * 1.0), ais.pos_enu));
    tr.recent_contributions.push_back(touchAt(
        SensorKind::ArpaTtm, tAt(i * 1.0), arpa.pos_enu,
        carry_reconstruction ? b_pub : 0.0));
    for (const auto& obs : extractPairs({tr}, tAt(i * 1.0))) {
      est.observe(obs);
    }
    const auto e = est.current();
    if (e.is_published) b_pub = e.bias_rad;
  }
  return est.biasRad();
}

}  // namespace

TEST(HeadingBiasClosedLoop, ConvergesToFullBiasWithReconstruction) {
  const double b_true = 3.0 * kDeg2Rad;
  const double b_hat = runClosedLoop(b_true, /*carry_reconstruction=*/true);
  EXPECT_NEAR(b_hat, b_true, 0.15 * kDeg2Rad);
  // Explicitly NOT the b/2 fixed point.
  EXPECT_GT(b_hat, 0.75 * b_true);
}

TEST(HeadingBiasClosedLoop, WithoutReconstructionConvergesToHalf) {
  // Teeth: the SAME closed loop, but the touch does not carry the applied
  // correction (the pre-fix behaviour) — converges to half the true bias.
  const double b_true = 3.0 * kDeg2Rad;
  const double b_hat = runClosedLoop(b_true, /*carry_reconstruction=*/false);
  EXPECT_NEAR(b_hat, 0.5 * b_true, 0.15 * kDeg2Rad);
}

}  // namespace navtracker
