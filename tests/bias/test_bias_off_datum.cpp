// W3.3 — AIS/ARPA pairs must form bearings about OWN-SHIP, not the ENU datum
// origin. The production ArpaAdapter never set Measurement::sensor_position_enu,
// so the touch carried (0,0) and the estimator measured the angle subtended at
// the datum origin — geometry-diluted and wrong whenever own-ship is far from
// the datum. Fix: ArpaAdapter populates sensor_position_enu with own-ship ENU.

#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/bias/AisArpaPairExtractor.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/projection/Projection.hpp"
#include "core/types/Track.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

geo::Datum kDatum({53.5, 8.0, 0.0});

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

Track::SourceTouch touch(SensorKind k, Timestamp t, Eigen::Vector2d v,
                         Eigen::Vector2d own) {
  Track::SourceTouch s;
  s.sensor = k;
  s.source_id = "arpa";
  s.time = t;
  s.value_enu = v;
  s.sensor_position_enu = own;
  s.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return s;
}

// Run the AIS/ARPA estimator for own-ship at `own`, gyro bias `b`, forming the
// touch's sensor_position_enu from `touch_own` (own-ship if the fix is in,
// origin if not).
double convergedBias(Eigen::Vector2d own, Eigen::Vector2d touch_own, double b) {
  HeadingBiasEstimator est{};
  const double R = 1500.0;
  for (int i = 0; i < 60; ++i) {
    const double theta_true = (30.0 + i * 19.0) * kDeg2Rad;  // compass
    const PointAndCov2D ais =
        projectRangeBearingToEnu(R, theta_true, 5.0, 0.5 * kDeg2Rad, 0, 0, own);
    const PointAndCov2D arpa =
        projectRangeBearingToEnu(R, theta_true + b, 5.0, 0.5 * kDeg2Rad, 0, 0, own);
    Track tr;
    tr.recent_contributions.push_back(
        touch(SensorKind::Ais, tAt(i * 1.0), ais.pos_enu, touch_own));
    tr.recent_contributions.push_back(
        touch(SensorKind::ArpaTtm, tAt(i * 1.0), arpa.pos_enu, touch_own));
    for (const auto& obs : extractPairs({tr}, tAt(i * 1.0))) est.observe(obs);
  }
  return est.biasRad();
}

}  // namespace

TEST(BiasOffDatum, ArpaAdapterPopulatesSensorPositionEnu) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.15;  // ~10 km east of the datum at 53.5°N
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  const Eigen::Vector3d own_enu = kDatum.toEnu({53.5, 8.15, 0.0});
  ASSERT_GT(std::hypot(own_enu.x(), own_enu.y()), 8000.0);  // ~10 km off datum

  ArpaAdapter adapter(kDatum, provider);
  ASSERT_TRUE(adapter.ingest(
      navtracker_test::makeNmea(
          "RATTM,01,1.0,90.0,T,12.0,90.0,T,0.0,0.0,N,TARG1,T,R,123456.78,A"),
      Timestamp::fromSeconds(5.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  // The fix: the ARPA touch carries own-ship ENU, not the origin.
  EXPECT_NEAR(out[0].sensor_position_enu.x(), own_enu.x(), 1.0);
  EXPECT_NEAR(out[0].sensor_position_enu.y(), own_enu.y(), 1.0);
}

TEST(BiasOffDatum, BearingsAboutOwnShipRecoverTrueBias) {
  const Eigen::Vector2d own(10000.0, 0.0);  // 10 km from datum origin
  const double b = 2.0 * kDeg2Rad;
  // Correct: touch carries own-ship -> bearings about own-ship -> recover b.
  const double b_own = convergedBias(own, own, b);
  EXPECT_NEAR(b_own, b, 0.2 * kDeg2Rad);
  // The datum-origin formula (touch carries (0,0)) dilutes the disagreement
  // ~10x and converges nowhere near the true bias — this is the teeth.
  const double b_origin = convergedBias(own, Eigen::Vector2d::Zero(), b);
  EXPECT_LT(b_origin, 0.5 * b);
}

}  // namespace navtracker
