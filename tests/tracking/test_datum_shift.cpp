#include "core/tracking/DatumShift.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

namespace {
Track makeTrack(double px, double py, double vx, double vy) {
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << px, py, vx, vy;
  t.covariance = Eigen::Matrix4d::Zero();
  t.covariance.diagonal() << 25.0, 25.0, 1.0, 1.0;
  return t;
}
}  // namespace

TEST(DatumShiftTest, PreservesGeodeticPositionUnderShift) {
  geo::Datum old_d(geo::Geodetic{53.5, 8.0, 0.0});
  geo::Datum new_d(geo::Geodetic{53.5, 8.5, 0.0});  // ~33 km east

  // Place a track at a known lat/lon by converting it through old_d.
  const auto enu = old_d.toEnu(geo::Geodetic{53.6, 8.2, 0.0});

  TrackManager mgr(2, 3);
  Track t = makeTrack(enu.x(), enu.y(), 1.0, 0.0);
  mgr.add(t);

  shiftTracksOnDatumChange(mgr, old_d, new_d);

  // After the shift, converting back through new_d should yield the
  // original lat/lon (within LTP tolerance).
  const auto& shifted = mgr.tracks()[0];
  const auto geo_after = new_d.toGeodetic(
      Eigen::Vector3d(shifted.state(0), shifted.state(1), 0.0));
  EXPECT_NEAR(geo_after.lat_deg, 53.6, 1e-4);
  EXPECT_NEAR(geo_after.lon_deg, 8.2, 1e-4);
}

TEST(DatumShiftTest, RotatesVelocityByConvergenceAngle) {
  geo::Datum old_d(geo::Geodetic{60.0, 0.0, 0.0});
  geo::Datum new_d(geo::Geodetic{60.0, 1.0, 0.0});   // ~55 km east at 60N
  TrackManager mgr(2, 3);
  Track t = makeTrack(0.0, 0.0, 1.0, 0.0);  // 1 m/s east
  mgr.add(t);

  const Eigen::Matrix2d R = datumAxisRotation(old_d, new_d);
  const double gamma_expected = 1.0 * 3.14159265358979 / 180.0
                                * std::sin(60.0 * 3.14159265358979 / 180.0);
  EXPECT_NEAR(std::atan2(R(1,0), R(0,0)), gamma_expected, 1e-9);

  shiftTracksOnDatumChange(mgr, old_d, new_d);

  // Velocity rotated by gamma: (1, 0) becomes (cos(gamma), sin(gamma)).
  const auto& shifted = mgr.tracks()[0];
  EXPECT_NEAR(shifted.state(2), std::cos(gamma_expected), 1e-6);
  EXPECT_NEAR(shifted.state(3), std::sin(gamma_expected), 1e-6);
}

TEST(DatumShiftTest, RotatesCovarianceBlocks) {
  geo::Datum old_d(geo::Geodetic{60.0, 0.0, 0.0});
  geo::Datum new_d(geo::Geodetic{60.0, 1.0, 0.0});

  TrackManager mgr(2, 3);
  Track t = makeTrack(0.0, 0.0, 0.0, 0.0);
  // Anisotropic covariance to confirm rotation does something.
  t.covariance = Eigen::Matrix4d::Zero();
  t.covariance(0, 0) = 100.0;
  t.covariance(1, 1) = 1.0;
  t.covariance(2, 2) = 4.0;
  t.covariance(3, 3) = 0.25;
  mgr.add(t);

  const Eigen::Matrix4d cov_before = mgr.tracks()[0].covariance;
  shiftTracksOnDatumChange(mgr, old_d, new_d);
  const Eigen::Matrix4d cov_after = mgr.tracks()[0].covariance;
  // Rotation by ~0.96 deg in the position block.
  EXPECT_NE(cov_after(0, 1), cov_before(0, 1));
}

TEST(DatumShiftTest, EmptyTrackListNoOp) {
  geo::Datum old_d(geo::Geodetic{53.5, 8.0, 0.0});
  geo::Datum new_d(geo::Geodetic{53.6, 8.0, 0.0});
  TrackManager mgr(2, 3);
  EXPECT_NO_FATAL_FAILURE(shiftTracksOnDatumChange(mgr, old_d, new_d));
}

}  // namespace navtracker
