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
  // Convention (W2.3): datumAxisRotation(old, new) returns the 2×2 rotation R
  // with  v_new = R · v_old  — it re-expresses a vector given in the OLD datum's
  // ENU axes into the NEW datum's ENU axes. R is the E,N block of the exact
  // linearised position map R_new · R_oldᵀ, which is a rotation by −γ, where
  //   γ = Δλ · sin(φ_mean)   (meridian convergence angle, Δλ = lon_new − lon_old).
  //
  // Worked example (this test): at 60°N, moving the datum 1° EAST gives
  // γ = +0.015115 rad. An east-pointing velocity (1, 0) in the old frame becomes
  //   R·(1,0) = (cos γ, −sin γ) = (0.999886, −0.015114)
  // i.e. it tips slightly SOUTH: sliding the origin east rotates the local
  // east/north axes so what was "due east" now has a small negative-north part.
  // The OLD code applied +γ (this test used to pin that sign — the bug).
  geo::Datum old_d(geo::Geodetic{60.0, 0.0, 0.0});
  geo::Datum new_d(geo::Geodetic{60.0, 1.0, 0.0});   // ~55 km east at 60N
  TrackManager mgr(2, 3);
  Track t = makeTrack(0.0, 0.0, 1.0, 0.0);  // 1 m/s east
  mgr.add(t);

  const Eigen::Matrix2d R = datumAxisRotation(old_d, new_d);
  const double gamma_expected = 1.0 * 3.14159265358979 / 180.0
                                * std::sin(60.0 * 3.14159265358979 / 180.0);
  // R rotates by −γ, so atan2(R(1,0), R(0,0)) == −γ.
  EXPECT_NEAR(std::atan2(R(1,0), R(0,0)), -gamma_expected, 1e-9);

  shiftTracksOnDatumChange(mgr, old_d, new_d);

  // Velocity rotated by −γ: (1, 0) becomes (cos(γ), −sin(γ)).
  const auto& shifted = mgr.tracks()[0];
  EXPECT_NEAR(shifted.state(2), std::cos(gamma_expected), 1e-6);
  EXPECT_NEAR(shifted.state(3), -std::sin(gamma_expected), 1e-6);
}

TEST(DatumShiftTest, WrapsDeltaLongitudeAcrossAntimeridian) {
  // W2.2: a recenter that crosses the ±180° antimeridian has a SMALL true
  // Δlongitude, but the unwrapped (lon_new − lon_old) is ~±360°. The rotation
  // must be computed from the wrapped Δλ, so crossing the seam is identical to
  // an equivalent step that does not cross it.
  geo::Datum old_d(geo::Geodetic{60.0, 179.5, 0.0});
  geo::Datum new_d(geo::Geodetic{60.0, -179.5, 0.0});  // +1° east, across seam
  // Equivalent non-crossing +1° step at the same latitude.
  geo::Datum ref_old(geo::Geodetic{60.0, 0.0, 0.0});
  geo::Datum ref_new(geo::Geodetic{60.0, 1.0, 0.0});

  const Eigen::Matrix2d R = datumAxisRotation(old_d, new_d);
  const Eigen::Matrix2d R_ref = datumAxisRotation(ref_old, ref_new);

  // Without wrapping, R would encode a ~−359° rotation instead of ~+1°.
  const double gamma = 1.0 * 3.14159265358979 / 180.0
                       * std::sin(60.0 * 3.14159265358979 / 180.0);
  EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), -gamma, 1e-6);
  EXPECT_NEAR(R(0, 0), R_ref(0, 0), 1e-9);
  EXPECT_NEAR(R(1, 0), R_ref(1, 0), 1e-9);
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
