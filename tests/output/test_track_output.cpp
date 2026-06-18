#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/output/TrackOutput.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

using navtracker::PositionGeodeticWithCov;
using navtracker::Timestamp;
using navtracker::toGeodeticWithCov;
using navtracker::toTrackOutput;
using navtracker::toVelocityOutput;
using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackOutput;
using navtracker::TrackStatus;
using navtracker::VelocityGeodeticWithSigma;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

// Build a 4D-state Track (px, py, vx, vy) with diagonal covariance.
// When build_4d is false, only the 2D position state/cov are set.
Track makeTrack(double px, double py, double vx, double vy,
                double sigma_pos = 5.0, double sigma_vel = 1.0,
                bool build_4d = true) {
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.last_update = Timestamp::fromSeconds(0.0);
  if (build_4d) {
    t.state.resize(4);
    t.state << px, py, vx, vy;
    t.covariance = Eigen::Matrix4d::Zero();
    t.covariance(0, 0) = sigma_pos * sigma_pos;
    t.covariance(1, 1) = sigma_pos * sigma_pos;
    t.covariance(2, 2) = sigma_vel * sigma_vel;
    t.covariance(3, 3) = sigma_vel * sigma_vel;
  } else {
    t.state.resize(2);
    t.state << px, py;
    t.covariance = Eigen::Matrix2d::Zero();
    t.covariance(0, 0) = sigma_pos * sigma_pos;
    t.covariance(1, 1) = sigma_pos * sigma_pos;
  }
  return t;
}

}  // namespace

// -----------------------------------------------------------------------------
// Position helpers
// -----------------------------------------------------------------------------

TEST(TrackOutputTest, GeodeticPositionRoundTripsAtDatum) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  const Eigen::Vector2d enu_xy(0.0, 0.0);
  const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();

  const PositionGeodeticWithCov out = toGeodeticWithCov(enu_xy, cov, datum);

  EXPECT_NEAR(out.lat_deg, 53.5, 1e-9);
  EXPECT_NEAR(out.lon_deg, 8.0, 1e-9);
  // At the datum, target_datum == datum, so rotation is identity and the
  // covariance is returned unchanged.
  EXPECT_NEAR(out.position_covariance_m2(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(out.position_covariance_m2(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(out.position_covariance_m2(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(out.position_covariance_m2(1, 0), 0.0, 1e-12);
}

TEST(TrackOutputTest, GeodeticPositionRoundTripsOffDatum) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  // Pick a non-trivial offset within auto-datum's recenter horizon.
  const Eigen::Vector2d enu_xy(12000.0, -3500.0);  // ~12 km east, 3.5 km south
  const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();

  const PositionGeodeticWithCov out = toGeodeticWithCov(enu_xy, cov, datum);

  // Round-trip the returned lat/lon back through datum.toEnu — should match
  // up to centimeter-level numerical precision (ENU↔ECF↔geodetic round-trip
  // accumulates small floating-point error at multi-km distances).
  const Eigen::Vector3d enu_back =
      datum.toEnu(Geodetic{out.lat_deg, out.lon_deg, 0.0});
  EXPECT_NEAR(enu_back.x(), enu_xy.x(), 0.1);
  EXPECT_NEAR(enu_back.y(), enu_xy.y(), 0.1);
}

TEST(TrackOutputTest, CovarianceRotatedAtHighLatitude) {
  // High-latitude datum at 60°N, 0°E. Track ~55 km east — about 1° of
  // longitude. Convergence angle gamma = delta_lon_rad * sin(mean_lat_rad).
  const Datum datum(Geodetic{60.0, 0.0, 0.0});
  // 1° lon at 60°N ≈ 55,597 m. Translate via datum to get the exact ENU.
  const Eigen::Vector3d enu3 = datum.toEnu(Geodetic{60.0, 1.0, 0.0});
  const Eigen::Vector2d enu_xy(enu3.x(), enu3.y());

  // Anisotropic covariance in datum-ENU: σ_east = 10 m, σ_north = 1 m.
  Eigen::Matrix2d cov_enu = Eigen::Matrix2d::Zero();
  cov_enu(0, 0) = 100.0;  // east variance
  cov_enu(1, 1) = 1.0;    // north variance

  const PositionGeodeticWithCov out = toGeodeticWithCov(enu_xy, cov_enu, datum);

  // Spec rule: gamma ≈ delta_lon_rad * sin(mean_lat_rad). At 60°N over
  // ~1° of longitude this is ≈ 0.0151 rad.
  const double approx_gamma = (1.0 * kDeg2Rad) * std::sin(60.0 * kDeg2Rad);
  EXPECT_NEAR(approx_gamma, 0.01512, 1e-4);

  // Compute the exact expected R using the actual lat/lon the helper
  // returned (which differs from a perfect 1° lon by a few mm due to the
  // ENU-vs-ellipsoid mapping).
  const double delta_lon_rad = (out.lon_deg - 0.0) * kDeg2Rad;
  const double mean_lat_rad = 0.5 * (60.0 + out.lat_deg) * kDeg2Rad;
  const double gamma = delta_lon_rad * std::sin(mean_lat_rad);
  const double s = std::sin(gamma);
  const double c = std::cos(gamma);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  const Eigen::Matrix2d expected = R * cov_enu * R.transpose();

  EXPECT_NEAR(out.position_covariance_m2(0, 0), expected(0, 0), 1e-9);
  EXPECT_NEAR(out.position_covariance_m2(1, 1), expected(1, 1), 1e-9);
  EXPECT_NEAR(out.position_covariance_m2(0, 1), expected(0, 1), 1e-9);
  EXPECT_NEAR(out.position_covariance_m2(1, 0), expected(1, 0), 1e-9);

  // Off-diagonal in the input was zero; after rotation it must differ by
  // (σ_east² − σ_north²) * sin·cos. With σ_east²=100, σ_north²=1, the
  // off-diagonal is well above any numerical-noise floor.
  const double expected_off = (100.0 - 1.0) * s * c;
  EXPECT_NEAR(out.position_covariance_m2(0, 1), expected_off, 1e-9);
  EXPECT_GT(std::abs(out.position_covariance_m2(0, 1)), 1e-3);
}

// -----------------------------------------------------------------------------
// Velocity helper
// -----------------------------------------------------------------------------

TEST(TrackOutputTest, SogCogFromEastVelocity) {
  // v_enu = (east=10, north=0) → SOG=10 m/s, COG=90° (east is clockwise of N).
  const Eigen::Vector2d v_enu(10.0, 0.0);
  const Eigen::Matrix2d v_cov = Eigen::Matrix2d::Identity();
  const VelocityGeodeticWithSigma out = toVelocityOutput(v_enu, v_cov, true);

  EXPECT_TRUE(out.is_valid);
  EXPECT_NEAR(out.sog_m_per_s, 10.0, 1e-12);
  EXPECT_NEAR(out.cog_deg, 90.0, 1e-9);

  // Polar Jacobian: with c=cos(90°)=0, s=sin(90°)=1, sog=10:
  //   J = [[s, c],     = [[1, 0],
  //        [c/sog, -s/sog]]   [0, -0.1]]
  // J * I * J^T = diag(1, 0.01). σ_sog=1 m/s; σ_cog=0.1 rad ≈ 5.7296°.
  EXPECT_NEAR(out.sigma_sog_m_per_s, 1.0, 1e-12);
  EXPECT_NEAR(out.sigma_cog_deg, 0.1 * kRad2Deg, 1e-9);
}

TEST(TrackOutputTest, SogCogFromNorthVelocity) {
  // v_enu = (east=0, north=10) → SOG=10 m/s, COG=0° (due north).
  const Eigen::Vector2d v_enu(0.0, 10.0);
  const Eigen::Matrix2d v_cov = Eigen::Matrix2d::Identity();
  const VelocityGeodeticWithSigma out = toVelocityOutput(v_enu, v_cov, true);

  EXPECT_TRUE(out.is_valid);
  EXPECT_NEAR(out.sog_m_per_s, 10.0, 1e-12);
  EXPECT_NEAR(out.cog_deg, 0.0, 1e-9);

  // With c=cos(0°)=1, s=sin(0°)=0, sog=10:
  //   J = [[0, 1], [0.1, 0]]. J * I * J^T = diag(1, 0.01).
  EXPECT_NEAR(out.sigma_sog_m_per_s, 1.0, 1e-12);
  EXPECT_NEAR(out.sigma_cog_deg, 0.1 * kRad2Deg, 1e-9);
}

TEST(TrackOutputTest, StationaryTrackHandlesSingularity) {
  // v = 0 → direction undefined; isotropic trace bound for σ_sog,
  // σ_cog forced to 0.
  const Eigen::Vector2d v_enu(0.0, 0.0);
  const Eigen::Matrix2d v_cov = Eigen::Matrix2d::Identity();
  const VelocityGeodeticWithSigma out = toVelocityOutput(v_enu, v_cov, true);

  EXPECT_TRUE(out.is_valid);
  EXPECT_NEAR(out.sog_m_per_s, 0.0, 1e-12);
  EXPECT_NEAR(out.cog_deg, 0.0, 1e-12);
  EXPECT_NEAR(out.sigma_cog_deg, 0.0, 1e-12);
  // σ_sog = sqrt(0.5 * trace(I_2)) = sqrt(0.5 * 2) = 1.
  EXPECT_NEAR(out.sigma_sog_m_per_s, 1.0, 1e-12);
}

TEST(TrackOutputTest, InvalidVelocityZeroesAllNumeric) {
  // is_valid=false → all numeric fields stay at default zero.
  const Eigen::Vector2d v_enu(7.0, -3.0);
  Eigen::Matrix2d v_cov;
  v_cov << 4.0, 0.5,
           0.5, 9.0;
  const VelocityGeodeticWithSigma out = toVelocityOutput(v_enu, v_cov, false);

  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.sog_m_per_s, 0.0);
  EXPECT_EQ(out.cog_deg, 0.0);
  EXPECT_EQ(out.sigma_sog_m_per_s, 0.0);
  EXPECT_EQ(out.sigma_cog_deg, 0.0);
}

// -----------------------------------------------------------------------------
// Integration: toTrackOutput
// -----------------------------------------------------------------------------

TEST(TrackOutputTest, TrackOutputFor4DTrack) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  // Place track ~10 km east, 2 km north at the datum, moving east at 5 m/s.
  Track t = makeTrack(10000.0, 2000.0, 5.0, 0.0,
                      /*sigma_pos=*/5.0, /*sigma_vel=*/0.5);
  t.id = TrackId{42};
  t.status = TrackStatus::Confirmed;
  t.last_update = Timestamp::fromSeconds(123.456);
  t.attributes.mmsi = 211000123u;
  t.attributes.name = std::string("MV TEST");
  t.attributes.vessel_type = std::string("CARGO");
  t.contributing_sources = {"ais", "arpa"};
  t.velocity_observed = true;  // confirmed, updated track → velocity observed

  const TrackOutput out = toTrackOutput(t, datum);

  // Metadata propagates verbatim.
  EXPECT_EQ(out.id.value, 42u);
  EXPECT_EQ(static_cast<int>(out.status),
            static_cast<int>(TrackStatus::Confirmed));
  EXPECT_EQ(out.last_update, Timestamp::fromSeconds(123.456));
  ASSERT_TRUE(out.attributes.mmsi.has_value());
  EXPECT_EQ(*out.attributes.mmsi, 211000123u);
  ASSERT_TRUE(out.attributes.name.has_value());
  EXPECT_EQ(*out.attributes.name, "MV TEST");
  ASSERT_EQ(out.contributing_sources.size(), 2u);
  EXPECT_EQ(out.contributing_sources[0], "ais");
  EXPECT_EQ(out.contributing_sources[1], "arpa");

  // Position round-trips through the datum (cm-level numerical precision).
  const Eigen::Vector3d back =
      datum.toEnu(Geodetic{out.position.lat_deg, out.position.lon_deg, 0.0});
  EXPECT_NEAR(back.x(), 10000.0, 0.1);
  EXPECT_NEAR(back.y(), 2000.0, 0.1);

  // Position covariance diagonal ~25 (σ_pos=5 → variance 25), preserved up to
  // the tiny convergence-angle rotation at 8°E + ~10 km east.
  EXPECT_NEAR(out.position.position_covariance_m2(0, 0), 25.0, 1e-3);
  EXPECT_NEAR(out.position.position_covariance_m2(1, 1), 25.0, 1e-3);

  // Velocity is valid, eastbound: SOG=5 m/s, COG=90°.
  EXPECT_TRUE(out.velocity.is_valid);
  EXPECT_NEAR(out.velocity.sog_m_per_s, 5.0, 1e-12);
  EXPECT_NEAR(out.velocity.cog_deg, 90.0, 1e-9);
  // σ_vel = 0.5 → v_cov = 0.25 * I. σ_sog = sqrt(0.25) = 0.5.
  EXPECT_NEAR(out.velocity.sigma_sog_m_per_s, 0.5, 1e-12);

  // No SourceTouch with the flag set → false.
  EXPECT_FALSE(out.covariance_is_default);
}

TEST(TrackOutputTest, VelocityInvalidWhenNotObserved) {
  // Review #13: a freshly-initiated track (velocity_observed=false) carries a
  // pure-prior velocity, so is_valid must be false even though v_cov.trace()>0.
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  Track t = makeTrack(1000.0, 500.0, 5.0, 0.0, /*sigma_pos=*/5.0,
                      /*sigma_vel=*/0.5);
  ASSERT_FALSE(t.velocity_observed);  // default for a just-built track

  const TrackOutput out = toTrackOutput(t, datum);
  EXPECT_FALSE(out.velocity.is_valid);

  // Flip the flag → valid (same kinematics).
  t.velocity_observed = true;
  const TrackOutput out2 = toTrackOutput(t, datum);
  EXPECT_TRUE(out2.velocity.is_valid);
  EXPECT_NEAR(out2.velocity.sog_m_per_s, 5.0, 1e-12);
}

TEST(TrackOutputTest, CovarianceIsDefaultForwardedFromRecentContributions) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  Track t = makeTrack(1000.0, 500.0, 5.0, 0.0, /*sigma_pos_m=*/5.0,
                      /*sigma_vel_m_per_s=*/0.5);
  Track::SourceTouch clean;
  clean.covariance_is_default = false;
  Track::SourceTouch dirty;
  dirty.covariance_is_default = true;
  t.recent_contributions.push_back(clean);
  t.recent_contributions.push_back(dirty);

  const TrackOutput out = toTrackOutput(t, datum);
  EXPECT_TRUE(out.covariance_is_default);
}

TEST(TrackOutputTest, CovarianceIsDefaultFalseWhenAllContributionsAreReal) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  Track t = makeTrack(1000.0, 500.0, 5.0, 0.0, 5.0, 0.5);
  Track::SourceTouch a, b;
  a.covariance_is_default = false;
  b.covariance_is_default = false;
  t.recent_contributions.push_back(a);
  t.recent_contributions.push_back(b);

  const TrackOutput out = toTrackOutput(t, datum);
  EXPECT_FALSE(out.covariance_is_default);
}

TEST(TrackOutputTest, TrackOutputFor2DTrackHasInvalidVelocity) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  // 2D state only — no velocity component.
  Track t = makeTrack(1000.0, 500.0, /*vx=*/0.0, /*vy=*/0.0,
                      /*sigma_pos=*/5.0, /*sigma_vel=*/0.0,
                      /*build_4d=*/false);

  const TrackOutput out = toTrackOutput(t, datum);

  EXPECT_FALSE(out.velocity.is_valid);
  EXPECT_EQ(out.velocity.sog_m_per_s, 0.0);
  EXPECT_EQ(out.velocity.cog_deg, 0.0);
  EXPECT_EQ(out.velocity.sigma_sog_m_per_s, 0.0);
  EXPECT_EQ(out.velocity.sigma_cog_deg, 0.0);

  // Position fields still populated from the 2D state.
  const Eigen::Vector3d back =
      datum.toEnu(Geodetic{out.position.lat_deg, out.position.lon_deg, 0.0});
  EXPECT_NEAR(back.x(), 1000.0, 0.1);
  EXPECT_NEAR(back.y(), 500.0, 0.1);
}

TEST(TrackOutputTest, TrackOutputForZeroVelocityCovarianceHasInvalidVelocity) {
  const Datum datum(Geodetic{53.5, 8.0, 0.0});
  // 4D state but the velocity covariance block is exactly zero — no
  // uncertainty information means we can't claim a meaningful velocity.
  Track t = makeTrack(2000.0, 1000.0, 3.0, 4.0,
                      /*sigma_pos=*/5.0, /*sigma_vel=*/0.0);
  // makeTrack already zeros the v-cov block when sigma_vel == 0; assert that
  // intent here for clarity.
  const Eigen::Matrix2d vcov_block = t.covariance.block<2, 2>(2, 2);
  ASSERT_EQ(vcov_block.trace(), 0.0);

  const TrackOutput out = toTrackOutput(t, datum);

  EXPECT_FALSE(out.velocity.is_valid);
  EXPECT_EQ(out.velocity.sog_m_per_s, 0.0);
  EXPECT_EQ(out.velocity.cog_deg, 0.0);
  EXPECT_EQ(out.velocity.sigma_sog_m_per_s, 0.0);
  EXPECT_EQ(out.velocity.sigma_cog_deg, 0.0);
}
