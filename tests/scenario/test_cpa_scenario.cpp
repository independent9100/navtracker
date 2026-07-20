// Integration test: drive the tracker with synthetic Position2D measurements
// on a known geometry, synthesise an own-ship Track via the Task-2 helper,
// then call computeCpaWithUncertainty and assert the truth CPA lies inside
// the 2-sigma band.
//
// Geometry. Own-ship sits stationary at the ENU origin. Target starts at
// (0, 1000) m, moving east at 10 m/s. At any time t the relative position
// is (10 t, 1000) m, separation = sqrt(100 t^2 + 1e6) m. The CPA is in the
// PAST at t = 0 (target moves away monotonically), so at t_ref = 10 s the
// closed-form formula hits the "past CPA" branch and reports the current
// distance as cpa, with sigma propagated from the joint position
// covariance via direction projection. Current distance at t_ref = 10 s is
// sqrt(100^2 + 1000^2) ~= 1004.99 m -- inside the 5 % band around 1000 m.
//
// The point of the test is not the geometry per se but the integration:
// the tracker is driven by 1 Hz Position2D measurements, so its covariance
// converges towards a small steady-state value; the own-ship Track carries
// a 1 m sigma position uncertainty; and the resulting predicted sigma_cpa
// has to be (a) positive and (b) large enough that the 2-sigma band
// contains the truth.

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/collision/Cpa.hpp"
#include "core/collision/CpaOwnShip.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker;

namespace {

constexpr double kTruthCpaM = 1000.0;
constexpr double kTRefSeconds = 10.0;
constexpr double kDThresholdM = 500.0;

// Build a perpendicular-pass scenario:
//   own-ship: stationary at ENU origin
//   target:   starts at (0, 1000) m, velocity (10, 0) m/s
//   1 Hz Position2D measurements, sigma = pos_noise_std_m
//   duration: 20 s (samples at t = 1, 2, ... 20)
Scenario buildPerpendicularPassScenario(double pos_noise_std_m,
                                        std::uint32_t seed) {
  std::vector<double> times;
  for (int i = 1; i <= 20; ++i) times.push_back(static_cast<double>(i));
  return buildStraightLineScenario(
      Eigen::Vector2d(0.0, 1000.0), Eigen::Vector2d(10.0, 0.0),
      times, pos_noise_std_m, seed);
}

// Drive the tracker and return the (single) confirmed track after running.
// Returns whether a track was successfully obtained; track is written to
// *out_track if returned true.
bool driveScenarioAndGetTrack(double pos_noise_std_m, std::uint32_t seed,
                              Track* out_track) {
  const Scenario s = buildPerpendicularPassScenario(pos_noise_std_m, seed);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(50.0);
  TrackManager mgr(/*confirm=*/2, /*delete=*/4);
  Tracker tracker(ekf, gnn, mgr, /*miss_timeout=*/30.0);

  // We don't care about OSPA here; just drive measurements through.
  (void)runScenario(s, tracker, mgr, /*ospa_cutoff=*/50.0);

  // Pick the largest-state confirmed track (there should be exactly one).
  for (const Track& t : mgr.tracks()) {
    if (t.status == TrackStatus::Confirmed && t.state.size() >= 4) {
      *out_track = t;
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(CpaScenario, PerpendicularPassTwoSigmaBandContainsTruth) {
  // Reference geometry: target_truth(t=10) = (100, 1000); separation 1004.99 m.
  Track target_track;
  ASSERT_TRUE(driveScenarioAndGetTrack(/*sigma=*/1.0, /*seed=*/13,
                                       &target_track));

  // Synthesise own-ship via Task-2 helper. The datum origin places lat/lon
  // pose at the ENU origin; velocity is zero; sigma_pos = 1 m.
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(kTRefSeconds);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d::Zero();
  pose.velocity_is_valid = false;
  const Track own_ship = synthesizeOwnShipTrack(pose, provider);

  // Run CPA with uncertainty at t_ref = 10 s.
  const CpaPrediction p = computeCpaWithUncertainty(
      own_ship, target_track, Timestamp::fromSeconds(kTRefSeconds),
      kDThresholdM);

  // Truth cpa = 1000 m; predicted should be within ~5 %.
  EXPECT_NEAR(p.cpa_distance_m, kTruthCpaM, 0.05 * kTruthCpaM)
      << "predicted CPA = " << p.cpa_distance_m;
  EXPECT_GT(p.sigma_cpa_m, 0.0);

  // 2-sigma band contains the truth value 1000 m.
  EXPECT_LT(std::abs(p.cpa_distance_m - kTruthCpaM), 2.0 * p.sigma_cpa_m)
      << "predicted = " << p.cpa_distance_m
      << ", sigma_cpa = " << p.sigma_cpa_m;

  // The truth (1000 m) is far above the alarm threshold (500 m); probability
  // should be tiny.
  EXPECT_LT(p.probability_below_threshold, 0.01)
      << "predicted CPA = " << p.cpa_distance_m
      << ", sigma_cpa = " << p.sigma_cpa_m
      << ", P = " << p.probability_below_threshold;

  std::fprintf(stderr,
               "\n[CpaScenario PerpendicularPass] sigma_pos_meas=1.0 m  "
               "predicted CPA = %.3f m  sigma_cpa = %.4f m  P(<%.0fm) = %.6f\n",
               p.cpa_distance_m, p.sigma_cpa_m, kDThresholdM,
               p.probability_below_threshold);
}

// Same scenario, but at multiple measurement noise levels. SUCCEED-only;
// the values are captured by the eval-log.
TEST(CpaScenario, PerpendicularPassNoiseSweepReport) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;

  struct Row {
    double sigma_pos_meas;
    double sigma_own_pos;
  };
  const Row rows[] = {
      {1.0, 1.0},
      {1.0, 5.0},
      {5.0, 1.0},
      {5.0, 5.0},
  };

  std::fprintf(stderr,
      "\n[CpaScenario PerpendicularPassSweep]\n"
      "  sigma_meas_m | sigma_own_m | predicted CPA |  sigma_cpa | P(<500m) | in_2sigma\n");

  for (const Row& r : rows) {
    Track target_track;
    if (!driveScenarioAndGetTrack(r.sigma_pos_meas, /*seed=*/13,
                                  &target_track)) {
      std::fprintf(stderr,
          "  %11.2f  | %10.2f  |  (no confirmed track)\n",
          r.sigma_pos_meas, r.sigma_own_pos);
      continue;
    }
    pose.position_std_m = r.sigma_own_pos;
    pose.velocity_enu = Eigen::Vector2d::Zero();
    pose.velocity_is_valid = false;
    const Track own_ship = synthesizeOwnShipTrack(pose, provider);
    const CpaPrediction p = computeCpaWithUncertainty(
        own_ship, target_track, Timestamp::fromSeconds(kTRefSeconds),
        kDThresholdM);
    const bool in_2sigma =
        std::abs(p.cpa_distance_m - kTruthCpaM) < 2.0 * p.sigma_cpa_m;
    std::fprintf(stderr,
        "  %11.2f  | %10.2f  |   %9.3f   | %9.4f  | %.6f | %s\n",
        r.sigma_pos_meas, r.sigma_own_pos,
        p.cpa_distance_m, p.sigma_cpa_m, p.probability_below_threshold,
        in_2sigma ? "yes" : "no ");
  }
  SUCCEED();
}

// With a non-zero own-ship velocity uncertainty, sigma_cpa must grow versus
// the sigma_v = 0 baseline; the predicted CPA mean is unchanged. Uses a
// future-CPA geometry so that the Jacobian's velocity-uncertainty terms enter
// sigma_cpa (the past-CPA branch only projects position covariance).
TEST(CpaScenario, PerpendicularPassVelocityUncertaintyGrowsSigmaCpa) {
  // Target at (-1000, 1000) m moving east at 10 m/s; reference time t_ref = 0.
  // Target's CPA wrt a stationary own-ship at origin occurs at t = 100 s,
  // CPA distance = 1000 m. So CPA is in the future and the general-case
  // branch of computeCpaWithUncertainty runs.
  Track target;
  target.id = TrackId{1};
  target.status = TrackStatus::Confirmed;
  target.last_update = Timestamp::fromSeconds(0.0);
  target.state.resize(4);
  target.state << -1000.0, 1000.0, 10.0, 0.0;
  target.covariance = Eigen::Matrix4d::Zero();
  target.covariance(0, 0) = 1.0;
  target.covariance(1, 1) = 1.0;
  target.covariance(2, 2) = 0.01;
  target.covariance(3, 3) = 0.01;

  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose_baseline;
  pose_baseline.time = Timestamp::fromSeconds(0.0);
  pose_baseline.lat_deg = 53.5;
  pose_baseline.lon_deg = 8.0;
  pose_baseline.position_std_m = 1.0;
  pose_baseline.velocity_enu = Eigen::Vector2d::Zero();
  pose_baseline.velocity_std_m_per_s = 0.0;
  pose_baseline.velocity_is_valid = false;
  const Track own_baseline = synthesizeOwnShipTrack(pose_baseline, provider);
  const CpaPrediction p_baseline = computeCpaWithUncertainty(
      own_baseline, target, Timestamp::fromSeconds(0.0), kDThresholdM);

  OwnShipPose pose_v = pose_baseline;
  pose_v.velocity_std_m_per_s = 1.0;
  pose_v.velocity_is_valid = true;
  const Track own_v = synthesizeOwnShipTrack(pose_v, provider);
  const CpaPrediction p_v = computeCpaWithUncertainty(
      own_v, target, Timestamp::fromSeconds(0.0), kDThresholdM);

  // Both predictions hit the future-CPA branch.
  ASSERT_GT(p_baseline.tcpa_seconds, 0.0);
  ASSERT_GT(p_v.tcpa_seconds, 0.0);
  // CPA mean unchanged by velocity uncertainty.
  EXPECT_NEAR(p_v.cpa_distance_m, p_baseline.cpa_distance_m, 1e-9);
  // sigma_cpa is strictly larger when velocity uncertainty is non-zero.
  EXPECT_GT(p_v.sigma_cpa_m, p_baseline.sigma_cpa_m);
}

// Velocity uncertainty sweep report: future-CPA geometry with σ_v in
// {0, 0.5, 1.0, 2.0} m/s. SUCCEED-only; table captured by eval-log.
// Target at (-1000, 1000) m moving east at 10 m/s; TCPA = 100 s, CPA = 1000 m.
TEST(CpaScenario, PerpendicularPassVelocityUncertaintySweepReport) {
  Track target;
  target.id = TrackId{1};
  target.status = TrackStatus::Confirmed;
  target.last_update = Timestamp::fromSeconds(0.0);
  target.state.resize(4);
  target.state << -1000.0, 1000.0, 10.0, 0.0;
  target.covariance = Eigen::Matrix4d::Zero();
  target.covariance(0, 0) = 1.0;
  target.covariance(1, 1) = 1.0;
  target.covariance(2, 2) = 0.01;
  target.covariance(3, 3) = 0.01;

  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d::Zero();

  struct Row {
    double sigma_pos_m;
    double sigma_v_m_per_s;
  };
  const Row rows[] = {
      {1.0, 0.0},
      {1.0, 0.5},
      {1.0, 1.0},
      {1.0, 2.0},
  };

  std::fprintf(stderr,
      "\n[CpaScenario PerpendicularPassVelocityUncertaintySweep] future-CPA\n"
      "  sigma_pos_m | sigma_v (m/s) | predicted CPA | sigma_cpa | P(<200m)\n");

  for (const Row& r : rows) {
    pose.position_std_m = r.sigma_pos_m;
    if (r.sigma_v_m_per_s > 0.0) {
      pose.velocity_std_m_per_s = r.sigma_v_m_per_s;
      pose.velocity_is_valid = true;
    } else {
      pose.velocity_std_m_per_s = 0.0;
      pose.velocity_is_valid = false;
    }
    const Track own_ship = synthesizeOwnShipTrack(pose, provider);
    const CpaPrediction p = computeCpaWithUncertainty(
        own_ship, target, Timestamp::fromSeconds(0.0), 200.0);
    std::fprintf(stderr,
        "  %10.1f  | %13.1f  |   %9.3f   | %9.4f  | %.6f\n",
        r.sigma_pos_m, r.sigma_v_m_per_s,
        p.cpa_distance_m, p.sigma_cpa_m, p.probability_below_threshold);
  }
  SUCCEED();
}
