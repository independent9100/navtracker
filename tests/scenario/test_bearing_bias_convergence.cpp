#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

constexpr double kBiasTrue = 0.0349;  // 2 degrees

Measurement makePosition(const Eigen::Vector2d& truth, double t_s,
                         double std_m, std::mt19937_64& rng) {
  std::normal_distribution<double> noise(0.0, std_m);
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;  // stand-in for any unbiased positioning source
  m.source_id = "anchor";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(truth.x() + noise(rng), truth.y() + noise(rng));
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}

Measurement makeBiasedBearing(const Eigen::Vector2d& truth,
                              const Eigen::Vector2d& sensor,
                              double t_s, double bias_rad, double std_rad,
                              std::mt19937_64& rng) {
  std::normal_distribution<double> noise(0.0, std_rad);
  const double beta_true = std::atan2(truth.y() - sensor.y(),
                                      truth.x() - sensor.x());
  const double beta_obs = beta_true + bias_rad + noise(rng);
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "eoir";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = beta_obs;
  m.value = v;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = std_rad * std_rad + 1e-9;
  m.covariance = R;
  m.sensor_position_enu = sensor;
  return m;
}

}  // namespace

TEST(BearingBiasConvergence, AnchoredBearingStreamReconstructsTrueBias) {
  // Stationary target at (800, 200). A stationary unbiased-positioning
  // source (stand-in for GPS / lidar / anchor radar) emits a Position2D
  // measurement at every cycle with std 1 m. A biased EO/IR sensor at
  // (0,0) emits a Bearing2D measurement at the same cycle, shifted by
  // kBiasTrue. The anchor keeps the EKF state from absorbing the bias;
  // the bearing stream's innovation relative to the anchored state is
  // therefore a clean measurement of b.
  const Eigen::Vector2d target(800.0, 200.0);
  const Eigen::Vector2d sensor(0.0, 0.0);
  std::mt19937_64 rng(7);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 1.0);
  GnnAssociator assoc(500.0);
  TrackManager mgr(1, 6);
  Tracker tracker(est, assoc, mgr, 200.0);

  HeadingBiasEstimator bias({});
  tracker.setBearingInnovationSink(&bias);

  const int N = 200;
  for (int i = 0; i < N; ++i) {
    const double t = 0.5 * (i + 1);
    tracker.process(makePosition(target, t, /*std_m=*/1.0, rng));
    // Slight time offset so Position2D and Bearing2D do not collide on
    // ts inside the associator's single-timestamp batch handling.
    tracker.process(
        makeBiasedBearing(target, sensor, t + 0.01, kBiasTrue, 0.01, rng));
  }

  const double err = std::abs(bias.biasRad() - kBiasTrue);
  EXPECT_LT(err, 0.5 * 3.14159265358979323846 / 180.0)
      << "bias=" << bias.biasRad() << " true=" << kBiasTrue
      << " accepted=" << bias.acceptedBearingObs()
      << " rej_range=" << bias.rejectedByRange()
      << " rej_state=" << bias.rejectedByStateVar()
      << " rej_outlier=" << bias.rejectedByOutlier();
  EXPECT_GT(bias.acceptedBearingObs(), 10u);
  EXPECT_TRUE(bias.current().is_published);
}
