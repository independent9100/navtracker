#include "core/scenario/Builders.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include "core/types/Ids.hpp"

namespace navtracker {
namespace {

Measurement makeMeasurement(const Eigen::Vector2d& noisy_pos,
                            double t_seconds,
                            double std_m) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::Ais;
  m.source_id = "sim";
  m.model = MeasurementModel::Position2D;
  m.value = noisy_pos;
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}

Measurement makeRangeBearingMeasurement(double noisy_r,
                                        double noisy_b,
                                        double t_seconds,
                                        double range_std,
                                        double bearing_std) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = "sim_rb";
  m.model = MeasurementModel::RangeBearing2D;
  m.value = Eigen::Vector2d(noisy_r, noisy_b);
  m.covariance = Eigen::Matrix2d::Zero();
  m.covariance(0, 0) = range_std * range_std + 1e-6;
  m.covariance(1, 1) = bearing_std * bearing_std + 1e-9;
  return m;
}

TruthSample makeTruth(const Eigen::Vector2d& pos,
                      const Eigen::Vector2d& vel,
                      double t_seconds,
                      std::uint64_t truth_id) {
  TruthSample ts;
  ts.time = Timestamp::fromSeconds(t_seconds);
  ts.truth_id = truth_id;
  ts.position = pos;
  ts.velocity = vel;
  return ts;
}

Measurement makeBearingMeasurement(double noisy_b,
                                   double t_seconds,
                                   double bearing_std) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::EoIr;  // bearing-only is camera-like
  m.source_id = "sim_b";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = noisy_b;
  m.value = v;
  Eigen::MatrixXd r(1, 1);
  r(0, 0) = bearing_std * bearing_std + 1e-9;
  m.covariance = r;
  return m;
}

}  // namespace

Scenario buildStraightLineScenario(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& velocity,
                                   const std::vector<double>& times,
                                   double pos_noise_std_m,
                                   std::uint32_t seed,
                                   std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_pos = start + velocity * t;
    s.truth.push_back(makeTruth(truth_pos, velocity, t, truth_id));
    const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                truth_pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildParallelTargetsScenario(const Eigen::Vector2d& start_a,
                                      const Eigen::Vector2d& start_b,
                                      const Eigen::Vector2d& velocity,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_a = start_a + velocity * t;
    const Eigen::Vector2d truth_b = start_b + velocity * t;
    s.truth.push_back(makeTruth(truth_a, velocity, t, 1));
    s.truth.push_back(makeTruth(truth_b, velocity, t, 2));
    const Eigen::Vector2d noisy_a(truth_a.x() + noise(rng),
                                  truth_a.y() + noise(rng));
    const Eigen::Vector2d noisy_b(truth_b.x() + noise(rng),
                                  truth_b.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_a, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_b, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildCrossingTargetsScenario(const Eigen::Vector2d& start_a,
                                      const Eigen::Vector2d& velocity_a,
                                      const Eigen::Vector2d& start_b,
                                      const Eigen::Vector2d& velocity_b,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_a = start_a + velocity_a * t;
    const Eigen::Vector2d truth_b = start_b + velocity_b * t;
    s.truth.push_back(makeTruth(truth_a, velocity_a, t, 1));
    s.truth.push_back(makeTruth(truth_b, velocity_b, t, 2));
    const Eigen::Vector2d noisy_a(truth_a.x() + noise(rng),
                                  truth_a.y() + noise(rng));
    const Eigen::Vector2d noisy_b(truth_b.x() + noise(rng),
                                  truth_b.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_a, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_b, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildOvertakingScenario(const Eigen::Vector2d& start_slow,
                                 const Eigen::Vector2d& velocity_slow,
                                 const Eigen::Vector2d& start_fast,
                                 const Eigen::Vector2d& velocity_fast,
                                 const std::vector<double>& times,
                                 double pos_noise_std_m,
                                 std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_slow = start_slow + velocity_slow * t;
    const Eigen::Vector2d truth_fast = start_fast + velocity_fast * t;
    s.truth.push_back(makeTruth(truth_slow, velocity_slow, t, 1));
    s.truth.push_back(makeTruth(truth_fast, velocity_fast, t, 2));
    const Eigen::Vector2d noisy_slow(truth_slow.x() + noise(rng),
                                     truth_slow.y() + noise(rng));
    const Eigen::Vector2d noisy_fast(truth_fast.x() + noise(rng),
                                     truth_fast.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_slow, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_fast, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildRangeBearingPassScenario(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& velocity,
                                       const std::vector<double>& times,
                                       double initial_position_std_m,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       std::uint32_t seed,
                                       std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> pos_noise(0.0, initial_position_std_m);
  std::normal_distribution<double> r_noise(0.0, range_std_m);
  std::normal_distribution<double> b_noise(0.0, bearing_std_rad);
  Scenario s;
  for (std::size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    const Eigen::Vector2d truth = start + velocity * t;
    s.truth.push_back(makeTruth(truth, velocity, t, truth_id));
    if (i == 0) {
      const Eigen::Vector2d noisy(truth.x() + pos_noise(rng),
                                  truth.y() + pos_noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, initial_position_std_m));
    } else {
      const double r = std::hypot(truth.x(), truth.y());
      const double b = std::atan2(truth.y(), truth.x());
      const double noisy_r = std::max(0.001, r + r_noise(rng));
      const double noisy_b = b + b_noise(rng);
      s.measurements.push_back(
          makeRangeBearingMeasurement(noisy_r, noisy_b, t, range_std_m, bearing_std_rad));
    }
  }
  return s;
}

Scenario buildBearingOnlyScenario(const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& velocity,
                                  const std::vector<double>& times,
                                  double initial_position_std_m,
                                  double bearing_std_rad,
                                  std::uint32_t seed,
                                  std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> pos_noise(0.0, initial_position_std_m);
  std::normal_distribution<double> b_noise(0.0, bearing_std_rad);
  Scenario s;
  for (std::size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    const Eigen::Vector2d truth = start + velocity * t;
    s.truth.push_back(makeTruth(truth, velocity, t, truth_id));
    if (i == 0) {
      const Eigen::Vector2d noisy(truth.x() + pos_noise(rng),
                                  truth.y() + pos_noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, initial_position_std_m));
    } else {
      const double b = std::atan2(truth.y(), truth.x());
      const double noisy_b = b + b_noise(rng);
      s.measurements.push_back(
          makeBearingMeasurement(noisy_b, t, bearing_std_rad));
    }
  }
  return s;
}

}  // namespace navtracker
