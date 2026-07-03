#include "core/scenario/Builders.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "core/types/Ids.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

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

Measurement makeArpaPositionMeasurement(const Eigen::Vector2d& noisy_pos,
                                        double t_seconds, double std_m,
                                        const std::string& source_id) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = source_id;
  m.model = MeasurementModel::Position2D;
  m.value = noisy_pos;
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}

Measurement makeShoreMeasurement(const Eigen::Vector2d& noisy_pos,
                                 double t_seconds, double std_m) {
  return makeArpaPositionMeasurement(noisy_pos, t_seconds, std_m, "sim_shore");
}

// Precondition: measurements in s are time-grouped (the same timestamp appears
// consecutively). Builders emit grouped; addFixedClutter/addAnchoredBoats/
// addUniformClutter sort (measurements AND truth) before returning. On
// ungrouped input this back-compare dedup would emit a time more than once,
// causing duplicate injections.
std::vector<double> distinctScanTimes(const Scenario& s) {
  std::vector<double> scan_times;
  for (const auto& m : s.measurements) {
    const double t = m.time.seconds();
    if (scan_times.empty() || scan_times.back() != t) scan_times.push_back(t);
  }
  return scan_times;
}

void sortMeasurementsByTime(Scenario& s) {
  // Compare exact Timestamp nanos (not lossy .seconds() doubles) — the same
  // ordering BenchRunner::flushScansUpTo / groupTruth bucket on downstream.
  std::stable_sort(s.measurements.begin(), s.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time < b.time;
                   });
}

// BenchRunner::groupTruth requires truth sorted by non-decreasing time — it
// only opens a new bucket when the timestamp changes, so an out-of-order run
// silently fragments into duplicate groups (a corrupt benchmark). Builders that
// emit truth strictly per-tick stay sorted for free, but addAnchoredBoats
// appends a whole second time-run onto pre-existing base truth. The additive
// builders sort truth explicitly so the returned Scenario is uniformly time-
// sorted in BOTH measurements and truth, regardless of composition order.
void sortTruthByTime(Scenario& s) {
  std::stable_sort(s.truth.begin(), s.truth.end(),
                   [](const TruthSample& a, const TruthSample& b) {
                     return a.time < b.time;
                   });
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

Scenario buildManeuveringTargetScenario(const Eigen::Vector2d& start,
                                        const Eigen::Vector2d& velocity,
                                        double straight_duration_s,
                                        double turn_duration_s,
                                        double omega_rad_s,
                                        double sample_dt_s,
                                        double pos_noise_std_m,
                                        std::uint32_t seed,
                                        std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  const double t_end_leg1 = straight_duration_s;
  const double t_end_turn = straight_duration_s + turn_duration_s;
  const double t_end_leg2 =
      straight_duration_s + turn_duration_s + straight_duration_s;
  const Eigen::Vector2d p_turn_start =
      start + velocity * straight_duration_s;

  auto turnPositionVelocity = [&](double tau)
      -> std::pair<Eigen::Vector2d, Eigen::Vector2d> {
    const double w = omega_rad_s;
    if (std::abs(w) < 1e-9) {
      return {p_turn_start + velocity * tau, velocity};
    }
    const double c = std::cos(w * tau);
    const double si = std::sin(w * tau);
    const double vx = velocity.x() * c - velocity.y() * si;
    const double vy = velocity.x() * si + velocity.y() * c;
    const double px = p_turn_start.x() +
                       (velocity.x() * si + velocity.y() * (c - 1.0)) / w;
    const double py = p_turn_start.y() +
                       (velocity.y() * si - velocity.x() * (c - 1.0)) / w;
    return std::make_pair(Eigen::Vector2d(px, py), Eigen::Vector2d(vx, vy));
  };
  const auto turn_end = turnPositionVelocity(turn_duration_s);
  const Eigen::Vector2d p_leg2_start = turn_end.first;
  const Eigen::Vector2d v_leg2 = turn_end.second;

  double t = 0.0;
  while (t <= t_end_leg2 + 1e-9) {
    Eigen::Vector2d truth_pos;
    Eigen::Vector2d truth_vel;
    if (t <= t_end_leg1) {
      truth_pos = start + velocity * t;
      truth_vel = velocity;
    } else if (t <= t_end_turn) {
      const auto pv = turnPositionVelocity(t - t_end_leg1);
      truth_pos = pv.first;
      truth_vel = pv.second;
    } else {
      truth_pos = p_leg2_start + v_leg2 * (t - t_end_turn);
      truth_vel = v_leg2;
    }
    s.truth.push_back(makeTruth(truth_pos, truth_vel, t, truth_id));
    const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                truth_pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
    t += sample_dt_s;
  }
  return s;
}

Scenario buildClutterCrossingScenario(const Eigen::Vector2d& start_a,
                                      const Eigen::Vector2d& velocity_a,
                                      const Eigen::Vector2d& start_b,
                                      const Eigen::Vector2d& velocity_b,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      int n_clutter_per_scan,
                                      const Eigen::Vector2d& clutter_min,
                                      const Eigen::Vector2d& clutter_max,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> ux(clutter_min.x(), clutter_max.x());
  std::uniform_real_distribution<double> uy(clutter_min.y(), clutter_max.y());
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
    for (int k = 0; k < n_clutter_per_scan; ++k) {
      const Eigen::Vector2d fp(ux(rng), uy(rng));
      s.measurements.push_back(makeMeasurement(fp, t, pos_noise_std_m));
    }
  }
  return s;
}

Scenario buildCrossingDropoutScenario(double velocity_x_mps,
                                      double y_offset_m,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      double dropout_start_s,
                                      double dropout_end_s,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  const Eigen::Vector2d va( velocity_x_mps, 0.0);
  const Eigen::Vector2d vb(-velocity_x_mps, 0.0);
  const double t_mid = times.empty()
      ? 0.0
      : 0.5 * (times.front() + times.back());
  const Eigen::Vector2d start_a(-velocity_x_mps * t_mid,  y_offset_m);
  const Eigen::Vector2d start_b( velocity_x_mps * t_mid, -y_offset_m);

  for (double t : times) {
    const Eigen::Vector2d truth_a = start_a + va * t;
    const Eigen::Vector2d truth_b = start_b + vb * t;
    s.truth.push_back(makeTruth(truth_a, va, t, 1));
    s.truth.push_back(makeTruth(truth_b, vb, t, 2));
    if (t >= dropout_start_s && t < dropout_end_s) continue;
    const Eigen::Vector2d noisy_a(truth_a.x() + noise(rng),
                                  truth_a.y() + noise(rng));
    const Eigen::Vector2d noisy_b(truth_b.x() + noise(rng),
                                  truth_b.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_a, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_b, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildBearingOnlyMovingSensorScenario(
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& sensor_start,
    const Eigen::Vector2d& sensor_velocity,
    const std::vector<double>& times,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed,
    std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> pos_noise(0.0, initial_position_std_m);
  std::normal_distribution<double> b_noise(0.0, bearing_std_rad);
  Scenario s;
  const Eigen::Vector2d zero_vel(0.0, 0.0);
  for (std::size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    s.truth.push_back(makeTruth(target_position, zero_vel, t, truth_id));
    const Eigen::Vector2d sensor_pos = sensor_start + sensor_velocity * t;
    if (i == 0) {
      const Eigen::Vector2d noisy(target_position.x() + pos_noise(rng),
                                  target_position.y() + pos_noise(rng));
      Measurement m = makeMeasurement(noisy, t, initial_position_std_m);
      m.sensor_position_enu = sensor_pos;
      s.measurements.push_back(std::move(m));
    } else {
      const double dx = target_position.x() - sensor_pos.x();
      const double dy = target_position.y() - sensor_pos.y();
      const double b = std::atan2(dy, dx);
      const double noisy_b = b + b_noise(rng);
      Measurement m = makeBearingMeasurement(noisy_b, t, bearing_std_rad);
      m.sensor_position_enu = sensor_pos;
      s.measurements.push_back(std::move(m));
    }
  }
  return s;
}

Scenario buildSpeedChangeScenario(const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& initial_velocity,
                                  double surge_start_s,
                                  double surge_duration_s,
                                  double surge_accel_mps2,
                                  double drift_decel_mps2,
                                  const std::vector<double>& times,
                                  double pos_noise_std_m,
                                  std::uint32_t seed,
                                  std::uint64_t truth_id) {
  // Three-phase along-track motion:
  //  [0, surge_start)        : constant velocity (initial_velocity)
  //  [surge_start, surge_end): constant acceleration `surge_accel_mps2`
  //                            along the initial heading (engine spool-up)
  //  [surge_end, end]        : constant deceleration `drift_decel_mps2`
  //                            along the initial heading (drift after
  //                            thrust loss). Speed clamped at zero.
  //
  // No heading change anywhere — this is the niche CV and CT both miss:
  // CV mis-models the speed change; CT only tracks rotational motion.
  // The noisy-CV mode (high accel PSD, no heading rotation) is exactly
  // the right model for this segment.
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);

  const double speed0 = initial_velocity.norm();
  Eigen::Vector2d heading = initial_velocity;
  if (speed0 > 1e-9) heading /= speed0;
  else heading = Eigen::Vector2d(1.0, 0.0);  // safe default

  const double surge_end_s = surge_start_s + surge_duration_s;
  const auto positionAt = [&](double t) -> std::pair<Eigen::Vector2d, Eigen::Vector2d> {
    double s;        // speed at t
    double dist;     // along-track distance at t
    if (t < surge_start_s) {
      s = speed0;
      dist = speed0 * t;
    } else if (t < surge_end_s) {
      const double dt = t - surge_start_s;
      s = speed0 + surge_accel_mps2 * dt;
      dist = speed0 * surge_start_s
           + speed0 * dt + 0.5 * surge_accel_mps2 * dt * dt;
    } else {
      const double s_end = speed0 + surge_accel_mps2 * surge_duration_s;
      const double dt = t - surge_end_s;
      // Decelerate; clamp at zero so the target doesn't run backwards.
      const double s_raw = s_end - drift_decel_mps2 * dt;
      s = std::max(0.0, s_raw);
      const double t_stop = (drift_decel_mps2 > 1e-9)
          ? s_end / drift_decel_mps2 : std::numeric_limits<double>::infinity();
      double dist_drift;
      if (dt < t_stop) {
        dist_drift = s_end * dt - 0.5 * drift_decel_mps2 * dt * dt;
      } else {
        dist_drift = 0.5 * s_end * t_stop;  // distance until stop
      }
      const double surge_dist =
          speed0 * surge_duration_s
        + 0.5 * surge_accel_mps2 * surge_duration_s * surge_duration_s;
      dist = speed0 * surge_start_s + surge_dist + dist_drift;
    }
    Eigen::Vector2d pos = start + heading * dist;
    Eigen::Vector2d vel = heading * s;
    return {pos, vel};
  };

  Scenario s;
  for (double t : times) {
    const auto [pos, vel] = positionAt(t);
    s.truth.push_back(makeTruth(pos, vel, t, truth_id));
    const Eigen::Vector2d noisy(pos.x() + noise(rng),
                                pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildParallelLaneScenario(int n_targets, double lane_spacing_m,
                                   const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& velocity,
                                   const std::vector<double>& times,
                                   double pos_noise_std_m, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const Eigen::Vector2d dir = velocity.normalized();
  const Eigen::Vector2d perp(-dir.y(), dir.x());
  Scenario s;
  for (double t : times) {
    for (int i = 0; i < n_targets; ++i) {
      const Eigen::Vector2d lane_start =
          start + perp * (static_cast<double>(i) * lane_spacing_m);
      const Eigen::Vector2d truth_pos = lane_start + velocity * t;
      s.truth.push_back(
          makeTruth(truth_pos, velocity, t, static_cast<std::uint64_t>(i + 1)));
      const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                  truth_pos.y() + noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
    }
  }
  return s;
}

Scenario buildCrossingAngleScenario(double crossing_angle_deg, double speed_mps,
                                    const Eigen::Vector2d& crossing_point,
                                    const std::vector<double>& times,
                                    double pos_noise_std_m, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const double th = crossing_angle_deg * kPi / 180.0;
  const Eigen::Vector2d vel_a(speed_mps, 0.0);
  const Eigen::Vector2d vel_b(speed_mps * std::cos(th), speed_mps * std::sin(th));
  const double t_mid =
      times.empty() ? 0.0 : 0.5 * (times.front() + times.back());
  const Eigen::Vector2d start_a = crossing_point - vel_a * t_mid;
  const Eigen::Vector2d start_b = crossing_point - vel_b * t_mid;
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d ta = start_a + vel_a * t;
    const Eigen::Vector2d tb = start_b + vel_b * t;
    s.truth.push_back(makeTruth(ta, vel_a, t, 1));
    s.truth.push_back(makeTruth(tb, vel_b, t, 2));
    const Eigen::Vector2d na(ta.x() + noise(rng), ta.y() + noise(rng));
    const Eigen::Vector2d nb(tb.x() + noise(rng), tb.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(na, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(nb, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildConvoyScenario(int n_targets, double gap_m, double speed_mps,
                             double overtaker_speed_mps,
                             const std::vector<double>& times,
                             double pos_noise_std_m, std::uint32_t seed) {
  constexpr double kOvertakerLateralOffsetM = 25.0;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const Eigen::Vector2d vel(speed_mps, 0.0);
  const Eigen::Vector2d vel_ot(overtaker_speed_mps, 0.0);
  const Eigen::Vector2d ot_start(
      -static_cast<double>(n_targets) * gap_m - 100.0, kOvertakerLateralOffsetM);
  Scenario s;
  for (double t : times) {
    for (int i = 0; i < n_targets; ++i) {
      const Eigen::Vector2d start_i(-static_cast<double>(i) * gap_m, 0.0);
      const Eigen::Vector2d truth_pos = start_i + vel * t;
      s.truth.push_back(
          makeTruth(truth_pos, vel, t, static_cast<std::uint64_t>(i + 1)));
      const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                  truth_pos.y() + noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
    }
    const Eigen::Vector2d ot_pos = ot_start + vel_ot * t;
    s.truth.push_back(makeTruth(ot_pos, vel_ot, t,
                                static_cast<std::uint64_t>(n_targets + 1)));
    const Eigen::Vector2d ot_noisy(ot_pos.x() + noise(rng),
                                   ot_pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(ot_noisy, t, pos_noise_std_m));
  }
  return s;
}

SyntheticShore buildSyntheticShore(const geo::Geodetic& datum_origin,
                                   double shore_y_m, double extent_m,
                                   double land_depth_m, double pier_width_m,
                                   double pier_length_m, int n_clutter,
                                   const CoastlinePriorParams& params) {
  const geo::Datum datum(datum_origin);
  const double S = shore_y_m;
  const double T = shore_y_m + land_depth_m;
  const double L = -extent_m, R = extent_m;
  const double pw = 0.5 * pier_width_m;
  const double pl = pier_length_m;
  // ENU outer ring: land interior above the shoreline with a pier notch
  // protruding into the water at x = 0.
  const std::vector<Eigen::Vector2d> ring_enu = {
      {L, S},      {-pw, S}, {-pw, S - pl}, {pw, S - pl},
      {pw, S},     {R, S},   {R, T},        {L, T}};
  LandPolygon poly;
  poly.outer.reserve(ring_enu.size());
  for (const auto& p : ring_enu) {
    const geo::Geodetic g =
        datum.toGeodetic(Eigen::Vector3d(p.x(), p.y(), 0.0));
    poly.outer.emplace_back(g.lon_deg, g.lat_deg);
  }
  CoastlineGeometry geometry({poly}, params);
  std::vector<Eigen::Vector2d> clutter;
  clutter.reserve(static_cast<std::size_t>(std::max(0, n_clutter)));
  const double y_inland = shore_y_m + 0.5 * land_depth_m;
  const double x0 = -0.8 * extent_m, x1 = 0.8 * extent_m;
  for (int i = 0; i < n_clutter; ++i) {
    const double frac =
        n_clutter <= 1 ? 0.5 : static_cast<double>(i) / (n_clutter - 1);
    clutter.emplace_back(x0 + frac * (x1 - x0), y_inland);
  }
  return SyntheticShore{std::move(geometry), datum, std::move(clutter)};
}

Scenario addFixedClutter(Scenario base, const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& points,
                         const std::string& source_id, double detection_prob,
                         double pos_noise_std_m, std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    for (const auto& pt : points) {
      if (u(rng) < detection_prob) {
        const Eigen::Vector2d noisy(pt.x() + noise(rng), pt.y() + noise(rng));
        base.measurements.push_back(
            makeArpaPositionMeasurement(noisy, t, pos_noise_std_m, source_id));
      }
    }
  }
  sortMeasurementsByTime(base);
  sortTruthByTime(base);
  base.datum = datum;
  return base;
}

Scenario addShoreClutter(Scenario base, const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& clutter_enu_points,
                         double detection_prob, double pos_noise_std_m,
                         std::uint32_t seed) {
  return addFixedClutter(std::move(base), datum, clutter_enu_points, "sim_shore",
                         detection_prob, pos_noise_std_m, seed);
}

Scenario addAnchoredBoats(Scenario base, const geo::Datum& datum,
                          const std::vector<Eigen::Vector2d>& boat_positions,
                          std::uint64_t truth_id_start, double detection_prob,
                          double pos_noise_std_m, std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    for (std::size_t b = 0; b < boat_positions.size(); ++b) {
      const Eigen::Vector2d& pos = boat_positions[b];
      const std::uint64_t id = truth_id_start + static_cast<std::uint64_t>(b);
      // Truth exists every scan regardless of detection (the boat is there).
      base.truth.push_back(makeTruth(pos, Eigen::Vector2d::Zero(), t, id));
      if (u(rng) < detection_prob) {
        const Eigen::Vector2d noisy(pos.x() + noise(rng), pos.y() + noise(rng));
        base.measurements.push_back(makeArpaPositionMeasurement(
            noisy, t, pos_noise_std_m, "sim_anchored"));
      }
    }
  }
  sortMeasurementsByTime(base);
  sortTruthByTime(base);
  base.datum = datum;
  return base;
}

Scenario addStopGoBoat(Scenario base, const geo::Datum& datum,
                       const Eigen::Vector2d& anchored_pos,
                       const Eigen::Vector2d& depart_velocity,
                       double move_start_s, std::uint64_t truth_id,
                       double detection_prob, double pos_noise_std_m,
                       std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    const bool moving = t >= move_start_s;
    const Eigen::Vector2d pos =
        moving ? anchored_pos + depart_velocity * (t - move_start_s)
               : anchored_pos;
    const Eigen::Vector2d vel =
        moving ? depart_velocity : Eigen::Vector2d::Zero();
    // Truth exists every scan under the one id — identity must survive the
    // static→moving transition.
    base.truth.push_back(makeTruth(pos, vel, t, truth_id));
    if (u(rng) < detection_prob) {
      const Eigen::Vector2d noisy(pos.x() + noise(rng), pos.y() + noise(rng));
      base.measurements.push_back(
          makeArpaPositionMeasurement(noisy, t, pos_noise_std_m, "sim_stopgo"));
    }
  }
  sortMeasurementsByTime(base);
  sortTruthByTime(base);
  base.datum = datum;
  return base;
}

Scenario addUniformClutter(Scenario base, const geo::Datum& datum,
                           const Eigen::Vector2d& box_min,
                           const Eigen::Vector2d& box_max, int n_per_scan,
                           std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(box_min.x(), box_max.x());
  std::uniform_real_distribution<double> uy(box_min.y(), box_max.y());
  for (double t : scan_times) {
    for (int i = 0; i < n_per_scan; ++i) {
      const Eigen::Vector2d fp(ux(rng), uy(rng));
      base.measurements.push_back(
          makeArpaPositionMeasurement(fp, t, /*std_m=*/1.0, "sim_clutter"));
    }
  }
  sortMeasurementsByTime(base);
  sortTruthByTime(base);
  base.datum = datum;
  return base;
}

}  // namespace navtracker
