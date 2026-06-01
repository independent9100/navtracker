#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/scenario/Truth.hpp"

namespace navtracker {

Scenario buildStraightLineScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

Scenario buildParallelTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two CV targets on opposing courses, optionally laterally offset. Truth is
// emitted in (A, B) order per step.
Scenario buildCrossingTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& velocity_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity_b,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two same-direction CV targets with a lateral offset. Truth is emitted in
// (slow, fast) order per step.
Scenario buildOvertakingScenario(
    const Eigen::Vector2d& start_slow,
    const Eigen::Vector2d& velocity_slow,
    const Eigen::Vector2d& start_fast,
    const Eigen::Vector2d& velocity_fast,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// One CV target observed by a sensor at the ENU origin: first sample is a
// Position2D measurement (to seed the track), subsequent samples are
// RangeBearing2D measurements with the supplied noise. Designed to stress
// nonlinear measurement filtering — pass the target close to the origin
// to exercise high curvature of h(x) = (r, atan2(py, px)).
Scenario buildRangeBearingPassScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double range_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// One CV target observed by a bearing-only sensor at the ENU origin. The
// first sample is a Position2D measurement with deliberately WIDE
// covariance to seed a broad prior; subsequent samples are Bearing2D only
// (scalar β = atan2(py, px)). Designed to stress filters: the posterior
// over range stays as wide as the prior, so a Kalman filter's Gaussian
// approximation degrades while a particle filter can represent the
// banana-shaped posterior faithfully.
Scenario buildBearingOnlyScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Target follows three legs: straight at constant velocity from `start` for
// `straight_duration_s`; then a constant-rate turn at `omega_rad_s` for
// `turn_duration_s` (omega > 0 = left turn in ENU); then straight again for
// `straight_duration_s`. Position2D measurements at every `sample_dt_s`.
Scenario buildManeuveringTargetScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    double straight_duration_s,
    double turn_duration_s,
    double omega_rad_s,
    double sample_dt_s,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Two crossing CV targets plus uniform-random clutter measurements per scan.
// All measurements at a given timestamp share that timestamp exactly, so
// `runScenarioBatched` will group them. `n_clutter_per_scan` false alarms
// are drawn uniformly within the box [clutter_min, clutter_max] each scan.
Scenario buildClutterCrossingScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& velocity_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity_b,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    int n_clutter_per_scan,
    const Eigen::Vector2d& clutter_min,
    const Eigen::Vector2d& clutter_max,
    std::uint32_t seed = 0);

// Two CV targets crossing at the origin with a brief sensor dropout. Targets
// move toward each other along the x axis at +/- velocity_x, with a small
// y offset (closest approach ~ 2 * y_offset_m). A sensor dropout zeroes the
// emitted measurements for any scan whose timestamp falls within
// [dropout_start_s, dropout_end_s); truth is still emitted so OSPA can be
// computed.
Scenario buildCrossingDropoutScenario(
    double velocity_x_mps,
    double y_offset_m,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    double dropout_start_s,
    double dropout_end_s,
    std::uint32_t seed = 0);

// Stationary target observed by a bearing-only sensor on a moving platform.
// Sensor starts at sensor_start and moves with sensor_velocity; target sits
// at target_position. Initial sample emits a wide Position2D seed at the
// target with covariance initial_position_std_m^2 (sensor_position_enu set
// to the sensor's initial location). Subsequent samples emit Bearing2D
// measurements whose sensor_position_enu is the sensor's ENU position at
// that timestamp. Range becomes observable via parallax over the run.
Scenario buildBearingOnlyMovingSensorScenario(
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& sensor_start,
    const Eigen::Vector2d& sensor_velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

}  // namespace navtracker
