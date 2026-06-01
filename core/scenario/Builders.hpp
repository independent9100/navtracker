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

}  // namespace navtracker
