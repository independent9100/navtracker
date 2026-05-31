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

}  // namespace navtracker
