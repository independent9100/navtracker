#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

/**
 * Effective sample size of a normalized weight vector: 1 / Σ w_i².
 * Equals N for uniform weights, 1 for a fully concentrated weight vector.
 */
double effectiveSampleSize(const Eigen::VectorXd& weights);

/**
 * Systematic resampling: returns N indices drawn from the CDF of `weights`
 * at the equally-spaced positions {u, u + 1/N, u + 2/N, ...}. Caller supplies
 * the starting offset `u ∈ [0, 1/N)` so the RNG is owned externally.
 * Weights are assumed normalized (Σ w_i = 1); behavior is undefined otherwise.
 */
std::vector<int> systematicResample(const Eigen::VectorXd& weights, double u);

}  // namespace navtracker
