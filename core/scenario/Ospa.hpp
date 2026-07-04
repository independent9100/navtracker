#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

/**
 * OSPA distance (Schuhmacher et al. 2008), p=2, cutoff c. Assignment is
 * optimal (min-cost) via the Hungarian algorithm on the clipped-distance²
 * cost matrix; the |X|−|Y| surplus is charged c² each, normalised by
 * max(|X|,|Y|). (The name keeps the historical "Greedy" suffix for
 * call-site stability; the implementation is no longer greedy.)
 */
double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff);

}  // namespace navtracker
