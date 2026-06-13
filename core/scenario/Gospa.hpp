#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

// Generalized OSPA (Rahmathullah, García-Fernández, Svensson 2017,
// arXiv:1601.05585).
//
//   GOSPA(X, Y; c, p, α) = ( min_π Σ_{(i,j)∈π} d(x_i, y_j)^p
//                          + (c^p / α) · (|X| + |Y| − 2|π|) )^(1/p)
//
// where d(x, y) = min(‖x − y‖, c). Compared to OSPA (Ospa.hpp):
//   - OSPA divides the total cost by max(|X|, |Y|) → bounded by c.
//   - GOSPA does NOT divide → cost grows with target count; missed and
//     false targets are charged c^p / α each (with α=2: each missed
//     and each false target contributes c^p / 2). This is what makes
//     GOSPA the conventional choice for multi-target tracking
//     evaluations — it surfaces cardinality errors directly instead of
//     folding them into a saturated cutoff.
//
// Defaults p=2, α=2 match the convention used throughout the GOSPA /
// PMBM literature.
//
// Assignment: greedy nearest-neighbour, same as ospaGreedy (matches our
// existing convention; equivalent to Hungarian within numerical noise on
// the small target counts in the bench scenarios — promote when
// profiling demands it).
double gospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                   const std::vector<Eigen::Vector2d>& est,
                   double cutoff,
                   double p = 2.0,
                   double alpha = 2.0);

}  // namespace navtracker
