#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

/**
 * Generalized OSPA (Rahmathullah, García-Fernández, Svensson 2017,
 * arXiv:1601.05585).
 *
 *   GOSPA(X, Y; c, p, α) = ( min_π Σ_{(i,j)∈π} d(x_i, y_j)^p
 *                          + (c^p / α) · (|X| + |Y| − 2|π|) )^(1/p)
 *
 * where d(x, y) = min(‖x − y‖, c). Compared to OSPA (Ospa.hpp):
 *   - OSPA divides the total cost by max(|X|, |Y|) → bounded by c.
 *   - GOSPA does NOT divide → cost grows with target count; missed and
 *     false targets are charged c^p / α each (with α=2: each missed
 *     and each false target contributes c^p / 2). This is what makes
 *     GOSPA the conventional choice for multi-target tracking
 *     evaluations — it surfaces cardinality errors directly instead of
 *     folding them into a saturated cutoff.
 *
 * Defaults p=2, α=2 match the convention used throughout the GOSPA /
 * PMBM literature.
 *
 * Assignment: optimal (min-cost) via the Hungarian algorithm on an
 * augmented (|X|+|Y|)² cost matrix with per-target miss/false dummy slots,
 * so a pair is matched only when d^p < 2·c^p/α. (The name retains the
 * historical "Greedy" suffix for call-site stability; the implementation is
 * no longer greedy — greedy NN could flip pairings in crossing geometry and
 * confound A/B comparisons.) Same convention as ospaGreedy.
 */

/**
 * GOSPA decomposition (Rahmathullah 2017 §IV).
 *
 * Math:
 *   The augmented-cost Hungarian solve partitions the assignment into
 *   three disjoint buckets:
 *     (a) truth_i ↔ est_j match:      row i in [0,n), col j in [0,m)
 *         → contributes d(x_i,y_j)^p to localization.
 *     (b) truth_i routed to its miss slot: col = m+i (diagonal)
 *         → contributes c^p/α to missed.
 *     (c) est_j routed to its false slot: row = n+j (diagonal)
 *         → contributes c^p/α to false_.
 *   total = localization + missed + false_  (pre-root quantity).
 *   gospaGreedy = pow(total, 1/p)  (the Rahmathullah headline metric).
 *
 * All fields are in the *pre-root* power-p space so the identity
 *   total = localization + missed + false_
 * holds exactly. Callers who want the rooted sub-costs can pow(x, 1/p)
 * themselves, but the sum-identity is only clean in power-p space.
 *
 * Assumptions: same as gospaGreedy (p>0, alpha>0, cutoff>0,
 *              positions in same Cartesian frame, metres).
 * Rationale:   makes every A/B legible — "is GOSPA moving because of
 *              misses or false tracks?" is now directly visible.
 * Improve next: per-target breakdown (which truth was missed, which
 *               est was false) for deep per-scenario diagnosis.
 */
struct GospaComponents {
  double total{0.0};        // localization + missed + false_ (pre-root)
  double localization{0.0}; // Σ d(x_i,y_j)^p over matched pairs
  double missed{0.0};       // n_missed * c^p/α
  double false_{0.0};       // n_false  * c^p/α
  int n_missed{0};          // number of unmatched truth points
  int n_false{0};           // number of unmatched estimate points
};

/**
 * Compute the decomposed GOSPA cost between `truth` and `est` point sets,
 * returning the localization / missed / false_ breakdown in pre-root
 * power-p space (see `GospaComponents`). `cutoff` = c, `p` and `alpha`
 * follow the GOSPA convention.
 */
GospaComponents gospaComponents(const std::vector<Eigen::Vector2d>& truth,
                                const std::vector<Eigen::Vector2d>& est,
                                double cutoff,
                                double p = 2.0,
                                double alpha = 2.0);

/**
 * Headline (rooted) GOSPA distance between `truth` and `est` point sets —
 * `pow(gospaComponents(...).total, 1/p)`. `cutoff` = c, `p` and `alpha`
 * follow the GOSPA convention.
 */
double gospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                   const std::vector<Eigen::Vector2d>& est,
                   double cutoff,
                   double p = 2.0,
                   double alpha = 2.0);

}  // namespace navtracker
