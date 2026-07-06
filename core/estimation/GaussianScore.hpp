#pragma once

#include <cmath>

#include <Eigen/Dense>

namespace navtracker {

// Mechanical-sympathy helpers for the Gaussian innovation score on the
// estimator hot path (PMBM/IMM likelihood + estimator update, evaluated per
// track × measurement × mode).
//
// Math: for an innovation y with covariance S the two quantities the hot path
// needs are the squared Mahalanobis distance yᵀS⁻¹y and log|S|. The historical
// code computed these with `S.determinant()` AND `S.inverse()` — two separate
// LU factorizations of the *same* matrix in the innermost loop. These helpers
// do it with ONE decomposition.
//
// Change class: B (epsilon). `lu.solve(y)` replaces forming S⁻¹ then
// multiplying, and `lu.determinant()` replaces `S.determinant()` (for small
// fixed dims Eigen's free-function determinant is a closed form, the LU one is
// a product of pivots) — so results move at floating-point rounding scale
// (~1e-15). The computed function is unchanged. Determinism is preserved (still
// a deterministic sequence of ops).
//
// The `safe_det` guard reproduces the prior EstimatorDefaults / ImmEstimator
// semantics EXACTLY: if det(S) is not (> 0 && finite), the log term uses a
// 1e-300 floor.

inline double safeLogDet(double det) {
  const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
  return std::log(safe_det);
}

// yᵀ S⁻¹ y and log|S| (guarded) from ONE decomposition. `S⁻¹` is never formed
// — the Mahalanobis term uses a triangular solve. Use for logLikelihood-style
// scoring, which needs both quantities but not the inverse matrix itself.
struct GaussianScore {
  double mahalanobis;   // yᵀ S⁻¹ y
  double log_det_safe;  // log(max(det(S), 0⁺))  — see safeLogDet
};

inline GaussianScore gaussianScore(const Eigen::VectorXd& y,
                                   const Eigen::MatrixXd& S) {
  const Eigen::PartialPivLU<Eigen::MatrixXd> lu(S);
  return {y.dot(lu.solve(y)), safeLogDet(lu.determinant())};
}

}  // namespace navtracker
