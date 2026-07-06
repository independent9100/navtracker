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

// S⁻¹ (formed explicitly) and det(S) (raw — caller applies its own guard) from
// ONE decomposition. Use at sites that genuinely need the inverse matrix AND
// the determinant, e.g. softUpdate() where a single S⁻¹ is reused across all
// gated measurements (forming the inverse once beats a solve per measurement)
// and the raw det feeds both the Gaussian normaliser and the gate volume.
struct InverseAndDet {
  Eigen::MatrixXd inverse;
  double det;
};

inline InverseAndDet luInverseDet(const Eigen::MatrixXd& S) {
  const Eigen::PartialPivLU<Eigen::MatrixXd> lu(S);
  return {lu.inverse(), lu.determinant()};
}

// ---------------------------------------------------------------------------
// Fixed-size 2×2 stack kernels (perf round 3, T3). The dominant HAXR
// measurement model is Position2D: the innovation covariance S is 2×2 and the
// measurement Jacobian H = [I₂ | 0] is a pure selector. Building S via a
// dynamic H·P·Hᵀ matmul and decomposing it with a dynamic-size LU heap-
// allocates several MatrixXd temporaries PER (track × measurement × mode) call
// — the cache-hostile allocation churn the round-2 profile flagged. These
// kernels take the already-selected 2×2 S on the STACK (Eigen::Matrix2d) and
// use the closed-form 2×2 inverse/determinant. No heap, no LU.
//
// Change class: B (epsilon). The closed-form 2×2 (yᵀS⁻¹y = (S₁₁y₀² −
// (S₀₁+S₁₀)y₀y₁ + S₀₀y₁²)/det, det = S₀₀S₁₁ − S₀₁S₁₀) is the same function as
// the dynamic-LU path up to floating-point order. The mahalanobis term uses
// the RAW det (matching yᵀS⁻¹y through the historical S.inverse()); the log
// term uses the safe_det guard — identical split to the dynamic path.

inline double mahalanobis2x2(const Eigen::Vector2d& y, const Eigen::Matrix2d& S) {
  const double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
  const double num = S(1, 1) * y(0) * y(0) -
                     (S(0, 1) + S(1, 0)) * y(0) * y(1) +
                     S(0, 0) * y(1) * y(1);
  return num / det;  // ±inf / NaN on a singular S, exactly as S.inverse() gives
}

inline GaussianScore gaussianScore2x2(const Eigen::Vector2d& y,
                                      const Eigen::Matrix2d& S) {
  const double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
  const double num = S(1, 1) * y(0) * y(0) -
                     (S(0, 1) + S(1, 0)) * y(0) * y(1) +
                     S(0, 0) * y(1) * y(1);
  return {num / det, safeLogDet(det)};
}

}  // namespace navtracker
