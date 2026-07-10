#pragma once

// Covariance intersection (CI) — the default track-to-track fusion rule.
//
// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------
// Two estimates (x1, P1) and (x2, P2) of the SAME state are fused by
//
//     P_f^{-1} = w P1^{-1} + (1 - w) P2^{-1}
//     x_f      = P_f ( w P1^{-1} x1 + (1 - w) P2^{-1} x2 ),   w in [0, 1].
//
// The weight w is chosen to minimize trace(P_f). trace(P_f(w)) is convex on
// [0, 1] (a standard CI result), so a 1-D line search finds the optimum; we
// use a fixed-iteration golden-section search (see ciOptimalOmega) for
// determinism (no convergence-dependent step count -> identical output on
// replay). N sources are fused sequentially in the caller's (canonical) order.
//
// ---------------------------------------------------------------------------
// Assumptions
// ---------------------------------------------------------------------------
//  * P1, P2 are symmetric positive-definite (finite certainty in every
//    direction). Edge validation upstream (ExternalTrack) rejects non-PSD
//    input covariances; CI itself assumes invertibility and does not re-check.
//  * Both estimates describe the same state in the same frame at the same time
//    (time alignment is the fuser's job, see docs/algorithms/t2t-fusion.md §1).
//  * Each input is CONSISTENT: E[(x_hat - x)(x_hat - x)^T] <= P. Under that
//    assumption CI is consistent for ANY unknown cross-correlation between the
//    two inputs — this is the entire justification for CI-by-default when the
//    pedigree cannot prove independence.
//
// ---------------------------------------------------------------------------
// Rationale
// ---------------------------------------------------------------------------
// Naive (independence-assuming) fusion, P_f^{-1} = P1^{-1} + P2^{-1}, is
// overconfident when the two inputs secretly share a sensor: it counts one
// piece of evidence twice. CI never becomes more certain than the more careful
// of the two inputs justifies, whatever the correlation. We pay for that safety
// with a looser covariance when the inputs really were independent — an
// acceptable, and reversible, price (see the IFusionRule port for the future
// independence-exploiting rule). trace (not determinant) is minimized because
// the position trace is the operationally meaningful spread; det is the
// classical alternative, noted in the algorithm doc's "ways to improve".
//
// ---------------------------------------------------------------------------
// Ways to improve / what to test next
// ---------------------------------------------------------------------------
//  * Batch CI: a single joint weight vector on the simplex over all N inputs
//    instead of sequential pairwise folding (removes the small order
//    dependence sequential folding introduces).
//  * det(P_f) or an information-theoretic objective instead of trace.
//  * Bar-Shalom / Campo fusion when a pair is provably independent (behind
//    IFusionRule) — tighter, but only valid with proven independence.

#include <Eigen/Core>
#include <Eigen/LU>
#include <utility>
#include <vector>

namespace navtracker::t2t {

// Default golden-section iteration count for the weight search. 40 iterations
// shrink the [0,1] bracket by phi^-40 (~1e-8), far below any operational
// tolerance, and are fixed (not convergence-gated) for deterministic replay.
inline constexpr int kDefaultOmegaIterations = 40;

// Result of a CI fuse: the fused estimate and the weight that produced it.
struct CiResult {
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  double omega = 0.5;  // weight applied to the FIRST input.
};

// Fuse two estimates at a GIVEN weight w in [0,1] (w on the first input).
// Pure; does not choose w. Used both by the optimizer's objective and by
// callers that want a fixed weight (e.g. tests).
inline CiResult covarianceIntersectAt(const Eigen::VectorXd& x1,
                                      const Eigen::MatrixXd& P1,
                                      const Eigen::VectorXd& x2,
                                      const Eigen::MatrixXd& P2, double w) {
  const Eigen::MatrixXd I1 = P1.inverse();
  const Eigen::MatrixXd I2 = P2.inverse();
  const Eigen::MatrixXd If = w * I1 + (1.0 - w) * I2;
  CiResult r;
  r.P = If.inverse();
  r.x = r.P * (w * (I1 * x1) + (1.0 - w) * (I2 * x2));
  r.omega = w;
  return r;
}

// Choose the weight w in [0,1] minimizing trace(P_f(w)) by fixed-iteration
// golden-section search. Deterministic: exactly `iterations` bracket shrinks,
// no early exit. The tie branch (exact-equal trace, e.g. P1 == P2) shrinks the
// bracket SYMMETRICALLY so the degenerate/flat objective returns w = 0.5.
inline double ciOptimalOmega(const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2,
                             int iterations = kDefaultOmegaIterations) {
  // Symmetry fast path: when P1 == P2 the trace objective is analytically flat
  // (trace(P_f) == trace(P1) for every w), so the minimizer is 0.5 by symmetry
  // convention. Return it exactly rather than let floating-point noise in the
  // line search drift the degenerate optimum. (isApprox uses a ~1e-12 relative
  // tolerance, so only genuinely-equal covariances take this path.)
  if (P1.isApprox(P2)) return 0.5;
  const Eigen::MatrixXd I1 = P1.inverse();
  const Eigen::MatrixXd I2 = P2.inverse();
  const auto traceAt = [&](double w) {
    const Eigen::MatrixXd If = w * I1 + (1.0 - w) * I2;
    return If.inverse().trace();
  };
  // phi = golden ratio; the interior probe offset is (b - a) / phi.
  constexpr double kInvPhi = 0.6180339887498949;  // 1 / phi
  double a = 0.0, b = 1.0;
  for (int i = 0; i < iterations; ++i) {
    const double span = (b - a) * kInvPhi;
    const double c = b - span;
    const double d = a + span;
    const double fc = traceAt(c);
    const double fd = traceAt(d);
    if (fc < fd) {
      b = d;  // minimum is left of d
    } else if (fc > fd) {
      a = c;  // minimum is right of c
    } else {
      a = c;  // exact tie (flat/symmetric objective) -> shrink both ends
      b = d;  // symmetrically, converging to the centre (0.5).
    }
  }
  return 0.5 * (a + b);
}

// Fuse two estimates with the trace-optimal weight.
inline CiResult covarianceIntersect(const Eigen::VectorXd& x1,
                                     const Eigen::MatrixXd& P1,
                                     const Eigen::VectorXd& x2,
                                     const Eigen::MatrixXd& P2,
                                     int iterations = kDefaultOmegaIterations) {
  const double w = ciOptimalOmega(P1, P2, iterations);
  return covarianceIntersectAt(x1, P1, x2, P2, w);
}

// Fuse N estimates by sequential left-fold in the given order:
// fuse(fuse(fuse(e0,e1),e2),...). The order is the CALLER's responsibility
// (the fuser sorts by source_tracker_id for determinism); the small
// order-dependence is documented in the algorithm doc. Requires >= 1 input;
// a single input passes through unchanged.
inline CiResult covarianceIntersectSequential(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& estimates,
    int iterations = kDefaultOmegaIterations) {
  CiResult acc;
  acc.x = estimates.front().first;
  acc.P = estimates.front().second;
  acc.omega = 1.0;
  for (std::size_t i = 1; i < estimates.size(); ++i) {
    acc = covarianceIntersect(acc.x, acc.P, estimates[i].first,
                              estimates[i].second, iterations);
  }
  return acc;
}

// The overconfident control: naive fusion assuming independence,
// P_f^{-1} = P1^{-1} + P2^{-1}. NOT a shipped fusion rule — provided so tests
// and the bench baseline can demonstrate WHY CI (trace(P_CI) >= trace(P_naive)
// always; the naive covariance is the one that becomes dangerously small when
// the inputs share a source). Do not use in production.
inline CiResult naiveIndependentFuse(const Eigen::VectorXd& x1,
                                     const Eigen::MatrixXd& P1,
                                     const Eigen::VectorXd& x2,
                                     const Eigen::MatrixXd& P2) {
  const Eigen::MatrixXd I1 = P1.inverse();
  const Eigen::MatrixXd I2 = P2.inverse();
  CiResult r;
  r.P = (I1 + I2).inverse();
  r.x = r.P * (I1 * x1 + I2 * x2);
  r.omega = 0.5;  // not meaningful for naive fusion; reported for symmetry.
  return r;
}

}  // namespace navtracker::t2t
