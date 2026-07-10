#pragma once

// T2tAssociator — one-shot track-to-track assignment: gate + Hungarian + soft
// identity terms. Stateless and pure (pairing hysteresis lives in the fuser,
// which owns the cross-cycle state). See docs/algorithms/t2t-fusion.md §1.2.
//
// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------
// For a fused-track prediction (x1,P1) and a candidate source track (x2,P2),
// both at the current time on the 2-D position block, the squared statistical
// distance is
//
//     d² = (x1 − x2)ᵀ (P1 + P2)⁻¹ (x1 − x2)
//
// Using P1+P2 ignores the cross-covariance, making the gate conservatively wide
// under positive correlation (documented, acceptable). The (P1+P2)⁻¹ is
// computed with a determinant-guarded closed-form 2×2 inverse (mirroring
// core/estimation/GaussianScore.hpp's single-decomposition, floored-det
// philosophy): a huge-covariance source (e.g. a bearing-quality position-only
// input) makes S large and well-conditioned, so d² is small and the pair GATES
// WIDE without poisoning anything; a pathological near-singular S (both covs
// near zero) is floored to +inf so the pair simply gates out rather than
// yielding inf/NaN.
//
// Pairs with d² > gate_chi2_position are gated out (+inf cost). Within the gate,
// the assignment cost is d² adjusted by SOFT identity terms — a bonus for a
// shared MMSI, a penalty for a conflicting one — both small relative to the
// gate so a strong kinematic match still wins (invariant 5).
//
// Assignment reuses the core Hungarian solver on the cost matrix; forbidden
// cells are +inf and every returned assignment is re-checked for a finite
// original cost before it is trusted.

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "core/t2t/T2tConfig.hpp"

namespace navtracker::t2t {

// The minimal view the associator needs of a track: its position-block estimate
// and MMSI hint. (Fused predictions and source tracks are both reduced to this.)
struct GateCandidate {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};    // ENU meters, at time t
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Zero()};  // 2×2 position cov, m², ENU
  std::optional<std::uint32_t> mmsi;
};

struct T2tAssignment {
  std::vector<std::pair<std::size_t, std::size_t>> matches;  // (fused_idx, source_idx)
  std::vector<std::size_t> unmatched_fused;
  std::vector<std::size_t> unmatched_sources;
};

class T2tAssociator {
 public:
  explicit T2tAssociator(T2tConfig cfg = {}) : cfg_(cfg) {}

  // Guarded squared-Mahalanobis on the position block, S = a.cov + b.cov.
  // Returns +inf when S is not safely invertible (degenerate) so the caller
  // gates the pair out. Small (gates wide) for a huge, well-conditioned S.
  double gateDistanceSq(const GateCandidate& a, const GateCandidate& b) const;

  // The assignment cost for an in-gate pair: d² with the soft MMSI adjustment
  // applied. (Exposed for testing that identity terms stay soft.)
  double assignmentCost(double d2, const std::optional<std::uint32_t>& a,
                        const std::optional<std::uint32_t>& b) const;

  // One-shot global assignment of fused predictions to source tracks.
  T2tAssignment associate(const std::vector<GateCandidate>& fused,
                          const std::vector<GateCandidate>& sources) const;

  const T2tConfig& config() const { return cfg_; }

 private:
  T2tConfig cfg_;
};

}  // namespace navtracker::t2t
