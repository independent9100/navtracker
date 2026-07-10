#include "core/t2t/T2tAssociator.hpp"

#include <cmath>
#include <limits>

#include "core/association/Hungarian.hpp"

namespace navtracker::t2t {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// Floor on det(S) below which S = P1+P2 is treated as degenerate. S is a sum of
// PSD covariances and is well-conditioned for any realistic (even huge) input;
// this only bites a pathological near-zero-covariance case, where we prefer a
// clean gate-out over inf/NaN. Same spirit as GaussianScore::safeLogDet.
constexpr double kGateDetFloor = 1e-12;

// Guarded closed-form 2×2 squared-Mahalanobis yᵀ S⁻¹ y.
double guardedMahalanobis2x2(const Eigen::Vector2d& y, const Eigen::Matrix2d& S) {
  const double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
  if (!std::isfinite(det) || det <= kGateDetFloor) return kInf;
  const double num = S(1, 1) * y(0) * y(0) - (S(0, 1) + S(1, 0)) * y(0) * y(1) +
                     S(0, 0) * y(1) * y(1);
  const double d2 = num / det;
  return std::isfinite(d2) ? d2 : kInf;
}

}  // namespace

double T2tAssociator::gateDistanceSq(const GateCandidate& a,
                                     const GateCandidate& b) const {
  const Eigen::Vector2d y = a.position - b.position;
  const Eigen::Matrix2d S = a.covariance + b.covariance;
  return guardedMahalanobis2x2(y, S);
}

double T2tAssociator::assignmentCost(double d2,
                                     const std::optional<std::uint32_t>& a,
                                     const std::optional<std::uint32_t>& b) const {
  double cost = d2;
  if (a.has_value() && b.has_value()) {
    if (*a == *b)
      cost -= cfg_.shared_mmsi_cost_bonus;      // corroboration: prefer
    else
      cost += cfg_.conflicting_mmsi_cost_penalty;  // conflict: discourage (soft)
  }
  return cost;
}

T2tAssignment T2tAssociator::associate(const std::vector<GateCandidate>& fused,
                                       const std::vector<GateCandidate>& sources) const {
  T2tAssignment out;
  const std::size_t nf = fused.size();
  const std::size_t ns = sources.size();

  // Degenerate shapes: everything unmatched.
  if (nf == 0 || ns == 0) {
    for (std::size_t i = 0; i < nf; ++i) out.unmatched_fused.push_back(i);
    for (std::size_t j = 0; j < ns; ++j) out.unmatched_sources.push_back(j);
    return out;
  }

  // Build the gated cost matrix. Out-of-gate cells are +inf (forbidden); in-gate
  // cells carry d² + soft MMSI adjustment. Keep the raw gate result so we can
  // re-check the Hungarian's picks (it may return a forced-forbidden pick).
  Eigen::MatrixXd cost(nf, ns);
  Eigen::MatrixXd gate_d2(nf, ns);
  for (std::size_t i = 0; i < nf; ++i) {
    for (std::size_t j = 0; j < ns; ++j) {
      const double d2 = gateDistanceSq(fused[i], sources[j]);
      gate_d2(i, j) = d2;
      cost(i, j) = (d2 <= cfg_.gate_chi2_position)
                       ? assignmentCost(d2, fused[i].mmsi, sources[j].mmsi)
                       : kInf;
    }
  }

  const std::vector<int> row_to_col = hungarianAssignment(cost);
  std::vector<bool> source_used(ns, false);
  for (std::size_t i = 0; i < nf; ++i) {
    const int j = row_to_col[i];
    // Trust the pick only if it is a real in-gate assignment (not a forced
    // forbidden fill returned when no feasible assignment existed).
    if (j >= 0 && gate_d2(i, static_cast<std::size_t>(j)) <= cfg_.gate_chi2_position) {
      out.matches.emplace_back(i, static_cast<std::size_t>(j));
      source_used[static_cast<std::size_t>(j)] = true;
    } else {
      out.unmatched_fused.push_back(i);
    }
  }
  for (std::size_t j = 0; j < ns; ++j)
    if (!source_used[j]) out.unmatched_sources.push_back(j);
  return out;
}

}  // namespace navtracker::t2t
