#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include <Eigen/Core>

namespace navtracker {

// Trajectory-time-aligned GOSPA (T-GOSPA), García-Fernández, Rahmathullah,
// Svensson 2020 ("A metric on the space of finite sets of trajectories
// for evaluation of multi-target tracking algorithms",
// arXiv:1605.01177v3 §III.B "LP relaxation of T-GOSPA").
//
// **Math.** Given truth trajectories X = {X^i} and estimated
// trajectories Y = {Y^j} each defined on a time grid k ∈ {1..T},
// T-GOSPA is the cost of an assignment π_k per time step plus a
// switching penalty whenever the assignment changes between scans:
//
//   T-GOSPA(X, Y; c, p, γ) =
//     ( Σ_{k=1..T} GOSPA_cost_k(X_k, Y_k, π_k)
//     + (γ^p) · #{k : π_k ≠ π_{k-1}} )^(1/p)
//
// where GOSPA_cost_k is the unrolled (un-pow-1/p) GOSPA cost at time
// k under assignment π_k.  γ is the switching penalty (typical
// 2·cutoff/α). The optimal π_k accounts for both the per-scan
// cost AND the switching penalty — formally solved by an LP relaxation;
// in practice the greedy approximation below picks per-scan optimal
// GOSPA pairings and charges γ when the matched truth↔est pair flips.
//
// **Assumptions.** Trajectory time stamps already aligned to a common
// integer scan index (caller is responsible). Truth and estimated use
// the same TrajectoryId type (std::uint64_t — track id). A truth or
// estimated trajectory may be present in only a subset of scans
// (handled as cardinality miss/false at the absent steps).
//
// **Rationale.** T-GOSPA is the only metric in the standard PMBM /
// TPMBM literature that surfaces TRAJECTORY-LEVEL errors:
// fragmentation (one truth → many short estimated tracks),
// id switching, and per-scan position error all decompose cleanly.
// Per-scan GOSPA hides the first two by re-solving the assignment
// every scan with no memory.
//
// **Ways to improve.** The greedy per-scan optimal + sum-over-time
// implementation here approximates the true LP-relaxed assignment.
// A full LP / network-flow solver gives a tighter bound (smaller
// reported metric) but adds a solver dependency. Revisit when the
// metric is used as an optimisation target rather than a comparator.
struct Trajectory {
  std::uint64_t id;
  // Time-indexed positions. `key` is the scan index (caller-assigned);
  // `value` is the 2-D ENU position at that scan. Missing keys =
  // trajectory absent at that scan.
  std::map<int, Eigen::Vector2d> samples;
};

// Compute T-GOSPA over time-indexed trajectories. `cutoff` is the
// per-pair maximum distance (clipped at d=cutoff in the per-scan
// GOSPA), `p` and `alpha` follow Ospa/Gospa conventions, and
// `switch_penalty` is γ (defaults to cutoff, conservative).
double tgospa(const std::vector<Trajectory>& truth,
              const std::vector<Trajectory>& est,
              double cutoff,
              double p = 2.0,
              double alpha = 2.0,
              double switch_penalty = -1.0);

}  // namespace navtracker
