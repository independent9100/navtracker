#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

/**
 * Solve a rectangular linear sum assignment problem (Hungarian / Jonker-
 * Volgenant style). Given an N×M cost matrix C, return an assignment
 * `row_to_col` of size N, where `row_to_col[i]` is the column assigned
 * to row i (or -1 if row i is unassigned because N > M).
 *
 * Math: minimize Σ_i C(i, row_to_col[i]) subject to each column used at
 * most once. Cells with cost == +∞ are treated as forbidden.
 *
 * This is the Kuhn-Munkres algorithm implemented for rectangular cost
 * matrices (the caller need not pad: if N > M, the surplus rows return -1).
 * Internally the matrix is padded to a square K×K with K = max(N,M) and the
 * Jonker-Volgenant augmenting-path loop runs in O(max(N,M)³).
 *
 * Rationale: TOMHT global hypothesis selection picks one leaf per track
 * tree subject to "each measurement used at most once across trees".
 * The cost matrix is built with rows = trees, columns = (measurements ∪
 * tree-specific miss slots); this solver returns the leaf assignment.
 *
 * Ways to improve: Murty's K-best assignment extends this to defer
 * commitment to a single global hypothesis (the "K" deferred-decision
 * step in textbook TOMHT). For now the K=1 single-best hypothesis is
 * the load-bearing correctness fix.
 */
std::vector<int> hungarianAssignment(const Eigen::MatrixXd& cost);

}  // namespace navtracker
