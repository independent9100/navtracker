#include "core/scenario/TGospa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "core/association/Hungarian.hpp"

namespace navtracker {

namespace {

// Per-scan GOSPA cost (UNPOWED — caller takes the 1/p root after
// summing over time + switch penalty). Returns total cost AND the
// matched (truth_id → est_id) pairs at this scan.
double gospaCostAndAssignment(
    const std::vector<std::pair<std::uint64_t, Eigen::Vector2d>>& truth,
    const std::vector<std::pair<std::uint64_t, Eigen::Vector2d>>& est,
    double cutoff, double p, double alpha,
    std::map<std::uint64_t, std::uint64_t>& assignment_out) {
  assignment_out.clear();
  if (truth.empty() && est.empty()) return 0.0;
  const int n = static_cast<int>(truth.size());
  const int m = static_cast<int>(est.size());
  const double miss = std::pow(cutoff, p) / alpha;
  const double cap = std::pow(cutoff, p);
  const int K = n + m;
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(K, K, kInf);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      const double d = (truth[i].second - est[j].second).norm();
      cost(i, j) = std::min(std::pow(d, p), cap);
    }
  }
  for (int i = 0; i < n; ++i) cost(i, m + i) = miss;
  for (int j = 0; j < m; ++j) cost(n + j, j) = miss;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      cost(n + i, m + j) = 0.0;
  const std::vector<int> row_to_col = hungarianAssignment(cost);
  double total = 0.0;
  for (int i = 0; i < K; ++i) {
    const int j = row_to_col[i];
    if (j >= 0) total += cost(i, j);
    if (i < n && j >= 0 && j < m) {
      assignment_out[truth[i].first] = est[j].first;
    }
  }
  return total;
}

}  // namespace

double tgospa(const std::vector<Trajectory>& truth,
              const std::vector<Trajectory>& est,
              double cutoff,
              double p,
              double alpha,
              double switch_penalty) {
  if (switch_penalty < 0.0) switch_penalty = cutoff;
  // Collect the union of scan indices present in either set.
  std::set<int> scans;
  for (const auto& t : truth)
    for (const auto& [k, _] : t.samples) scans.insert(k);
  for (const auto& t : est)
    for (const auto& [k, _] : t.samples) scans.insert(k);

  double total = 0.0;
  std::map<std::uint64_t, std::uint64_t> prev_assignment;
  for (int k : scans) {
    // Snapshot present-at-this-scan truth / est entries.
    std::vector<std::pair<std::uint64_t, Eigen::Vector2d>> truth_k, est_k;
    for (const auto& t : truth) {
      auto it = t.samples.find(k);
      if (it != t.samples.end()) truth_k.push_back({t.id, it->second});
    }
    for (const auto& t : est) {
      auto it = t.samples.find(k);
      if (it != t.samples.end()) est_k.push_back({t.id, it->second});
    }
    std::map<std::uint64_t, std::uint64_t> assignment;
    total += gospaCostAndAssignment(truth_k, est_k, cutoff, p, alpha,
                                    assignment);
    // Switching penalty: for every truth id whose matched est id
    // changed from the previous scan, charge γ^p once.
    if (!prev_assignment.empty()) {
      const double sp = std::pow(switch_penalty, p);
      for (const auto& [tid, eid] : assignment) {
        auto it = prev_assignment.find(tid);
        if (it != prev_assignment.end() && it->second != eid) {
          total += sp;
        }
      }
    }
    prev_assignment = std::move(assignment);
  }
  return std::pow(total, 1.0 / p);
}

}  // namespace navtracker
