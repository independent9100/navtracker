#include "core/estimation/Resampling.hpp"

namespace navtracker {

double effectiveSampleSize(const Eigen::VectorXd& weights) {
  return 1.0 / weights.squaredNorm();
}

std::vector<int> systematicResample(const Eigen::VectorXd& weights, double u) {
  const int N = static_cast<int>(weights.size());
  std::vector<int> idx(N);
  double c = weights(0);
  int i = 0;
  for (int j = 0; j < N; ++j) {
    const double Uj = u + static_cast<double>(j) / static_cast<double>(N);
    while (Uj > c && i < N - 1) {
      ++i;
      c += weights(i);
    }
    idx[j] = i;
  }
  return idx;
}

}  // namespace navtracker
