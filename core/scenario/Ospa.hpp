#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff);

}  // namespace navtracker
