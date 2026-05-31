#include "core/estimation/SigmaPoints.hpp"

#include <Eigen/Cholesky>

namespace navtracker {

SigmaPoints computeSigmaPoints(const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov,
                               double alpha,
                               double beta,
                               double kappa) {
  const int n = static_cast<int>(mean.size());
  const double scale = alpha * alpha * (n + kappa);  // = n + lambda
  const double lambda = scale - n;

  const Eigen::LLT<Eigen::MatrixXd> llt(scale * cov);
  const Eigen::MatrixXd L = llt.matrixL();

  SigmaPoints sp;
  sp.points.resize(n, 2 * n + 1);
  sp.points.col(0) = mean;
  for (int i = 0; i < n; ++i) {
    sp.points.col(i + 1) = mean + L.col(i);
    sp.points.col(i + 1 + n) = mean - L.col(i);
  }

  sp.Wm.resize(2 * n + 1);
  sp.Wc.resize(2 * n + 1);
  sp.Wm(0) = lambda / scale;
  sp.Wc(0) = lambda / scale + (1.0 - alpha * alpha + beta);
  const double w = 1.0 / (2.0 * scale);
  for (int i = 1; i < 2 * n + 1; ++i) {
    sp.Wm(i) = w;
    sp.Wc(i) = w;
  }
  return sp;
}

}  // namespace navtracker
