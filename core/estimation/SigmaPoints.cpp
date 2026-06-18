#include "core/estimation/SigmaPoints.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace navtracker {

SigmaPoints computeSigmaPoints(const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov,
                               double alpha,
                               double beta,
                               double kappa) {
  const int n = static_cast<int>(mean.size());
  const double scale = alpha * alpha * (n + kappa);  // = n + lambda
  const double lambda = scale - n;

  const Eigen::MatrixXd S = scale * cov;
  const Eigen::LLT<Eigen::MatrixXd> llt(S);
  Eigen::MatrixXd L;
  if (llt.info() == Eigen::Success) {
    L = llt.matrixL();
  } else {
    // S is not positive-definite (singular or marginally indefinite cov,
    // e.g. after a degenerate update). A raw Cholesky would emit NaN
    // columns that poison every sigma point and propagate through the
    // whole UKF/IMM step. Fall back to a symmetric PSD matrix square
    // root via eigendecomposition with negative eigenvalues clamped to
    // zero — any matrix square root is a valid sigma-point spread
    // (Julier/Uhlmann), and this one stays finite.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S);
    const Eigen::VectorXd d = es.eigenvalues().cwiseMax(0.0);
    L = es.eigenvectors() * d.cwiseSqrt().asDiagonal();
  }

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
