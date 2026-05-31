#include <gtest/gtest.h>
#include "core/estimation/SigmaPoints.hpp"

using navtracker::computeSigmaPoints;
using navtracker::SigmaPoints;

TEST(SigmaPoints, WeightsSumToOne) {
  const Eigen::VectorXd mean = Eigen::Vector4d(1.0, 2.0, 3.0, 4.0);
  const Eigen::MatrixXd cov = Eigen::Matrix4d::Identity();
  const SigmaPoints sp = computeSigmaPoints(mean, cov, 1e-3, 2.0, 0.0);
  EXPECT_EQ(sp.points.cols(), 9);
  EXPECT_NEAR(sp.Wm.sum(), 1.0, 1e-12);
}

TEST(SigmaPoints, WeightedSumReconstructsMean) {
  const Eigen::VectorXd mean = Eigen::Vector4d(5.0, -2.0, 1.0, 0.5);
  Eigen::Matrix4d cov = Eigen::Matrix4d::Identity();
  cov(0, 0) = 4.0;
  cov(2, 2) = 9.0;
  const SigmaPoints sp = computeSigmaPoints(mean, cov, 1e-3, 2.0, 0.0);
  Eigen::VectorXd recon = Eigen::VectorXd::Zero(4);
  for (int i = 0; i < sp.points.cols(); ++i) recon += sp.Wm(i) * sp.points.col(i);
  EXPECT_TRUE(recon.isApprox(mean, 1e-9));
}

TEST(SigmaPoints, WeightedOuterSumReconstructsCovariance) {
  const Eigen::VectorXd mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d cov;
  cov << 4.0, 1.0, 0.0, 0.0,
         1.0, 9.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.5,
         0.0, 0.0, 0.5, 2.0;
  const SigmaPoints sp = computeSigmaPoints(mean, cov, 1e-3, 2.0, 0.0);
  Eigen::Matrix4d recon = Eigen::Matrix4d::Zero();
  for (int i = 0; i < sp.points.cols(); ++i) {
    const Eigen::VectorXd diff = sp.points.col(i) - mean;
    recon += sp.Wc(i) * diff * diff.transpose();
  }
  EXPECT_TRUE(recon.isApprox(cov, 1e-9));
}
