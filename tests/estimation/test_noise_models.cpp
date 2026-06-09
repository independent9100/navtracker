#include <memory>

#include <gtest/gtest.h>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/NoiseModels.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GaussianNoiseModel;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::StudentTNoiseModel;
using navtracker::Timestamp;
using navtracker::Track;

TEST(NoiseModels, GaussianAlwaysScaleOne) {
  GaussianNoiseModel g;
  Eigen::VectorXd y(2);
  y << 100.0, 100.0;  // huge outlier
  Eigen::MatrixXd S = Eigen::Matrix2d::Identity();
  EXPECT_DOUBLE_EQ(g.covarianceScale(y, S), 1.0);
}

TEST(NoiseModels, StudentTInlierScaleNearOne) {
  StudentTNoiseModel t(4.0);
  // Inlier: delta^2 ~ d = 2 → scale = (nu + d)/(nu + d) = 1.
  Eigen::VectorXd y(2);
  y << 1.0, 1.0;  // delta^2 = 2 against identity S
  Eigen::MatrixXd S = Eigen::Matrix2d::Identity();
  // (4 + 2)/(4 + 2) = 1.0
  EXPECT_NEAR(t.covarianceScale(y, S), 1.0, 1e-12);
}

TEST(NoiseModels, StudentTOutlierInflates) {
  StudentTNoiseModel t(4.0);
  Eigen::VectorXd y(2);
  y << 10.0, 10.0;  // delta^2 = 200 against identity S
  Eigen::MatrixXd S = Eigen::Matrix2d::Identity();
  // (4 + 200)/(4 + 2) = 34
  EXPECT_NEAR(t.covarianceScale(y, S), 34.0, 1e-9);
  EXPECT_GT(t.covarianceScale(y, S), 1.0);
}

TEST(NoiseModels, StudentTNeverBelowOne) {
  StudentTNoiseModel t(4.0);
  Eigen::VectorXd y(2);
  y << 0.0, 0.0;  // delta^2 = 0 → raw (4)/(6) < 1, clamped to 1
  Eigen::MatrixXd S = Eigen::Matrix2d::Identity();
  EXPECT_DOUBLE_EQ(t.covarianceScale(y, S), 1.0);
}

// End-to-end: an outlier position update pulls a Gaussian EKF much
// farther than a Student-t EKF. The robust filter resists the outlier.
TEST(NoiseModels, StudentTEkfResistsOutlierVsGaussian) {
  auto motion = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator gauss(motion, 5.0);  // null noise model = Gaussian
  const EkfEstimator robust(motion, 5.0,
                            std::make_shared<StudentTNoiseModel>(4.0));

  Track t0;
  t0.last_update = Timestamp::fromSeconds(0.0);
  t0.state = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  t0.covariance = Eigen::Matrix4d::Identity() * 10.0;

  // Measurement far from the prediction (a clutter/outlier hit at 50 m
  // when the track sits at the origin with σ≈3 m).
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(50.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity() * 1.0;

  Track tg = t0, tr = t0;
  gauss.update(tg, z);
  robust.update(tr, z);

  // Both move toward the outlier, but the robust filter moves much less.
  EXPECT_GT(tg.state(0), tr.state(0));
  EXPECT_LT(tr.state(0), 0.5 * tg.state(0))
      << "robust x=" << tr.state(0) << " gauss x=" << tg.state(0);
}
