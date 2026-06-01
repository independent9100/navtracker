#include <gtest/gtest.h>
#include "core/estimation/Resampling.hpp"

using navtracker::effectiveSampleSize;
using navtracker::systematicResample;

TEST(Resampling, EssIsNForUniformWeights) {
  Eigen::VectorXd w = Eigen::VectorXd::Constant(8, 1.0 / 8.0);
  EXPECT_NEAR(effectiveSampleSize(w), 8.0, 1e-12);
}

TEST(Resampling, EssIsOneForConcentratedWeights) {
  Eigen::VectorXd w(4);
  w << 1.0, 0.0, 0.0, 0.0;
  EXPECT_NEAR(effectiveSampleSize(w), 1.0, 1e-12);
}

TEST(Resampling, UniformWeightsGiveIdentityMapping) {
  const int N = 4;
  Eigen::VectorXd w = Eigen::VectorXd::Constant(N, 1.0 / N);
  const std::vector<int> idx = systematicResample(w, 0.5 / N);
  ASSERT_EQ(idx.size(), 4u);
  for (int i = 0; i < N; ++i) EXPECT_EQ(idx[i], i);
}

TEST(Resampling, ConcentratedWeightReplicatesTopParticle) {
  Eigen::VectorXd w(4);
  w << 1.0, 0.0, 0.0, 0.0;
  const std::vector<int> idx = systematicResample(w, 0.1);
  for (int i : idx) EXPECT_EQ(i, 0);
}

TEST(Resampling, DeterministicForSameOffset) {
  Eigen::VectorXd w(5);
  w << 0.05, 0.05, 0.30, 0.50, 0.10;
  const auto a = systematicResample(w, 0.07);
  const auto b = systematicResample(w, 0.07);
  EXPECT_EQ(a, b);
}

TEST(Resampling, EmptyWeightsGiveEmptyIndices) {
  const Eigen::VectorXd w;
  const std::vector<int> idx = systematicResample(w, 0.5);
  EXPECT_TRUE(idx.empty());
}
