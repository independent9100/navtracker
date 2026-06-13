#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Consistency.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IInnovationSink.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {

InnovationEvent makeEvent(SensorKind sensor, MeasurementModel model,
                         const std::string& source_id,
                         const Eigen::VectorXd& nu,
                         const Eigen::MatrixXd& S,
                         const Eigen::MatrixXd& R) {
  InnovationEvent e;
  e.time = Timestamp::fromSeconds(0.0);
  e.track_id = TrackId{1};
  e.sensor = sensor;
  e.source_id = source_id;
  e.model = model;
  e.residual = nu;
  e.S = S;
  e.R = R;
  e.dim = static_cast<std::size_t>(nu.size());
  return e;
}

}  // namespace

// ε̄ⁿⁱˢ for N=5000 IID draws from N(0, S) lands inside the Wilson-Hilferty
// 95% band of m. Repeated for m=1 (Bearing2D) and m=2 (Position2D).
TEST(Consistency, NisFromIidGaussianResiduals_m2) {
  NisCollector nis;
  std::mt19937 rng(0xC0FFEE);
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Matrix2d S;
  S << 4.0, 0.5, 0.5, 9.0;
  Eigen::LLT<Eigen::Matrix2d> llt(S);
  const Eigen::Matrix2d L = llt.matrixL();
  for (int i = 0; i < 5000; ++i) {
    const Eigen::Vector2d z(n(rng), n(rng));
    const Eigen::Vector2d nu = L * z;  // ν ∼ N(0, S)
    nis.onInnovation(makeEvent(SensorKind::Ais, MeasurementModel::Position2D,
                               "tester", nu, S, S));
  }
  const auto stats = nis.finalize();
  ASSERT_EQ(stats.size(), 1u);
  const auto& s = stats.begin()->second;
  EXPECT_EQ(s.n, 5000u);
  EXPECT_FALSE(s.low_sample);
  EXPECT_GT(s.mean, s.band_lo);
  EXPECT_LT(s.mean, s.band_hi);
  // Coverage at 95% should sit close to 0.95 (within ±0.02 for N=5000).
  EXPECT_NEAR(s.coverage_95, 0.95, 0.02);
}

TEST(Consistency, NisFromIidGaussianResiduals_m1) {
  NisCollector nis;
  std::mt19937 rng(0xABCDE);
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Matrix<double, 1, 1> S;
  S(0, 0) = 0.01;  // rad²
  const double L = std::sqrt(S(0, 0));
  for (int i = 0; i < 5000; ++i) {
    Eigen::Matrix<double, 1, 1> nu;
    nu(0) = L * n(rng);
    nis.onInnovation(makeEvent(SensorKind::EoIr, MeasurementModel::Bearing2D,
                               "cam0", nu, S, S));
  }
  const auto stats = nis.finalize();
  const auto& s = stats.begin()->second;
  EXPECT_GT(s.mean, s.band_lo);
  EXPECT_LT(s.mean, s.band_hi);
}

// α̂ = ε̄ⁿⁱˢ / m matches the true scaling when R_true = α·R_claimed and
// R dominates HPHᵀ. Verified at α ∈ {0.5, 1, 4}.
TEST(Consistency, NisFittedAlpha) {
  for (double alpha : {0.5, 1.0, 4.0}) {
    NisCollector nis;
    std::mt19937 rng(static_cast<unsigned>(alpha * 1000));
    std::normal_distribution<double> n(0.0, 1.0);
    Eigen::Matrix2d R_claimed = 4.0 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d HPH = 1e-6 * Eigen::Matrix2d::Identity();  // R ≫ HPHᵀ
    Eigen::Matrix2d S = HPH + R_claimed;
    Eigen::Matrix2d R_true = alpha * R_claimed;
    Eigen::LLT<Eigen::Matrix2d> llt(HPH + R_true);
    const Eigen::Matrix2d L = llt.matrixL();
    for (int i = 0; i < 5000; ++i) {
      const Eigen::Vector2d z(n(rng), n(rng));
      const Eigen::Vector2d nu = L * z;
      nis.onInnovation(makeEvent(SensorKind::Ais, MeasurementModel::Position2D,
                                 "tester", nu, S, R_claimed));
    }
    const auto stats = nis.finalize();
    const auto& s = stats.begin()->second;
    EXPECT_NEAR(s.alpha_hat, alpha, 0.05 * alpha)
        << "alpha=" << alpha << " mean=" << s.mean;
  }
}

// Below the 30-sample minimum, ε̄ is still emitted but the band is NaN
// and low_sample is true.
TEST(Consistency, BandSuppressedBelowMin) {
  NisCollector nis;
  Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
  for (int i = 0; i < 10; ++i) {
    nis.onInnovation(makeEvent(SensorKind::Ais, MeasurementModel::Position2D,
                               "x", Eigen::Vector2d(0.1, 0.1), S, S));
  }
  const auto stats = nis.finalize();
  const auto& s = stats.begin()->second;
  EXPECT_EQ(s.n, 10u);
  EXPECT_TRUE(s.low_sample);
  EXPECT_TRUE(std::isnan(s.band_lo));
  EXPECT_TRUE(std::isnan(s.band_hi));
}

// Zero R (degenerate adapter) yields a singular S. LDLT bails; the
// sample is dropped and counted but the running mean is unaffected.
TEST(Consistency, SingularSDropped) {
  NisCollector nis;
  Eigen::Matrix2d good = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d zero = Eigen::Matrix2d::Zero();
  for (int i = 0; i < 50; ++i) {
    nis.onInnovation(makeEvent(SensorKind::Ais, MeasurementModel::Position2D,
                               "x", Eigen::Vector2d(0.0, 0.0), good, good));
  }
  for (int i = 0; i < 5; ++i) {
    nis.onInnovation(makeEvent(SensorKind::Ais, MeasurementModel::Position2D,
                               "x", Eigen::Vector2d(0.5, 0.5), zero, zero));
  }
  const auto stats = nis.finalize();
  const auto& s = stats.begin()->second;
  EXPECT_EQ(s.n, 50u);                 // only good samples counted
  EXPECT_EQ(s.dropped_singular, 5u);   // singular dropped, counted
  EXPECT_EQ(nis.totalDroppedSingular(), 5u);
}

// NEES from a hand-crafted BenchResult: single truth and single track,
// P_xy = I, errors e ∼ N(0, I) → ε̄ⁿᵉᵉˢ ≈ 2.
TEST(Consistency, NeesFromKnownFilter) {
  BenchResult result;
  std::mt19937 rng(42);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int k = 0; k < 2000; ++k) {
    BenchStep step;
    step.time = Timestamp::fromSeconds(static_cast<double>(k));
    TruthStateSnapshot t;
    t.truth_id = 1;
    t.position = Eigen::Vector2d::Zero();
    t.velocity = Eigen::Vector2d::Zero();
    step.truth.push_back(t);
    TrackStateSnapshot tr;
    tr.id = TrackId{42};
    tr.position = Eigen::Vector2d(n(rng), n(rng));  // e ∼ N(0, I)
    tr.velocity = Eigen::Vector2d::Zero();
    tr.pos_covariance = Eigen::Matrix2d::Identity();
    step.tracks.push_back(tr);
    result.steps.push_back(std::move(step));
  }
  const auto nees = computeNees(result, 100.0);
  EXPECT_EQ(nees.n, 2000u);
  EXPECT_FALSE(nees.low_sample);
  EXPECT_NEAR(nees.mean, 2.0, 0.15);     // E[χ²_2] = 2
  EXPECT_NEAR(nees.beta_hat, 1.0, 0.1);
  EXPECT_GT(nees.mean, nees.band_lo);
  EXPECT_LT(nees.mean, nees.band_hi);
}
