#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::PmbmTracker;

namespace {

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

PmbmTracker::Config cfg(bool pda) {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  c.use_pda_soft_detected_branch = pda;
  return c;
}

// y-coordinate of the highest-existence Bernoulli (the established track).
double dominantMeanY(const PmbmTracker& t) {
  double best_r = -1.0, y = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      if (b.existence_probability > best_r && b.mean.size() >= 2) {
        best_r = b.existence_probability;
        y = b.mean(1);
      }
  return y;
}

// Establish a straight-line track along y=0 (v=(10,0)), then a scan with TWO
// gated returns: a gate-CLOSER clutter at y=-4 (near the y=0 prediction) and the
// real return at y=+6. Returns the established track's y after that scan.
double runClutterPull(bool pda) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker t(ekf, cfg(pda));
  for (int k = 0; k <= 7; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(10.0 * k, 0.0, k)});
  }
  t.predict(Timestamp::fromSeconds(8));
  // clutter (80,-4) is closer to the (80,0) prediction than the real (80,6),
  // so K=1 GNN hard-assigns the track to the clutter.
  t.processBatch({posMeas(80.0, -4.0, 8), posMeas(80.0, 6.0, 8)});
  return dominantMeanY(t);
}

}  // namespace

// The established track must not be dragged onto a gate-closer clutter return.
// Hard GNN commits fully to the clutter (y pulled negative); the PDA soft update
// blends the unclaimed real return back in, keeping the state nearer y=0/real.
TEST(PmbmPdaSoftUpdate, ResistsGateCloserClutterPull) {
  const double y_hard = runClutterPull(false);
  const double y_pda = runClutterPull(true);
  EXPECT_LT(y_hard, -1.5);          // hard: dragged toward the clutter at y=-4
  EXPECT_GT(y_pda, y_hard + 1.0);   // PDA: pulled back toward the real return
}

// Reduce-to-hard guard: with a single gated return per scan the pool is size 1,
// so the PDA soft update is byte-identical to the hard update.
TEST(PmbmPdaSoftUpdate, SingleGatedMeasurementReducesToHard) {
  auto run = [](bool pda) {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator ekf{motion, 5.0};
    PmbmTracker t(ekf, cfg(pda));
    for (int k = 0; k <= 9; ++k) {
      t.predict(Timestamp::fromSeconds(k));
      t.processBatch({posMeas(10.0 * k, 0.0, k)});
    }
    return dominantMeanY(t);
  };
  EXPECT_DOUBLE_EQ(run(false), run(true));
}
