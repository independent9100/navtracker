// SPRT confirmation mechanism test. Verifies the flag routes confirmation
// through the Wald LLR threshold and that a clean, well-tracked target
// confirms under SPRT. (Head-to-head SPRT-vs-M-of-N quality is a
// benchmark concern, not a unit test — measured results live in
// docs/baselines; SPRT currently underperforms M-of-N, hence default off.)

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker;

namespace {

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "sim";
  return m;
}

MhtTracker::Config sprtConfig() {
  MhtTracker::Config cfg;
  cfg.gate_threshold = 20.0;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_density = 1e-4;  // physical scale: hits accrue positive LLR
  cfg.use_sprt_confirm = true;
  cfg.sprt_alpha = 0.01;
  cfg.sprt_beta = 0.01;
  return cfg;
}

}  // namespace

TEST(MhtSprtConfirm, CleanTargetConfirmsUnderSprt) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  MhtTracker tracker(est, sprtConfig());

  // A single target moving at 2 m/s east, one clean detection per scan.
  for (int k = 0; k < 8; ++k) {
    std::vector<Measurement> scan = {posMeas(2.0 * k, 0.0, k)};
    tracker.processBatch(scan);
  }

  bool any_confirmed = false;
  for (const auto& tr : tracker.tracks())
    if (tr.status == TrackStatus::Confirmed) any_confirmed = true;
  EXPECT_TRUE(any_confirmed)
      << "a clean sustained target should confirm under SPRT";
}

TEST(MhtSprtConfirm, SingleDetectionDoesNotConfirm) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  MhtTracker tracker(est, sprtConfig());

  // One lone detection: LLR after a single hit must not cross T_confirm
  // (ln(0.99/0.01) ≈ 4.6) hard enough to confirm a one-scan track.
  tracker.processBatch({posMeas(0.0, 0.0, 0.0)});
  for (const auto& tr : tracker.tracks())
    EXPECT_NE(tr.status, TrackStatus::Confirmed)
        << "a one-scan track should not be Confirmed";
}
