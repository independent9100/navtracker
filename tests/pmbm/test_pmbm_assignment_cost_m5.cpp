#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorActivity.hpp"

// ── Backlog #34 M5: the assignment cost must equal the applied posterior ─────
//
// Defect: the Murty cost for a Bernoulli detection cell was −log(r·ℓ), omitting
// the detection-pricing term −log(p_D/(1−r·p_D)) that the APPLIED hypothesis
// weight carries. Under the deployable K=1 hard-commit, the K-best argmin then
// differs from the maximum-posterior assignment → the tracker commits to the
// wrong child.
//
// These tests pin the invariant directly, using the tracker's OWN applied
// weights as ground truth: run the identical scenario at K=large (enumerate
// every child; pruneAndNormalise ranks them by the applied posterior weight, so
// the dominant child IS the posterior argmax) and at K=1 (hard-commit to the
// cost-argmin). The two MUST agree. The scenario is a detect-vs-birth contest on
// a confident track where detection is the posterior optimum but the buggy cost
// (missing the term) commits to a phantom birth.
//
// TEETH: with the term removed, K=1 reverts to the birth child while K=large
// still ranks detection first → the K=1 assertion goes RED. Covered for both
// use_sensor_activity OFF (legacy compute_miss_pD baseline) and ON (conditional
// surveillance-miss opp.p_D baseline) — the two forms the reconciled cost must
// match.

using navtracker::ChannelKind;
using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::ISensorActivity;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::MissOpportunity;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::Bernoulli;
using navtracker::pmbm::GlobalHypothesis;
using navtracker::pmbm::PmbmTracker;
using navtracker::pmbm::PoissonComponent;

namespace {

Eigen::Vector4d cvState(double px, double py) {
  return Eigen::Vector4d(px, py, 0.0, 0.0);
}
Eigen::Matrix4d posCov(double sp, double sv) {
  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = P(1, 1) = sp * sp;
  P(2, 2) = P(3, 3) = sv * sv;
  return P;
}
Measurement pos2d(double t, double x, double y, double sigma) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = SensorKind::Lidar;
  z.source_id = "r0";
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
  return z;
}

// Fake surveillance profile: always reports a completed sweep that covered the
// track (surveillance_miss=true) with a fixed p_D — deliberately DIFFERENT from
// Config::probability_of_detection so a test can prove the reconciled cost pulls
// its miss-baseline from opp.p_D (not the config scalar).
struct AlwaysSurveillanceMiss : ISensorActivity {
  double pd;
  explicit AlwaysSurveillanceMiss(double p) : pd(p) {}
  MissOpportunity evaluate(const Eigen::Vector2d&, const Eigen::Vector2d&,
                           std::optional<std::uint32_t>,
                           std::optional<std::uint64_t>, Timestamp,
                           Timestamp) const override {
    MissOpportunity o;
    o.surveillance_miss = true;
    o.p_D = pd;
    return o;
  }
  std::optional<ChannelKind> channelKindFor(SensorKind) const override {
    return ChannelKind::Surveillance;
  }
};

// Run the detect-vs-birth scenario and return the existence of the confident
// track (Bernoulli id=1) in the dominant (max-weight) global hypothesis.
// A value of 1.0 ⇒ the track detected the measurement; < 1.0 ⇒ it was missed
// (and the measurement was explained as a phantom birth instead).
double confidentTrackExistence(int k_best, const ISensorActivity* activity) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;   // isolate the update from predict decay
  cfg.clutter_intensity = 0.1;      // birth is cheap enough that the buggy cost
                                    // prefers it over detecting the track
  cfg.k_best_per_hypothesis = k_best;
  cfg.adaptive_k_best = false;
  cfg.max_global_hypotheses = 50;
  cfg.min_new_bernoulli_existence = 0.0;
  cfg.use_sensor_activity = (activity != nullptr);

  PmbmTracker tracker(ekf, cfg);
  if (activity != nullptr) tracker.setSensorActivity(activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Confident track at the origin.
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  Bernoulli a;
  a.id = 1;
  a.existence_probability = 0.99;
  a.mean = cvState(0.0, 0.0);
  a.covariance = posCov(2.0, 0.5);
  a.last_update = Timestamp::fromSeconds(0.0);
  h.bernoullis.push_back(std::move(a));
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  // A PPP component at the origin so the measurement can alternatively be
  // explained as a new-target birth.
  PoissonComponent p;
  p.weight = 1.0;
  p.mean = cvState(0.0, 0.0);
  p.covariance = posCov(5.0, 1.0);
  tracker.mutableDensityForTesting().ppp.push_back(std::move(p));

  // One measurement exactly at the track's predicted position (zero
  // innovation → high likelihood).
  tracker.processBatch({pos2d(0.0, 0.0, 0.0, 0.5)});

  const auto& mbm = tracker.density().mbm;
  EXPECT_FALSE(mbm.empty());
  const GlobalHypothesis* dom = nullptr;
  for (const auto& g : mbm)
    if (dom == nullptr || g.weight > dom->weight) dom = &g;
  if (dom == nullptr) return -1.0;
  for (const auto& b : dom->bernoullis)
    if (b.id == 1) return b.existence_probability;
  return -1.0;  // track 1 absent from the dominant hypothesis
}

}  // namespace

TEST(PmbmAssignmentCostM5, K1CommitsToMaxPosteriorAssignmentLegacy) {
  // Ground truth from the tracker's own applied posterior: with every child
  // enumerated, detection by the confident track is the max-weight explanation.
  const double kbig = confidentTrackExistence(/*k_best=*/20, /*activity=*/nullptr);
  ASSERT_NEAR(kbig, 1.0, 1e-9)
      << "detection must be the posterior argmax (it wins the K=large ranking)";
  // The K=1 hard-commit MUST select that same assignment. Pre-fix the buggy
  // cost commits to the phantom birth and this track is missed (r≈0.908).
  const double k1 = confidentTrackExistence(/*k_best=*/1, /*activity=*/nullptr);
  EXPECT_NEAR(k1, 1.0, 1e-9)
      << "M5: K=1 cost-argmin must equal the max-posterior assignment (legacy "
         "miss baseline)";
}

TEST(PmbmAssignmentCostM5, K1CommitsToMaxPosteriorAssignmentSensorActivity) {
  // Same invariant through the use_sensor_activity path: the reconciled cost
  // must pull its miss baseline from opp.p_D (here 0.8, ≠ the config's 0.9).
  AlwaysSurveillanceMiss activity(0.8);
  const double kbig = confidentTrackExistence(/*k_best=*/20, &activity);
  ASSERT_NEAR(kbig, 1.0, 1e-9)
      << "detection must be the posterior argmax under sensor-activity too";
  const double k1 = confidentTrackExistence(/*k_best=*/1, &activity);
  EXPECT_NEAR(k1, 1.0, 1e-9)
      << "M5: K=1 must match the posterior under the conditional "
         "surveillance-miss (opp.p_D) baseline";
}
