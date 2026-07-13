// Cl-4 adoption zone tests (ADR-0003, 2026-07-12). Companion to the ADR-0001
// no-birth-zone validator in test_pmbm_land_model.cpp. The deployable config
// imm_cv_ct_pmbm_coverage_land_ivgate narrows the blocked offshore strip from
// 50 m to 25 m (Config::coastline_prior_params, see test_config.cpp). These
// tests exercise the BIRTH consequence at the shipped floor (0.10) through a
// land model that reproduces the exact shoreline ramp:
//
//   c(d) = clamp((W_off - d) / (W_off + W_in), 0, 1),  d = signed offshore
//   distance (m; d>0 water, d<0 inland), W_in = 50 m.
//   r_new(c) = 0.1(1-c) / (0.1(1-c) + 0.9);  admit iff r_new >= floor = 0.10
//              ⇔ c = 0 ⇔ d >= W_off  (the pinned-existence collapse, §3.2.2).
//
// So at floor 0.10 the admit boundary IS W_off: a vessel offshore of W_off
// births at full strength (r_new = 0.1); inside it is gated out. W_off = 25 m
// therefore revives the 25–50 m band that the old 50 m strip suppressed, while
// keeping the 0–25 m band blocked.
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ILandModel.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::PmbmTracker;

namespace {

// Exact shipped shoreline ramp (CoastlineGeometry::priorAtGeodetic), keyed on
// the measurement's x = offshore distance in metres (shore at x = 0).
struct RampLand : navtracker::ILandModel {
  explicit RampLand(double off, double in = 50.0) : w_off(off), w_in(in) {}
  double w_off;
  double w_in;
  double clutterPrior(const Eigen::Vector2d& p) const override {
    const double c = (w_off - p.x()) / (w_off + w_in);
    return std::clamp(c, 0.0, 1.0);
  }
};

Measurement posMeas(double x_offshore, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x_offshore, 0.0);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

// Faithful to the deployable config: adaptive birth, target 0.1, floor 0.10
// (== makeCoverageLandPmbmConfig()). The floor is what turns suppression into
// a hard no-birth boundary at c > 0.
PmbmTracker::Config candidateCfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.min_new_bernoulli_existence = 0.10;  // shipped floor
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  c.use_land_model = true;
  return c;
}

// Max birth existence after one scan at the given offshore distance and W_off.
double maxBirthExistence(double w_off, double x_offshore) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker t(ekf, candidateCfg());
  RampLand land{w_off};
  t.setLandModel(&land);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(x_offshore, 0.0)});
  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  return maxr;
}

constexpr double kW25 = 25.0;  // the deployable config's offshore half-width
constexpr double kW50 = 50.0;  // the pre-Cl-4 default (teeth reference)

}  // namespace

// THE CHANGE: a vessel at ~30 m offshore now BIRTHS under the 25 m strip
// (30 m >= 25 m ⇒ c = 0 ⇒ r_new = 0.1, clears the floor).
TEST(Cl4NearShoreZone, RevivesAtThirtyMetresUnderCandidate) {
  EXPECT_GT(maxBirthExistence(kW25, 30.0), 0.05)
      << "a 30 m-offshore vessel must birth under W_off=25 m";
}

// TEETH (encoded): the same 30 m vessel is SUPPRESSED under the old 50 m strip
// (30 m < 50 m ⇒ c = 0.2 ⇒ r_new ≈ 0.082 < 0.10 ⇒ gated). Flipping W back to
// 50 turns the birth off — this is the red the RevivesAtThirtyMetres test
// would show if the narrowing were reverted.
TEST(Cl4NearShoreZone, ThirtyMetresSuppressedUnderOldFiftyMetreStrip) {
  EXPECT_LT(maxBirthExistence(kW50, 30.0), 0.01)
      << "under W_off=50 m the 30 m vessel must be gated (the pre-Cl-4 zone)";
}

// STILL BLOCKED: a return at ~10 m offshore stays suppressed even under 25 m
// (10 m < 25 m ⇒ c = 0.2 ⇒ gated) — the residual 0–25 m blind band (ADR-0003
// accepted cost).
TEST(Cl4NearShoreZone, TenMetresStillSuppressedUnderCandidate) {
  EXPECT_LT(maxBirthExistence(kW25, 10.0), 0.01)
      << "a 10 m-offshore return must stay gated under W_off=25 m";
}

// UNTOUCHED (the existing ADR-0001 validator): a vessel at 60 m births under
// both strips (60 m >= 50 m >= 25 m ⇒ c = 0 either way).
TEST(Cl4NearShoreZone, SixtyMetresUntouchedBothStrips) {
  EXPECT_GT(maxBirthExistence(kW25, 60.0), 0.05);
  EXPECT_GT(maxBirthExistence(kW50, 60.0), 0.05);
}
