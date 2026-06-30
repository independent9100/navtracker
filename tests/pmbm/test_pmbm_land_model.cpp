#include <gtest/gtest.h>

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

// prior = 1.0 east of x=500 (land), 0.0 west (water).
struct HalfPlaneLand : navtracker::ILandModel {
  double clutterPrior(const Eigen::Vector2d& p) const override {
    return p.x() > 500.0 ? 1.0 : 0.0;
  }
};

// Constant mid prior everywhere (soft suppression).
struct SoftLand : navtracker::ILandModel {
  double clutterPrior(const Eigen::Vector2d&) const override { return 0.5; }
};

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;  // clutter-invariant: r_new independent of lambda_C
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  return c;
}

}  // namespace

// Hard gate: birth at a position well inland (prior=1.0) → rho_target=0 → no
// Bernoulli with meaningful existence survives pruning.
TEST(PmbmLandModel, HardGateDropsBirthOnLand) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  PmbmTracker t(ekf, c);
  HalfPlaneLand land;
  t.setLandModel(&land);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(1000.0, 0.0, 0.0)});  // x=1000 > 500 → land → dropped

  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  EXPECT_LT(maxr, 1e-6);
}

// Open water: birth at x=0 (prior=0.0) → scale=1.0 → existence unchanged ~0.1.
TEST(PmbmLandModel, OpenWaterBirthUnchanged) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  PmbmTracker t(ekf, c);
  HalfPlaneLand land;
  t.setLandModel(&land);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(0.0, 0.0, 0.0)});  // x=0 < 500 → water → normal birth

  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  EXPECT_NEAR(maxr, 0.1, 0.05);
}

// Soft prior: prior=0.5 everywhere → lambda_birth halved → r_new < 0.1 but > 0.
// With birth_existence_target=0.1 and c=0.5:
//   lb = (0.1/0.9)*lambda_z; after scale: 0.5*lb
//   r_new = 0.5*lb / (0.5*lb + lambda_z) = (1/18)*lambda_z / ((1+1/18)*lambda_z)
//         = 1/19 ≈ 0.053
TEST(PmbmLandModel, SoftPriorHalvesBirth) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  PmbmTracker t(ekf, c);
  SoftLand land;
  t.setLandModel(&land);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(0.0, 0.0, 0.0)});  // prior=0.5 everywhere

  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  EXPECT_LT(maxr, 0.1);   // suppressed below the 0.1 open-water birth
  EXPECT_GT(maxr, 0.0);   // but not dropped
}

// Null land model (use_land_model defaults false, no setLandModel): birth
// happens normally at ~0.1 even for a "land" position.
TEST(PmbmLandModel, NullLandModelBitIdentical) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();  // use_land_model defaults false
  PmbmTracker t(ekf, c);          // no setLandModel
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(1000.0, 0.0, 0.0)});  // would be "land" if model were wired

  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  EXPECT_NEAR(maxr, 0.1, 0.05);  // birth happens normally
}
