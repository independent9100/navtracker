// Review #18 (VERIFY): confirm the heading-bias correction and the
// per-sensor position-bias correction are NOT both folding the same
// physical offset (no "double debiasing") on the ARPA / EO-IR path.
//
// Conclusion locked in by these tests: the two corrections are applied by
// independent mechanisms on disjoint quantities —
//   * heading bias  → angular, platform-global, removed *upstream in the
//     adapter* via IHeadingBiasProvider (rotates the reported bearing);
//   * per-sensor bias → translational/bearing, sensor-specific, applied in
//     the Tracker via ISensorBiasProvider (applyBiasCorrection).
// They use different provider interfaces and different keys, so no code
// path applies the same correction twice. The per-sensor estimator only
// ever sees the residual left after the adapter's heading correction (see
// SensorBiasPairExtractor.hpp). System-level separability of a far-target
// heading offset vs a per-sensor position offset relies on multi-range
// geometry; that is the documented design, not a code defect.

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/BiasCorrection.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "ports/ISensorBiasProvider.hpp"

using namespace navtracker;

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

class StubHeadingBias : public IHeadingBiasProvider {
 public:
  HeadingBiasEstimate value;
  HeadingBiasEstimate current() const override { return value; }
};

OwnShipProvider makeOwnShip() {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);
  return provider;
}

CameraDetection eastTarget() {
  CameraDetection d;
  d.time = Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;  // east
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;
  return d;
}

}  // namespace

// applyBiasCorrection applies the per-sensor position bias exactly once
// (and inflates R by P_b), independent of any heading correction.
TEST(BiasCorrectionOrthogonality, PositionBiasAppliedExactlyOnce) {
  FixedSensorBiasProvider provider;
  const SensorBiasKey key{SensorKind::Ais, "ais"};
  const Eigen::Vector2d b(3.0, -2.0);
  provider.setPositionBias(key, b, Eigen::Matrix2d::Identity() * 4.0);

  Measurement z;
  z.sensor = SensorKind::Ais;
  z.source_id = "ais";
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 10.0);
  z.covariance = Eigen::Matrix2d::Identity() * 25.0;

  const Measurement out = applyBiasCorrection(z, &provider);
  EXPECT_DOUBLE_EQ(out.value(0), 10.0 - 3.0);
  EXPECT_DOUBLE_EQ(out.value(1), 10.0 + 2.0);
  // R inflated by P_b once: 25 + 4.
  EXPECT_DOUBLE_EQ(out.covariance(0, 0), 29.0);
  EXPECT_DOUBLE_EQ(out.covariance(1, 1), 29.0);
}

// The two corrections compose orthogonally: the adapter's heading
// correction changes the projected position, and applyBiasCorrection then
// shifts it by exactly the per-sensor bias — the position shift is
// independent of whether a heading bias is present, so neither re-applies
// the other.
TEST(BiasCorrectionOrthogonality, HeadingAndPerSensorCorrectionsAreDisjoint) {
  geo::Datum datum({53.5, 8.0, 0.0});

  // Adapter output WITHOUT heading bias.
  OwnShipProvider os0 = makeOwnShip();
  EoIrAdapter a0(datum, os0);
  a0.ingest(eastTarget());
  const Measurement m_none = a0.poll().front();

  // Adapter output WITH a 5° heading bias (rotates the bearing upstream).
  StubHeadingBias hb;
  hb.value.bias_rad = 5.0 * kDeg2Rad;
  hb.value.variance_rad2 = 0.0;
  hb.value.is_published = true;
  OwnShipProvider os1 = makeOwnShip();
  EoIrAdapter a1(datum, os1, EoIrAdapterConfig{}, &hb);
  a1.ingest(eastTarget());
  const Measurement m_head = a1.poll().front();

  // The heading bias must have moved the projected position.
  EXPECT_GT((m_head.value - m_none.value).norm(), 1.0);

  // Now apply a per-sensor position bias to the heading-corrected
  // measurement. The shift must be exactly -b, regardless of the heading
  // correction already baked in (no double counting).
  FixedSensorBiasProvider sensor_bias;
  const SensorBiasKey key{m_head.sensor, m_head.source_id};
  const Eigen::Vector2d b(7.0, -4.0);
  sensor_bias.setPositionBias(key, b, Eigen::Matrix2d::Identity() * 1e-6);

  const Measurement m_both = applyBiasCorrection(m_head, &sensor_bias);
  EXPECT_NEAR(m_both.value(0), m_head.value(0) - b.x(), 1e-6);
  EXPECT_NEAR(m_both.value(1), m_head.value(1) - b.y(), 1e-6);
}
