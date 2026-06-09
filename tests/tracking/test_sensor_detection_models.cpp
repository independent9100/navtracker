#include <vector>

#include <gtest/gtest.h>

#include "core/tracking/SensorDetectionModels.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

using navtracker::AdaptiveSensorDetectionModel;
using navtracker::DetectionParams;
using navtracker::FixedSensorDetectionModel;
using navtracker::ISensorDetectionModel;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;

namespace {

Measurement make(SensorKind s, MeasurementModel m) {
  Measurement z;
  z.sensor = s;
  z.model = m;
  z.value = Eigen::Vector2d::Zero();  // dummy
  return z;
}

}  // namespace

TEST(FixedSensorDetectionModel, ReturnsDefaultsForUnknownSensor) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  const auto p =
      m.paramsFor(make(SensorKind::Lidar, MeasurementModel::Position2D));
  EXPECT_DOUBLE_EQ(p.probability_of_detection, 0.9);
  EXPECT_DOUBLE_EQ(p.clutter_intensity, 1e-4);
}

TEST(FixedSensorDetectionModel, PerSensorOverrideTakesPrecedence) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  // EO/IR is high-clutter; ARPA is moderate; AIS is clean.
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D,
        DetectionParams{0.6, 5e-2});  // rad^-1
  m.set(SensorKind::ArpaTtm, MeasurementModel::RangeBearing2D,
        DetectionParams{0.85, 1e-3});  // (m·rad)^-1
  m.set(SensorKind::Ais, MeasurementModel::Position2D,
        DetectionParams{0.98, 1e-9});

  const auto eo =
      m.paramsFor(make(SensorKind::EoIr, MeasurementModel::Bearing2D));
  EXPECT_DOUBLE_EQ(eo.probability_of_detection, 0.6);
  EXPECT_DOUBLE_EQ(eo.clutter_intensity, 5e-2);

  const auto arpa =
      m.paramsFor(make(SensorKind::ArpaTtm, MeasurementModel::RangeBearing2D));
  EXPECT_DOUBLE_EQ(arpa.probability_of_detection, 0.85);
  EXPECT_DOUBLE_EQ(arpa.clutter_intensity, 1e-3);

  const auto ais =
      m.paramsFor(make(SensorKind::Ais, MeasurementModel::Position2D));
  EXPECT_DOUBLE_EQ(ais.probability_of_detection, 0.98);
  EXPECT_DOUBLE_EQ(ais.clutter_intensity, 1e-9);
}

TEST(FixedSensorDetectionModel,
     DimensionalIndependenceAcrossUnitsIsAllowed) {
  // The whole point of the new port: a bearing sensor's clutter
  // intensity in rad^-1 and a position sensor's in m^-2 can coexist
  // — the table treats them as independent scalars, not "the same
  // density."
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D,
        DetectionParams{0.6, 0.1});  // 0.1 rad^-1 — sensible
  m.set(SensorKind::Ais, MeasurementModel::Position2D,
        DetectionParams{0.98, 1e-8});  // 1e-8 m^-2 — sensible

  const auto bearing =
      m.paramsFor(make(SensorKind::EoIr, MeasurementModel::Bearing2D));
  const auto position =
      m.paramsFor(make(SensorKind::Ais, MeasurementModel::Position2D));
  // Verify the two clutter rates differ by orders of magnitude — exactly
  // the unit discrepancy a global scalar λ_C cannot encode.
  EXPECT_GT(bearing.clutter_intensity / position.clutter_intensity, 1e5);
}

TEST(FixedSensorDetectionModel, ObserveIsNoOp) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  std::vector<ISensorDetectionModel::ScanObservation> bundle;
  bundle.push_back({SensorKind::ArpaTtm, MeasurementModel::RangeBearing2D, 20,
                    {{0, 0}, {100, 100}}});
  m.observe(bundle);
  const auto p =
      m.paramsFor(make(SensorKind::ArpaTtm, MeasurementModel::RangeBearing2D));
  EXPECT_DOUBLE_EQ(p.clutter_intensity, 1e-4);
}

TEST(AdaptiveSensorDetectionModel, PerSensorBucketsTrackIndependently) {
  // ARPA gets steady 10 false-alarms / scan in a 200x200m box → λ_C^(arpa)
  // converges to 10/40000 = 2.5e-4. AIS stays clean (0 unassociated) →
  // its λ_C stays at init.
  AdaptiveSensorDetectionModel m(DetectionParams{0.9, 1e-4}, /*alpha=*/0.3);
  std::vector<Eigen::Vector2d> arpa_pts = {
      {0, 0}, {200, 0}, {0, 200}, {200, 200}};
  std::vector<Eigen::Vector2d> ais_pts = arpa_pts;

  for (int s = 0; s < 100; ++s) {
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        {SensorKind::ArpaTtm, MeasurementModel::Position2D, 10, arpa_pts},
        {SensorKind::Ais, MeasurementModel::Position2D, 0, ais_pts}};
    m.observe(bundle);
  }
  EXPECT_NEAR(
      m.densityFor(SensorKind::ArpaTtm, MeasurementModel::Position2D),
      10.0 / 40000.0, 5e-5);
  // AIS bucket sees zero unassociated → rate→0; with area>0, λ_C falls to
  // the floor (min_density), well below init.
  EXPECT_LT(m.densityFor(SensorKind::Ais, MeasurementModel::Position2D),
            1e-4);
}

TEST(AdaptiveSensorDetectionModel,
     OneSensorClutterDoesNotPolluteAnother) {
  // Regression: the legacy single-sensor adaptive model would smear
  // a noisy sensor's rate across the whole pipeline because λ_C was
  // global. The per-sensor port must isolate buckets.
  AdaptiveSensorDetectionModel m(DetectionParams{0.9, 1e-4}, 0.3);
  std::vector<Eigen::Vector2d> pts = {
      {0, 0}, {200, 0}, {0, 200}, {200, 200}};
  for (int s = 0; s < 100; ++s) {
    m.observe({{SensorKind::EoIr, MeasurementModel::Position2D, 30, pts}});
  }
  const double eo = m.densityFor(SensorKind::EoIr, MeasurementModel::Position2D);
  const double ais = m.densityFor(SensorKind::Ais, MeasurementModel::Position2D);
  EXPECT_GT(eo, 1e-4);             // noisy sensor adapted up
  EXPECT_DOUBLE_EQ(ais, 1e-4);     // clean sensor untouched (no bucket)
}

TEST(AdaptiveSensorDetectionModel, BearingBucketKeepsInitDensity) {
  // Bearing-only λ_C is in rad^-1 — the 2-d ENU bounding-box area is
  // dimensionally wrong for it, so the adaptive model leaves Bearing2D
  // buckets at their init density. A proper bearing rate estimator needs
  // an angular FOV (TODO).
  AdaptiveSensorDetectionModel m(DetectionParams{0.6, 0.1}, 0.3);
  m.setSensorInit(SensorKind::EoIr, MeasurementModel::Bearing2D,
                  DetectionParams{0.6, 0.1});
  std::vector<Eigen::Vector2d> dummy_pts;  // bearings carry no position
  for (int s = 0; s < 100; ++s) {
    m.observe({{SensorKind::EoIr, MeasurementModel::Bearing2D, 5, dummy_pts}});
  }
  EXPECT_DOUBLE_EQ(
      m.densityFor(SensorKind::EoIr, MeasurementModel::Bearing2D), 0.1);
}
