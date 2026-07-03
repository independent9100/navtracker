#include <cmath>
#include <string>
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

// --- Coverage-conditioned miss P_D ---------------------------------------
//
// The miss branch of the MHT score asks "could this sensor have detected
// the track at all?". A sensor whose coverage excludes the track (e.g. a
// lidar with ~140 m range and a track at 500 m) must contribute P_D = 0,
// i.e. zero miss penalty — otherwise every out-of-coverage scan bleeds
// score from a perfectly healthy track.

TEST(FixedSensorDetectionModel, MissPdReturnsTablePdInsideCoverage) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::Lidar, MeasurementModel::Position2D,
        DetectionParams{0.7, 5e-6, /*max_range_m=*/100.0});
  const double pd = m.missDetectionProbability(
      SensorKind::Lidar, MeasurementModel::Position2D,
      /*track_pos_enu=*/Eigen::Vector2d(50.0, 0.0),
      /*sensor_pos_enu=*/Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(pd, 0.7);
}

TEST(FixedSensorDetectionModel, MissPdZeroBeyondMaxRange) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::Lidar, MeasurementModel::Position2D,
        DetectionParams{0.7, 5e-6, /*max_range_m=*/100.0});
  const double pd = m.missDetectionProbability(
      SensorKind::Lidar, MeasurementModel::Position2D,
      Eigen::Vector2d(500.0, 0.0), Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(pd, 0.0);
}

TEST(FixedSensorDetectionModel, MissPdDefaultEntryIsUnbounded) {
  // No table entry → defaults, whose max_range is infinite: coverage
  // never truncates (legacy behaviour for sensors without range info).
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  const double pd = m.missDetectionProbability(
      SensorKind::Ais, MeasurementModel::Position2D,
      Eigen::Vector2d(1e6, 0.0), Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(pd, 0.9);
}

// ---------------------------------------------------------------------------
// Source-keyed entries: two physical sensors sharing a SensorKind (EO and
// IR cameras are both SensorKind::EoIr) calibrate independently when the
// table carries entries keyed by Measurement::source_id. Lookup precedence:
// (sensor, model, source_id) exact > (sensor, model) kind-wide > defaults.

namespace {

Measurement makeWithSource(SensorKind s, MeasurementModel m,
                           const std::string& source) {
  Measurement z = make(s, m);
  z.source_id = source;
  return z;
}

}  // namespace

TEST(FixedSensorDetectionModel, SourceKeyedEntryTakesPrecedence) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D,
        DetectionParams{0.6, 0.5});  // kind-wide combined entry
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, "eo",
        DetectionParams{0.8, 0.9});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, "ir",
        DetectionParams{0.4, 0.3});

  const auto eo = m.paramsFor(
      makeWithSource(SensorKind::EoIr, MeasurementModel::Bearing2D, "eo"));
  EXPECT_DOUBLE_EQ(eo.probability_of_detection, 0.8);
  EXPECT_DOUBLE_EQ(eo.clutter_intensity, 0.9);

  const auto ir = m.paramsFor(
      makeWithSource(SensorKind::EoIr, MeasurementModel::Bearing2D, "ir"));
  EXPECT_DOUBLE_EQ(ir.probability_of_detection, 0.4);
  EXPECT_DOUBLE_EQ(ir.clutter_intensity, 0.3);
}

TEST(FixedSensorDetectionModel, UnknownSourceFallsBackToKindThenDefaults) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D,
        DetectionParams{0.6, 0.5});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, "eo",
        DetectionParams{0.8, 0.9});

  // Source not in the table → kind-wide entry.
  const auto other = m.paramsFor(
      makeWithSource(SensorKind::EoIr, MeasurementModel::Bearing2D, "tv"));
  EXPECT_DOUBLE_EQ(other.probability_of_detection, 0.6);

  // Neither source nor kind entry → defaults.
  const auto none = m.paramsFor(
      makeWithSource(SensorKind::Lidar, MeasurementModel::Position2D, "x"));
  EXPECT_DOUBLE_EQ(none.probability_of_detection, 0.9);
}

TEST(ISensorDetectionModel, ThreeArgParamsForDefaultsToKindLookup) {
  // Models that don't override the source-aware lookup keep behaving
  // exactly as before — the base implementation ignores source_id.
  AdaptiveSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  const auto p = m.paramsFor(SensorKind::Ais, MeasurementModel::Position2D,
                             "any_source");
  EXPECT_DOUBLE_EQ(p.probability_of_detection, 0.9);
  EXPECT_DOUBLE_EQ(p.clutter_intensity, 1e-4);
}

// ---------------------------------------------------------------------------
// Azimuth-sector coverage. DetectionParams::{sector_center_rad,
// sector_width_rad} bound the sensor's field of view about its position,
// in the ENU math convention (atan2(dy, dx), CCW from east — the same
// convention as Bearing2D). A track outside the sector cannot have been
// detected → missDetectionProbability returns 0 (no miss penalty), same
// contract as max_range_m. Default width 2π = full circle = legacy.

TEST(FixedSensorDetectionModel, OutOfSectorTrackHasZeroMissPd) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  DetectionParams cam{0.8, 0.5};
  cam.sector_center_rad = 0.0;            // looking east
  cam.sector_width_rad = M_PI / 2.0;      // ±45°
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, cam);

  // Track due north of the sensor — 90° off the sector centre.
  const double p_d = m.missDetectionProbability(
      SensorKind::EoIr, MeasurementModel::Bearing2D,
      /*track=*/Eigen::Vector2d(0.0, 100.0),
      /*sensor=*/Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(p_d, 0.0);
}

TEST(FixedSensorDetectionModel, InSectorTrackKeepsTablePd) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  DetectionParams cam{0.8, 0.5};
  cam.sector_center_rad = 0.0;
  cam.sector_width_rad = M_PI / 2.0;
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, cam);

  // 30° off-centre — inside the ±45° sector.
  const double az = M_PI / 6.0;
  const double p_d = m.missDetectionProbability(
      SensorKind::EoIr, MeasurementModel::Bearing2D,
      Eigen::Vector2d(100.0 * std::cos(az), 100.0 * std::sin(az)),
      Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(p_d, 0.8);
}

TEST(FixedSensorDetectionModel, DefaultSectorIsFullCircle) {
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D,
        DetectionParams{0.8, 0.5});
  // No sector configured: any azimuth keeps the table P_D.
  for (double az = -3.0; az <= 3.0; az += 0.7) {
    const double p_d = m.missDetectionProbability(
        SensorKind::EoIr, MeasurementModel::Bearing2D,
        Eigen::Vector2d(50.0 * std::cos(az), 50.0 * std::sin(az)),
        Eigen::Vector2d(0.0, 0.0));
    EXPECT_DOUBLE_EQ(p_d, 0.8) << "az=" << az;
  }
}

TEST(FixedSensorDetectionModel, SectorWrapsAcrossPi) {
  // Sector centred on the ±π seam (looking west). A track just south of
  // due west sits at azimuth ≈ −(π − ε): in-sector only if the angular
  // difference is wrapped.
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  DetectionParams cam{0.7, 0.5};
  cam.sector_center_rad = M_PI;
  cam.sector_width_rad = M_PI / 2.0;
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, cam);

  const double in_sector = m.missDetectionProbability(
      SensorKind::EoIr, MeasurementModel::Bearing2D,
      Eigen::Vector2d(-100.0, -10.0), Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(in_sector, 0.7);

  const double out_of_sector = m.missDetectionProbability(
      SensorKind::EoIr, MeasurementModel::Bearing2D,
      Eigen::Vector2d(100.0, 10.0), Eigen::Vector2d(0.0, 0.0));
  EXPECT_DOUBLE_EQ(out_of_sector, 0.0);
}

TEST(FixedSensorDetectionModel, SourceKeyedMissPdRespectsSourceEntry) {
  // missDetectionProbability with a source_id picks the source-keyed
  // entry — EO and IR charge different miss penalties.
  FixedSensorDetectionModel m(DetectionParams{0.9, 1e-4});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, "eo",
        DetectionParams{0.8, 0.9});
  m.set(SensorKind::EoIr, MeasurementModel::Bearing2D, "ir",
        DetectionParams{0.4, 0.3});

  const Eigen::Vector2d track(100.0, 0.0), sensor(0.0, 0.0);
  EXPECT_DOUBLE_EQ(
      m.missDetectionProbability(SensorKind::EoIr,
                                 MeasurementModel::Bearing2D, track, sensor,
                                 "eo"),
      0.8);
  EXPECT_DOUBLE_EQ(
      m.missDetectionProbability(SensorKind::EoIr,
                                 MeasurementModel::Bearing2D, track, sensor,
                                 "ir"),
      0.4);
}

// R8.4 / increment 6b — CoverageSector::fromReturns self-estimates the swept
// footprint from a scan's returns (the same self-estimation pattern as the
// clutter-adaptive bar). The swept sector is the largest CONTIGUOUS cluster of
// return bearings about the sensor (a physical burst sweeps only a small arc);
// range is the farthest return in that cluster (padded).
TEST(CoverageSector, FromReturnsEstimatesSweptSectorAndRange) {
  using Cov = ISensorDetectionModel::CoverageSector;
  const Eigen::Vector2d sensor(0.0, 0.0);
  // A tight cluster: bearings 0°, ~5.7°, ~11.3° (a realistic ~11° burst span);
  // farthest return (100, 20).
  const std::vector<Eigen::Vector2d> pts = {
      {100.0, 0.0}, {100.0, 10.0}, {100.0, 20.0}};
  const Cov c = Cov::fromReturns(sensor, pts, /*az_pad_rad=*/0.0,
                                 /*range_pad_frac=*/0.0);
  ASSERT_TRUE(c.valid);
  for (const auto& p : pts) EXPECT_TRUE(c.covers(p)) << "built-from return not covered";
  EXPECT_NEAR(c.max_range_m, std::hypot(100.0, 20.0), 1e-9);
  EXPECT_FALSE(c.covers(Eigen::Vector2d(1000.0, 50.0)))  // in sector, out of range
      << "beyond-range point wrongly covered";
  EXPECT_FALSE(c.covers(Eigen::Vector2d(-50.0, -50.0)))  // out of sector
      << "opposite-sector point wrongly covered";
}

// The multi-cluster guard (6c): a wide burst whose returns form two clusters
// with a large internal gap is TWO echo clusters sharing one timestamp — an
// 80 ms burst cannot physically sweep across a 90° gap. The estimator must keep
// only the largest contiguous cluster and NOT claim the unswept gap as observed
// (claiming it would decay cells the sensor never looked at — the unsafe
// direction, and it lands hardest on the dense close_approach clip).
TEST(CoverageSector, FromReturnsKeepsLargestClusterNotTheInterClusterGap) {
  using Cov = ISensorDetectionModel::CoverageSector;
  const Eigen::Vector2d sensor(0.0, 0.0);
  const std::vector<Eigen::Vector2d> pts = {
      {100.0, 0.0}, {100.0, 5.0}, {100.0, 10.0},  // 3-return cluster near 0–5.7°
      {-17.0, 97.0}, {-20.0, 98.0}};               // 2-return cluster near 100°
  const Cov c = Cov::fromReturns(sensor, pts, /*az_pad_rad=*/0.0,
                                 /*range_pad_frac=*/0.0);  // default ~20° gap
  ASSERT_TRUE(c.valid);
  EXPECT_TRUE(c.covers(Eigen::Vector2d(100.0, 0.0)));   // kept (larger) cluster
  EXPECT_TRUE(c.covers(Eigen::Vector2d(100.0, 10.0)));
  EXPECT_FALSE(c.covers(Eigen::Vector2d(70.0, 70.0)))   // 45° — the unswept gap
      << "inter-cluster gap wrongly claimed as swept (over-claim, unsafe)";
  EXPECT_FALSE(c.covers(Eigen::Vector2d(-17.0, 97.0)))  // dropped smaller cluster
      << "dropped cluster should wait for its own burst";
}

// Degenerate inputs: empty ⇒ invalid (consumer assumes full coverage); a single
// return ⇒ a narrow sector at its bearing widened by the padding.
TEST(CoverageSector, FromReturnsDegenerateCases) {
  using Cov = ISensorDetectionModel::CoverageSector;
  EXPECT_FALSE(Cov::fromReturns(Eigen::Vector2d(0.0, 0.0), {}).valid);

  const Cov one = Cov::fromReturns(Eigen::Vector2d(0.0, 0.0), {{100.0, 0.0}},
                                   /*az_pad_rad=*/0.1, /*range_pad_frac=*/0.0);
  ASSERT_TRUE(one.valid);
  EXPECT_TRUE(one.covers(Eigen::Vector2d(100.0, 0.0)));
  EXPECT_FALSE(one.covers(Eigen::Vector2d(0.0, 100.0)));  // 90° off, outside pad
}

// The circular span handles the ±180° wrap: a tight cluster straddling due-west
// (≈ ±175°, a ~10° span) must yield the NARROW western arc, not the wide eastern
// complement.
TEST(CoverageSector, FromReturnsHandlesWraparound) {
  using Cov = ISensorDetectionModel::CoverageSector;
  const Cov c = Cov::fromReturns(Eigen::Vector2d(0.0, 0.0),
                                 {{-100.0, 9.0}, {-100.0, -9.0}},  // ≈ ±175°
                                 /*az_pad_rad=*/0.0, /*range_pad_frac=*/0.0);
  ASSERT_TRUE(c.valid);
  EXPECT_TRUE(c.covers(Eigen::Vector2d(-100.0, 0.0)));   // due west — inside the arc
  EXPECT_FALSE(c.covers(Eigen::Vector2d(100.0, 0.0)));   // due east — the wide gap
}
