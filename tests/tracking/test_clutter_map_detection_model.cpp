#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/tracking/SensorDetectionModels.hpp"

using navtracker::ClutterMapParams;
using navtracker::ClutterMapSensorDetectionModel;
using navtracker::DetectionParams;
using navtracker::FixedSensorDetectionModel;
using navtracker::ISensorDetectionModel;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;

namespace {

constexpr double kRadarPd = 0.8;
constexpr double kRadarLambda = 1e-5;   // m^-2
constexpr double kCamPd = 0.6;
constexpr double kCamLambda = 0.5;      // rad^-1

Measurement posMeas(double x, double y, double t,
                    SensorKind sensor = SensorKind::Lidar,
                    const std::string& source = {}) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.sensor = sensor;
  m.source_id = source;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity();
  return m;
}

Measurement bearingMeas(double az, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.sensor = SensorKind::EoIr;
  m.model = MeasurementModel::Bearing2D;
  m.value = Eigen::VectorXd::Constant(1, az);
  m.covariance = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
  return m;
}

Measurement rangeBearingMeas(double r, double az, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::RangeBearing2D;
  m.value = Eigen::Vector2d(r, az);
  m.covariance = Eigen::Matrix2d::Identity();
  return m;
}

std::shared_ptr<FixedSensorDetectionModel> makeInner() {
  auto inner = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{0.9, 1e-4});
  inner->set(SensorKind::Lidar, MeasurementModel::Position2D,
             DetectionParams{kRadarPd, kRadarLambda});
  inner->set(SensorKind::EoIr, MeasurementModel::Bearing2D,
             DetectionParams{kCamPd, kCamLambda});
  inner->set(SensorKind::ArpaTtm, MeasurementModel::RangeBearing2D,
             DetectionParams{0.7, 2e-6});
  return inner;
}

// One position-sensor scan observation at time t: `all` lists every
// reported ENU position, `unassoc` the subset that gated to no track.
ISensorDetectionModel::ScanObservation posScan(
    double t, const std::vector<Eigen::Vector2d>& all,
    const std::vector<Eigen::Vector2d>& unassoc) {
  ISensorDetectionModel::ScanObservation o;
  o.sensor = SensorKind::Lidar;
  o.model = MeasurementModel::Position2D;
  o.num_unassociated = static_cast<int>(unassoc.size());
  o.positions = all;
  o.time = Timestamp::fromSeconds(t);
  o.clutter_positions = unassoc;
  return o;
}

ISensorDetectionModel::ScanObservation bearingScan(
    double t, const std::vector<double>& all,
    const std::vector<double>& unassoc) {
  ISensorDetectionModel::ScanObservation o;
  o.sensor = SensorKind::EoIr;
  o.model = MeasurementModel::Bearing2D;
  o.num_unassociated = static_cast<int>(unassoc.size());
  o.time = Timestamp::fromSeconds(t);
  o.bearings = all;
  o.clutter_bearings = unassoc;
  return o;
}

}  // namespace

// Before any observations the decorator is transparent: every lookup —
// kind-wide, source-keyed, and measurement-resolved — must reproduce the
// wrapped table exactly.
TEST(ClutterMapDetectionModel, PassthroughBeforeObservations) {
  ClutterMapSensorDetectionModel m(makeInner(), ClutterMapParams{});

  const auto kind = m.paramsFor(SensorKind::Lidar,
                                MeasurementModel::Position2D);
  EXPECT_DOUBLE_EQ(kind.probability_of_detection, kRadarPd);
  EXPECT_DOUBLE_EQ(kind.clutter_intensity, kRadarLambda);

  const auto pos = m.paramsFor(posMeas(123.0, -45.0, 1.0));
  EXPECT_DOUBLE_EQ(pos.probability_of_detection, kRadarPd);
  EXPECT_DOUBLE_EQ(pos.clutter_intensity, kRadarLambda);

  const auto brg = m.paramsFor(bearingMeas(0.3, 1.0));
  EXPECT_DOUBLE_EQ(brg.probability_of_detection, kCamPd);
  EXPECT_DOUBLE_EQ(brg.clutter_intensity, kCamLambda);
}

// Persistent unassociated returns in one cell must raise the local λ
// above the table baseline, while a far-away cell stays at baseline.
TEST(ClutterMapDetectionModel, LearnsHotCellFromUnassociatedReturns) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 5.0;
  ClutterMapSensorDetectionModel m(makeInner(), p);

  // 30 scans at 1 Hz, 3 clutter returns per scan near (100, 100).
  for (int k = 0; k < 30; ++k) {
    const double t = static_cast<double>(k);
    const std::vector<Eigen::Vector2d> pts = {
        {100.0, 100.0}, {105.0, 95.0}, {95.0, 105.0}};
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        posScan(t, pts, pts)};
    m.observe(bundle);
  }

  const double hot =
      m.paramsFor(posMeas(100.0, 100.0, 30.0)).clutter_intensity;
  const double cold =
      m.paramsFor(posMeas(5000.0, 5000.0, 30.0)).clutter_intensity;
  EXPECT_GT(hot, 10.0 * kRadarLambda);
  EXPECT_DOUBLE_EQ(cold, kRadarLambda);
}

// A cell that keeps producing *associated* returns (a tracked target's
// wake) carries evidence the sensor surveys it and clutter is absent —
// its λ must fall below the table baseline.
TEST(ClutterMapDetectionModel, AssociatedTrafficDrivesCellBelowBaseline) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 5.0;
  ClutterMapSensorDetectionModel m(makeInner(), p);

  for (int k = 0; k < 60; ++k) {
    const double t = static_cast<double>(k);
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        posScan(t, {{200.0, 200.0}}, /*unassoc=*/{})};
    m.observe(bundle);
  }

  const double lam =
      m.paramsFor(posMeas(200.0, 200.0, 60.0)).clutter_intensity;
  EXPECT_LT(lam, kRadarLambda);
  EXPECT_GE(lam, p.min_ratio * kRadarLambda);
}

// The resolved λ is clamped to [baseline·min_ratio, baseline·max_ratio]
// no matter how extreme the cell statistics get.
TEST(ClutterMapDetectionModel, ClampsToRatioBandAroundBaseline) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 1.0;  // converge fast
  ClutterMapSensorDetectionModel m(makeInner(), p);

  for (int k = 0; k < 100; ++k) {
    const double t = static_cast<double>(k);
    // 200 clutter returns per scan in one cell → unclamped λ would be
    // 200 / 2500 m² = 0.08 m⁻², 8000× the 1e-5 baseline.
    std::vector<Eigen::Vector2d> pts(200, Eigen::Vector2d(100.0, 100.0));
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        posScan(t, pts, pts)};
    m.observe(bundle);
  }
  const double hot =
      m.paramsFor(posMeas(100.0, 100.0, 100.0)).clutter_intensity;
  EXPECT_DOUBLE_EQ(hot, p.max_ratio * kRadarLambda);
}

// λ is interpolated at the query position: midway between a hot cell
// center and a never-touched neighbour the value sits strictly between
// the hot λ and the baseline.
TEST(ClutterMapDetectionModel, InterpolatesBetweenCellCenters) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 2.0;
  ClutterMapSensorDetectionModel m(makeInner(), p);

  // Hot cell centered at (25, 25) — cell [0,50)×[0,50).
  for (int k = 0; k < 40; ++k) {
    const double t = static_cast<double>(k);
    const std::vector<Eigen::Vector2d> pts = {{25.0, 25.0}, {25.0, 25.0}};
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        posScan(t, pts, pts)};
    m.observe(bundle);
  }

  const double at_center =
      m.paramsFor(posMeas(25.0, 25.0, 40.0)).clutter_intensity;
  const double midway =
      m.paramsFor(posMeas(50.0, 25.0, 40.0)).clutter_intensity;
  const double far_cell =
      m.paramsFor(posMeas(125.0, 25.0, 40.0)).clutter_intensity;
  EXPECT_GT(at_center, midway);
  EXPECT_GT(midway, far_cell);
  EXPECT_DOUBLE_EQ(far_cell, kRadarLambda);
}

// The bearing map is OFF by default: bearings cannot initiate tracks,
// so a real target whose track has lapsed keeps feeding "unassociated"
// bearings at its own azimuth — the map then raises λ exactly where the
// target is and blocks its re-confirmation (measured death spiral on
// AutoFerry: sc17 lifetime 0.90 → 0.25, sc5 0.91 → 0.31; see the
// 2026-06-12 evaluation-log entry). Until the clutter proxy can tell a
// trackless target from shoreline, bearing observations must leave the
// model transparent for Bearing2D queries.
TEST(ClutterMapDetectionModel, BearingObservationsIgnoredByDefault) {
  ClutterMapParams p;
  p.time_constant_s = 5.0;
  ClutterMapSensorDetectionModel m(makeInner(), p);

  for (int k = 0; k < 40; ++k) {
    const double t = static_cast<double>(k);
    const std::vector<double> az = {0.0, 0.01, 0.02};
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        bearingScan(t, az, az)};
    m.observe(bundle);
  }
  const auto dp = m.paramsFor(bearingMeas(0.0, 40.0));
  EXPECT_DOUBLE_EQ(dp.clutter_intensity, kCamLambda);
  EXPECT_DOUBLE_EQ(dp.probability_of_detection, kCamPd);
}

// Opt-in bearing map (enable_bearing_map): a 1-D circular azimuth map —
// persistent clutter bearings near azimuth 0 raise λ there but not on
// the opposite side, and the map wraps at ±π.
TEST(ClutterMapDetectionModel, BearingMapLearnsAzimuthAndWraps) {
  ClutterMapParams p;
  p.enable_bearing_map = true;
  p.bearing_cell_rad = 2.0 * M_PI / 72.0;  // 5°
  p.time_constant_s = 5.0;
  ClutterMapSensorDetectionModel m(makeInner(), p);

  for (int k = 0; k < 40; ++k) {
    const double t = static_cast<double>(k);
    // Shoreline structure: clutter at azimuth ~0 and at the ±π seam.
    const std::vector<double> az = {0.0, 0.01, M_PI - 0.001};
    std::vector<ISensorDetectionModel::ScanObservation> bundle = {
        bearingScan(t, az, az)};
    m.observe(bundle);
  }

  const double at_zero =
      m.paramsFor(bearingMeas(0.0, 40.0)).clutter_intensity;
  const double at_quarter =
      m.paramsFor(bearingMeas(M_PI / 2.0, 40.0)).clutter_intensity;
  // Query just across the seam from the +π training bearings.
  const double across_seam =
      m.paramsFor(bearingMeas(-M_PI + 0.001, 40.0)).clutter_intensity;
  EXPECT_GT(at_zero, 2.0 * kCamLambda);
  EXPECT_DOUBLE_EQ(at_quarter, kCamLambda);
  EXPECT_GT(across_seam, kCamLambda);
}

// RangeBearing2D measurement space ((m·rad)^-1) has no map yet — the
// decorator must pass the wrapped table through untouched even after
// observations for that sensor.
TEST(ClutterMapDetectionModel, RangeBearingPassesThrough) {
  ClutterMapSensorDetectionModel m(makeInner(), ClutterMapParams{});
  ISensorDetectionModel::ScanObservation o;
  o.sensor = SensorKind::ArpaTtm;
  o.model = MeasurementModel::RangeBearing2D;
  o.num_unassociated = 50;
  o.positions = {{10.0, 10.0}};
  o.time = Timestamp::fromSeconds(1.0);
  std::vector<ISensorDetectionModel::ScanObservation> bundle = {o};
  for (int k = 0; k < 20; ++k) m.observe(bundle);

  const auto dp = m.paramsFor(rangeBearingMeas(100.0, 0.5, 20.0));
  EXPECT_DOUBLE_EQ(dp.clutter_intensity, 2e-6);
  EXPECT_DOUBLE_EQ(dp.probability_of_detection, 0.7);
}

// The EWMA weight is time-based (τ seconds), not scan-counted: the same
// number of clutter scans moves the cell further when the scans span
// more time. (Multi-rate lesson: 10 scans is 0.6 s for a 16 Hz camera
// and 100 s for AIS.)
TEST(ClutterMapDetectionModel, ConvergenceIsTimeBasedNotScanBased) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 10.0;

  ClutterMapSensorDetectionModel fast(makeInner(), p);
  ClutterMapSensorDetectionModel slow(makeInner(), p);
  for (int k = 0; k < 10; ++k) {
    const std::vector<Eigen::Vector2d> pts = {{100.0, 100.0}};
    std::vector<ISensorDetectionModel::ScanObservation> hi = {
        posScan(0.0625 * k, pts, pts)};  // 16 Hz
    std::vector<ISensorDetectionModel::ScanObservation> lo = {
        posScan(2.0 * k, pts, pts)};     // 0.5 Hz
    fast.observe(hi);
    slow.observe(lo);
  }
  const double lam_fast =
      fast.paramsFor(posMeas(100.0, 100.0, 1.0)).clutter_intensity;
  const double lam_slow =
      slow.paramsFor(posMeas(100.0, 100.0, 20.0)).clutter_intensity;
  // Same scan count, ~32× the elapsed time → the slow stream must have
  // pulled the cell much closer to its per-scan count.
  EXPECT_GT(lam_slow, lam_fast);
}

// P_D, coverage range, and sector fields are never touched by the map —
// only clutter_intensity is position-resolved. Source-keyed entries in
// the wrapped model keep working through the decorator.
TEST(ClutterMapDetectionModel, OnlyLambdaIsRemappedAndSourceKeysSurvive) {
  auto inner = makeInner();
  DetectionParams ir{0.4, kCamLambda};
  ir.max_range_m = 900.0;
  inner->set(SensorKind::EoIr, MeasurementModel::Bearing2D, "ir", ir);
  ClutterMapSensorDetectionModel m(inner, ClutterMapParams{});

  Measurement z = bearingMeas(0.0, 1.0);
  z.source_id = "ir";
  const auto dp = m.paramsFor(z);
  EXPECT_DOUBLE_EQ(dp.probability_of_detection, 0.4);
  EXPECT_DOUBLE_EQ(dp.max_range_m, 900.0);
  EXPECT_DOUBLE_EQ(dp.clutter_intensity, kCamLambda);

  // Coverage-conditioned miss P_D goes through the inner table too.
  const double pd = m.missDetectionProbability(
      SensorKind::EoIr, MeasurementModel::Bearing2D,
      Eigen::Vector2d(1000.0, 0.0), Eigen::Vector2d::Zero(), "ir");
  EXPECT_DOUBLE_EQ(pd, 0.0);
}

// Clutter evidence carries per-return weights (1 − existence of the
// claiming hypothesis, from the tracker's global solve): a return
// claimed by a half-confident track must move the cell half as far as
// a fully unclaimed one. Empty weight vectors mean weight 1.0 per
// return (back-compat with binary labeling).
TEST(ClutterMapDetectionModel, WeightedEvidenceScalesCellUpdate) {
  ClutterMapParams p;
  p.cell_size_m = 50.0;
  p.time_constant_s = 2.0;
  ClutterMapSensorDetectionModel full(makeInner(), p);
  ClutterMapSensorDetectionModel half(makeInner(), p);

  for (int k = 0; k < 40; ++k) {
    const double t = static_cast<double>(k);
    auto o = posScan(t, {{100.0, 100.0}}, {{100.0, 100.0}});
    std::vector<ISensorDetectionModel::ScanObservation> b_full = {o};
    o.clutter_position_weights = {0.5};
    std::vector<ISensorDetectionModel::ScanObservation> b_half = {o};
    full.observe(b_full);
    half.observe(b_half);
  }

  const double lam_full =
      full.paramsFor(posMeas(100.0, 100.0, 40.0)).clutter_intensity;
  const double lam_half =
      half.paramsFor(posMeas(100.0, 100.0, 40.0)).clutter_intensity;
  EXPECT_GT(lam_full, lam_half);
  EXPECT_GT(lam_half, kRadarLambda);
  // Converged cell rates are ~1.0 vs ~0.5 → the interpolated λ above
  // the shared baseline should be roughly double.
  EXPECT_NEAR(lam_full / lam_half, 2.0, 0.25);
}
