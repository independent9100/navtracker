#include <gtest/gtest.h>
#include "core/scenario/Builders.hpp"

using navtracker::buildParallelTargetsScenario;
using navtracker::buildStraightLineScenario;
using navtracker::Scenario;

TEST(Builders, StraightLineProducesTruthAndMeasurementsAtSameTimes) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 0.0, 1, 42);
  ASSERT_EQ(s.truth.size(), 3u);
  ASSERT_EQ(s.measurements.size(), 3u);
  EXPECT_DOUBLE_EQ(s.truth[1].position.x(), 20.0);
  EXPECT_DOUBLE_EQ(s.truth[1].position.y(), 0.0);
  EXPECT_EQ(s.truth[1].truth_id, 42u);
  EXPECT_DOUBLE_EQ(s.measurements[1].value(0), 20.0);
  EXPECT_DOUBLE_EQ(s.measurements[1].time.seconds(), 2.0);
}

TEST(Builders, ParallelTargetsEmitsTwoTruthPerTime) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 500.0),
      Eigen::Vector2d(10.0, 0.0), times, 0.0, 1);
  ASSERT_EQ(s.truth.size(), 4u);
  ASSERT_EQ(s.measurements.size(), 4u);
  EXPECT_NE(s.truth[0].truth_id, s.truth[1].truth_id);
}

TEST(Builders, DeterministicForSameSeed) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario a = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  const Scenario b = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.measurements[i].value(0), b.measurements[i].value(0));
    EXPECT_DOUBLE_EQ(a.measurements[i].value(1), b.measurements[i].value(1));
  }
}

TEST(Builders, RangeBearingPassFirstIsPositionThenRangeBearing) {
  const std::vector<double> times{0.0, 1.0, 2.0};
  const Scenario s = navtracker::buildRangeBearingPassScenario(
      Eigen::Vector2d(1000.0, 0.0), Eigen::Vector2d(-50.0, 0.0),
      times, 5.0, 10.0, 0.087, 7);
  ASSERT_EQ(s.measurements.size(), 3u);
  EXPECT_EQ(s.measurements[0].model, navtracker::MeasurementModel::Position2D);
  EXPECT_EQ(s.measurements[1].model, navtracker::MeasurementModel::RangeBearing2D);
  EXPECT_EQ(s.measurements[2].model, navtracker::MeasurementModel::RangeBearing2D);
}

TEST(Builders, ClutterCrossingEmitsTwoRealPlusNFalsePerScan) {
  std::vector<double> times{1.0, 2.0};
  const navtracker::Scenario s = navtracker::buildClutterCrossingScenario(
      Eigen::Vector2d(-100.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      Eigen::Vector2d( 100.0, 0.0), Eigen::Vector2d(-10.0, 0.0),
      times, 1.0, /*clutter*/ 3,
      Eigen::Vector2d(-500.0, -500.0), Eigen::Vector2d(500.0, 500.0),
      11);
  // 2 timestamps * (2 real + 3 clutter) = 10 measurements
  EXPECT_EQ(s.measurements.size(), 10u);
  EXPECT_EQ(s.truth.size(), 4u);
}

TEST(Builders, CrossingDropoutOmitsMeasurementsInDropoutWindow) {
  std::vector<double> times;
  for (int i = 0; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const navtracker::Scenario s = navtracker::buildCrossingDropoutScenario(
      /*vx*/ 5.0, /*y*/ 1.0, times, /*noise*/ 0.0,
      /*dropout_start*/ 9.0, /*dropout_end*/ 12.0, 17);
  // Truth: 2 per timestamp * 21 timestamps = 42.
  EXPECT_EQ(s.truth.size(), 42u);
  // Measurements: 2 per timestamp * (21 - 3 dropout) = 36.
  EXPECT_EQ(s.measurements.size(), 36u);
}

TEST(Builders, BearingOnlyMovingSensorAttachesSensorPose) {
  std::vector<double> times{0.0, 1.0, 2.0};
  const navtracker::Scenario s =
      navtracker::buildBearingOnlyMovingSensorScenario(
          Eigen::Vector2d(1000.0, 0.0),
          Eigen::Vector2d(0.0, 0.0),
          Eigen::Vector2d(5.0, 0.0),
          times, 50.0, 0.05, /*seed*/ 99);
  ASSERT_EQ(s.measurements.size(), 3u);
  EXPECT_EQ(s.measurements[0].model, navtracker::MeasurementModel::Position2D);
  EXPECT_DOUBLE_EQ(s.measurements[0].sensor_position_enu.x(), 0.0);
  EXPECT_EQ(s.measurements[1].model, navtracker::MeasurementModel::Bearing2D);
  EXPECT_NEAR(s.measurements[1].sensor_position_enu.x(), 5.0, 1e-9);
  EXPECT_NEAR(s.measurements[2].sensor_position_enu.x(), 10.0, 1e-9);
}

TEST(Builders, ManeuveringTargetHasStraightThenTurnThenStraight) {
  const navtracker::Scenario s = navtracker::buildManeuveringTargetScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      /*straight*/ 5.0, /*turn*/ 5.0, /*omega*/ 0.2,
      /*dt*/ 1.0, /*noise*/ 0.0, /*seed*/ 1);
  ASSERT_EQ(s.truth.size(), 16u);
  EXPECT_NEAR(s.truth[5].position.x(), 50.0, 1e-9);
  EXPECT_NEAR(s.truth[5].position.y(),  0.0, 1e-9);
  EXPECT_GT(std::abs(s.truth[10].velocity.y()), 1e-3);
}

#include <cmath>
using navtracker::buildConvoyScenario;
using navtracker::buildCrossingAngleScenario;
using navtracker::buildParallelLaneScenario;

TEST(Builders, ParallelLaneEmitsNTargetsPerScanSpacedPerpendicular) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario s = buildParallelLaneScenario(
      /*n_targets=*/4, /*lane_spacing_m=*/50.0,
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),  // heading +x
      times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 8u);          // 4 targets x 2 scans
  ASSERT_EQ(s.measurements.size(), 8u);
  // Lanes offset perpendicular to +x velocity => along +y, spacing 50 m.
  EXPECT_DOUBLE_EQ(s.truth[0].position.y(), 0.0);
  EXPECT_DOUBLE_EQ(s.truth[1].position.y(), 50.0);
  EXPECT_DOUBLE_EQ(s.truth[3].position.y(), 150.0);
  EXPECT_EQ(s.truth[0].truth_id, 1u);
  EXPECT_EQ(s.truth[3].truth_id, 4u);
}

TEST(Builders, CrossingAngleVelocitiesSubtendRequestedAngle) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildCrossingAngleScenario(
      /*crossing_angle_deg=*/60.0, /*speed_mps=*/20.0,
      Eigen::Vector2d(0.0, 0.0), times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 6u);          // 2 targets x 3 scans
  const Eigen::Vector2d va = s.truth[0].velocity;
  const Eigen::Vector2d vb = s.truth[1].velocity;
  const double ang = std::atan2(va.x() * vb.y() - va.y() * vb.x(),
                                va.dot(vb));
  EXPECT_NEAR(std::abs(ang) * 180.0 / M_PI, 60.0, 1e-6);
  EXPECT_NEAR(va.norm(), 20.0, 1e-9);
  EXPECT_NEAR(vb.norm(), 20.0, 1e-9);
}

TEST(Builders, ConvoyEmitsConvoyPlusOvertaker) {
  const std::vector<double> times{1.0};
  const Scenario s = buildConvoyScenario(
      /*n_targets=*/3, /*gap_m=*/80.0, /*speed_mps=*/5.0,
      /*overtaker_speed_mps=*/15.0, times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 4u);          // 3 convoy + 1 overtaker
  EXPECT_EQ(s.truth[3].truth_id, 4u);     // overtaker emitted last
  EXPECT_GT(s.truth[3].velocity.x(), s.truth[0].velocity.x());  // faster
}

TEST(Builders, GeometryBuildersDeterministicForSameSeed) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario a = buildConvoyScenario(2, 80.0, 5.0, 15.0, times, 4.0, 9);
  const Scenario b = buildConvoyScenario(2, 80.0, 5.0, 15.0, times, 4.0, 9);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.measurements[i].value(0), b.measurements[i].value(0));
    EXPECT_DOUBLE_EQ(a.measurements[i].value(1), b.measurements[i].value(1));
  }
}

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineModel.hpp"
using navtracker::addShoreClutter;
using navtracker::buildSyntheticShore;
using navtracker::CoastlineModel;
using navtracker::SyntheticShore;

TEST(ShoreClutter, SyntheticShoreClutterPointsReadHardGatePrior) {
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0},
      /*shore_y_m=*/500.0, /*extent_m=*/1500.0, /*land_depth_m=*/400.0,
      /*pier_width_m=*/40.0, /*pier_length_m=*/150.0, /*n_clutter=*/30);
  ASSERT_EQ(shore.clutter_enu_points.size(), 30u);
  const CoastlineModel model(shore.geometry, shore.datum);
  // Every deep-inland clutter point sits in the hard-gate plateau (c ~ 1).
  for (const auto& p : shore.clutter_enu_points) {
    EXPECT_GE(model.clutterPrior(p), 0.95);
  }
  // A point far out to sea reads ~0 (open water).
  EXPECT_NEAR(model.clutterPrior(Eigen::Vector2d(0.0, -500.0)), 0.0, 1e-9);
}

TEST(ShoreClutter, InjectorAddsFixedReturnsWithoutTruth) {
  // Two-scan base scenario, one real target.
  const std::vector<double> times{1.0, 2.0};
  navtracker::Scenario base = navtracker::buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0), times, 0.0, 1, 1);
  const std::size_t base_truth = base.truth.size();
  const std::size_t base_meas = base.measurements.size();
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0}, 500.0, 1500.0, 400.0,
      40.0, 150.0, /*n_clutter=*/5);
  navtracker::Scenario s = addShoreClutter(
      base, shore.datum, shore.clutter_enu_points,
      /*detection_prob=*/1.0, /*pos_noise_std_m=*/0.0, /*seed=*/7);
  // P_D = 1 => 5 clutter returns x 2 scans added; truth unchanged.
  EXPECT_EQ(s.truth.size(), base_truth);
  EXPECT_EQ(s.measurements.size(), base_meas + 10u);
  EXPECT_TRUE(s.datum.has_value());
  std::size_t shore_count = 0;
  for (const auto& m : s.measurements) {
    if (m.source_id == "sim_shore") {
      ++shore_count;
      EXPECT_EQ(m.sensor, navtracker::SensorKind::ArpaTtm);
    }
  }
  EXPECT_EQ(shore_count, 10u);
  // Measurements remain time-sorted after injection.
  for (std::size_t i = 1; i < s.measurements.size(); ++i) {
    EXPECT_LE(s.measurements[i - 1].time.seconds(),
              s.measurements[i].time.seconds());
  }
}

TEST(ShoreClutter, ClutterPositionsRepeatAcrossScans) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  navtracker::Scenario base = navtracker::buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0), times, 0.0, 1, 1);
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0}, 500.0, 1500.0, 400.0,
      40.0, 150.0, /*n_clutter=*/1);
  navtracker::Scenario s = addShoreClutter(base, shore.datum,
                                           shore.clutter_enu_points,
                                           /*detection_prob=*/1.0,
                                           /*pos_noise_std_m=*/0.0, /*seed=*/3);
  // With zero noise, the single shore point lands at the same ENU position
  // on every scan (fixed-position process, not redrawn).
  std::vector<Eigen::Vector2d> shore_pos;
  for (const auto& m : s.measurements)
    if (m.source_id == "sim_shore") shore_pos.push_back(m.value.head<2>());
  ASSERT_EQ(shore_pos.size(), 3u);
  EXPECT_DOUBLE_EQ(shore_pos[0].x(), shore_pos[1].x());
  EXPECT_DOUBLE_EQ(shore_pos[1].x(), shore_pos[2].x());
  EXPECT_DOUBLE_EQ(shore_pos[0].y(), shore_pos[2].y());
}
