#include <gtest/gtest.h>

#include "core/benchmark/Config.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/scenario/Builders.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
class TinyStraightLine : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"tiny_line", true, 2};
  }
  Scenario generate(std::uint64_t seed) override {
    std::vector<double> times;
    for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
    return buildStraightLineScenario(
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(10, 0),
        times, 1.0,
        static_cast<std::uint32_t>(seed),
        1);
  }
};
}  // namespace

TEST(Sweep, RowCountMatchesMatrix) {
  // Select ekf_cv_gnn explicitly (by label, not position) so the test is
  // robust to the canonical-config ordering in defaultConfigs().
  std::vector<Config> all = defaultConfigs();
  std::vector<Config> configs;
  for (const auto& c : all)
    if (c.label == "ekf_cv_gnn") configs.push_back(c);
  ASSERT_EQ(configs.size(), 1u);
  std::vector<std::unique_ptr<ScenarioRun>> scenarios;
  scenarios.push_back(std::make_unique<TinyStraightLine>());

  SweepParams p;
  p.run_id = "test_run";
  p.synthetic_seeds = 2;

  const auto rows = runSweep(configs, scenarios, p);
  // 1 config * 1 scenario * 2 seeds → some number of metric rows.
  // 11 kinematic/quality metrics (OSPA mean/p95 + GOSPA mean/p95/rms +
  // lifetime/breaks/switches/RMSE × 4) + 6 NEES metrics + 6 NIS metrics
  // per (sensor, model, source_id) source key contributing innovations
  // + 7 per-truth metrics (lifetime/breaks/switches/RMSE × 4 + rmse_n)
  // per truth_id. TinyStraightLine has one source and one truth target;
  // total per seed = 11 + 6 + 6 + 7 = 30.
  EXPECT_EQ(rows.size(), 2u * 30u);
  std::size_t nees_seen = 0;
  std::size_t nis_seen = 0;
  std::size_t per_truth_seen = 0;
  for (const auto& r : rows) {
    EXPECT_EQ(r.run_id, "test_run");
    EXPECT_EQ(r.config, "ekf_cv_gnn");
    EXPECT_EQ(r.scenario, "tiny_line");
    if (r.metric.rfind("nees_", 0) == 0) ++nees_seen;
    if (r.metric.rfind("nis_", 0) == 0) ++nis_seen;
    if (r.metric.find(":truth_") != std::string::npos) ++per_truth_seen;
  }
  EXPECT_EQ(nees_seen, 12u);  // 6 NEES * 2 seeds
  EXPECT_GE(nis_seen, 12u);   // ≥ 6 NIS * 2 seeds (≥ 1 source)
  EXPECT_EQ(per_truth_seen, 14u);  // 7 per-truth * 1 truth * 2 seeds
}

// --- Per-scenario per-sensor detection tables -----------------------------

TEST(Sweep, DetectionModelForBuildsPerSensorTable) {
  ScenarioDescriptor desc;
  desc.label = "with_table";
  desc.detection_table.push_back(
      {navtracker::SensorKind::Lidar, navtracker::MeasurementModel::Position2D,
       navtracker::DetectionParams{0.7, 5e-6, 140.0}});
  desc.detection_table.push_back(
      {navtracker::SensorKind::EoIr, navtracker::MeasurementModel::Bearing2D,
       navtracker::DetectionParams{0.6, 0.5}});

  navtracker::MhtTracker::Config cfg;
  const auto model = detectionModelFor(desc, cfg);
  ASSERT_TRUE(model);
  const auto lidar = model->paramsFor(navtracker::SensorKind::Lidar,
                                      navtracker::MeasurementModel::Position2D);
  EXPECT_DOUBLE_EQ(lidar.probability_of_detection, 0.7);
  EXPECT_DOUBLE_EQ(lidar.clutter_intensity, 5e-6);
  EXPECT_DOUBLE_EQ(lidar.max_range_m, 140.0);
  const auto eo = model->paramsFor(navtracker::SensorKind::EoIr,
                                   navtracker::MeasurementModel::Bearing2D);
  EXPECT_DOUBLE_EQ(eo.probability_of_detection, 0.6);
  // Sensors not in the table fall back to the tracker config defaults.
  const auto other = model->paramsFor(navtracker::SensorKind::Ais,
                                      navtracker::MeasurementModel::Position2D);
  EXPECT_DOUBLE_EQ(other.probability_of_detection,
                   cfg.probability_of_detection);
  EXPECT_DOUBLE_EQ(other.clutter_intensity, cfg.clutter_density);
}

TEST(Sweep, DetectionModelForEmptyTableReturnsNull) {
  // No table → null, and the sweep falls back to the legacy scalar
  // clutter_density override.
  ScenarioDescriptor desc;
  desc.label = "no_table";
  navtracker::MhtTracker::Config cfg;
  EXPECT_FALSE(detectionModelFor(desc, cfg));
}

TEST(Sweep, DetectionModelForBuildsSourceKeyedEntries) {
  // An entry with a source_id calibrates one physical sensor unit (EO
  // vs IR cameras share SensorKind::EoIr); empty source_id stays a
  // kind-wide entry that unknown sources fall back to.
  ScenarioDescriptor desc;
  desc.label = "with_sources";
  desc.detection_table.push_back(
      {navtracker::SensorKind::EoIr, navtracker::MeasurementModel::Bearing2D,
       navtracker::DetectionParams{0.6, 0.5}});
  desc.detection_table.push_back(
      {navtracker::SensorKind::EoIr, navtracker::MeasurementModel::Bearing2D,
       navtracker::DetectionParams{0.8, 0.9}, "cam_eo"});
  desc.detection_table.push_back(
      {navtracker::SensorKind::EoIr, navtracker::MeasurementModel::Bearing2D,
       navtracker::DetectionParams{0.4, 0.3}, "cam_ir"});

  navtracker::MhtTracker::Config cfg;
  const auto model = detectionModelFor(desc, cfg);
  ASSERT_TRUE(model);
  const auto eo = model->paramsFor(navtracker::SensorKind::EoIr,
                                   navtracker::MeasurementModel::Bearing2D,
                                   "cam_eo");
  EXPECT_DOUBLE_EQ(eo.probability_of_detection, 0.8);
  EXPECT_DOUBLE_EQ(eo.clutter_intensity, 0.9);
  const auto ir = model->paramsFor(navtracker::SensorKind::EoIr,
                                   navtracker::MeasurementModel::Bearing2D,
                                   "cam_ir");
  EXPECT_DOUBLE_EQ(ir.probability_of_detection, 0.4);
  const auto other = model->paramsFor(navtracker::SensorKind::EoIr,
                                      navtracker::MeasurementModel::Bearing2D,
                                      "cam_unknown");
  EXPECT_DOUBLE_EQ(other.probability_of_detection, 0.6);
}

TEST(Sweep, DetectionModelForWrapsClutterMapWhenRequested) {
  // Backlog item 5: the clutter-map ablation wraps the scenario's fixed
  // table in a ClutterMapSensorDetectionModel. The wrap must preserve
  // the table lookups (transparent before observations) and must NOT
  // happen for the default fixed path or for table-less scenarios.
  ScenarioDescriptor desc;
  desc.label = "with_table";
  desc.detection_table.push_back(
      {navtracker::SensorKind::EoIr, navtracker::MeasurementModel::Bearing2D,
       navtracker::DetectionParams{0.6, 0.5}});

  navtracker::MhtTracker::Config cfg;
  const auto wrapped = detectionModelFor(desc, cfg, /*use_clutter_map=*/true);
  ASSERT_TRUE(wrapped);
  EXPECT_NE(
      dynamic_cast<navtracker::ClutterMapSensorDetectionModel*>(wrapped.get()),
      nullptr);
  const auto p = wrapped->paramsFor(navtracker::SensorKind::EoIr,
                                    navtracker::MeasurementModel::Bearing2D);
  EXPECT_DOUBLE_EQ(p.probability_of_detection, 0.6);
  EXPECT_DOUBLE_EQ(p.clutter_intensity, 0.5);

  const auto fixed = detectionModelFor(desc, cfg);
  ASSERT_TRUE(fixed);
  EXPECT_EQ(
      dynamic_cast<navtracker::ClutterMapSensorDetectionModel*>(fixed.get()),
      nullptr);

  ScenarioDescriptor no_table;
  no_table.label = "no_table";
  EXPECT_FALSE(detectionModelFor(no_table, cfg, /*use_clutter_map=*/true));
}
