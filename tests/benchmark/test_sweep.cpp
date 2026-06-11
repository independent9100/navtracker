#include <gtest/gtest.h>

#include "core/benchmark/Config.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/benchmark/Sweep.hpp"
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
  // 1 config * 1 scenario * 2 seeds * 8 metrics = 16 rows
  EXPECT_EQ(rows.size(), 16u);
  for (const auto& r : rows) {
    EXPECT_EQ(r.run_id, "test_run");
    EXPECT_EQ(r.config, "ekf_cv_gnn");
    EXPECT_EQ(r.scenario, "tiny_line");
  }
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
