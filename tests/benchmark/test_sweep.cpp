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
  std::vector<Config> configs = {defaultConfigs()[0]};  // ekf_cv_gnn
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
