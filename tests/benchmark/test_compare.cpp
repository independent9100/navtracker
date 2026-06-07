#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/Comparator.hpp"

using namespace navtracker::benchmark;

TEST(Comparator, MarksImprovementsAndRegressions) {
  // baseline ospa_mean = 100; improvement = 80 (lower is better -> up arrow)
  ComparisonInput a{}, b{};
  a.prov.run_id = "baseline";
  b.prov.run_id = "improv";
  a.rows = {{"baseline", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 100.0, "m"}};
  b.rows = {{"improv", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 80.0, "m"}};

  std::ostringstream os;
  renderComparison(os, {a, b});
  const std::string s = os.str();
  EXPECT_NE(s.find(u8"▲"), std::string::npos);
  EXPECT_NE(s.find("100"), std::string::npos);
  EXPECT_NE(s.find("80"), std::string::npos);
}

TEST(Comparator, MarksRegressionsForLowerIsBetterMetrics) {
  // ospa goes from 80 to 100 -> regression (down arrow)
  ComparisonInput a{}, b{};
  a.prov.run_id = "baseline";
  b.prov.run_id = "regress";
  a.rows = {{"baseline", "cfg", "scen", 0, "ospa_mean", 80.0, "m"}};
  b.rows = {{"regress", "cfg", "scen", 0, "ospa_mean", 100.0, "m"}};

  std::ostringstream os;
  renderComparison(os, {a, b});
  EXPECT_NE(os.str().find(u8"▼"), std::string::npos);
}

TEST(Comparator, LifetimeRatioImprovementIsPositiveDelta) {
  // lifetime_ratio is HIGHER-is-better; 0.9 -> 0.95 is improvement (up arrow)
  ComparisonInput a{}, b{};
  a.prov.run_id = "baseline";
  b.prov.run_id = "improv";
  a.rows = {{"baseline", "cfg", "scen", 0, "lifetime_ratio", 0.90, "ratio"}};
  b.rows = {{"improv", "cfg", "scen", 0, "lifetime_ratio", 0.95, "ratio"}};

  std::ostringstream os;
  renderComparison(os, {a, b});
  EXPECT_NE(os.str().find(u8"▲"), std::string::npos);
}
