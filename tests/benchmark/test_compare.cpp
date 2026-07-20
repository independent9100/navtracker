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

// #36 M23: signed/target metrics (card_err_mean, nees_*, nis_coverage_95,
// nis_trace_ratio) are neither lower- nor higher-is-better — the "better"
// direction depends on a target the pure renderer doesn't know. The old
// isLowerBetter default rendered a (false) improvement up-arrow whenever such a
// metric merely decreased. They must carry a neutral, non-directional
// indicator so a reader compares the raw numbers against the target instead of
// trusting a wrong arrow.
TEST(Comparator, TargetMetricDecreaseIsNotFlaggedAsImprovement) {
  // nis_trace_ratio targets 1.0; 1.0 -> 0.5 is WORSE but numerically lower.
  ComparisonInput a{}, b{};
  a.prov.run_id = "baseline";
  b.prov.run_id = "input";
  a.rows = {{"baseline", "cfg", "scen", 0, "nis_trace_ratio", 1.0, ""}};
  b.rows = {{"input", "cfg", "scen", 0, "nis_trace_ratio", 0.5, ""}};

  std::ostringstream os;
  renderComparison(os, {a, b});
  const std::string s = os.str();
  EXPECT_EQ(s.find(u8"▲"), std::string::npos) << s;  // no false improvement
  EXPECT_EQ(s.find(u8"▼"), std::string::npos) << s;  // and no false regression
  EXPECT_NE(s.find("0.5"), std::string::npos);        // raw value still shown
}

TEST(Comparator, SignedAndTargetMetricsAllGetNeutralIndicator) {
  for (const char* m : {"card_err_mean", "nees_mean", "nees_pos",
                        "nis_coverage_95", "nis_trace_ratio"}) {
    ComparisonInput a{}, b{};
    a.prov.run_id = "baseline";
    b.prov.run_id = "input";
    a.rows = {{"baseline", "cfg", "scen", 0, m, 2.0, ""}};
    b.rows = {{"input", "cfg", "scen", 0, m, 3.0, ""}};

    std::ostringstream os;
    renderComparison(os, {a, b});
    const std::string s = os.str();
    EXPECT_EQ(s.find(u8"▲"), std::string::npos) << m << ":\n" << s;
    EXPECT_EQ(s.find(u8"▼"), std::string::npos) << m << ":\n" << s;
  }
}
