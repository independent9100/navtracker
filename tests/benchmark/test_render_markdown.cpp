#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/MarkdownRenderer.hpp"

using namespace navtracker::benchmark;

TEST(MarkdownRenderer, EmitsScenarioSections) {
  CsvProvenance p;
  p.run_id = "test";
  std::vector<MetricRow> rows = {
      {"test", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 10.0, "m"},
      {"test", "ekf_cv_gnn", "crossing", 1, "ospa_mean", 12.0, "m"},
      {"test", "ekf_cv_gnn", "overtaking", 0, "ospa_mean", 5.0, "m"},
  };
  std::ostringstream os;
  renderMarkdown(os, p, rows);
  const auto s = os.str();
  EXPECT_NE(s.find("## crossing"), std::string::npos);
  EXPECT_NE(s.find("## overtaking"), std::string::npos);
  EXPECT_NE(s.find("ekf_cv_gnn"), std::string::npos);
  // mean of {10, 12} = 11
  EXPECT_NE(s.find("11"), std::string::npos);
}
