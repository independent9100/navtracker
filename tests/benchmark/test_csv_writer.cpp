#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/CsvWriter.hpp"

using namespace navtracker::benchmark;

TEST(CsvWriter, EmitsHeaderBlockAndRows) {
  CsvProvenance p;
  p.run_id = "2026-06-07_baseline";
  p.started_at_utc = "2026-06-07T10:14:22Z";
  p.git_sha = "abc1234 (clean)";
  p.build_type = "Release";
  p.compiler = "gcc 13.2.0";
  p.host = "linux x86_64";
  p.seeds = {0, 1};
  p.config_count = 1;
  p.scenario_count = 1;
  p.total_runs = 2;
  p.elapsed_seconds = 1.5;

  std::vector<MetricRow> rows = {
      {"2026-06-07_baseline", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 73.42, "m"},
  };

  std::ostringstream os;
  writeCsv(os, p, rows);
  const auto s = os.str();
  EXPECT_NE(s.find("# run_id: 2026-06-07_baseline"), std::string::npos);
  EXPECT_NE(s.find("# git_sha: abc1234 (clean)"), std::string::npos);
  EXPECT_NE(s.find("run_id,config,scenario,seed,metric,value,unit"), std::string::npos);
  EXPECT_NE(s.find("2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_mean,73.42,m"),
            std::string::npos);
}
