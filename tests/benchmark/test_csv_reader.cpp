#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/CsvReader.hpp"
#include "core/benchmark/CsvWriter.hpp"

using namespace navtracker::benchmark;

TEST(CsvReader, RoundTripsThroughWriter) {
  CsvProvenance p;
  p.run_id = "r1";
  p.started_at_utc = "2026-06-07T10:14:22Z";
  p.git_sha = "abc1234 (clean)";
  p.build_type = "Release";
  p.compiler = "gcc 13.2.0";
  p.host = "linux x86_64";
  p.seeds = {0, 1, 2};
  p.config_count = 1;
  p.scenario_count = 1;
  p.total_runs = 3;
  p.elapsed_seconds = 2.5;

  std::vector<MetricRow> rows = {
      {"r1", "cfg", "scen", 0, "ospa_mean", 12.5, "m"},
      {"r1", "cfg", "scen", 1, "ospa_mean", 13.0, "m"},
      {"r1", "cfg", "scen", 2, "lifetime_ratio", 0.95, "ratio"},
  };

  std::ostringstream wos;
  writeCsv(wos, p, rows);
  std::istringstream rin(wos.str());
  const auto doc = readCsv(rin);

  EXPECT_EQ(doc.provenance.run_id, "r1");
  EXPECT_EQ(doc.provenance.git_sha, "abc1234 (clean)");
  EXPECT_EQ(doc.provenance.seeds.size(), 3u);
  EXPECT_EQ(doc.provenance.seeds[2], 2u);
  ASSERT_EQ(doc.rows.size(), 3u);
  EXPECT_EQ(doc.rows[0].metric, "ospa_mean");
  EXPECT_NEAR(doc.rows[0].value, 12.5, 1e-9);
  EXPECT_EQ(doc.rows[2].metric, "lifetime_ratio");
  EXPECT_NEAR(doc.rows[2].value, 0.95, 1e-9);
}
