// D2 GOSPA cross-validation exporter. Verifies writeBenchStatesCsv /
// writeOurGospaCsv emit exactly what an external scorer needs: per-scan
// (truth, track) ENU positions and our per-scan GOSPA decomposition that
// matches core/scenario/Gospa.hpp on the same point sets.
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/GospaExport.hpp"
#include "core/scenario/Gospa.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
std::string tmpPath(const std::string& name) {
  const char* dir = std::getenv("TEST_TMPDIR");
  std::string base = dir ? dir : "/tmp";
  return base + "/gospa_export_test_" + name;
}

std::string slurp(const std::string& path) {
  std::ifstream is(path);
  std::stringstream ss;
  ss << is.rdbuf();
  return ss.str();
}

BenchResult twoStepResult() {
  BenchResult r;
  // Step 0: two truth, one track near truth #1, one spurious far track.
  BenchStep s0;
  s0.time = Timestamp::fromSeconds(1.0);
  s0.truth = {{1u, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d::Zero()},
              {2u, Eigen::Vector2d(100.0, 0.0), Eigen::Vector2d::Zero()}};
  TrackStateSnapshot t_close;
  t_close.id = TrackId{7u};
  t_close.position = Eigen::Vector2d(3.0, 4.0);  // 5 m from truth #1 (< cutoff)
  TrackStateSnapshot t_far;
  t_far.id = TrackId{8u};
  t_far.position = Eigen::Vector2d(500.0, 500.0);  // far from all truth
  s0.tracks = {t_close, t_far};
  // Step 1: one truth, no tracks (pure miss).
  BenchStep s1;
  s1.time = Timestamp::fromSeconds(2.0);
  s1.truth = {{1u, Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d::Zero()}};
  r.steps = {s0, s1};
  return r;
}
}  // namespace

TEST(GospaExport, StatesCsvHasHeaderAndOneRowPerObject) {
  const auto r = twoStepResult();
  const std::string path = tmpPath("states.csv");
  writeBenchStatesCsv(r, path);
  const std::string csv = slurp(path);

  EXPECT_EQ(csv.substr(0, csv.find('\n')), "scan,time_s,kind,id,east_m,north_m");
  // 2 truth + 2 track (scan 0) + 1 truth (scan 1) = 5 data rows + 1 header.
  std::size_t lines = 0;
  for (char ch : csv)
    if (ch == '\n') ++lines;
  EXPECT_EQ(lines, 6u);
  EXPECT_NE(csv.find("0,1,truth,1,0,0"), std::string::npos);
  EXPECT_NE(csv.find("0,1,track,7,3,4"), std::string::npos);
  EXPECT_NE(csv.find("0,1,track,8,500,500"), std::string::npos);
  EXPECT_NE(csv.find("1,2,truth,1,10,10"), std::string::npos);
}

TEST(GospaExport, OurGospaCsvMatchesGospaComponents) {
  const auto r = twoStepResult();
  const double c = 20.0;  // harbor gospa_cutoff_m
  const std::string path = tmpPath("ours.csv");
  writeOurGospaCsv(r, c, path);

  // Recompute the reference values the CSV must reproduce, scan by scan.
  std::vector<Eigen::Vector2d> truth0 = {{0, 0}, {100, 0}};
  std::vector<Eigen::Vector2d> est0 = {{3, 4}, {500, 500}};
  const GospaComponents g0 = gospaComponents(truth0, est0, c);
  const double d0 = gospaGreedy(truth0, est0, c);
  // Sanity on the geometry: truth#1 matched (loc = 5^2 = 25), truth#2 missed,
  // far track false → n_missed = n_false = 1.
  EXPECT_DOUBLE_EQ(g0.localization, 25.0);
  EXPECT_EQ(g0.n_missed, 1);
  EXPECT_EQ(g0.n_false, 1);

  std::vector<Eigen::Vector2d> truth1 = {{10, 10}};
  std::vector<Eigen::Vector2d> est1 = {};
  const GospaComponents g1 = gospaComponents(truth1, est1, c);
  EXPECT_EQ(g1.n_missed, 1);
  EXPECT_EQ(g1.n_false, 0);

  std::ifstream is(path);
  std::string header;
  std::getline(is, header);
  EXPECT_EQ(header,
            "scan,time_s,gospa,localisation,missed,false,n_missed,n_false");

  auto readRow = [&](int expect_scan) {
    std::string line;
    std::getline(is, line);
    for (char& ch : line)
      if (ch == ',') ch = ' ';
    std::istringstream ls(line);
    int scan, nm, nf;
    double t, gospa, loc, missed, fals;
    ls >> scan >> t >> gospa >> loc >> missed >> fals >> nm >> nf;
    EXPECT_EQ(scan, expect_scan);
    return std::make_tuple(gospa, loc, missed, fals, nm, nf);
  };

  {
    auto [gospa, loc, missed, fals, nm, nf] = readRow(0);
    EXPECT_NEAR(gospa, d0, 1e-9);
    EXPECT_NEAR(loc, g0.localization, 1e-9);
    EXPECT_NEAR(missed, g0.missed, 1e-9);
    EXPECT_NEAR(fals, g0.false_, 1e-9);
    EXPECT_EQ(nm, g0.n_missed);
    EXPECT_EQ(nf, g0.n_false);
  }
  {
    auto [gospa, loc, missed, fals, nm, nf] = readRow(1);
    EXPECT_NEAR(gospa, gospaGreedy(truth1, est1, c), 1e-9);
    EXPECT_NEAR(loc, g1.localization, 1e-9);
    EXPECT_NEAR(missed, g1.missed, 1e-9);
    EXPECT_NEAR(fals, g1.false_, 1e-9);
    EXPECT_EQ(nm, g1.n_missed);
    EXPECT_EQ(nf, g1.n_false);
  }
}
