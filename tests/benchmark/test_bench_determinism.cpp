#include <gtest/gtest.h>

#include <functional>
#include <iomanip>
#include <sstream>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
// Wall-clock performance measurements are not part of the tracker's
// deterministic output; they legitimately vary run-to-run and must be
// excluded from a byte-identical determinism check. This covers
// "wall_seconds" and the per-scan latency rows added in perf round 2
// (scan_proc_ms_*). NOTE: scan_interval_s and n_scans are DATA-derived
// (scan timestamps / scan count), hence deterministic, so they stay in the
// hash — the check then also guards their determinism.
bool isWallClockMetric(const std::string& m) {
  return m == "wall_seconds" || m.rfind("scan_proc_ms", 0) == 0;
}
std::size_t hashRows(const std::vector<MetricRow>& rows) {
  std::ostringstream os;
  // #24 (W3 assertion-quality#4): stream at full round-trip precision (17 sig
  // figs for double). The old default ostringstream format (~6 sig figs) hashed
  // a sub-6th-figure non-determinism (e.g. FP reduction-order drift from
  // unordered-container iteration in an OSPA/GOSPA/NEES sum) as IDENTICAL, so
  // the byte-identical-output invariant (CLAUDE.md #4) was only checked to 6
  // digits. Same-binary repeated runs are bit-identical when truly deterministic.
  os << std::setprecision(17);
  for (const auto& r : rows) {
    if (isWallClockMetric(r.metric)) continue;
    os << r.run_id << ',' << r.config << ',' << r.scenario << ','
       << r.seed << ',' << r.metric << ',' << r.value << ',' << r.unit << '\n';
  }
  return std::hash<std::string>{}(os.str());
}
}  // namespace

TEST(BenchDeterminism, RepeatedSweepProducesIdenticalRows) {
  // #24 (W3 required-scenarios#3): the old test hashed only configs.front()
  // (imm_cv_ct_mht — the in-source comment "ekf_cv_gnn" is stale), leaving the
  // headline PMBM tracker's determinism guarded elsewhere only by two aggregate
  // scalars. Also cover the PMBM path here so byte-identical output is checked
  // for both the MHT and PMBM pipelines.
  auto configs = defaultConfigs();
  std::vector<Config> c1;
  for (const auto& c : configs)
    if (c.label == "imm_cv_ct_mht" || c.label == "imm_cv_ct_pmbm") c1.push_back(c);
  ASSERT_EQ(c1.size(), 2u)
      << "expected imm_cv_ct_mht + imm_cv_ct_pmbm in defaultConfigs()";

  SweepParams p;
  p.run_id = "det";
  p.synthetic_seeds = 2;

  // Fresh scenario vectors per call — generate() may mutate internal RNG
  // state, and the vector itself is moved-from on first use.
  auto scenarios_a = defaultSimScenarios();
  std::vector<std::unique_ptr<ScenarioRun>> s1;
  s1.push_back(std::move(scenarios_a.front()));
  const auto rows1 = runSweep(c1, s1, p);

  auto scenarios_b = defaultSimScenarios();
  std::vector<std::unique_ptr<ScenarioRun>> s2;
  s2.push_back(std::move(scenarios_b.front()));
  const auto rows2 = runSweep(c1, s2, p);

  ASSERT_EQ(rows1.size(), rows2.size())
      << "Row counts differ — non-deterministic structure.";
  EXPECT_EQ(hashRows(rows1), hashRows(rows2))
      << "Row contents differ — non-deterministic values.";

  // If hash equality fails, print the differing rows for diagnosis.
  if (hashRows(rows1) != hashRows(rows2)) {
    for (std::size_t i = 0; i < rows1.size(); ++i) {
      if (isWallClockMetric(rows1[i].metric)) continue;
      if (rows1[i].value != rows2[i].value) {
        std::cerr << "First differing row at index " << i << ": "
                  << rows1[i].config << " " << rows1[i].scenario << " "
                  << rows1[i].seed << " " << rows1[i].metric << " : "
                  << rows1[i].value << " vs " << rows2[i].value << "\n";
        break;
      }
    }
  }
}
