// M3 two-tracker-view scenarios (ticket §6.2): 1 disjoint, 2 shared-AIS (the
// double-counting NEES gate), 3 unknown/absent pedigree, 7 determinism.
// Sim-gated: skips unless the sim_multisensor fixtures are reachable
// (SIMMS_DIR=<main tree>/tests/fixtures/sim_multisensor). Assertions are
// directional/banded (#24); the quantitative CI-vs-naive table lives in
// docs/baselines/2026-07-11_t2t_gates.md. These numbers are PRELIMINARY until
// the combined adversarial review runs clean on the M2 engine.

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <set>
#include <string>

#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
#include "core/benchmark/Consistency.hpp"
#include "core/benchmark/Metrics.hpp"
#include "tests/benchmark/T2tViewHarness.hpp"

namespace navtracker::t2t::bench {
namespace {

using navtracker::benchmark::BenchResult;
using navtracker::benchmark::computeMetrics;
using navtracker::benchmark::computeNees;

std::unique_ptr<navtracker::benchmark::ScenarioRun> findSim(const std::string& label) {
  for (auto& s : navtracker::benchmark::defaultSimMultisensorScenarios())
    if (s->descriptor().label == label) return std::move(s);
  return nullptr;
}

constexpr double kGate = 100.0;  // assoc gate for scoring (m), bench default

bool has(const std::set<IndependenceClass>& s, IndependenceClass c) {
  return s.count(c) != 0;
}

// Scenario 1: disjoint sensors -> provably independent. Fusion consistent and
// no worse than the better single input.
TEST(T2tScenarioRun, DisjointSourcesFuseConsistentlyAndNoWorseThanBest) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  if (full.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  ArmSpec a{"radar_tracker", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais_tracker", {SensorKind::Ais}, usedStreams({"ais"})};
  CovarianceIntersectionRule ci;
  BenchResult ra, rb;
  std::set<IndependenceClass> classes;
  const BenchResult fused =
      fuseTwoViews(*run, full, a, b, &ci, T2tConfig{}, &ra, &rb, &classes);
  ASSERT_FALSE(fused.steps.empty());

  const double g_fused = computeMetrics(fused, {}).gospa_mean;
  const double g_a = computeMetrics(ra, {}).gospa_mean;
  const double g_b = computeMetrics(rb, {}).gospa_mean;
  const auto nees = computeNees(fused, kGate);
  std::cout << "[t2t_disjoint] gospa fused=" << g_fused << " radar=" << g_a
            << " ais=" << g_b << " | NEES n=" << nees.n << " mean=" << nees.mean
            << " median=" << nees.median << " cov95=" << nees.coverage_95 << "\n";

  EXPECT_GT(nees.n, 0);
  EXPECT_LT(nees.median, 6.0);  // 2-DoF consistent (~1.4 median); generous band
  EXPECT_LE(g_fused, std::min(g_a, g_b) * 1.25);  // no worse than best input
  // The cross-arm (radar+ais) fusion is correctly ProvablyIndependent. (Note:
  // PossiblyCorrelated may ALSO appear when one arm momentarily has two of its
  // own tracks for a vessel and both co-fuse — those share that arm's stream,
  // so the verdict is correctly PossiblyCorrelated; we do not forbid it.)
  EXPECT_TRUE(has(classes, IndependenceClass::ProvablyIndependent));
}

// Scenario 2: THE double-counting NEES gate. Two trackers that BOTH fused the
// same AIS stream (maximal sharing) — the literal hazard. CI-fused stays
// consistent; the naive independence-assuming baseline is materially
// overconfident (higher NEES). A radar-diluted partial-sharing arm is also
// measured (radar dominates A there, so the effect is small — reported, not
// gated). Numbers -> docs/baselines/2026-07-11_t2t_gates.md.
TEST(T2tScenarioRun, SharedAisDoubleCountingNeesGate) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  if (full.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  CovarianceIntersectionRule ci;
  NaiveFusionRule naive;

  // --- Maximal sharing: both arms consume ONLY the shared AIS stream. ---
  ArmSpec a_ais{"a_ais", {SensorKind::Ais}, usedStreams({"ais"})};
  ArmSpec b_ais{"b_ais", {SensorKind::Ais}, usedStreams({"ais"})};
  std::set<IndependenceClass> classes;
  const BenchResult max_ci =
      fuseTwoViews(*run, full, a_ais, b_ais, &ci, T2tConfig{}, nullptr, nullptr, &classes);
  const BenchResult max_naive = fuseTwoViews(*run, full, a_ais, b_ais, &naive, T2tConfig{});
  ASSERT_FALSE(max_ci.steps.empty());
  const auto ci_n = computeNees(max_ci, kGate);
  const auto nv_n = computeNees(max_naive, kGate);
  std::cout << "[t2t_shared_ais MAXIMAL] CI    NEES n=" << ci_n.n << " mean=" << ci_n.mean
            << " median=" << ci_n.median << " cov95=" << ci_n.coverage_95 << "\n";
  std::cout << "[t2t_shared_ais MAXIMAL] naive NEES n=" << nv_n.n << " mean=" << nv_n.mean
            << " median=" << nv_n.median << " cov95=" << nv_n.coverage_95
            << " | ratio mean=" << (nv_n.mean / ci_n.mean)
            << " median=" << (nv_n.median / ci_n.median) << "\n";

  // --- Realistic partial sharing (radar dilutes A): report only. ---
  ArmSpec a_ra{"a_ra", {SensorKind::ArpaTtm, SensorKind::Ais}, usedStreams({"radar", "ais"})};
  ArmSpec b_a{"b_a", {SensorKind::Ais}, usedStreams({"ais"})};
  const BenchResult p_ci = fuseTwoViews(*run, full, a_ra, b_a, &ci, T2tConfig{});
  const BenchResult p_naive = fuseTwoViews(*run, full, a_ra, b_a, &naive, T2tConfig{});
  const auto p_ci_n = computeNees(p_ci, kGate);
  const auto p_nv_n = computeNees(p_naive, kGate);
  std::cout << "[t2t_shared_ais PARTIAL] CI median=" << p_ci_n.median
            << " naive median=" << p_nv_n.median
            << " | ratio median=" << (p_nv_n.median / p_ci_n.median) << "\n";

  // Gate on the maximal case. CI stays consistent; naive is materially more
  // overconfident (mean NEES ratio ~1.7x measured, near the theoretical 2x for
  // a duplicated estimate — the mean captures the effect where both arms
  // genuinely co-fuse; transient single-contributor scans dilute the median).
  EXPECT_GT(ci_n.n, 0);
  EXPECT_LT(ci_n.median, 4.0);                   // CI stays consistent
  EXPECT_GT(nv_n.mean, ci_n.mean * 1.4);         // naive double-counts (generous)
  EXPECT_TRUE(has(classes, IndependenceClass::PossiblyCorrelated));
}

// Point 2 (ticket §10 ruling): per-arm NEES calibration sweep. At the loader's
// 30 m AIS default each arm is under-confident (NEES << 2), so the double-count
// shows only as a ratio, not a χ² band violation. Sweep the AIS σ (test-local,
// via the harness lever) to find where each arm's OWN NEES ≈ 2; at that σ the
// maximal-sharing double-count should push NAIVE out of the band while CI stays
// in. Characterization (prints -> gates doc). No generator/fixture change.
TEST(T2tScenarioRun, PerArmNeesCalibrationAndBandViolationGate) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  if (full.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  CovarianceIntersectionRule ci;
  NaiveFusionRule naive;
  ArmSpec a_ais{"a_ais", {SensorKind::Ais}, usedStreams({"ais"})};
  ArmSpec b_ais{"b_ais", {SensorKind::Ais}, usedStreams({"ais"})};

  // Characterization sweep (prints -> docs/baselines/2026-07-11_t2t_gates.md).
  for (double sig : {30.0, 16.0, 14.0, 12.0, 11.0, 10.0, 9.0, 8.0}) {
    const BenchResult arm = runArm(*run, armView(full, a_ais.sensors), sig);
    const auto an = computeNees(arm, kGate);
    const BenchResult mc = fuseTwoViews(*run, full, a_ais, b_ais, &ci, T2tConfig{},
                                        nullptr, nullptr, nullptr, sig);
    const BenchResult mn = fuseTwoViews(*run, full, a_ais, b_ais, &naive, T2tConfig{},
                                        nullptr, nullptr, nullptr, sig);
    const auto cn = computeNees(mc, kGate);
    const auto nn = computeNees(mn, kGate);
    std::cout << "[calib sigma=" << sig << "] perarm NEES mean=" << an.mean
              << " median=" << an.median << " cov95=" << an.coverage_95
              << " || CI mean=" << cn.mean << " band=[" << cn.band_lo << ","
              << cn.band_hi << "] cov95=" << cn.coverage_95
              << " || naive mean=" << nn.mean << " cov95=" << nn.coverage_95
              << " | ratio=" << (nn.mean / cn.mean) << "\n";
  }

  // Calibrated gate. σ = 12 m is where the AIS arm's OWN mean NEES ≈ 2 (the
  // arbiter's literal target). At that calibration the double-count becomes a
  // real χ² BAND VIOLATION, not just a ratio: CI mean stays inside the band,
  // naive mean breaches band_hi. NOTE (honest caveat): the fused-NEES
  // distribution is heavy-tailed here (median ≈ 0.5 ≪ mean ≈ 2), so the
  // mean-in-band result is tail-driven and fragile. The robust, calibration-
  // INVARIANT evidence is what we hard-assert: (a) naive ≥ 1.4× CI at the mean,
  // (b) naive breaches band_hi, (c) CI covers truth strictly better than naive
  // (coverage is outlier-insensitive). See the gates doc for the full sweep and
  // the σ=16 coverage-calibrated cross-check.
  constexpr double kCalibratedAisSigma = 12.0;
  const BenchResult perarm =
      runArm(*run, armView(full, a_ais.sensors), kCalibratedAisSigma);
  const auto pa = computeNees(perarm, kGate);
  const BenchResult cal_ci = fuseTwoViews(*run, full, a_ais, b_ais, &ci, T2tConfig{},
                                          nullptr, nullptr, nullptr, kCalibratedAisSigma);
  const BenchResult cal_nv = fuseTwoViews(*run, full, a_ais, b_ais, &naive, T2tConfig{},
                                          nullptr, nullptr, nullptr, kCalibratedAisSigma);
  const auto cn = computeNees(cal_ci, kGate);
  const auto nn = computeNees(cal_nv, kGate);

  EXPECT_GT(pa.mean, 1.5);                    // calibration moved per-arm NEES to ≈2
  EXPECT_LT(pa.mean, 3.0);                    // (was 0.42 at the 30 m default)
  EXPECT_GT(nn.mean, cn.mean * 1.4);          // double-count (calibration-invariant)
  EXPECT_GT(nn.mean, cn.band_hi);             // naive BREACHES the χ² band
  EXPECT_GT(cn.coverage_95, nn.coverage_95);  // CI covers truth better (robust)
}

// Scenario 3: same data as (2, partial) but pedigrees all-Unknown, and one arm
// ABSENT. CI is pedigree-blind -> fused estimate byte-identical to explicit-
// Unknown; class stays PossiblyCorrelated.
TEST(T2tScenarioRun, UnknownAndAbsentPedigreeMatchExplicitUnknown) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  if (full.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  const std::set<SensorKind> a_sensors{SensorKind::Ais};
  const std::set<SensorKind> b_sensors{SensorKind::Ais};
  CovarianceIntersectionRule ci;

  ArmSpec a_unknown{"a", a_sensors, allUnknown()};
  ArmSpec b_unknown{"b", b_sensors, allUnknown()};
  std::set<IndependenceClass> classes;
  const BenchResult f_unknown = fuseTwoViews(*run, full, a_unknown, b_unknown, &ci,
                                             T2tConfig{}, nullptr, nullptr, &classes);

  ArmSpec a_absent{"a", a_sensors, std::nullopt};  // pedigree ABSENT
  ArmSpec b_absent{"b", b_sensors, allUnknown()};
  const BenchResult f_absent = fuseTwoViews(*run, full, a_absent, b_absent, &ci, T2tConfig{});

  ASSERT_EQ(f_unknown.steps.size(), f_absent.steps.size());
  for (std::size_t k = 0; k < f_unknown.steps.size(); ++k) {
    ASSERT_EQ(f_unknown.steps[k].tracks.size(), f_absent.steps[k].tracks.size());
    for (std::size_t i = 0; i < f_unknown.steps[k].tracks.size(); ++i) {
      EXPECT_EQ(f_unknown.steps[k].tracks[i].id.value,
                f_absent.steps[k].tracks[i].id.value);
      EXPECT_EQ(f_unknown.steps[k].tracks[i].position,
                f_absent.steps[k].tracks[i].position);  // byte-identical
    }
  }
  EXPECT_TRUE(has(classes, IndependenceClass::PossiblyCorrelated));
}

// Scenario 7: the fused output is deterministic on replay.
TEST(T2tScenarioRun, FusedOutputIsDeterministic) {
  auto run = findSim("sim_ms_crossing");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  if (full.measurements.empty())
    GTEST_SKIP() << "sim_multisensor fixtures unreachable (set SIMMS_DIR)";

  ArmSpec a{"radar", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais", {SensorKind::Ais}, usedStreams({"ais"})};
  CovarianceIntersectionRule ci;
  const BenchResult x = fuseTwoViews(*run, full, a, b, &ci, T2tConfig{});
  const BenchResult y = fuseTwoViews(*run, full, a, b, &ci, T2tConfig{});

  ASSERT_EQ(x.steps.size(), y.steps.size());
  for (std::size_t k = 0; k < x.steps.size(); ++k) {
    ASSERT_EQ(x.steps[k].tracks.size(), y.steps[k].tracks.size());
    for (std::size_t i = 0; i < x.steps[k].tracks.size(); ++i) {
      EXPECT_EQ(x.steps[k].tracks[i].id.value, y.steps[k].tracks[i].id.value);
      EXPECT_EQ(x.steps[k].tracks[i].position, y.steps[k].tracks[i].position);
    }
  }
}

}  // namespace
}  // namespace navtracker::t2t::bench
