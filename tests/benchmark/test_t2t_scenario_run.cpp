// M3 two-tracker-view scenarios (ticket §6.2): 1 disjoint, 2 shared-AIS (the
// double-counting NEES gate), 3 unknown/absent pedigree, 7 determinism.
// Sim-gated: skips unless the sim_multisensor fixtures are reachable
// (SIMMS_DIR=<main tree>/tests/fixtures/sim_multisensor). Assertions are
// directional/banded (#24); the quantitative CI-vs-naive table lives in
// docs/baselines/2026-07-11_t2t_gates.md. These numbers are PRELIMINARY until
// the combined adversarial review runs clean on the M2 engine.

#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include <algorithm>
#include <cstdint>
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
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

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
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

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
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

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
  // #24: the coverage comparison is the intended ROBUST double-count gate
  // (coverage is outlier-insensitive), but a bare `>` between two adaptive
  // fractions is the sunset-6c shape. Require a real margin: measured CI cov95
  // ~0.901 vs naive ~0.826 (gap ~0.075), so +0.03 leaves ~0.045 headroom while
  // still failing if CI stops covering truth better than naive.
  EXPECT_GT(cn.coverage_95, nn.coverage_95 + 0.03)  // CI covers truth better (robust, w/ margin)
      << "CI did not cover truth materially better than naive: CI cov95="
      << cn.coverage_95 << " naive cov95=" << nn.coverage_95;
}

// Scenario 3: same data as (2, partial) but pedigrees all-Unknown, and one arm
// ABSENT. CI is pedigree-blind -> fused estimate byte-identical to explicit-
// Unknown; class stays PossiblyCorrelated.
TEST(T2tScenarioRun, UnknownAndAbsentPedigreeMatchExplicitUnknown) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

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
  std::set<IndependenceClass> absent_classes;
  const BenchResult f_absent =
      fuseTwoViews(*run, full, a_absent, b_absent, &ci, T2tConfig{}, nullptr, nullptr,
                   &absent_classes);

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
  // The absent run must resolve to the SAME independence verdict(s) as the
  // explicit all-Unknown run — asserted on the ABSENT run's own classes (the
  // position check above is pedigree-blind under CI, so it cannot guard this;
  // combined-review, m1-contracts-adapter lens).
  EXPECT_EQ(absent_classes, classes);
}

// Scenario 7: the fused output is deterministic on replay.
TEST(T2tScenarioRun, FusedOutputIsDeterministic) {
  auto run = findSim("sim_ms_crossing");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

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

// Scenario 4: t2t_dropout. B (AIS) goes silent 60 s mid-scenario, then resumes.
// EXPECT fused-track continuity (no id churn, no spurious second fused track on
// B's return) and covariance inflate-then-recover. Latency sub-case (B 2 s late
// throughout) reports the memoryless late-report BEHAVIOR explicitly.
TEST(T2tScenarioRun, DropoutContinuityAndLatencySkew) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

  double tmin = 1e18, tmax = -1e18;
  for (const auto& s : full.truth) {
    const double t = s.time.seconds();
    if (t < tmin) tmin = t;
    if (t > tmax) tmax = t;
  }
  const double mid = 0.5 * (tmin + tmax);
  const double drop_lo = mid - 30.0, drop_hi = mid + 30.0;  // 60 s silence

  CovarianceIntersectionRule ci;
  ArmSpec a{"radar", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais", {SensorKind::Ais}, usedStreams({"ais"})};

  // --- Dropout ---
  RecordingFusedSink sink;
  ArmPerturb pa, pb;
  pb.drop_from_s = drop_lo;
  pb.drop_to_s = drop_hi;
  const BenchResult fused =
      fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, pa, pb, &sink);
  ASSERT_FALSE(fused.steps.empty());

  std::set<std::uint64_t> all_ids, pre_ids, post_ids;
  double tr_in = 0, tr_out = 0;
  int n_in = 0, n_out = 0;
  for (const auto& step : fused.steps) {
    const double t = step.time.seconds();
    const bool in_drop = (t >= drop_lo && t < drop_hi);
    for (const auto& tk : step.tracks) {
      all_ids.insert(tk.id.value);
      const double tr = tk.pos_covariance.trace();
      if (in_drop) { tr_in += tr; ++n_in; } else { tr_out += tr; ++n_out; }
      if (t >= drop_lo - 5.0 && t < drop_lo) pre_ids.insert(tk.id.value);
      if (t >= drop_hi && t < drop_hi + 15.0) post_ids.insert(tk.id.value);
    }
  }
  const double mean_in = n_in ? tr_in / n_in : 0.0;
  const double mean_out = n_out ? tr_out / n_out : 0.0;
  int survived = 0;
  for (auto id : pre_ids)
    if (post_ids.count(id)) ++survived;

  // No-dropout baseline: the head-on's per-arm MHT already churns at CPA (the
  // known duplicate-track conveyor), so a raw fused-id count is not a clean
  // continuity metric. Compare against baseline to isolate DROPOUT-CAUSED churn.
  std::set<std::uint64_t> base_ids;
  const BenchResult base =
      fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, ArmPerturb{}, ArmPerturb{});
  for (const auto& step : base.steps)
    for (const auto& tk : step.tracks) base_ids.insert(tk.id.value);
  std::cout << "[t2t_dropout] inits=" << sink.count("init") << " deletes=" << sink.count("delete")
            << " distinct_ids=" << all_ids.size() << " (baseline no-drop=" << base_ids.size()
            << ") | cov-trace in-drop=" << mean_in << " out=" << mean_out
            << " | pre_ids=" << pre_ids.size() << " survived=" << survived << "\n";

  // Robust, churn-tolerant claims (#24): (a) covariance inflates while B is
  // silent (radar-only) then recovers when AIS returns; (b) a fused id spans the
  // whole dropout (target not lost); (c) the target is re-fused after B returns;
  // (d) the dropout does not ADD substantial fused-id churn beyond the baseline.
  // #24: covariance inflates while B (AIS) is silent, then recovers. Bare `>`
  // between two adaptive traces → require a real multiplicative margin. Losing
  // AIS for 60 s leaves radar-only covariance far larger (measured in-drop trace
  // ≫ out), so 1.2× is trivially cleared yet still fails if inflation stops.
  EXPECT_GT(mean_in, mean_out * 1.2)                                    // (a)
      << "covariance did not materially inflate during the AIS dropout: in="
      << mean_in << " out=" << mean_out;
  EXPECT_GE(survived, 1);                                               // (b)
  EXPECT_FALSE(post_ids.empty());                                       // (c)
  EXPECT_LE(static_cast<int>(all_ids.size()),
            static_cast<int>(base_ids.size()) + 2);                     // (d)

  // --- Latency skew sub-case: B reports 2 s late throughout (BEHAVIOR REPORT). ---
  ArmPerturb pb_late;
  pb_late.time_offset_s = 2.0;
  RecordingFusedSink sink_late;
  const BenchResult f_skew =
      fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, pa, pb_late, &sink_late);
  const BenchResult f_sync =
      fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, pa, ArmPerturb{});
  const auto g_skew = computeMetrics(f_skew, {});
  const auto g_sync = computeMetrics(f_sync, {});
  std::set<std::uint64_t> skew_ids, sync_ids;
  for (const auto& step : f_skew.steps)
    for (const auto& tk : step.tracks) skew_ids.insert(tk.id.value);
  for (const auto& step : f_sync.steps)
    for (const auto& tk : step.tracks) sync_ids.insert(tk.id.value);
  std::cout << "[t2t_latency 2s] BEHAVIOR: B's offset reports are ACCEPTED (per-source "
               "monotonic, NOT cross-source stale); the fuser advances now_ to the latest "
               "timestamp, predicts the fused estimate there, and fuses -> a bounded lag, no "
               "rejection, no id churn. gospa skew="
            << g_skew.gospa_mean << " sync=" << g_sync.gospa_mean
            << " inits=" << sink_late.count("init") << " distinct_ids skew=" << skew_ids.size()
            << " sync=" << sync_ids.size() << "\n";
  // Memoryless late-report handling is graceful: the skew does not fragment
  // tracks beyond the un-skewed baseline, and the lag degradation is bounded.
  EXPECT_LE(static_cast<int>(skew_ids.size()), static_cast<int>(sync_ids.size()) + 2);
  EXPECT_LE(g_skew.gospa_mean, g_sync.gospa_mean * 2.0 + 5.0);
}

// Scenario 5: t2t_conflict. B is deliberately biased (+150 m) and overconfident
// (claims 5 m sigma -> 25 m^2). CHARACTERIZATION (banded, no pass/fail heroics):
// how much does the CI fuse limit the damage vs the naive independence merge?
// Seeds the input-validation / de-weighting ways-to-improve entry.
TEST(T2tScenarioRun, ConflictBiasedOverconfidentSourceCharacterization) {
  auto run = findSim("sim_ms_headon");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

  CovarianceIntersectionRule ci;
  NaiveFusionRule naive;
  ArmSpec a{"radar", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais", {SensorKind::Ais}, usedStreams({"ais"})};
  ArmPerturb pa, pb;
  pb.pos_bias = Eigen::Vector2d(150.0, 0.0);  // +150 m east
  pb.cov_override_m2 = 25.0;                   // claims 5 m sigma (overconfident)

  const BenchResult f_ci = fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, pa, pb);
  const BenchResult f_nv = fuseTwoViewsPerturbed(*run, full, a, b, &naive, T2tConfig{}, pa, pb);
  const auto ci_g = computeMetrics(f_ci, {});
  const auto nv_g = computeMetrics(f_nv, {});
  const auto ci_n = computeNees(f_ci, kGate);
  const auto nv_n = computeNees(f_nv, kGate);
  std::cout << "[t2t_conflict] CI gospa=" << ci_g.gospa_mean << " NEES mean=" << ci_n.mean
            << " cov95=" << ci_n.coverage_95 << " || naive gospa=" << nv_g.gospa_mean
            << " NEES mean=" << nv_n.mean << " cov95=" << nv_n.coverage_95 << "\n";
  // Characterization only: naive trusts the tight-but-lying source more, so it is
  // at least as overconfident as CI (higher/equal NEES). CI does NOT fully
  // protect against a biased-confident source -> motivates input de-weighting
  // (ways-to-improve, algorithm-doc §4). Directional + generous; no exact pin.
  EXPECT_GT(ci_n.n, 0);
  // #24: the old 0.9 factor slacked in the WRONG direction — it passed even when
  // naive was up to 10% LESS overconfident than CI, i.e. it did not guard CI's
  // conservatism at all (the whole point of the characterization). The claimed
  // mechanism is naive ≥ CI (naive trusts the tight-lying source more), so assert
  // that direction; a regression making CI as/more overconfident than naive fails.
  EXPECT_GE(nv_n.mean, ci_n.mean)
      << "naive must be at least as overconfident as CI on a biased-confident "
         "source; naive NEES=" << nv_n.mean << " CI NEES=" << ci_n.mean;
}

// Scenario 6: t2t_cross. Two crossing targets, per-vessel MMSI on both arms.
// A/B on the MMSI-conflict penalty: it must not INCREASE wrong pairings (fused
// id_switches). Invariant 5 (external id is NEVER the fusion key) is asserted
// here in scenario context AND at the unit level
// (T2tAssociator.ConflictingMmsiStillAssociatesWhenKinematicsAgree).
TEST(T2tScenarioRun, CrossMmsiConflictReducesWrongPairings) {
  auto run = findSim("sim_ms_crossing");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

  CovarianceIntersectionRule ci;
  ArmSpec a{"radar", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais", {SensorKind::Ais}, usedStreams({"ais"})};
  ArmPerturb pa, pb;
  pa.mmsi_from_truth = true;
  pb.mmsi_from_truth = true;

  T2tConfig on;  // default conflicting_mmsi_cost_penalty=6.0, shared bonus=2.0
  T2tConfig off;
  off.conflicting_mmsi_cost_penalty = 0.0;
  off.shared_mmsi_cost_bonus = 0.0;
  const BenchResult f_on = fuseTwoViewsPerturbed(*run, full, a, b, &ci, on, pa, pb);
  const BenchResult f_off = fuseTwoViewsPerturbed(*run, full, a, b, &ci, off, pa, pb);
  const auto g_on = computeMetrics(f_on, {});
  const auto g_off = computeMetrics(f_off, {});

  int max_tracks = 0;
  for (const auto& step : f_on.steps)
    max_tracks = std::max(max_tracks, static_cast<int>(step.tracks.size()));
  std::cout << "[t2t_cross] CHARACTERIZATION (report-only; #24 forbids gating marginal "
               "association outcomes): MMSI penalty ON id_switches="
            << g_on.id_switches << " gospa=" << g_on.gospa_mean
            << " || OFF id_switches=" << g_off.id_switches << " gospa=" << g_off.gospa_mean
            << " | max_confirmed_tracks(ON)=" << max_tracks
            << ". On this churn-dominated crossing sim the penalty is within noise "
               "(id_switches driven by inherited per-arm lifecycle, not cross-pairing; MMSI is "
               "ambiguous exactly at the co-located crossing). The penalty's clean value is the "
               "controlled associator A/B "
               "(T2tAssociator.MmsiConflictPenaltyBreaksKinematicTieToCorrectPairing) and its "
               "soft-cost correctness is unit-level.\n";
  // HARD assert only the robust invariant (not the marginal A/B, per #24):
  // invariant 5 — external identity is never the fusion key, so MMSI conflicts
  // at the crossing must NOT prevent fusion when kinematics agree; both crossing
  // targets are still fused.
  EXPECT_GE(max_tracks, 2);
}

// b24-2 (T2T invariant 5, END-TO-END). max_tracks>=2 guards the WRONG direction:
// a conflict wrongly PREVENTING fusion SPLITS each vessel into separate radar/ais
// tracks -> MORE tracks, which a lower bound cannot catch. With the per-fused-
// track FUSER arm-id bookkeeping (b24-2) we can gate the real invariant: both
// targets are GENUINELY FUSED (each fed by BOTH arms), not split. Setup: two
// crossing targets; radar tags per-vessel MMSI, AIS reports one SPOOFED constant
// MMSI -> every cross-arm pair carries an MMSI CONFLICT. The soft penalty is
// uniform (all pairs conflict) so it is a no-op and kinematics fuses correctly.
// If external id were the fusion KEY (teeth: forbid conflicting-MMSI pairs), the
// conflict would prevent fusion and both vessels would split into single-arm
// tracks -> the two-arm gate below goes red.
TEST(T2tScenarioRun, CrossSpoofedMmsiStillFusesBothTargetsEndToEnd) {
  auto run = findSim("sim_ms_crossing");
  ASSERT_TRUE(run);
  const auto full = run->generate(0);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      full.measurements.empty(),
      "sim_multisensor fixtures unreachable (set SIMMS_DIR)");

  CovarianceIntersectionRule ci;
  ArmSpec a{"radar", {SensorKind::ArpaTtm}, usedStreams({"radar"})};
  ArmSpec b{"ais", {SensorKind::Ais}, usedStreams({"ais"})};
  ArmPerturb pa, pb;
  pa.mmsi_from_truth = true;    // radar: consistent per-vessel MMSI (A, B)
  pb.force_mmsi = 999000999u;   // AIS: one spoofed MMSI on BOTH targets -> conflict

  const BenchResult fused =
      fuseTwoViewsPerturbed(*run, full, a, b, &ci, T2tConfig{}, pa, pb);
  ASSERT_FALSE(fused.steps.empty());

  auto twoArm = [](const benchmark::TrackStateSnapshot& s) {
    bool r = false, ai = false;
    for (const auto& arm : s.contributing_fuser_arm_ids) {
      if (arm == "radar") r = true;
      if (arm == "ais") ai = true;
    }
    return r && ai;
  };
  int steps_with_2_fused = 0, max_two_arm = 0, steps_with_any_two_arm = 0;
  for (const auto& step : fused.steps) {
    int n2 = 0;
    for (const auto& tk : step.tracks)
      if (twoArm(tk)) ++n2;
    max_two_arm = std::max(max_two_arm, n2);
    if (n2 >= 2) ++steps_with_2_fused;
    if (n2 >= 1) ++steps_with_any_two_arm;
  }
  std::cerr << "[t2t_cross_spoof] steps=" << fused.steps.size()
            << " steps_with_>=2_two-arm-fused=" << steps_with_2_fused
            << " steps_with_>=1=" << steps_with_any_two_arm
            << " max_two_arm=" << max_two_arm << "\n";
  // Invariant 5, end-to-end: both crossing targets are GENUINELY fused (each fed
  // by radar AND ais) simultaneously (max_two_arm>=2), and that holds on a
  // sustained fraction of the run (measured 261/598 ≈ 44%; floor 100 ≈ 17% keeps
  // >2.5x headroom under the fixed seed's per-arm churn). If external id were the
  // fusion KEY, the spoofed-MMSI conflict would PREVENT fusion and each vessel
  // would split into single-arm tracks -> zero two-arm-fused tracks -> both red.
  // (The pre-b24-2 gate `max_tracks>=2` could not see this: a split raises the
  // track count, satisfying a lower bound.)
  EXPECT_GE(max_two_arm, 2)
      << "the two crossing targets were never BOTH genuinely fused (each fed by "
         "radar+ais) at one step — spoofed MMSI prevented fusion (invariant 5)";
  EXPECT_GT(steps_with_2_fused, 100)
      << "genuine 2-target 2-arm fusion was not sustained (measured ~261): "
      << steps_with_2_fused;
}

}  // namespace
}  // namespace navtracker::t2t::bench
