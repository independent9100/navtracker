// Stage 1b-ii increment 6c — coverage-aware vs universal decay, validated on
// real philos clips. The mechanism (6a model + 6b producer + multi-cluster
// guard) is unit-tested in isolation; this is its ON-REAL-DATA payoff
// measurement, the A/B the 2026-07-03 steer named. FINDINGS in the eval-log
// (2026-07-03); the assertions here lock the mechanism's INVARIANTS, not the
// (real-data, non-contract) A/B numbers.
//
//   * sunset_cruise — the sunset label optimistically predicted coverage-aware
//     would "resolve the loiterer (rank202, returns cease t~94) as a departed
//     vessel". The measurement DISPROVED that: after t94 the loiterer's cell is
//     swept 0/283 scans, so radar has NO observed-empty evidence to decay on —
//     coverage-aware CORRECTLY holds it as a conserved hazard (its departure is a
//     camera fact, invisible to radar). That is the mechanism's contract, not a
//     bug (an observability probe is the discriminator). The real, defensible
//     win is STRUCTURE PRESENCE: coverage-aware holds off-beam structure that
//     universal forgets, and — by construction (it decays a subset of universal's
//     cells) — never holds LESS presence than universal over any region.
//   * close_approach — KEEP_MIXED presence under the suppressor: presence
//     (track OR a hazard's keep-clear ring, the ADR-0002 conservation test) must
//     not fall below the land baseline (no object suppressed into nothing).
//
// These are measurement tests: they print the timelines and assert only the
// mechanism's structural invariants — the sector gate actually bites; presence
// is monotone ≥ universal; a pinned-but-unswept cell is correct not buggy;
// the suppressor never drops KEEP_MIXED presence below baseline.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/benchmark/ExistenceLabel.hpp"
#include "tests/replay/PhilosLabelReplay.hpp"

namespace navtracker {
namespace {

using benchmark::ExistenceLabel;
using benchmark::ExistenceLabelClass;
using replay_test::ClipRun;
using replay_test::HazardSnapshot;
using replay_test::labelEnu;

constexpr double kRad2Deg = 57.29577951308232;

// Median of a copy (empty → 0).
double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

// Does any emitted hazard sit within `radius_m` of `center` on this scan?
// (Same co-location convention used to score a track against a region.)
bool hazardAt(const replay_test::ScanTracks& scan, const Eigen::Vector2d& center,
              double radius_m) {
  for (const HazardSnapshot& h : scan.hazards)
    if ((h.center - center).norm() <= radius_m) return true;
  return false;
}
bool trackAt(const replay_test::ScanTracks& scan, const Eigen::Vector2d& center,
             double radius_m) {
  for (const auto& tr : scan.tracks)
    if ((tr.pos - center).norm() <= radius_m) return true;
  return false;
}
// ADR-0002 conservation-correct hazard presence: is `center` inside some
// emitted hazard's KEEP-CLEAR ring? (A suppressed birth is guaranteed to be —
// conservation by construction — even when the hazard centre is not co-located,
// e.g. a large astern structure whose ring reaches the point.) This is the
// operator-facing "a keep-clear zone represents this location" test, distinct
// from hazardAt()'s "a structure is pinned ON this cell" co-location test.
bool hazardKeepClearAt(const replay_test::ScanTracks& scan,
                       const Eigen::Vector2d& center) {
  for (const HazardSnapshot& h : scan.hazards)
    if ((h.center - center).norm() <= h.keep_clear_m) return true;
  return false;
}

// Any CHART-CORROBORATED hazard within `radius_m` of `center` this scan?
bool corroboratedHazardAt(const replay_test::ScanTracks& scan,
                          const Eigen::Vector2d& center, double radius_m) {
  for (const HazardSnapshot& h : scan.hazards)
    if (h.corroborated && (h.center - center).norm() <= radius_m) return true;
  return false;
}
// Any CAMERA-OBSERVED-EMPTY hazard within `radius_m` of `center` this scan?
bool cameraEmptyHazardAt(const replay_test::ScanTracks& scan,
                         const Eigen::Vector2d& center, double radius_m) {
  for (const HazardSnapshot& h : scan.hazards)
    if (h.camera_empty && (h.center - center).norm() <= radius_m) return true;
  return false;
}

void reportSectors(const ClipRun& run, const std::string& tag) {
  std::vector<double> w = run.sector_widths_rad;
  double mn = 1e18, mx = 0.0;
  for (double x : w) {
    mn = std::min(mn, x);
    mx = std::max(mx, x);
  }
  std::cout << "  [" << tag << "] valid sectors=" << w.size()
            << " full_circle=" << run.sector_full_circle
            << " width_deg: min=" << (w.empty() ? 0.0 : mn * kRad2Deg)
            << " median=" << median(w) * kRad2Deg
            << " max=" << mx * kRad2Deg << "\n";
}

void reportHazardTimeline(const ClipRun& run, const std::string& tag) {
  std::size_t mn = 1000000, mx = 0, final = 0;
  for (const auto& s : run.history) {
    mn = std::min(mn, s.hazards.size());
    mx = std::max(mx, s.hazards.size());
  }
  if (!run.history.empty()) final = run.history.back().hazards.size();
  std::cout << "  [" << tag << "] hazards/scan: min=" << mn << " max=" << mx
            << " final=" << final << "\n";
}

// Last RELATIVE time at which a hazard covers `center`; -1 if never. Used to see
// whether structure pinned on the loiterer decays OUT after returns cease.
double lastHazardCoverRel(const ClipRun& run, const Eigen::Vector2d& center,
                          double radius_m) {
  double last = -1.0;
  for (const auto& s : run.history)
    if (hazardAt(s, center, radius_m))
      last = std::max(last, s.t_unix - run.clip_start_unix);
  return last;
}
// Number of scans a hazard covers `center` (structure-stability surface).
long hazardCoverScans(const ClipRun& run, const Eigen::Vector2d& center,
                      double radius_m) {
  long n = 0;
  for (const auto& s : run.history)
    if (hazardAt(s, center, radius_m)) ++n;
  return n;
}

// OBSERVABILITY of `center` in rel-time window [lo,hi): scans where some
// coverage sector covers it, out of total scans in the window. The bug-vs-
// correct discriminator: a cell pinned as a hazard while OFTEN observable-empty
// would be a decay bug; a cell pinned while RARELY observable is correct
// protection of an unswept cell (radar cannot see it depart).
std::pair<long, long> observability(const ClipRun& run,
                                    const Eigen::Vector2d& center, double lo,
                                    double hi) {
  long obs = 0, total = 0;
  for (const auto& ss : run.sector_history) {
    const double rel = ss.t_unix - run.clip_start_unix;
    if (rel < lo || rel >= hi) continue;
    ++total;
    for (const auto& sec : ss.sectors)
      if (sec.covers(center)) {
        ++obs;
        break;
      }
  }
  return {obs, total};
}

}  // namespace

// ── sunset_cruise: loiterer decay-out (recovery) + structure stability A/B ──
TEST(PhilosCoverageDecay6c, SunsetCoverageAwareHoldsStructureAndProtectsUnsweptCells) {
  const ClipRun uni =
      replay_test::runClip("sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector");
  if (!uni.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const ClipRun cov = replay_test::runClip(
      "sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector_coverage");
  ASSERT_TRUE(cov.valid);
  const auto labels = replay_test::loadLabels("sunset_cruise_labels.csv");
  ASSERT_FALSE(labels.empty());

  std::cout << "\n=== 6c sunset_cruise: universal vs coverage-aware decay ===\n";
  reportSectors(uni, "universal");
  reportSectors(cov, "coverage ");
  reportHazardTimeline(uni, "universal");
  reportHazardTimeline(cov, "coverage ");

  // Locate labels of interest.
  const ExistenceLabel* loit = nullptr;
  std::vector<const ExistenceLabel*> structure;
  for (const auto& l : labels) {
    if (l.region_id == "loiterer_v2") loit = &l;
    if (l.label == ExistenceLabelClass::SuppressStructure) structure.push_back(&l);
  }
  ASSERT_NE(loit, nullptr);

  const Eigen::Vector2d lc = labelEnu(uni.datum, *loit);
  std::cout << "\n  loiterer_v2 (r=" << loit->radius_m
            << "m, window t=" << loit->t_start_s << ".." << loit->t_end_s
            << "): last scan with a hazard on it —\n"
            << "    universal: " << lastHazardCoverRel(uni, lc, loit->radius_m)
            << " s   coverage: " << lastHazardCoverRel(cov, lc, loit->radius_m)
            << " s   (want: decays out AFTER returns cease ~94, BEFORE clip end)\n";
  // Bug-vs-correct: after returns cease (~94), is the loiterer cell actually
  // being swept? If rarely observable, coverage-aware is CORRECTLY protecting an
  // unswept cell (radar can't see it depart); if often observable-and-pinned,
  // the decay gate has a hole.
  const auto obs_after = observability(cov, lc, 94.0, 1e9);
  const auto obs_before = observability(cov, lc, loit->t_start_s, 94.0);
  std::cout << "    loiterer cell observability (coverage arm): "
            << "before t94 " << obs_before.first << "/" << obs_before.second
            << " scans swept, after t94 " << obs_after.first << "/"
            << obs_after.second << " scans swept\n";

  std::cout << "\n  structure-hazard stability (scans a hazard covers each "
               "SUPPRESS region):\n";
  long uni_astern = 0, cov_astern = 0;
  for (const auto* s : structure) {
    const Eigen::Vector2d sc = labelEnu(uni.datum, *s);
    const long u = hazardCoverScans(uni, sc, s->radius_m);
    const long v = hazardCoverScans(cov, sc, s->radius_m);
    std::cout << "    " << s->region_id << " (r=" << s->radius_m
              << "m): universal=" << u << "  coverage=" << v << "\n";
    if (s->region_id == "astern_blob") {
      uni_astern = u;
      cov_astern = v;
    }
  }
  std::cout << std::flush;

  // ── Load-bearing assertions (the A/B numbers themselves stay in the eval-log,
  //    not frozen as thresholds; these are the mechanism's INVARIANTS) ──

  // (1) The sector mechanism actually bites on philos — sectors are sub-circle
  //     (median ≈ 3° swept + the 2×5° az pad), never collapsing to full circle.
  EXPECT_FALSE(cov.sector_widths_rad.empty())
      << "coverage arm estimated no sectors — producer not wired?";
  EXPECT_EQ(cov.sector_full_circle, 0)
      << "some bursts collapsed to full circle → the coverage gate goes inert";
  EXPECT_LT(median(cov.sector_widths_rad) * kRad2Deg, 90.0)
      << "estimated sectors are not sub-circle → the coverage gate is inert";

  // (2) MONOTONICITY (the structural property): coverage-aware decay forgets a
  //     cell only when it is observed empty — a SUBSET of the scans universal
  //     decays it — so per-cell persistence is pointwise ≥ universal, hence the
  //     emitted hazard set is a superset and presence is held AT LEAST as long
  //     over every labelled region. Coverage-aware can never LOSE presence vs
  //     universal. Checked across all labels.
  for (const auto& l : labels) {
    const Eigen::Vector2d c = labelEnu(uni.datum, l);
    EXPECT_GE(hazardCoverScans(cov, c, l.radius_m),
              hazardCoverScans(uni, c, l.radius_m))
        << "coverage-aware held region " << l.region_id
        << " FEWER scans than universal — monotonicity violated";
  }

  // (3) It STRICTLY improves real off-beam structure presence: astern_blob is a
  //     large real structure out of camera FOV, rarely swept as own-ship departs
  //     → universal wrongly forgets it, coverage-aware protects it.
  EXPECT_GT(cov_astern, uni_astern)
      << "coverage-aware did not improve astern_blob structure presence";

  // (4) The loiterer pin under coverage-aware is CORRECT protection, not a decay
  //     bug: after returns cease (~t94) its cell is essentially never swept, so
  //     there is no observed-empty evidence to decay on. (Its departure is a
  //     CAMERA fact, invisible to radar — the motivation for AIS/chart
  //     corroboration next.) A hole in the gate would show it swept-and-pinned.
  const auto post = observability(cov, lc, 94.0, 1e9);
  EXPECT_LE(post.first, post.second / 20)
      << "loiterer cell was swept " << post.first << "/" << post.second
      << " scans after t94 yet stayed pinned → coverage decay gate has a hole";
}

// ── close_approach: KEEP_MIXED presence under the suppressor (both arms) ──
// The suppressor-safety axis (detector vs the land baseline, NOT coverage vs
// universal): ADR 0002 forbids suppressing a KEEP_MIXED object into NOTHING.
// Presence = a track OR an emitted hazard covers the region. Measured against
// the land track-only baseline the R8.6 KEEP-stress benchmark established.
TEST(PhilosCoverageDecay6c, CloseApproachKeepMixedPresenceHeldUnderSuppressor) {
  const ClipRun land =
      replay_test::runClip("close_approach", "imm_cv_ct_pmbm_land");
  if (!land.valid) GTEST_SKIP() << "close_approach fixtures not reachable";
  const ClipRun uni = replay_test::runClip(
      "close_approach", "imm_cv_ct_pmbm_occupancy_detector");
  ASSERT_TRUE(uni.valid);
  const ClipRun cov = replay_test::runClip(
      "close_approach", "imm_cv_ct_pmbm_occupancy_detector_coverage");
  ASSERT_TRUE(cov.valid);
  const auto labels = replay_test::loadLabels("close_approach_labels.csv");
  ASSERT_FALSE(labels.empty());

  reportSectors(cov, "coverage ");

  std::cout << "\n=== 6c close_approach KEEP_MIXED presence (track OR hazard) ==="
               "\n";
  // Per region: fraction of active scans with presence, under each config.
  auto presenceFrac = [&](const ClipRun& run, const ExistenceLabel& l) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long covered = 0, active = 0;
    for (const auto& s : run.history) {
      if (!l.activeAtUnix(s.t_unix, run.clip_start_unix)) continue;
      ++active;
      if (trackAt(s, c, l.radius_m) || hazardKeepClearAt(s, c)) ++covered;
    }
    return active ? static_cast<double>(covered) / active : 0.0;
  };

  for (const auto& l : labels) {
    if (l.label != ExistenceLabelClass::KeepMixed) continue;
    const double fl = presenceFrac(land, l);
    const double fu = presenceFrac(uni, l);
    const double fc = presenceFrac(cov, l);
    std::cout << "  " << l.region_id << " (r=" << l.radius_m
              << "m): presence land=" << fl << " detector(uni)=" << fu
              << " detector(cov)=" << fc << "\n";
    // ADR 0002 conservation: the suppressor must not DROP presence below the
    // land baseline (a KEEP_MIXED object suppressed into nothing). Track-loss to
    // suppression must be backfilled by an emitted hazard. Small numerical slack.
    EXPECT_GE(fu, fl - 0.02)
        << "universal detector lost KEEP_MIXED presence on " << l.region_id;
    EXPECT_GE(fc, fl - 0.02)
        << "coverage detector lost KEEP_MIXED presence on " << l.region_id;
  }
  std::cout << std::flush;
}

// ── chart corroboration (increment 6): confirm structure, flag departed vessels ──
// The densified charted radar-visible structure (radar_structure_points.geojson)
// labels each emitted live hazard chart-confirmed or not. Chart-CONFIRMED = real
// structure (suppression justified, high operator confidence). UNcorroborated =
// the eviction candidate (increment 8): a departed vessel that pinned a cell has
// no chart/AIS/camera backing. This is the discriminator radar+coverage alone
// could not provide on the loiterer (6c: swept 0/283 after t94).
TEST(PhilosCoverageDecay6c, SunsetChartCorroborationLabelsStructureNotDepartedVessel) {
  const ClipRun run = replay_test::runClip(
      "sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector_coverage",
      /*load_chart_structure=*/true);
  if (!run.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const auto labels = replay_test::loadLabels("sunset_cruise_labels.csv");
  ASSERT_FALSE(labels.empty());

  long haz = 0, corr = 0;
  for (const auto& s : run.history)
    for (const HazardSnapshot& h : s.hazards) {
      ++haz;
      if (h.corroborated) ++corr;
    }
  std::cout << "\n=== 6c sunset_cruise chart corroboration (coverage+chart) ===\n"
            << "  hazard-scans=" << haz << " chart-corroborated=" << corr
            << " (" << (haz ? 100.0 * corr / haz : 0.0) << "%)\n";

  const ExistenceLabel* loit = nullptr;
  for (const auto& l : labels) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long on = 0, on_corr = 0;
    for (const auto& s : run.history) {
      if (hazardAt(s, c, l.radius_m)) ++on;
      if (corroboratedHazardAt(s, c, l.radius_m)) ++on_corr;
    }
    std::cout << "  " << l.region_id << " (r=" << l.radius_m
              << "m): hazard-scans " << on << ", chart-corroborated " << on_corr
              << "\n";
    if (l.region_id == "loiterer_v2") loit = &l;
  }
  std::cout << std::flush;
  ASSERT_NE(loit, nullptr);

  // Load-bearing invariants:
  // (1) The mechanism fires on real data — SOME emitted structure is chart-
  //     confirmed (the philos inner harbour has charted piers/wharves).
  EXPECT_GT(corr, 0) << "no live hazard was chart-corroborated — chart wiring "
                        "or densified fixture is broken";
  // (2) The loiterer pin is NOT chart-corroborated: it is a departed vessel in
  //     open water, not charted structure. This is exactly the departed-vs-held
  //     discriminator — chart ABSENCE flags it as the eviction candidate.
  const Eigen::Vector2d lc = labelEnu(run.datum, *loit);
  long loit_corr = 0;
  for (const auto& s : run.history)
    if (corroboratedHazardAt(s, lc, loit->radius_m)) ++loit_corr;
  EXPECT_EQ(loit_corr, 0)
      << "the loiterer (a departed vessel) was chart-corroborated as structure — "
         "false coincidence with the charted layer";
}

// ── camera corroboration (increment 6, camera): observed-empty flags VACATED ──
// ── cells that chart + radar could not resolve. ──
// Coverage detector + chart + camera on sunset_cruise. Camera-observed-empty
// flags cells the camera watches (in-FOV, live frame) with no detection at their
// bearing, sustained. Findings (eval-log): `ferry_v1_a` — the ferry's OUTBOUND
// berth, vacated after its t≈98 transition to ferry_v1_b — is the clean
// demonstration (flagged on many scans; a real vessel that MOVED, its stale pin
// correctly marked departed). The loiterer IS flagged (its bearing is cleanly
// empty after t100 — 0 detections within ±10°) but only where its post-departure
// hazard, made intermittent by the adaptive-bar flicker, coincides with the
// matured empty-streak. astern_blob ("astern of own-ship", out of the center
// FOV) is NEVER flagged — absence there is unobserved, not evidence (the
// coverage-aware principle in the camera modality). Every camera-flagged cell
// here is chart-UNconfirmed: the departed-vessel eviction candidates.
TEST(PhilosCoverageDecay6c, SunsetCameraObservedEmptyFlagsVacatedCells) {
  const ClipRun run = replay_test::runClip(
      "sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector_coverage",
      /*load_chart_structure=*/true, /*load_camera=*/true);
  if (!run.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const auto labels = replay_test::loadLabels("sunset_cruise_labels.csv");
  ASSERT_FALSE(labels.empty());

  long haz = 0, cam_empty = 0;
  for (const auto& s : run.history)
    for (const HazardSnapshot& h : s.hazards) {
      ++haz;
      if (h.camera_empty) ++cam_empty;
    }
  std::cout << "\n=== 6c sunset_cruise camera corroboration (coverage+chart+camera) ==="
            << "\n  hazard-scans=" << haz << " camera-observed-empty=" << cam_empty
            << "\n";

  const ExistenceLabel* loit = nullptr;
  const ExistenceLabel* astern = nullptr;
  for (const auto& l : labels) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long on = 0, corr = 0, cam = 0;
    for (const auto& s : run.history) {
      if (hazardAt(s, c, l.radius_m)) ++on;
      if (corroboratedHazardAt(s, c, l.radius_m)) ++corr;
      if (cameraEmptyHazardAt(s, c, l.radius_m)) ++cam;
    }
    std::cout << "  " << l.region_id << ": hazard-scans " << on
              << ", chart-corroborated " << corr << ", camera-observed-empty "
              << cam << "\n";
    if (l.region_id == "loiterer_v2") loit = &l;
    if (l.region_id == "astern_blob") astern = &l;
  }
  // Diagnostic: loiterer hazard presence vs camera-empty over the departure
  // window (are hazard-scans and the clean camera-empty window overlapping?).
  {
    const Eigen::Vector2d lc = labelEnu(run.datum, *loit);
    std::cout << "  loiterer timeline [rel_t: H=hazard C=camera_empty]:";
    int shown = 0;
    for (const auto& s : run.history) {
      const double rel = s.t_unix - run.clip_start_unix;
      if (rel < 92.0 || rel > 120.0) continue;
      const bool h = hazardAt(s, lc, loit->radius_m);
      const bool c = cameraEmptyHazardAt(s, lc, loit->radius_m);
      if (h && (shown++ % 8 == 0))
        std::cout << " " << (int)rel << (c ? ":HC" : ":H");
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
  ASSERT_NE(loit, nullptr);
  ASSERT_NE(astern, nullptr);

  auto countCam = [&](const ExistenceLabel& l) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long n = 0;
    for (const auto& s : run.history)
      if (cameraEmptyHazardAt(s, c, l.radius_m)) ++n;
    return n;
  };
  auto countCorr = [&](const ExistenceLabel& l) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long n = 0;
    for (const auto& s : run.history)
      if (corroboratedHazardAt(s, c, l.radius_m)) ++n;
    return n;
  };

  const ExistenceLabel* ferry_a = nullptr;
  for (const auto& l : labels)
    if (l.region_id == "ferry_v1_a") ferry_a = &l;
  ASSERT_NE(ferry_a, nullptr);

  // (1) The mechanism fires on real philos — the ferry's VACATED outbound berth
  //     (a real vessel that moved to ferry_v1_b) is robustly camera-observed-empty.
  EXPECT_GT(countCam(*ferry_a), 5)
      << "the vacated ferry berth was not camera-observed-empty — camera wiring "
         "or FOV gate broken";
  // (2) The loiterer's cleanly-empty bearing IS caught where its (intermittent)
  //     post-departure hazard coincides with the matured empty-streak.
  EXPECT_GT(countCam(*loit), 0)
      << "the loiterer was never camera-observed-empty";
  // (3) Every camera-flagged cell here is chart-UNconfirmed → the eviction
  //     candidates (departed vessels, not charted structure).
  EXPECT_EQ(countCorr(*loit), 0) << "loiterer unexpectedly chart-corroborated";
  EXPECT_EQ(countCorr(*ferry_a), 0) << "vacated ferry berth chart-corroborated";
  // (4) astern_blob is OUT of the center FOV → never camera-flagged (absence
  //     unobserved is not evidence); it is held by chart (31/31) instead.
  EXPECT_EQ(countCam(*astern), 0)
      << "astern_blob (out of center FOV) was camera-flagged — FOV gate leaked";
}

}  // namespace navtracker
