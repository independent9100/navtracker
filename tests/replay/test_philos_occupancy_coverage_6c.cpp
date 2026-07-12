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
//     win is STRUCTURE PRESENCE: coverage-aware RETAINS off-beam structure it
//     cannot re-sweep. This is asserted as a SINGLE-RUN property of the coverage
//     arm (it keeps emitting structure hazards, never collapsing to empty) — NOT
//     as a cross-config "coverage ≥ universal over every region" guarantee. That
//     monotonicity claim is FALSE: the layer is feedback-coupled to the tracker
//     and the clutter-adaptive bar is non-monotone in persistence, so per-region
//     coverage-vs-universal presence flips with the margin (see the removed
//     check (2) in the sunset test). The one-sided "the guard only ADDS mass"
//     invariant holds only with inputs held fixed and is unit-tested in
//     LiveOccupancyModel.ShadowGuardOnlyAddsMassOnFixedInputs.
//   * close_approach — KEEP_MIXED presence under the suppressor: presence
//     (track OR a hazard's keep-clear ring, the ADR-0002 conservation test) must
//     not fall below the land baseline (no object suppressed into nothing).
//
// These are measurement tests: they print the timelines and assert only the
// mechanism's structural invariants — the sector gate actually bites; the
// coverage arm retains structure (single-run, never collapsing to empty); a
// pinned-but-unswept cell is correct not buggy; camera-observed-empty is proven
// by the config-independent CELL streak (not the adaptive-bar-fragile emitted
// hazard flag); the suppressor never drops KEEP_MIXED presence below baseline.
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

  // (2) [REMOVED 2026-07-07 — no valid full-pipeline monotonicity invariant.]
  //
  //     This PREVIOUSLY asserted "coverage-aware holds every labelled region for
  //     ≥ as many EMITTED HAZARD-scans as universal", derived from per-cell
  //     persistence "→ hazard superset". BOTH forms are wrong as full-pipeline
  //     invariants:
  //       (a) emitted-hazard: the clutter-ADAPTIVE bar (median × factor) is
  //           NON-monotone in persistence — raising a cell's mass raises the
  //           median → raises the bar → can DE-EMIT a marginal region whose mass
  //           ROSE. The LOS guard exposed it: the failing region flipped between
  //           midriver_grp and astern_blob as the guard range margin changed
  //           (50→0/15, 150→0/0, 250→93/0, 400→606/60) — an epsilon-knife-edge.
  //       (b) per-cell persistence: NOT monotone between these configs either —
  //           the occupancy layer is FEEDBACK-COUPLED to the tracker: occupancy
  //           persistence → birth suppression → tracker → which returns are
  //           claimed → clutter weights (1−r) → the touches fed back into
  //           occupancy. So `cov` and `uni` are different full-pipeline runs with
  //           different touch sequences; per-cell mass diverges non-monotonically
  //           (measured deficit 0.064 on this clip).
  //
  //     Asserting a property of the DIFFERENCE between two full-pipeline runs of a
  //     feedback system pins an INCIDENTAL, not an invariant — the old check was
  //     true by luck. The guard's genuine invariant (identical inputs ⇒ it only
  //     skips decays ⇒ persistence only rises) is provable ONLY with inputs held
  //     fixed, and is asserted where it holds:
  //     LiveOccupancyModel.ShadowGuardOnlyAddsMassOnFixedInputs
  //     (tests/static/test_live_occupancy_model.cpp). Checks (1)/(3)/(4) below are
  //     honest measured expectations about SPECIFIC behaviours, not claimed
  //     invariants — a legitimate thing for a gate to hold. KEEP conservation
  //     gates (never lose a real vessel) remain absolute, untouched. See the
  //     2026-07-07 eval-log entry + backlog "adaptive/threshold & feedback-coupled
  //     A/B decision robustness".

  // (3) The coverage arm RETAINS structure — it never collapses to empty over the
  //     clip. Stated as a SINGLE-RUN, banded floor on the coverage run alone: a
  //     hazard is emitted on ≥ 90% of scans. This is NOT the old cross-config
  //     per-region A/B (`cov_astern > uni_astern`), which was the SAME invalid
  //     invariant removed in (2): astern_blob's emission class flips with the
  //     adaptive-bar margin (universal=13/coverage=0 here — the exact
  //     midriver_grp↔astern_blob knife-edge the guard exposed), so a coverage>uni
  //     per-region comparison pins an incidental. The one-sided "guard only adds
  //     mass" invariant is proven with fixed inputs in
  //     LiveOccupancyModel.ShadowGuardOnlyAddsMassOnFixedInputs.
  long cov_nonempty = 0;
  for (const auto& s : cov.history)
    if (!s.hazards.empty()) ++cov_nonempty;
  ASSERT_FALSE(cov.history.empty());
  EXPECT_GE(static_cast<double>(cov_nonempty) / cov.history.size(), 0.90)
      << "coverage arm emitted no hazard on " << (cov.history.size() - cov_nonempty)
      << "/" << cov.history.size()
      << " scans → it is not retaining structure (should never collapse to empty)";
  (void)uni_astern;
  (void)cov_astern;

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

  int keep_mixed_seen = 0;
  for (const auto& l : labels) {
    if (l.label != ExistenceLabelClass::KeepMixed) continue;
    ++keep_mixed_seen;
    const double fl = presenceFrac(land, l);
    const double fu = presenceFrac(uni, l);
    const double fc = presenceFrac(cov, l);
    std::cout << "  " << l.region_id << " (r=" << l.radius_m
              << "m): presence land=" << fl << " detector(uni)=" << fu
              << " detector(cov)=" << fc << "\n";
    // ADR 0002 conservation: the suppressor must not DROP KEEP_MIXED presence
    // below the land baseline (an object suppressed into nothing). This is an
    // inherently RELATIVE invariant — the land baseline itself is region-varying
    // (measured: sailing_dock ~0.96 but far_bank_line ~0.49, a far/partly-covered
    // region), so a fixed absolute floor would be wrong. #24: the old additive
    // `fu >= fl - 0.02` slack is fragile (flips on a ~3% cross-config drift). A
    // RATIO floor is robust (scales with the baseline) and toothy: the detector
    // arm must retain ≥80% of land's presence. Measured ratios are ≥1.0 in every
    // arm/region (detector ≥ land), so 0.8 carries ≥20% headroom; a suppression-
    // into-nothing regression drops the arm well under 0.8·land and goes red.
    EXPECT_GE(fu, 0.8 * fl)
        << "universal detector suppressed KEEP_MIXED presence below the land "
           "baseline on " << l.region_id << " (fu=" << fu << " land=" << fl << ")";
    EXPECT_GE(fc, 0.8 * fl)
        << "coverage detector suppressed KEEP_MIXED presence below the land "
           "baseline on " << l.region_id << " (fc=" << fc << " land=" << fl << ")";
  }
  // #24 (W3 assertion-quality coverage-note-8): guard the filtered loop — if the
  // labels file ever drops/renames KEEP_MIXED, the loop would match nothing and
  // this whole conservation test would pass with zero assertions run. The sibling
  // test_philos_close_approach_labels.cpp:141 already carries this guard.
  ASSERT_GT(keep_mixed_seen, 0)
      << "no KEEP_MIXED region found in close_approach labels — the conservation "
         "loop never executed (fixture/label regression)";
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
  // #24: banded floor, not a bare >0. Measured chart-corroborated ~4819 of 6128
  // hazard-scans (~79%); a floor of 3000 (~62% of measured) catches a PARTIAL
  // corroboration collapse, not only total chart-wiring death.
  EXPECT_GT(corr, 3000) << "chart corroboration collapsed (measured ~4819): corr="
                        << corr << " — chart wiring or densified fixture broken";
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

  // Config-INDEPENDENT: scans on which a camera-observed-empty STREAK matured on a
  // cell within the region — the raw "the camera proved this cell empty" fact,
  // which does not depend on the persistence bar or membership hysteresis (unlike
  // the emitted-hazard flag, whose centroid drifts). This is what the camera
  // actually proved on the loiterer (clean bearing, 0 detections ±10° over 20 s).
  auto streakMaturedScans = [&](const ExistenceLabel& l) {
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long n = 0;
    for (const auto& s : run.history)
      for (const auto& cell : s.camera_empty_cells)
        if ((cell - c).norm() <= l.radius_m) { ++n; break; }
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

  // Config-independent camera-empty CELL streaks per region (the raw "the camera
  // proved this cell empty" fact — independent of the adaptive persistence bar).
  std::cout << "  camera-empty CELL streaks matured (config-independent): "
            << "ferry_v1_a=" << streakMaturedScans(*ferry_a)
            << " loiterer_v2=" << streakMaturedScans(*loit)
            << " astern_blob=" << streakMaturedScans(*astern) << "\n"
            << std::flush;

  // (1) The mechanism fires on real philos — the ferry's VACATED outbound berth
  //     is proven empty by the camera CELL streak maturing (config-independent —
  //     what the camera actually observed). This does NOT use the emitted-hazard
  //     camera flag (countCam): that requires the berth pin to survive the
  //     adaptive persistence bar into the post-vacate window, but under the
  //     current stack the ferry berth decays first (0 emitted hazards after t98 —
  //     see the eviction A/B, test SunsetCameraEviction*), so countCam is a #24
  //     knife-edge. This is the c0ac493 loiterer fix applied to the ferry.
  // #24: banded floor (measured 41 matured scans) — catches a FOV-gate/streak
  // regression that matures only a scan or two, not just total camera death.
  EXPECT_GT(streakMaturedScans(*ferry_a), 20)
      << "the vacated ferry berth cell's camera-observed-empty streak collapsed "
         "(measured ~41): " << streakMaturedScans(*ferry_a)
      << " — camera wiring or FOV gate broken";
  // (2) The loiterer's cleanly-empty bearing is proven by the CELL streak
  //     maturing (config-independent — what the camera actually observed), not by
  //     the fragile hazard∧streak coincidence (which the frozen detector's
  //     membership hysteresis, membership_exit_factor=0.6, legitimately shifts —
  //     see the 2026-07-05 held-out freeze decision). Both the loiterer (case 2)
  //     and the ferry (case 1) are now asserted on the config-independent cell
  //     streak, not the adaptive-bar-fragile emitted-hazard flag.
  // #24: banded floor (measured 203 matured scans over the ~20 s clean window).
  EXPECT_GT(streakMaturedScans(*loit), 100)
      << "the loiterer's camera-observed-empty streak collapsed (measured ~203): "
      << streakMaturedScans(*loit);
  // (3) Every camera-flagged cell here is chart-UNconfirmed → the eviction
  //     candidates (departed vessels, not charted structure).
  EXPECT_EQ(countCorr(*loit), 0) << "loiterer unexpectedly chart-corroborated";
  EXPECT_EQ(countCorr(*ferry_a), 0) << "vacated ferry berth chart-corroborated";
  // (4) astern_blob is OUT of the center FOV → its cell streak never matures
  //     (absence unobserved is not evidence); it is held by chart instead. Tested
  //     on the config-independent streak, not countCam: astern is de-emitted under
  //     the adaptive bar on this clip (see the structure-presence test), so a
  //     countCam==0 would be trivially true regardless of the FOV gate; the streak
  //     tests the FOV gate directly (it is populated whether or not a hazard is
  //     emitted on the cell).
  EXPECT_EQ(streakMaturedScans(*astern), 0)
      << "astern_blob (out of center FOV) had a matured camera-empty streak — "
         "FOV gate leaked";
}

// ── increment (ii): camera EVICTION as BEHAVIOUR — real-data DEMO (A/B) ──
// Coverage detector + chart + camera on sunset_cruise, eviction OFF vs ON (the
// only variable). Ground truth from the labels + 6c measurements: the loiterer
// (rank202, returns cease ~t94, uncharted, 134 m from charted structure) and the
// ferry's vacated OUTBOUND berth (ferry_v1_a, uncharted, camera-observed-empty)
// are DEPARTED vessels that coverage-aware decay + chart could NOT clear (swept
// 0/283; chart abstains). Camera eviction, keyed by cell, spends their frozen
// pins — including the loiterer, whose intermittent (adaptive-bar-flicker) hazard
// under-flagged the label-only increment (i) but is caught here on re-entry.
// astern_blob is chart-CONFIRMED (16 m, 31/31) → HELD regardless of camera
// (evidence precedence). This is a DEMONSTRATION on real data; promotion gates on
// the synthetic model scenario (LiveOccupancyModel.EvictionScene* + adaptive-bar
// flicker), per the circularity rule.
TEST(PhilosCoverageDecay6c, SunsetCameraEvictionRemovesDepartedPinsHoldsChartStructure) {
  const ClipRun off = replay_test::runClip(
      "sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector_coverage",
      /*load_chart_structure=*/true, /*load_camera=*/true, /*evict_camera=*/false);
  if (!off.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const ClipRun on = replay_test::runClip(
      "sunset_cruise", "imm_cv_ct_pmbm_occupancy_detector_coverage",
      /*load_chart_structure=*/true, /*load_camera=*/true, /*evict_camera=*/true);
  ASSERT_TRUE(on.valid);
  const auto labels = replay_test::loadLabels("sunset_cruise_labels.csv");
  ASSERT_FALSE(labels.empty());

  auto totalHaz = [](const ClipRun& r) {
    long n = 0;
    for (const auto& s : r.history) n += static_cast<long>(s.hazards.size());
    return n;
  };
  auto hazAt = [&](const ClipRun& r, const ExistenceLabel& l) {
    const Eigen::Vector2d c = labelEnu(r.datum, l);
    long n = 0;
    for (const auto& s : r.history)
      if (hazardAt(s, c, l.radius_m)) ++n;
    return n;
  };

  const ExistenceLabel *loit = nullptr, *ferry = nullptr, *astern = nullptr;
  for (const auto& l : labels) {
    if (l.region_id == "loiterer_v2") loit = &l;
    if (l.region_id == "ferry_v1_a") ferry = &l;
    if (l.region_id == "astern_blob") astern = &l;
  }
  ASSERT_NE(loit, nullptr);
  ASSERT_NE(ferry, nullptr);
  ASSERT_NE(astern, nullptr);

  const long haz_off = totalHaz(off), haz_on = totalHaz(on);
  const long loit_off = hazAt(off, *loit), loit_on = hazAt(on, *loit);
  const long ferry_off = hazAt(off, *ferry), ferry_on = hazAt(on, *ferry);
  const long astern_off = hazAt(off, *astern), astern_on = hazAt(on, *astern);
  std::cout
      << "\n=== increment(ii) sunset_cruise camera EVICTION A/B (coverage+chart+camera) ==="
      << "\n  total hazard-scans:                off=" << haz_off << " on=" << haz_on
      << "\n  loiterer_v2  (uncharted, departed): off=" << loit_off << " on=" << loit_on
      << "\n  ferry_v1_a   (uncharted, vacated) : off=" << ferry_off << " on=" << ferry_on
      << "\n  astern_blob  (chart-confirmed)    : off=" << astern_off << " on=" << astern_on
      << "\n" << std::flush;

  // Hazard-scans in a region, split at a departure time `t_dep` into
  // {before, after}. The AFTER window is the departed/vacated phantom (camera
  // sees it empty → the eviction target); the BEFORE window is the vessel still
  // present (camera sees detections → correctly NOT evicted).
  auto splitAt = [&](const ClipRun& r, const ExistenceLabel& l, double t_dep) {
    const Eigen::Vector2d c = labelEnu(r.datum, l);
    std::pair<long, long> pp{0, 0};
    for (const auto& s : r.history) {
      if (!hazardAt(s, c, l.radius_m)) continue;
      ((s.t_unix - r.clip_start_unix) < t_dep ? pp.first : pp.second)++;
    }
    return pp;
  };
  const auto loit_off_pp = splitAt(off, *loit, 100.0);   // loiterer clean-empty at t100
  const auto loit_on_pp = splitAt(on, *loit, 100.0);
  const auto ferry_off_pp = splitAt(off, *ferry, 98.0);  // ferry vacates berth-a at t98
  const auto ferry_on_pp = splitAt(on, *ferry, 98.0);
  std::cout
      << "  loiterer   before/after t100:  off=" << loit_off_pp.first << "/"
      << loit_off_pp.second << "  on=" << loit_on_pp.first << "/" << loit_on_pp.second
      << "\n  ferry_v1_a before/after t98:   off=" << ferry_off_pp.first << "/"
      << ferry_off_pp.second << "  on=" << ferry_on_pp.first << "/" << ferry_on_pp.second
      << "\n" << std::flush;

  // What this DEMONSTRATES on real data (correctness is gated on the synthetic
  // model scenario, per the circularity rule — this clip has no truth):
  //
  // (1) Eviction removes real phantom mass overall (392 hazard-scans here).
  EXPECT_LT(haz_on, haz_off) << "eviction removed no hazard mass";
  // (2) The vacated ferry berth's POST-move window (t≥98) is never INCREASED by
  //     eviction — one-sided, because eviction can only spend pins, never add
  //     them. The old strict `on < off` pinned a residual post-move phantom to
  //     evict, but under the current stack the berth pin decays out before the
  //     empty window (0/0 here — the same "not a persistent post-departure
  //     phantom" caveat the eval-log already records for the loiterer below), so
  //     there is nothing left to remove there. The strict "eviction removes real
  //     mass" claim is carried by the aggregate (1); post-move eviction
  //     correctness is gated on the synthetic EvictionScene* (this clip has no
  //     truth).
  EXPECT_LE(ferry_on_pp.second, ferry_off_pp.second)
      << "eviction INCREASED the vacated ferry berth post-move phantom — "
         "impossible unless eviction is adding pins";
  // (3) Eviction RESPECTS a present vessel: the loiterer's BEFORE-departure
  //     hazards (vessel still there, camera sees detections at its bearing → the
  //     streak resets → never matured) are retained, not wrongly evicted.
  EXPECT_GE(loit_on_pp.first, loit_off_pp.first)
      << "eviction wrongly removed hazards while the vessel was still present";
  // (4) Chart-confirmed structure is HELD regardless of camera (evidence
  //     precedence): eviction never REDUCES astern_blob — one-sided GE, robust.
  //     The prior `astern_on > 0` existence pin is dropped: astern_blob is
  //     de-emitted under the adaptive bar on this clip (universal=13/coverage=0 —
  //     the #24 midriver_grp↔astern_blob knife-edge), so its emission is not an
  //     invariant here; the chart-hold-beats-camera precedence is gated on the
  //     synthetic EvictionScene*.
  EXPECT_GE(astern_on, astern_off) << "chart-confirmed astern_blob wrongly evicted";

  // HONEST caveats recorded for the eval-log (Layer-2 / truth questions):
  //  • The loiterer is NOT a persistent post-departure phantom in this config —
  //    the adaptive bar fades it (off has just 1 hazard-scan after t100), so
  //    there is almost nothing for eviction to remove there; the SYNTHETIC
  //    flicker gate carries the loiterer pathology instead.
  //  • Eviction also removed ~145 ferry hazards BEFORE the move (t<98), where the
  //    camera intermittently saw the docked berth empty. Whether that is correct
  //    (ferry tracked, or the berth pin is already phantom) or over-eviction of a
  //    present-but-unseen vessel needs kinematic truth — a Layer-2 measurement.
}

}  // namespace navtracker
