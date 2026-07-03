// R8.6 — label-scored replay of the zero-AIS `close_approach` clip: the standing
// KEEP-STRESS benchmark. close_approach is the densest scene in the philos
// dataset (Charles River sailing basin, regatta-density dinghies, ~330-550
// plots per 10 s) and has zero AIS, so — like sunset_cruise — the only
// evaluation surface is the video-derived region labels
// (tests/fixtures/philos/labels/close_approach_labels.csv). Both labelled
// regions are KEEP_MIXED (vessels AND structure): a confirmed track OR an
// emitted static hazard satisfies the presence gate.
//
// Run through imm_cv_ct_pmbm_land (no suppression) to record the KEEP baseline a
// future suppressor is compared against (flatness of tracks_on_keep is the
// increment-6 A/B, not a fixed threshold). What is gated HERE today: a loose
// catastrophic-drop floor on the aggregate, and a per-region majority-of-scans
// coverage floor that a craft-deleting config would trip. Harness in
// tests/replay/PhilosLabelReplay.hpp, shared with the sunset_cruise gates.
#include <algorithm>
#include <iostream>
#include <map>
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
using replay_test::labelEnu;

ClipRun runCloseApproach() {
  return replay_test::runClip("close_approach", "imm_cv_ct_pmbm_land");
}
std::vector<ExistenceLabel> loadCloseApproachLabels() {
  return replay_test::loadLabels("close_approach_labels.csv");
}

}  // namespace

// R8.6 item 4 — decomposition baseline. Reported (not a threshold gate): these
// are the KEEP-stress numbers a suppression mechanism is judged against.
// tracks_on_keep must NOT fall when a suppressor is later scored on this clip.
TEST(PhilosCloseApproachLabels, LabelAwareDecompositionKeepStressBaseline) {
  const ClipRun run = runCloseApproach();
  if (!run.valid) GTEST_SKIP() << "close_approach fixtures not reachable";
  const auto labels = loadCloseApproachLabels();
  ASSERT_FALSE(labels.empty());

  const auto d = replay_test::decompose(run, labels);
  std::cout << "\n=== R8.6 label-aware philos decomposition (close_approach, "
               "imm_cv_ct_pmbm_land) — KEEP-STRESS BASELINE ===\n"
            << "  scans=" << run.history.size() << "\n"
            << "  tracks_on_keep   = " << d.tracks_on_keep
            << "  (confirmed track-scans in KEEP_MIXED regions)\n"
            << "  false_on_suppress= " << d.false_on_suppress
            << "  (no SUPPRESS regions labelled on this clip → structurally 0)\n"
            << "  false_unlabeled  = " << d.false_unlabeled << "  (remainder)\n"
            << std::flush;
  // Regression floor on the recorded baseline (tracks_on_keep = 5570 on
  // 2026-07-03 under imm_cv_ct_pmbm_land). The floor is deliberately loose — it
  // guards against a CATASTROPHIC drop in KEEP mass (a base-config change or a
  // future suppressor deleting the majority of real craft), not flatness. The
  // flatness comparison (a suppressor's tracks_on_keep vs this land baseline) is
  // the increment-6 A/B, not a fixed threshold here.
  constexpr long kTracksOnKeepFloor = 2500;  // ≈45% of the 5570 baseline
  EXPECT_GE(d.tracks_on_keep, kTracksOnKeepFloor)
      << "KEEP mass on close_approach collapsed vs the 5570 baseline — a base "
         "config change or suppressor deleted the majority of real craft";
  // false_on_suppress is structurally 0 here (the fixture has no SUPPRESS_STRUCTURE
  // row — asserted in CloseApproachFixtureLoads); this guards that decompose()
  // does not miscount a KEEP_MIXED track as suppress-mass (cross-contamination).
  EXPECT_EQ(d.false_on_suppress, 0);
}

// R8.6 — KEEP_MIXED coverage gate. For each KEEP_MIXED region, the fraction of
// ACTIVE scans holding >=1 confirmed track within radius must clear a floor.
// This is deliberately stronger than a whole-clip existential "any track ever
// grazed the region": the dense scene satisfies that ~6x per scan for free and
// it cannot tell a real dinghy from a phantom. The per-scan coverage FRACTION
// does discriminate — a suppressor that deletes a region's real craft drives its
// coverage toward 0 and trips the floor. Presence-gated: under the non-suppressing
// land config no static hazards are emitted, so this scores TRACK presence only;
// the OR-hazard branch is added when a suppressor config is first scored on this
// clip (increment 6; see PhilosLabelReplay.hpp). Must pass TODAY — documents
// current-behaviour KEEP safety in the densest scene.
TEST(PhilosCloseApproachLabels, KeepMixedRegionsTrackedOnMajorityOfScans) {
  const ClipRun run = runCloseApproach();
  if (!run.valid) GTEST_SKIP() << "close_approach fixtures not reachable";
  const auto labels = loadCloseApproachLabels();
  ASSERT_FALSE(labels.empty());

  // Per-region coverage baselines measured 2026-07-03 under imm_cv_ct_pmbm_land.
  // sailing_dock is the strong high-confidence dock, tracked ~96% of scans;
  // far_bank_line is a distant med-confidence float/mooring line intermittently
  // detected at range, ~49%. The gate is 70% of each region's OWN baseline — a
  // suppressor (or base-config change) that erodes a region's real-craft
  // coverage by >30% relative trips it, while the honest range-limited
  // intermittency of the far bank is not punished. A single blanket floor cannot
  // do both (it would let a suppressor gut the strong region undetected).
  const std::map<std::string, double> baseline_cov = {
      {"sailing_dock", 0.96}, {"far_bank_line", 0.49}};

  std::cout << "\n=== R8.6 KEEP_MIXED coverage (close_approach, "
               "imm_cv_ct_pmbm_land) ===\n";
  int keep_mixed_seen = 0;
  for (const auto& l : labels) {
    if (l.label != ExistenceLabelClass::KeepMixed) continue;  // this clip: all
    ++keep_mixed_seen;
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    long active = 0, covered = 0;
    double best = 1e18;
    for (const auto& scan : run.history) {
      if (!l.activeAtUnix(scan.t_unix, run.clip_start_unix)) continue;
      ++active;
      bool hit = false;
      for (const auto& tr : scan.tracks) {
        const double dist = (tr.pos - c).norm();
        best = std::min(best, dist);
        if (dist <= l.radius_m) hit = true;
      }
      if (hit) ++covered;
    }
    const double frac = active ? static_cast<double>(covered) / active : 0.0;
    const auto it = baseline_cov.find(l.region_id);
    const double floor = it != baseline_cov.end() ? 0.7 * it->second : 0.3;
    std::cout << "  " << l.region_id << " (r=" << l.radius_m << "m): " << covered
              << "/" << active << " active scans covered (frac " << frac
              << ", floor " << floor << "), closest track " << best << "m\n";
    ASSERT_GT(active, 0) << l.region_id << " has no active scans";
    EXPECT_GE(frac, floor)
        << "KEEP_MIXED region " << l.region_id << " tracked on only " << frac
        << " of active scans (floor " << floor << " = 70% of the "
        << (it != baseline_cov.end() ? it->second : 0.0)
        << " baseline) — real craft in the region are being dropped";
  }
  EXPECT_EQ(keep_mixed_seen, 2) << "expected both close_approach KEEP_MIXED regions";
  std::cout << std::flush;
}

}  // namespace navtracker
