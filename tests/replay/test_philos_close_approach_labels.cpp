// R8.6 — label-scored replay of the zero-AIS `close_approach` clip: the standing
// KEEP-STRESS benchmark. close_approach is the densest scene in the philos
// dataset (Charles River sailing basin, regatta-density dinghies, ~330-550
// plots per 10 s) and has zero AIS, so — like sunset_cruise — the only
// evaluation surface is the video-derived region labels
// (tests/fixtures/philos/labels/close_approach_labels.csv). Both labelled
// regions are KEEP_MIXED (vessels AND structure): a confirmed track OR an
// emitted static hazard satisfies the presence gate.
//
// Run through imm_cv_ct_pmbm_land (no suppression) to establish the baseline any
// future suppressor must hold: tracks_on_keep MUST NOT fall here. Harness in
// tests/replay/PhilosLabelReplay.hpp, shared with the sunset_cruise gates.
#include <algorithm>
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
            << "  (confirmed track-scans in KEEP_MIXED regions — MUST NOT fall)\n"
            << "  false_on_suppress= " << d.false_on_suppress
            << "  (no SUPPRESS regions labelled on this clip → expect 0)\n"
            << "  false_unlabeled  = " << d.false_unlabeled << "  (remainder)\n"
            << std::flush;
  // The densest clip: the tracker holds many confirmed tracks in the KEEP_MIXED
  // regions. This is the value a suppressor must not erode.
  EXPECT_GT(d.tracks_on_keep, 0);
  // No SUPPRESS_STRUCTURE region is labelled on this clip.
  EXPECT_EQ(d.false_on_suppress, 0);
}

// R8.6 — KEEP_MIXED canary. Each KEEP_MIXED region must contain >=1 confirmed
// track within its radius during its (whole-clip) window. Presence-gated; under
// the non-suppressing land config no static hazards are emitted, so this reduces
// to track presence. Must pass TODAY — documents current-behaviour safety in the
// densest scene, and is the gate a suppressor must not break.
TEST(PhilosCloseApproachLabels, KeepMixedCanariesHaveTracks) {
  const ClipRun run = runCloseApproach();
  if (!run.valid) GTEST_SKIP() << "close_approach fixtures not reachable";
  const auto labels = loadCloseApproachLabels();
  ASSERT_FALSE(labels.empty());

  std::cout << "\n=== R8.6 KEEP_MIXED canaries (close_approach, "
               "imm_cv_ct_pmbm_land) ===\n";
  for (const auto& l : labels) {
    ASSERT_EQ(l.label, ExistenceLabelClass::KeepMixed) << l.region_id;
    const Eigen::Vector2d c = labelEnu(run.datum, l);
    double best = 1e18;
    bool covered = false;
    for (const auto& scan : run.history) {
      if (!l.activeAtUnix(scan.t_unix, run.clip_start_unix)) continue;
      for (const auto& tr : scan.tracks) {
        const double dist = (tr.pos - c).norm();
        best = std::min(best, dist);
        if (dist <= l.radius_m) covered = true;
      }
    }
    std::cout << "  " << l.region_id << " (r=" << l.radius_m
              << "m): closest confirmed track = " << best << "m -> "
              << (covered ? "COVERED" : "MISSED") << "\n";
    EXPECT_TRUE(covered) << "KEEP_MIXED region " << l.region_id
                         << " has no confirmed track within " << l.radius_m
                         << " m (closest " << best << " m)";
  }
  std::cout << std::flush;
}

}  // namespace navtracker
