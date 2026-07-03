// R8.2 + R8.3 — label-aware philos metric decomposition + binary gates on the
// zero-AIS `sunset_cruise` clip. The replay harness (runClip / loadLabels /
// decompose) is shared with the R8.6 close_approach KEEP-stress benchmark; see
// tests/replay/PhilosLabelReplay.hpp.
//
// sunset_cruise has NO AIS and no radar-truth, so there is no kinematic truth
// to score GOSPA against — the ONLY evaluation surface is the video-derived
// region labels (tests/fixtures/philos/labels/sunset_cruise_labels.csv). This
// is exactly the "partial truth: existence labels" case R8 defines. We run the
// clip through the canonical coastal PMBM (imm_cv_ct_pmbm_land) and:
//
//   R8.2 decomposition — per confirmed track-scan, classify its position vs the
//     active labels: false_on_suppress (in a SUPPRESS region), tracks_on_keep
//     (in a KEEP region), false_unlabeled (elsewhere). Reported, un-gameable: a
//     config that "wins" by deleting the ferry shows tracks_on_keep fall.
//   R8.3 gates — (1) KEEP canary: each KEEP region has >=1 confirmed track
//     within radius during its window; (2) stop->go: a single track id holds
//     the ferry across the t~90 transition (r11 -> r16 regions) and reports
//     motion by t~110-116. Both must pass TODAY under imm_cv_ct_pmbm_land
//     (no suppression active); they document current-behaviour safety.
#include <algorithm>
#include <cstdint>
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

ClipRun runSunset() {
  return replay_test::runClip("sunset_cruise", "imm_cv_ct_pmbm_land");
}
std::vector<ExistenceLabel> loadLabels() {
  return replay_test::loadLabels("sunset_cruise_labels.csv");
}

}  // namespace

// R8.2 — label-aware decomposition. Reported, not a threshold gate here (the
// numbers become the A/B surface increment 6 is judged against); a couple of
// sanity bounds keep the instrument honest.
TEST(PhilosSunsetLabels, LabelAwareDecomposition) {
  const ClipRun run = runSunset();
  if (!run.valid) {
    GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  }
  const auto labels = loadLabels();
  ASSERT_FALSE(labels.empty());

  const auto d = replay_test::decompose(run, labels);
  std::cout << "\n=== R8.2 label-aware philos decomposition (sunset_cruise, "
               "imm_cv_ct_pmbm_land) ===\n"
            << "  scans=" << run.history.size() << "\n"
            << "  tracks_on_keep   = " << d.tracks_on_keep
            << "  (confirmed track-scans in KEEP regions — must NOT fall)\n"
            << "  false_on_suppress= " << d.false_on_suppress
            << "  (confirmed track-scans in SUPPRESS regions — a suppressor should shrink)\n"
            << "  false_unlabeled  = " << d.false_unlabeled << "  (remainder)\n"
            << std::flush;
  // Instrument sanity: the clip has real moving vessels, so SOME confirmed
  // track-scans land in KEEP regions under the current tracker.
  EXPECT_GT(d.tracks_on_keep, 0);
}

// R8.3 gate 1 — KEEP canary. Each KEEP region must contain >=1 confirmed track
// within its radius during its window. Must pass TODAY under land.
TEST(PhilosSunsetLabels, KeepCanariesHaveTracks) {
  const ClipRun run = runSunset();
  if (!run.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const auto labels = loadLabels();
  ASSERT_FALSE(labels.empty());

  std::cout << "\n=== R8.3 KEEP canaries (sunset_cruise, imm_cv_ct_pmbm_land) ===\n";
  for (const auto& l : labels) {
    if (l.label == ExistenceLabelClass::SuppressStructure) continue;  // KEEP set
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
    EXPECT_TRUE(covered) << "KEEP region " << l.region_id
                         << " has no confirmed track within " << l.radius_m
                         << " m during its window (closest " << best << " m)";
  }
  std::cout << std::flush;
}

// R8.3 gate 2 — stop->go. A single confirmed track id holds the ferry across
// the t~90 transition (ferry_v1_a window -> ferry_v1_b window) and reports
// motion (SOG above threshold) by t~110-116. Real-data instance of the ADR
// 0002 rule-3 recovery.
TEST(PhilosSunsetLabels, FerryStopGoKeepsStableIdAndReportsMotion) {
  const ClipRun run = runSunset();
  if (!run.valid) GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  const auto labels = loadLabels();
  const ExistenceLabel* a = nullptr;
  const ExistenceLabel* b = nullptr;
  for (const auto& l : labels) {
    if (l.region_id == "ferry_v1_a") a = &l;
    if (l.region_id == "ferry_v1_b") b = &l;
  }
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  const Eigen::Vector2d ca = labelEnu(run.datum, *a);
  const Eigen::Vector2d cb = labelEnu(run.datum, *b);

  // ids seen inside region a during a's window; inside region b during b's
  // window; and the max SOG each id reports late (t in [110,116]).
  std::map<std::uint64_t, bool> in_a, in_b;
  std::map<std::uint64_t, double> late_sog;
  for (const auto& scan : run.history) {
    const double rel = scan.t_unix - run.clip_start_unix;
    for (const auto& tr : scan.tracks) {
      if (a->activeAtUnix(scan.t_unix, run.clip_start_unix) &&
          (tr.pos - ca).norm() <= a->radius_m)
        in_a[tr.id] = true;
      if (b->activeAtUnix(scan.t_unix, run.clip_start_unix) &&
          (tr.pos - cb).norm() <= b->radius_m)
        in_b[tr.id] = true;
      if (rel >= 110.0 && rel <= 116.0)
        late_sog[tr.id] = std::max(late_sog[tr.id], tr.vel.norm());
    }
  }
  // Stable ids: present in the ferry region both before and after the stop->go.
  std::vector<std::uint64_t> stable;
  for (const auto& kv : in_a)
    if (in_b.count(kv.first)) stable.push_back(kv.first);

  std::cout << "\n=== R8.3 stop->go (sunset_cruise, imm_cv_ct_pmbm_land) ===\n"
            << "  ids in ferry_v1_a window: " << in_a.size()
            << " | in ferry_v1_b window: " << in_b.size()
            << " | stable across transition: " << stable.size() << "\n";
  double best_late = 0.0;
  for (auto id : stable) {
    const double s = late_sog.count(id) ? late_sog[id] : 0.0;
    best_late = std::max(best_late, s);
    std::cout << "  stable id " << id << ": late SOG=" << s << " m/s\n";
  }
  std::cout << std::flush;

  EXPECT_FALSE(stable.empty())
      << "no confirmed track kept a stable id across the ferry stop->go transition";
  EXPECT_GT(best_late, 0.5)
      << "the ferry track does not report motion (SOG) by t~110-116";
}

}  // namespace navtracker
