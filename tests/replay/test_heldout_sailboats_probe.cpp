// Held-out pass PROBE (2026-07-05). Runs the FROZEN Stage-1b-ii detector on the
// three held-out clips (sailboats_busy scored against the locked
// pre-registration docs/baselines/2026-07-05_heldout_preregistration_sailboats_busy.md;
// almost_cross / ais_ferry_far measured, not bet on) and dumps the evidence to
// score the eight predictions: the structure-hazard map (union over the clip,
// geo-referenced from own-ship) and the track statistics. This is a diagnostic
// dump, not a pass/fail gate — the eight predictions are scored by hand from the
// output into the held-out results doc. Fixture-gated (skips when absent).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "tests/replay/PhilosLabelReplay.hpp"
#include "tests/support/FixtureGuard.hpp"

using namespace navtracker;
using namespace navtracker::replay_test;

namespace {

double compassBearingDeg(const Eigen::Vector2d& enu) {
  // ENU: x=East, y=North. Compass bearing from North, clockwise.
  double b = std::atan2(enu.x(), enu.y()) * 180.0 / 3.14159265358979323846;
  if (b < 0) b += 360.0;
  return b;
}

// Returns true if the clip's fixtures were present and the run analysed.
// (GTEST_SKIP in a helper does NOT skip the enclosing TEST, so the fixture
// gate is applied at the TEST body via the returned validity — W2.7.)
bool dumpRun(const std::string& clip, const std::string& config) {
  const ClipRun run = runClip(clip, config, false, false, false);
  std::cout << "\n===== " << clip << " / " << config << " =====\n";
  if (!run.valid) {
    std::cout << "  (fixtures absent — skipped)\n";
    return false;
  }

  // Track stats (per-scan Confirmed track snapshots).
  std::set<std::uint64_t> ids;
  long track_scans = 0;
  std::size_t peak_hazards = 0;
  std::vector<Eigen::Vector2d> haz_all;  // every hazard centre over the clip
  for (const auto& s : run.history) {
    for (const auto& t : s.tracks) ids.insert(t.id);
    track_scans += static_cast<long>(s.tracks.size());
    peak_hazards = std::max(peak_hazards, s.hazards.size());
    for (const auto& h : s.hazards) haz_all.push_back(h.center);
  }

  std::cout << "  scans=" << run.history.size()
            << "  distinct_confirmed_ids=" << ids.size()
            << "  total_confirmed_track_scans=" << track_scans
            << "  peak_simultaneous_hazards=" << peak_hazards << "\n";

  // Union of hazard locations: greedily cluster centres within 60 m (one basin
  // feature = one cluster), report each with geo-reference from own-ship (~datum
  // origin; own-ship loiters in a ~130 m box, so range/bearing are ±~130 m).
  struct Clust { Eigen::Vector2d sum; int n; int seen; };
  std::vector<Clust> clusters;
  for (const auto& c : haz_all) {
    bool merged = false;
    for (auto& cl : clusters) {
      if ((cl.sum / cl.n - c).norm() < 60.0) {
        cl.sum += c; cl.n += 1; cl.seen += 1; merged = true; break;
      }
    }
    if (!merged) clusters.push_back({c, 1, 1});
  }
  std::cout << "  hazard clusters (structure emitted; extended_cells_min=1 ⇒ even "
               "compact cells appear):\n";
  if (clusters.empty()) std::cout << "    (none)\n";
  for (const auto& cl : clusters) {
    const Eigen::Vector2d m = cl.sum / cl.n;
    const auto g = run.datum.toGeodetic(Eigen::Vector3d(m.x(), m.y(), 0.0));
    std::cout << "    ENU(" << m.x() << "," << m.y() << ")  range="
              << m.norm() << " m  bearing=" << compassBearingDeg(m)
              << "°  lat/lon=" << g.lat_deg << "," << g.lon_deg
              << "  scan-hits=" << cl.seen << "\n";
  }

  // Pred-3 anchor: the far-bank KEEP_MIXED group from close_approach, cross-
  // validated here from a different day. Print where it lands in this clip's frame.
  const Eigen::Vector3d fb =
      run.datum.toEnu(navtracker::geo::Geodetic{42.357, -71.0837, 0.0});
  const Eigen::Vector2d fb2(fb.x(), fb.y());
  std::cout << "  [pred3] far-bank 42.357,-71.0837 → ENU(" << fb2.x() << ","
            << fb2.y() << ") range=" << fb2.norm() << " m bearing="
            << compassBearingDeg(fb2) << "°\n";
  return true;
}

}  // namespace

TEST(HeldoutSailboats, Probe) {
  // Scored clip — frozen detector (structure preds) + land baseline (pred 6).
  // W2.7: gate the whole probe on the scored clip's fixtures. Before, a silent
  // return inside dumpRun (with no assertions here) made this PASS with fixtures
  // absent — invisible to both the skip diff AND strict mode. Now strict mode
  // FAILs if the scored clip is unreachable; otherwise it skips.
  const bool ran = dumpRun("sailboats_busy", "imm_cv_ct_pmbm_occupancy_detector");
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !ran, "held-out sailboats_busy fixtures absent");
  dumpRun("sailboats_busy", "imm_cv_ct_pmbm_land");
  // Measured (no predictions), for context / tuning.
  dumpRun("almost_cross", "imm_cv_ct_pmbm_occupancy_detector");
  dumpRun("ais_ferry_far", "imm_cv_ct_pmbm_occupancy_detector");
  SUCCEED();
}
