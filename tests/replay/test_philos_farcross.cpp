// Pre-water ticket-10 smoke tests for the two released philos clips
// (ais_ferry_far, almost_cross). Skip-guarded on fixture presence (all of
// tests/fixtures/ is gitignored). These assert the clips LOAD, are
// DETERMINISTIC, and — for almost_cross, which has no truth of any kind — the
// ADR-0002 anchorage/persistence canary: the persistent radar returns must
// yield confirmed tracks that survive to end-of-clip (never vanish into
// nothing). Full per-config accuracy for ais_ferry_far lives in the bench
// (docs/baselines/2026-07-06_philos_farcross.md); it cannot be produced for
// almost_cross because the bench harness is truth-driven and this clip has
// empty truth — hence the direct-tracker canary here.
//
// Direct-tracker harness mirrors tests/replay/test_philos_ospa.cpp: a fixed 1 s
// evaluation clock rather than truth-timestamp snapshots, so it works on a
// truthless clip.

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::replay {
namespace {

std::string clipDir(const char* clip) {
  return std::string("tests/fixtures/philos/out/") + clip + "/";
}
bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

// Drive an EKF+GNN tracker over `measurements` on a fixed 1 s clock; return the
// number of confirmed tracks alive at end-of-clip and the count of distinct
// track ids ever confirmed. Config-independent smoke harness (the bench carries
// the per-config accuracy numbers for clips that have truth).
struct DriveResult {
  std::size_t final_tracks;
  std::size_t unique_ids;
};
DriveResult driveTracker(const std::vector<Measurement>& measurements) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator est(motion, 3.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(3, 8);
  Tracker tracker(est, assoc, mgr, 5.0);
  std::set<std::uint64_t> seen;
  for (const auto& m : measurements) {
    tracker.process(m);
    for (const auto& t : mgr.tracks()) seen.insert(t.id.value);
  }
  return {mgr.size(), seen.size()};
}

}  // namespace

TEST(PhilosFarCross, AisFerryFarLoadsIsDeterministicAndTrackable) {
  const std::string dir = clipDir("ais_ferry_far");
  const std::string own = dir + "ownship.csv", ais_p = dir + "ais.csv",
                    plots = dir + "radar_plots.csv";
  if (!fileExists(own) || !fileExists(ais_p) || !fileExists(plots)) {
    GTEST_SKIP() << "ais_ferry_far fixture absent — run the philos batch first.";
  }

  const auto poses = loadOwnshipCsv(own);
  ASSERT_FALSE(poses.empty());
  OwnShipProvider provider(poses.size() + 1);
  feedOwnshipHistory(provider, poses);
  ASSERT_TRUE(provider.hasDatum());

  const auto ais = loadAisCsv(ais_p, provider.datum(), "ais");
  const auto radar =
      loadPlotCsvBodyFrame(plots, provider, SensorKind::ArpaTtm, "philos_radar");
  EXPECT_FALSE(radar.empty());
  EXPECT_FALSE(ais.empty()) << "ais_ferry_far carries AIS (20 MMSIs)";

  // Determinism: reloading yields byte-identical measurement streams.
  const auto radar2 =
      loadPlotCsvBodyFrame(plots, provider, SensorKind::ArpaTtm, "philos_radar");
  ASSERT_EQ(radar.size(), radar2.size());
  for (std::size_t i = 0; i < radar.size(); ++i) {
    EXPECT_EQ(radar[i].time.nanos(), radar2[i].time.nanos());
    EXPECT_EQ(radar[i].value, radar2[i].value);
  }

  // The fused stream forms tracks (radar-only accuracy numbers are in the
  // bench doc; here we only assert trackability + determinism).
  std::vector<Measurement> both = radar;
  both.insert(both.end(), ais.begin(), ais.end());
  std::sort(both.begin(), both.end(),
            [](const Measurement& a, const Measurement& b) { return a.time < b.time; });
  const DriveResult r = driveTracker(both);
  EXPECT_GT(r.unique_ids, 0u);
  std::cerr << "[farcross/ais_ferry_far] radar=" << radar.size()
            << " ais=" << ais.size() << " final_tracks=" << r.final_tracks
            << " unique_ids=" << r.unique_ids << "\n";
}

TEST(PhilosFarCross, AlmostCrossHasNoAisAndPersistsRadarTracks) {
  const std::string dir = clipDir("almost_cross");
  const std::string own = dir + "ownship.csv", ais_p = dir + "ais.csv",
                    plots = dir + "radar_plots.csv";
  if (!fileExists(own) || !fileExists(plots)) {
    GTEST_SKIP() << "almost_cross fixture absent — run the philos batch first.";
  }

  const auto poses = loadOwnshipCsv(own);
  ASSERT_FALSE(poses.empty());
  OwnShipProvider provider(poses.size() + 1);
  feedOwnshipHistory(provider, poses);

  // almost_cross carries zero AIS and no radar_truth — documents the "no honest
  // truth" fact this clip is scored under (mechanics only).
  const auto ais = fileExists(ais_p)
                       ? loadAisCsv(ais_p, provider.datum(), "ais")
                       : std::vector<Measurement>{};
  EXPECT_TRUE(ais.empty()) << "almost_cross is expected to carry no AIS";
  EXPECT_FALSE(fileExists(dir + "radar_truth.csv"))
      << "almost_cross is expected to have no radar_truth";

  const auto radar =
      loadPlotCsvBodyFrame(plots, provider, SensorKind::ArpaTtm, "philos_radar");
  ASSERT_FALSE(radar.empty());

  // Determinism.
  const auto radar2 =
      loadPlotCsvBodyFrame(plots, provider, SensorKind::ArpaTtm, "philos_radar");
  ASSERT_EQ(radar.size(), radar2.size());

  // ADR-0002 anchorage/persistence canary: the persistent radar returns must
  // surface as confirmed tracks that survive to end-of-clip — a radar contact
  // must never be suppressed into NOTHING. (No truth exists to score accuracy;
  // this asserts presence, not correctness.)
  const DriveResult r = driveTracker(radar);
  EXPECT_GT(r.unique_ids, 0u) << "radar returns produced no confirmed track — "
                                 "ADR-0002 'never vanish into nothing' violated";
  EXPECT_GT(r.final_tracks, 0u) << "no track survived to end-of-clip";
  std::cerr << "[farcross/almost_cross] radar=" << radar.size()
            << " ais=0 final_tracks=" << r.final_tracks
            << " unique_ids=" << r.unique_ids << "\n";
}

}  // namespace navtracker::replay
