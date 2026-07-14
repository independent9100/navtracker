// Multi-sensor fusion integration test on Philos ais_ferry_near.
//
// Wires together:
//   - OwnShipProvider seeded from ownship.csv (lat, lon, heading)
//   - Radar plots via PlotCsvReplayAdapter body-frame mode
//   - AIS reports via AisCsvReplayAdapter
//   - EKF + GNN tracker
//   - OSPA scored against the same AIS reports (used as truth in
//     position space)
//
// The test runs two configurations and prints both numbers:
//   (A) radar-only:    plots through tracker, AIS as truth
//   (B) radar+AIS:     both as measurements, AIS still as truth
//
// (B)'s "truth from same source as measurement" is the standard
// caveat: it scores consistency, not absolute accuracy. The two
// numbers together tell you whether AIS fusion materially changes
// the tracker's behaviour under real radar noise.

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Ospa.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::replay {
namespace {

const std::string kOwnshipCsv =
    navtracker_test::srcAbs("tests/fixtures/philos/out/ais_ferry_near/ownship.csv");
const std::string kAisCsv =
    navtracker_test::srcAbs("tests/fixtures/philos/out/ais_ferry_near/ais.csv");
const std::string kPlotsCsv = navtracker_test::srcAbs(
    "tests/fixtures/philos/out/ais_ferry_near/radar_plots.csv");

bool fileExists(const char* path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

std::vector<Eigen::Vector2d> trackPositions(const TrackManager& mgr) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(mgr.size());
  for (const auto& t : mgr.tracks()) out.emplace_back(t.state(0), t.state(1));
  return out;
}

struct RunStats {
  double mean_ospa;
  std::size_t n_windows;
  std::size_t final_tracks;
  std::size_t unique_ids;
  double wall_s;
};

RunStats runConfig(const std::vector<Measurement>& measurements,
                   const std::vector<Measurement>& truth_as_measurement,
                   const char* tag) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator est(motion, 3.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(3, 8);
  Tracker tracker(est, assoc, mgr, 5.0);

  const Timestamp t0 = measurements.front().time;
  const Timestamp t1 = measurements.back().time;

  Timestamp next_eval = Timestamp::fromSeconds(t0.seconds() + 1.0);
  std::vector<double> ospa_per_window;
  double sum_ospa = 0.0;
  std::set<std::uint64_t> seen;

  auto evalWindow = [&](Timestamp t_eval) {
    std::vector<Eigen::Vector2d> truth_now;
    const double t = t_eval.seconds();
    for (const auto& s : truth_as_measurement) {
      const double dt = s.time.seconds() - t;
      if (dt >= -0.5 && dt <= 0.5)
        truth_now.emplace_back(s.value(0), s.value(1));
    }
    const auto tracks_now = trackPositions(mgr);
    const double d = ospaGreedy(truth_now, tracks_now, 100.0);
    ospa_per_window.push_back(d);
    sum_ospa += d;
    for (const auto& trk : mgr.tracks()) seen.insert(trk.id.value);
  };

  const auto t_start = std::chrono::steady_clock::now();
  for (const auto& m : measurements) {
    while (m.time >= next_eval) {
      evalWindow(next_eval);
      next_eval = Timestamp::fromSeconds(next_eval.seconds() + 1.0);
      if (next_eval > t1) break;
    }
    tracker.process(m);
  }
  evalWindow(t1);
  const double wall_s = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t_start)
                            .count();

  const double mean_ospa = ospa_per_window.empty()
                               ? 0.0
                               : sum_ospa / static_cast<double>(ospa_per_window.size());

  std::cerr << "[Philos/" << tag << "] "
            << "measurements=" << measurements.size()
            << "  windows=" << ospa_per_window.size()
            << "  mean OSPA=" << mean_ospa << " m"
            << "  final tracks=" << mgr.size()
            << "  unique ids=" << seen.size()
            << "  wall=" << wall_s << "s\n";

  return {mean_ospa, ospa_per_window.size(), mgr.size(), seen.size(), wall_s};
}

}  // namespace

TEST(PhilosOspa, AisFerryNearMultiSensorBaseline) {
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !fileExists(kOwnshipCsv.c_str()) || !fileExists(kAisCsv.c_str()) ||
          !fileExists(kPlotsCsv.c_str()),
      "Philos fixture data missing — run the philos batch first.");

  // 1. Build an OwnShipProvider primed with the entire ownship history
  //    so plot/AIS adapters can look up pose at-or-before any time.
  //    Default history is 16; we have hundreds of poses and want to
  //    retain them all so per-plot pose lookup works across the scene.
  const auto poses = loadOwnshipCsv(kOwnshipCsv);
  ASSERT_FALSE(poses.empty()) << "ownship CSV parsed empty";
  OwnShipProvider provider(/*history_size=*/poses.size() + 1);
  feedOwnshipHistory(provider, poses);
  ASSERT_TRUE(provider.hasDatum());
  std::cerr << "[Philos] ownship: " << poses.size() << " poses, datum at "
            << provider.datum().origin().lat_deg << "°N, "
            << provider.datum().origin().lon_deg << "°E\n";

  // 2. Load AIS as both potential measurements and truth.
  const auto ais = loadAisCsv(kAisCsv, provider.datum(), "ais");
  ASSERT_FALSE(ais.empty()) << "AIS CSV parsed empty";

  // 3. Load radar plots in body frame, projected via ownship pose.
  const auto radar = loadPlotCsvBodyFrame(kPlotsCsv, provider,
                                          SensorKind::ArpaTtm, "philos_radar");
  ASSERT_FALSE(radar.empty()) << "radar plots CSV parsed empty";

  std::cerr << "[Philos] AIS measurements: " << ais.size()
            << "  radar measurements: " << radar.size() << "\n";

  // ----- Config A: radar-only, AIS-as-truth. -----
  RunStats a = runConfig(radar, ais, "radar_only");

  // ----- Config B: radar + AIS fused. -----
  std::vector<Measurement> both;
  both.reserve(radar.size() + ais.size());
  both.insert(both.end(), radar.begin(), radar.end());
  both.insert(both.end(), ais.begin(), ais.end());
  std::sort(both.begin(), both.end(),
            [](const Measurement& x, const Measurement& y) { return x.time < y.time; });
  RunStats b = runConfig(both, ais, "radar+ais");

  // Sanity: both runs must form at least one track. We don't gate on
  // OSPA — this baseline is detector- and lifecycle-dominated (same
  // caveat as the HAXR test); the *numbers* are the deliverable, and
  // the SOTA-roadmap items will move them later.
  EXPECT_GT(a.unique_ids, 0u);
  EXPECT_GT(b.unique_ids, 0u);
  EXPECT_GT(a.n_windows, 0u);
  EXPECT_GT(b.n_windows, 0u);
}

}  // namespace navtracker::replay
