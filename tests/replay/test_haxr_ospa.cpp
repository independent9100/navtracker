// Real-OSPA-on-real-radar test. Feeds the HAXR kattwyk_08 plot CSV
// through the tracker (EKF + GNN baseline) and scores against the AIS
// CSV that ships with the dataset.
//
// The fixture lives under tests/fixtures/haxr_cfar/. This test will be
// gtest-skipped if the CSVs are not present so the rest of the suite
// keeps building on machines that have not pulled the data bundle.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "tests/support/FixtureGuard.hpp"

#include "adapters/replay/HaxrTruthLoader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/scenario/TruthResample.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Ospa.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker::replay {
namespace {

// Default to the smaller threshold=40 / 2000-cycle CSV. Override either
// path via env vars for the first-cut iteration; once we trust the
// pipeline we point at kattwyk_08_full.csv for the full hour.
const std::string kDefaultPlotsCsv =
    navtracker_test::srcAbs("tests/fixtures/haxr_cfar/out/kattwyk_08_t40.csv");
const std::string kAisCsv =
    navtracker_test::srcAbs("data/dlr/kattwyk_08-UTC.csv");
const std::string kStationsCsv =
    navtracker_test::srcAbs("data/dlr/stations.csv");

const char* envOrDefault(const char* var, const char* def) {
  const char* v = std::getenv(var);
  return (v && *v) ? v : def;
}

bool fileExists(const char* path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

// Pull the current track centroids out of the manager so we can hand a
// vector<Vector2d> to ospaGreedy.
std::vector<Eigen::Vector2d> trackPositions(const TrackManager& mgr) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(mgr.size());
  for (const auto& t : mgr.tracks()) {
    out.emplace_back(t.state(0), t.state(1));
  }
  return out;
}

}  // namespace

TEST(HaxrOspa, KattwykHourEkfGnnBaseline) {
  const char* plots_csv =
      envOrDefault("HAXR_PLOTS_CSV", kDefaultPlotsCsv.c_str());
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(
      !fileExists(plots_csv) || !fileExists(kAisCsv.c_str()) ||
          !fileExists(kStationsCsv.c_str()),
      "HAXR data not present at " << plots_csv << " / " << kAisCsv << " / "
                                  << kStationsCsv
                                  << " — run the haxr_cfar Python fixture "
                                     "first.");

  const auto t_start = std::chrono::steady_clock::now();
  const auto stations = loadStations(kStationsCsv);
  ASSERT_FALSE(stations.empty()) << "stations.csv parsed empty";

  auto plots = loadPlotCsv(plots_csv, stations, SensorKind::ArpaTtm, "kattwyk");
  ASSERT_FALSE(plots.empty()) << "plot CSV parsed empty";
  std::cerr << "[HAXR] loaded " << plots.size() << " plots from " << plots_csv
            << "\n";

  auto truth = loadHaxrTruth(kAisCsv, "kattwyk", stations);
  ASSERT_FALSE(truth.empty()) << "AIS truth parsed empty";
  std::cerr << "[HAXR] loaded " << truth.size() << " raw AIS truth samples\n";
  // W6.2: the AIS truth is sparse (~10-20 s between fixes) and carries no
  // SOG/COG, and loadHaxrTruth hardcodes velocity to zero. Resample onto a 1 Hz
  // clock (sorted first) so velocity is finite-differenced from consecutive
  // fixes AND the OSPA windows below score against dense, non-stale truth. The
  // pre-fix mean OSPA (~199.5 m) was pegged at the 200 m cutoff because most
  // 1 Hz windows found no fix — truth sparsity, not tracker error. Shared helper,
  // same as the philos/bench replay paths.
  std::sort(truth.begin(), truth.end(),
            [](const auto& a, const auto& b) { return a.time < b.time; });
  truth = resampleTruthToClock(truth, /*period_s=*/1.0, /*max_gap_s=*/30.0);
  std::cerr << "[HAXR] resampled to " << truth.size() << " truth samples @ 1 Hz\n";

  // Restrict to the plots' time window so we score against in-window truth.
  const Timestamp t0 = plots.front().time;
  const Timestamp t1 = plots.back().time;
  std::cerr << "[HAXR] plot window " << t0.seconds() << " .. " << t1.seconds()
            << " (" << (t1.seconds() - t0.seconds()) << "s)\n";

  // Tracker. Default EKF + GNN baseline, settings sized for HAXR
  // measurement noise (sigma_r ≈ 27 m, sigma_az ≈ 1.3°).
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(3, 8);
  Tracker tracker(est, assoc, mgr, 5.0);

  // Stream the plots through the tracker. After every ~1 s of plot time,
  // pull the current track set and the truth set in that window for an
  // OSPA snapshot.
  std::vector<double> ospa_per_window;
  Timestamp next_eval = Timestamp::fromSeconds(t0.seconds() + 1.0);
  double sum_ospa = 0.0;
  std::set<std::uint64_t> seen_ids;

  auto evalWindow = [&](Timestamp t_eval) {
    // Truth: any sample within +-0.5 s of the evaluation tick.
    std::vector<Eigen::Vector2d> truth_now;
    const double t = t_eval.seconds();
    for (const auto& s : truth) {
      const double dt = s.time.seconds() - t;
      if (dt >= -0.5 && dt <= 0.5) truth_now.push_back(s.position);
    }
    const auto tracks_now = trackPositions(mgr);
    const double d = ospaGreedy(truth_now, tracks_now, 200.0);
    ospa_per_window.push_back(d);
    sum_ospa += d;
    for (const auto& trk : mgr.tracks()) seen_ids.insert(trk.id.value);
  };

  std::size_t processed = 0;
  for (const auto& m : plots) {
    while (m.time >= next_eval) {
      evalWindow(next_eval);
      next_eval = Timestamp::fromSeconds(next_eval.seconds() + 1.0);
      if (next_eval > t1) break;
    }
    tracker.process(m);
    if (++processed % 50000 == 0) {
      std::cerr << "[HAXR] processed " << processed << "/" << plots.size()
                << "  tracks=" << mgr.size() << "\n";
    }
  }
  // One final window at the tail.
  evalWindow(t1);

  ASSERT_FALSE(ospa_per_window.empty());
  const double mean_ospa = sum_ospa / static_cast<double>(ospa_per_window.size());

  const auto t_done = std::chrono::steady_clock::now();
  const double wall_s = std::chrono::duration<double>(t_done - t_start).count();

  // Report rather than assert: the goal of this first cut is to *land*
  // a real-data OSPA number for the SOTA-roadmap baseline, not to gate
  // CI on a specific value yet.
  std::cerr << "[HAXR/kattwyk_08 EKF+GNN] "
            << "windows=" << ospa_per_window.size()
            << "  mean OSPA=" << mean_ospa << " m"
            << "  final tracks=" << mgr.size()
            << "  unique ids seen=" << seen_ids.size()
            << "  wall=" << wall_s << "s\n";

  // Sanity: the tracker must form at least one confirmed track and the
  // mean OSPA must be a finite number under the cutoff.
  EXPECT_GT(seen_ids.size(), 0u);
  EXPECT_LT(mean_ospa, 200.0);
}

}  // namespace navtracker::replay
