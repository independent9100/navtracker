// R8.2 + R8.3 — label-aware philos metric decomposition + binary gates on the
// zero-AIS `sunset_cruise` clip.
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
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/land/GeoJsonCoastline.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/ExistenceLabel.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/benchmark/Sweep.hpp"  // detectionModelFor
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineModel.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {
namespace {

using benchmark::ExistenceLabel;
using benchmark::ExistenceLabelClass;

std::string srcDir() { return std::string(NAVTRACKER_SOURCE_DIR); }
std::string clipDir() {
  return srcDir() + "/tests/fixtures/philos/out/sunset_cruise";
}
bool fileExists(const std::string& p) {
  std::ifstream f(p);
  return static_cast<bool>(f);
}

struct ConfirmedTrack {
  std::uint64_t id;
  Eigen::Vector2d pos;
  Eigen::Vector2d vel;
};
struct ScanTracks {
  double t_unix;
  std::vector<ConfirmedTrack> tracks;
};
struct SunsetRun {
  geo::Datum datum{geo::Geodetic{0, 0, 0}};
  double clip_start_unix{0.0};
  std::vector<ScanTracks> history;
  bool valid{false};
};

// Run sunset_cruise (radar-only) through the tracker built from `config_label`,
// capturing all Confirmed tracks after every scan. Mirrors Sweep's PMBM wiring
// (detection model / land model / live-occupancy layer) so land vs detector is
// a pure config swap.
SunsetRun runSunset(const std::string& config_label) {
  SunsetRun run;
  const std::string own = clipDir() + "/ownship.csv";
  const std::string plots = clipDir() + "/radar_plots.csv";
  if (!fileExists(own) || !fileExists(plots)) return run;  // fixtures absent

  const auto poses = navtracker::replay::loadOwnshipCsv(own);
  if (poses.empty()) return run;
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);
  run.clip_start_unix = poses.front().time.seconds();

  Scenario scen;
  scen.measurements = navtracker::replay::loadPlotCsvBodyFrame(
      plots, provider, SensorKind::ArpaTtm, "philos_radar");
  if (scen.measurements.empty()) return run;
  std::sort(scen.measurements.begin(), scen.measurements.end(),
            [](const Measurement& a, const Measurement& b) {
              return a.time < b.time;
            });
  scen.datum = provider.datum();
  run.datum = *scen.datum;

  const auto all = benchmark::defaultConfigs();
  const benchmark::Config* c = nullptr;
  for (const auto& cc : all)
    if (cc.label == config_label) c = &cc;
  if (!c) return run;

  auto est = c->build_estimator();
  pmbm::PmbmTracker::Config cfg =
      c->pmbm_config ? c->pmbm_config() : pmbm::PmbmTracker::Config{};

  // Philos radar detection table (same as the philos replay descriptor).
  benchmark::ScenarioDescriptor desc;
  desc.label = "sunset_cruise";
  desc.detection_table = {
      {SensorKind::ArpaTtm, MeasurementModel::Position2D,
       DetectionParams{0.07, 2.7e-6, /*max_range_m=*/1000.0}}};
  MhtTracker::Config carrier;
  carrier.probability_of_detection = cfg.probability_of_detection;
  carrier.clutter_density = cfg.clutter_intensity;
  auto det = benchmark::detectionModelFor(desc, carrier, c->use_clutter_map);

  pmbm::PmbmTracker tracker(*est, cfg);
  if (det) tracker.setSensorDetectionModel(det);

  std::shared_ptr<CoastlineModel> land;
  if (c->use_land_model) {
    const std::string coast = srcDir() + "/tests/fixtures/philos/boston.geojson";
    if (fileExists(coast)) {
      auto geom = loadCoastlineGeoJson(coast, CoastlinePriorParams{});
      land = std::make_shared<CoastlineModel>(std::move(geom), *scen.datum);
      tracker.setLandModel(land.get());
    }
  }
  std::shared_ptr<LiveOccupancyModel> occ;
  if (c->use_live_occupancy_model) {
    auto op = c->live_occupancy_params.value_or(LiveOccupancyParams{});
    if (c->occupancy_adaptive_clutter_bar) op.clutter_adaptive = true;
    occ = std::make_shared<LiveOccupancyModel>(*scen.datum, op);
    tracker.setStaticObstacleModel(occ.get());
    tracker.setLiveOccupancyFeed(occ.get());
  }

  benchmark::PmbmPostScanHook hook = [&](const pmbm::PmbmTracker& t,
                                         Timestamp scan_t) {
    ScanTracks st;
    st.t_unix = scan_t.seconds();
    for (const Track& tr : t.tracks()) {
      if (tr.status != TrackStatus::Confirmed || tr.state.size() < 2) continue;
      Eigen::Vector2d vel = tr.state.size() >= 4
                                ? Eigen::Vector2d(tr.state(2), tr.state(3))
                                : Eigen::Vector2d::Zero();
      st.tracks.push_back({tr.id.value, Eigen::Vector2d(tr.state(0), tr.state(1)),
                           vel});
    }
    run.history.push_back(std::move(st));
  };
  benchmark::runBenchPmbm(scen, tracker, hook);
  run.valid = true;
  return run;
}

std::vector<ExistenceLabel> loadLabels() {
  const std::string path =
      srcDir() + "/tests/fixtures/philos/labels/sunset_cruise_labels.csv";
  std::ifstream f(path);
  if (!f.good()) return {};
  return benchmark::parseExistenceLabels(f);
}

Eigen::Vector2d labelEnu(const geo::Datum& d, const ExistenceLabel& l) {
  const Eigen::Vector3d e = d.toEnu(geo::Geodetic{l.lat_deg, l.lon_deg, 0.0});
  return Eigen::Vector2d(e.x(), e.y());
}

}  // namespace

// R8.2 — label-aware decomposition. Reported, not a threshold gate here (the
// numbers become the A/B surface increment 6 is judged against); a couple of
// sanity bounds keep the instrument honest.
TEST(PhilosSunsetLabels, LabelAwareDecomposition) {
  const SunsetRun run = runSunset("imm_cv_ct_pmbm_land");
  if (!run.valid) {
    GTEST_SKIP() << "sunset_cruise fixtures not reachable";
  }
  const auto labels = loadLabels();
  ASSERT_FALSE(labels.empty());

  // Precompute label ENU centres.
  std::vector<Eigen::Vector2d> center;
  for (const auto& l : labels) center.push_back(labelEnu(run.datum, l));

  long false_on_suppress = 0, tracks_on_keep = 0, false_unlabeled = 0;
  for (const auto& scan : run.history) {
    for (const auto& tr : scan.tracks) {
      // Which labels (active now) contain this track?
      bool in_suppress = false, in_keep = false;
      for (std::size_t i = 0; i < labels.size(); ++i) {
        const auto& l = labels[i];
        if (!l.activeAtUnix(scan.t_unix, run.clip_start_unix)) continue;
        if ((tr.pos - center[i]).norm() > l.radius_m) continue;
        if (l.label == ExistenceLabelClass::SuppressStructure)
          in_suppress = true;
        else  // KEEP_VESSEL / KEEP_ANCHORAGE / UNKNOWN default-KEEP
          in_keep = true;
      }
      if (in_keep) ++tracks_on_keep;       // KEEP dominates: a track here is right
      else if (in_suppress) ++false_on_suppress;
      else ++false_unlabeled;
    }
  }
  std::cout << "\n=== R8.2 label-aware philos decomposition (sunset_cruise, "
               "imm_cv_ct_pmbm_land) ===\n"
            << "  scans=" << run.history.size() << "\n"
            << "  tracks_on_keep   = " << tracks_on_keep
            << "  (confirmed track-scans in KEEP regions — must NOT fall)\n"
            << "  false_on_suppress= " << false_on_suppress
            << "  (confirmed track-scans in SUPPRESS regions — a suppressor should shrink)\n"
            << "  false_unlabeled  = " << false_unlabeled
            << "  (remainder)\n"
            << std::flush;
  // Instrument sanity: the clip has real moving vessels, so SOME confirmed
  // track-scans land in KEEP regions under the current tracker.
  EXPECT_GT(tracks_on_keep, 0);
}

// R8.3 gate 1 — KEEP canary. Each KEEP region must contain >=1 confirmed track
// within its radius during its window. Must pass TODAY under land.
TEST(PhilosSunsetLabels, KeepCanariesHaveTracks) {
  const SunsetRun run = runSunset("imm_cv_ct_pmbm_land");
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
        const double d = (tr.pos - c).norm();
        best = std::min(best, d);
        if (d <= l.radius_m) covered = true;
      }
    }
    std::cout << "  " << l.region_id << " (r=" << l.radius_m
              << "m): closest confirmed track = " << best
              << "m -> " << (covered ? "COVERED" : "MISSED") << "\n";
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
  const SunsetRun run = runSunset("imm_cv_ct_pmbm_land");
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
