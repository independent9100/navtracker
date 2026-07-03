#pragma once
// Shared harness for LABEL-SCORED replay of a zero-AIS / no-radar-truth philos
// clip (R8.2/R8.3 sunset_cruise, R8.6 close_approach, later HAXR). These clips
// have no kinematic truth, so the only evaluation surface is the video-derived
// existence/region labels (tests/fixtures/philos/labels/<clip>_labels.csv,
// loaded by core/benchmark/ExistenceLabel). Labels are NEVER converted to
// TruthSamples (would be circular + corrupt GOSPA localisation).
//
// runClip() mirrors Sweep's PMBM wiring (detection model / land model /
// live-occupancy layer) so a land-vs-detector comparison is a pure config swap;
// it captures every Confirmed track after each scan. decompose() scores those
// track-scans against the labels into the three un-gameable columns.
//
// NOTE (R8.6): KEEP_MIXED is presence-gated (a track OR an emitted static hazard
// satisfies). This harness captures TRACKS only — correct and complete under
// non-suppressing configs (imm_cv_ct_pmbm_land emits no hazards), which is where
// the KEEP-stress baseline is established. The hazard branch is added here when a
// suppressor config (increment 6 detector) is first scored against these clips.
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

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
namespace replay_test {

struct ConfirmedTrack {
  std::uint64_t id;
  Eigen::Vector2d pos;
  Eigen::Vector2d vel;
};
struct ScanTracks {
  double t_unix;
  std::vector<ConfirmedTrack> tracks;
};
struct ClipRun {
  geo::Datum datum{geo::Geodetic{0, 0, 0}};
  double clip_start_unix{0.0};
  std::vector<ScanTracks> history;
  bool valid{false};
};

inline std::string srcDir() { return std::string(NAVTRACKER_SOURCE_DIR); }
inline std::string clipDir(const std::string& clip_name) {
  return srcDir() + "/tests/fixtures/philos/out/" + clip_name;
}
inline bool fileExists(const std::string& p) {
  std::ifstream f(p);
  return static_cast<bool>(f);
}

// Run philos clip `clip_name` (radar-only) through the tracker built from
// `config_label`, capturing all Confirmed tracks after every scan.
inline ClipRun runClip(const std::string& clip_name,
                       const std::string& config_label) {
  ClipRun run;
  const std::string own = clipDir(clip_name) + "/ownship.csv";
  const std::string plots = clipDir(clip_name) + "/radar_plots.csv";
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
  desc.label = clip_name;
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
      st.tracks.push_back(
          {tr.id.value, Eigen::Vector2d(tr.state(0), tr.state(1)), vel});
    }
    run.history.push_back(std::move(st));
  };
  benchmark::runBenchPmbm(scen, tracker, hook);
  run.valid = true;
  return run;
}

inline std::vector<benchmark::ExistenceLabel> loadLabels(
    const std::string& labels_filename) {
  const std::string path =
      srcDir() + "/tests/fixtures/philos/labels/" + labels_filename;
  std::ifstream f(path);
  if (!f.good()) return {};
  return benchmark::parseExistenceLabels(f);
}

inline Eigen::Vector2d labelEnu(const geo::Datum& d,
                                const benchmark::ExistenceLabel& l) {
  const Eigen::Vector3d e = d.toEnu(geo::Geodetic{l.lat_deg, l.lon_deg, 0.0});
  return Eigen::Vector2d(e.x(), e.y());
}

// Label-aware decomposition of confirmed track-scans. KEEP dominates: a track
// inside any active KEEP/KEEP_MIXED/KEEP_ANCHORAGE/UNKNOWN region is "right"
// (tracks_on_keep — MUST NOT fall under a suppressor). A track only in a
// SUPPRESS region is false mass a suppressor should shrink (false_on_suppress).
// Everything else is false_unlabeled.
struct Decomposition {
  long tracks_on_keep = 0;
  long false_on_suppress = 0;
  long false_unlabeled = 0;
};
inline Decomposition decompose(
    const ClipRun& run,
    const std::vector<benchmark::ExistenceLabel>& labels) {
  std::vector<Eigen::Vector2d> center;
  center.reserve(labels.size());
  for (const auto& l : labels) center.push_back(labelEnu(run.datum, l));

  Decomposition d;
  for (const auto& scan : run.history) {
    for (const auto& tr : scan.tracks) {
      bool in_suppress = false, in_keep = false;
      for (std::size_t i = 0; i < labels.size(); ++i) {
        const auto& l = labels[i];
        if (!l.activeAtUnix(scan.t_unix, run.clip_start_unix)) continue;
        if ((tr.pos - center[i]).norm() > l.radius_m) continue;
        if (l.label == benchmark::ExistenceLabelClass::SuppressStructure)
          in_suppress = true;
        else  // KEEP_VESSEL / KEEP_MIXED / KEEP_ANCHORAGE / UNKNOWN default-KEEP
          in_keep = true;
      }
      if (in_keep) ++d.tracks_on_keep;
      else if (in_suppress) ++d.false_on_suppress;
      else ++d.false_unlabeled;
    }
  }
  return d;
}

}  // namespace replay_test
}  // namespace navtracker
