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
#include "adapters/replay/CameraBearingCsvReader.hpp"
#include "adapters/static/GeoJsonStaticObstacles.hpp"
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
// One emitted static hazard (LiveOccupancyModel structure component), captured in
// the datum's ENU frame so a hazard's presence at a labelled region can be scored
// the same way a track is. Populated only when an occupancy model is wired
// (empty otherwise → track-only configs are bit-identical).
struct HazardSnapshot {
  Eigen::Vector2d center;
  double footprint_m;
  double keep_clear_m;
  bool corroborated = false;   // chart-confirmed (increment 6); false unless charts wired
  bool camera_empty = false;   // camera-observed-empty; false unless camera wired
};
struct ScanTracks {
  double t_unix;
  std::vector<ConfirmedTrack> tracks;
  std::vector<HazardSnapshot> hazards;
  // Anchor-frame cell centres whose camera-observed-empty STREAK has matured this
  // scan — INDEPENDENT of structure membership / hazard emission (the raw "camera
  // proved this cell empty" fact). Empty unless an occupancy model + camera wired.
  std::vector<Eigen::Vector2d> camera_empty_cells;
  // Raw per-cell EWMA occupancy mass this scan (anchor-ENU centre, persistence),
  // exactly LiveOccupancyModel::persistenceCells(). Captured ONLY when runClip's
  // `capture_persistence` is set (default off ⇒ empty ⇒ byte-identical for every
  // existing caller); the LOS/shadow probe reads it to watch a labelled cell's mass
  // decay across a shadow interval. Snapshot is taken AFTER observe() has applied
  // this scan's coverage-aware decay + touch update.
  std::vector<std::pair<Eigen::Vector2d, double>> persistence_cells;
};
struct ClipRun {
  geo::Datum datum{geo::Geodetic{0, 0, 0}};
  double clip_start_unix{0.0};
  std::vector<ScanTracks> history;
  bool valid{false};
  // 6c coverage-sector introspection: the width (rad) of every VALID sector the
  // occupancy feed actually decayed against, plus how many collapsed to the
  // degenerate full circle. Empty unless estimate_coverage_sector is on. Lets a
  // clip assert the mechanism bit (median sector ≪ 360°) before trusting an A/B.
  std::vector<double> sector_widths_rad;
  long sector_full_circle = 0;
  // Largest fraction of live cells the LOS/shadow guard protected in any scan
  // (LiveOccupancyModel::peakGuardProtectedFraction). 0 unless an occupancy model
  // with the guard enabled was wired. A robustness signal: shadow sectors are
  // narrow, so a sane scene keeps this small.
  double peak_guard_protected_frac = 0.0;
  // Per-scan sectors (current-datum ENU) so a point's OBSERVABILITY can be
  // reconstructed post-hoc — the bug-vs-correct discriminator for a cell that
  // stays pinned as a hazard (is it genuinely unswept, or swept-empty and not
  // decaying?). Single fixed datum on these clips ⇒ current == anchor ENU.
  struct ScanSectors {
    double t_unix;
    std::vector<ISensorDetectionModel::CoverageSector> sectors;
    // This scan's raw position returns (ENU), so a probe can recompute the
    // LOS/shadow wedges post-hoc and measure how often the guard fires on a
    // labelled cell. Captured only when `capture_returns` is set on the recorder
    // (the shadow-guard probe); empty otherwise ⇒ no memory cost for other tests.
    std::vector<Eigen::Vector2d> returns;
  };
  std::vector<ScanSectors> sector_history;
};

// Transparent decorator over the LiveOccupancyModel feed face: records each
// bundle's valid coverage-sector widths, then forwards observe() unchanged. The
// occupancy model's behaviour is untouched (same call, same order) — this only
// observes what the producer estimated.
struct RecordingOccupancyFeed : ILiveOccupancyFeed {
  ILiveOccupancyFeed* inner = nullptr;
  std::vector<double>* widths = nullptr;
  long* full_circle = nullptr;
  ClipRun* run = nullptr;  // for per-scan sector_history
  bool capture_returns = false;  // LOS-guard probe: keep per-scan raw returns
  void observe(const std::vector<ISensorDetectionModel::ScanObservation>&
                   by_sensor) override {
    ClipRun::ScanSectors ss;
    ss.t_unix = by_sensor.empty() ? 0.0 : by_sensor.front().time.seconds();
    for (const auto& obs : by_sensor) {
      if (capture_returns)
        ss.returns.insert(ss.returns.end(), obs.positions.begin(),
                          obs.positions.end());
      if (!obs.coverage.valid) continue;
      if (widths) widths->push_back(obs.coverage.sector_width_rad);
      if (full_circle &&
          obs.coverage.sector_width_rad >= DetectionParams::kFullCircleRad)
        ++(*full_circle);
      ss.sectors.push_back(obs.coverage);
    }
    if (run) run->sector_history.push_back(std::move(ss));
    inner->observe(by_sensor);
  }
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
// Group loaded camera bearing-Measurements (sorted by time) into per-timestamp
// live CameraObservation frames. FOV = the center camera (~48° HFOV, optical
// axis on the bow → fov_center = the bow's absolute ENU math bearing); match
// tolerance ≈ 3σ at the fixture's σ≈3.5°. A conservative half-FOV (< the
// empirical ~23° envelope) under-claims observability (the safe direction).
inline std::vector<LiveOccupancyModel::CameraObservation> buildCameraFrames(
    const std::vector<Measurement>& cam, const OwnShipProvider& provider) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeg2Rad = kPi / 180.0;
  constexpr double kHalfFovRad = 22.0 * kDeg2Rad;
  constexpr double kMatchTolRad = 10.0 * kDeg2Rad;
  std::vector<LiveOccupancyModel::CameraObservation> frames;
  std::size_t i = 0;
  while (i < cam.size()) {
    LiveOccupancyModel::CameraObservation f;
    f.t_unix = cam[i].time.seconds();
    f.sensor_enu = cam[i].sensor_position_enu;
    f.fov_half_width_rad = kHalfFovRad;
    f.match_tolerance_rad = kMatchTolRad;
    const auto pose = provider.poseAtOrBefore(cam[i].time);
    f.fov_center_rad =
        pose ? (kPi / 2.0 - pose->heading_true_deg * kDeg2Rad) : 0.0;
    std::size_t j = i;
    while (j < cam.size() && cam[j].time.seconds() == f.t_unix) {
      if (cam[j].value.size() >= 1)
        f.detection_bearings_rad.push_back(cam[j].value(0));
      ++j;
    }
    frames.push_back(std::move(f));
    i = j;
  }
  return frames;
}

// Explicit clip source so a non-philos clip (e.g. a sim_multisensor scenario dir)
// can reuse the SAME wiring as the philos label replays. Every existing caller
// goes through runClip(clip_name, ...), which fills this from the philos fixture
// layout and reproduces the historical behaviour byte-for-byte.
struct ClipInputs {
  std::string ownship_csv;
  std::string plots_csv;
  std::string camera_csv;          // "" ⇒ no camera even if load_camera set
  std::string coastline_geojson;   // "" ⇒ no land model even if config asks
  std::string chart_structure_geojson;  // "" ⇒ no chart even if load_chart set
  std::string radar_source_id = "philos_radar";
  double radar_max_range_m = 1000.0;  // detection-model coverage range
};

inline ClipRun runClipInputs(const ClipInputs& in,
                             const std::string& config_label,
                             bool load_chart_structure = false,
                             bool load_camera = false, bool evict_camera = false,
                             bool capture_persistence = false,
                             const LiveOccupancyParams* occ_params_override =
                                 nullptr) {
  ClipRun run;
  const std::string& own = in.ownship_csv;
  const std::string& plots = in.plots_csv;
  if (!fileExists(own) || !fileExists(plots)) return run;  // fixtures absent

  const auto poses = navtracker::replay::loadOwnshipCsv(own);
  if (poses.empty()) return run;
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);
  run.clip_start_unix = poses.front().time.seconds();

  Scenario scen;
  scen.measurements = navtracker::replay::loadPlotCsvBodyFrame(
      plots, provider, SensorKind::ArpaTtm, in.radar_source_id);
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

  // Radar detection table (philos replay descriptor; max range per clip source).
  benchmark::ScenarioDescriptor desc;
  desc.label = in.radar_source_id;
  desc.detection_table = {
      {SensorKind::ArpaTtm, MeasurementModel::Position2D,
       DetectionParams{0.07, 2.7e-6, in.radar_max_range_m}}};
  MhtTracker::Config carrier;
  carrier.probability_of_detection = cfg.probability_of_detection;
  carrier.clutter_density = cfg.clutter_intensity;
  auto det = benchmark::detectionModelFor(desc, carrier, c->use_clutter_map);

  pmbm::PmbmTracker tracker(*est, cfg);
  if (det) tracker.setSensorDetectionModel(det);

  std::shared_ptr<CoastlineModel> land;
  if (c->use_land_model && !in.coastline_geojson.empty()) {
    const std::string& coast = in.coastline_geojson;
    if (fileExists(coast)) {
      auto geom = loadCoastlineGeoJson(coast, CoastlinePriorParams{});
      land = std::make_shared<CoastlineModel>(std::move(geom), *scen.datum);
      tracker.setLandModel(land.get());
    }
  }
  std::shared_ptr<LiveOccupancyModel> occ;
  RecordingOccupancyFeed rec;  // outlives runBenchPmbm; used only when occ wired
  if (c->use_live_occupancy_model) {
    // occ_params_override lets a probe force a specific LiveOccupancyParams (e.g.
    // toggle the LOS/shadow guard for an A/B on the same clip); nullptr ⇒ the
    // config's params, byte-identical for every non-probe caller.
    auto op = occ_params_override
                  ? *occ_params_override
                  : c->live_occupancy_params.value_or(LiveOccupancyParams{});
    if (c->occupancy_adaptive_clutter_bar) op.clutter_adaptive = true;
    // Increment-ii camera eviction (real-data DEMO only — promotion gates on the
    // synthetic model scenario). When on, a camera-observed-empty, chart-UNconfirmed
    // structure cell is spent; chart-confirmed cells are held (evidence precedence).
    if (evict_camera) op.evict_camera_empty = true;
    occ = std::make_shared<LiveOccupancyModel>(*scen.datum, op);
    tracker.setStaticObstacleModel(occ.get());
    rec.inner = occ.get();
    rec.widths = &run.sector_widths_rad;
    rec.full_circle = &run.sector_full_circle;
    rec.run = &run;
    rec.capture_returns = capture_persistence;  // LOS-guard probe reuses this gate
    tracker.setLiveOccupancyFeed(&rec);  // transparent; records sector widths
    // Chart corroboration (increment 6): feed the densified charted radar-visible
    // structure so emitted live hazards can be confirmed. Label only — no effect
    // on hazards/suppression/tracks (so occupancy-config runs stay bit-identical
    // with load_chart_structure=false).
    if (load_chart_structure && !in.chart_structure_geojson.empty()) {
      const std::string& charts = in.chart_structure_geojson;
      if (fileExists(charts))
        occ->setChartedStructure(loadStaticObstaclesGeoJson(charts));
    }
  }
  // Camera-observed-empty feed (increment 6): the frames are fed in the post-scan
  // hook in time order (streak advances per antenna scan; the resulting hazard
  // flag lags by at most one scan — negligible vs the sustain window).
  std::vector<LiveOccupancyModel::CameraObservation> cam_frames;
  std::size_t cam_cursor = 0;
  if (load_camera && occ && !in.camera_csv.empty()) {
    const std::string& campath = in.camera_csv;
    if (fileExists(campath))
      cam_frames = buildCameraFrames(
          navtracker::replay::loadCameraBearingsCsv(campath, provider), provider);
  }

  benchmark::PmbmPostScanHook hook = [&](const pmbm::PmbmTracker& t,
                                         Timestamp scan_t) {
    // Advance the camera-observed-empty streak with every frame up to this scan.
    if (occ && load_camera)
      while (cam_cursor < cam_frames.size() &&
             cam_frames[cam_cursor].t_unix <= scan_t.seconds())
        occ->observeCamera(cam_frames[cam_cursor++]);

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
    // Capture the emitted hazard set (ENU) so a KEEP_MIXED region can be
    // satisfied by a hazard as well as a track, and so structure decay-out is
    // observable. Empty for non-occupancy configs.
    if (occ) {
      const auto& obs = occ->obstacles();
      for (std::size_t i = 0; i < obs.size(); ++i) {
        const Eigen::Vector3d e = run.datum.toEnu(obs[i].position);
        st.hazards.push_back({Eigen::Vector2d(e.x(), e.y()),
                              obs[i].footprint_radius_m,
                              obs[i].keep_clear_radius_m,
                              occ->obstacleCorroborated(i),
                              occ->obstacleCameraObservedEmpty(i)});
      }
      st.camera_empty_cells = occ->cameraObservedEmptyCells();  // raw per-cell streak
      // Raw per-cell EWMA occupancy mass (opt-in; default off ⇒ byte-identical for
      // every existing caller). The LOS/shadow probe watches a labelled cell's mass
      // across a shadow interval. Snapshot AFTER this scan's decay + touch update.
      if (capture_persistence) st.persistence_cells = occ->persistenceCells();
    }
    run.history.push_back(std::move(st));
  };
  benchmark::runBenchPmbm(scen, tracker, hook);
  if (occ) run.peak_guard_protected_frac = occ->peakGuardProtectedFraction();
  run.valid = true;
  return run;
}

// Philos label-replay entry point (unchanged signature; byte-identical to the
// historical behaviour for every existing caller). Fills ClipInputs from the
// philos fixture layout and delegates to runClipInputs.
inline ClipRun runClip(const std::string& clip_name,
                       const std::string& config_label,
                       bool load_chart_structure = false,
                       bool load_camera = false, bool evict_camera = false,
                       bool capture_persistence = false,
                       const LiveOccupancyParams* occ_params_override = nullptr) {
  ClipInputs in;
  in.ownship_csv = clipDir(clip_name) + "/ownship.csv";
  in.plots_csv = clipDir(clip_name) + "/radar_plots.csv";
  in.camera_csv = clipDir(clip_name) + "/camera_bearings.csv";
  in.coastline_geojson = srcDir() + "/tests/fixtures/philos/boston.geojson";
  in.chart_structure_geojson =
      srcDir() + "/tests/fixtures/philos/charts/radar_structure_points.geojson";
  in.radar_source_id = "philos_radar";
  in.radar_max_range_m = 1000.0;
  return runClipInputs(in, config_label, load_chart_structure, load_camera,
                       evict_camera, capture_persistence, occ_params_override);
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
