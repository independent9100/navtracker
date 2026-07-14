#include "adapters/benchmark/ReplayScenarioRun.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
#include "adapters/replay/AutoferryJsonReplay.hpp"
#include "adapters/replay/HaxrTruthLoader.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "adapters/replay/RadarTruthCsvReader.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/scenario/Truth.hpp"
#include "core/scenario/TruthResample.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {
namespace benchmark {

namespace {

// Philos fixture clip. The four CSVs live under
// tests/fixtures/philos/out/<clip>/. The clip is selectable at runtime via the
// PHILOS_CLIP env var (see envOr + generate()); default reproduces the
// historical ais_ferry_near behaviour bit-identically. Relative paths resolve
// against the process working directory (project root when run from build/).
constexpr const char* kPhilosDefaultClip = "ais_ferry_near";

// HAXR kattwyk_08 fixture paths. Mirrors tests/replay/test_haxr_ospa.cpp.
constexpr const char* kHaxrPlotsCsv =
    "tests/fixtures/haxr_cfar/out/kattwyk_08_t40.csv";
constexpr const char* kHaxrAisCsv = "data/dlr/kattwyk_08-UTC.csv";
constexpr const char* kHaxrStationsCsv = "data/dlr/stations.csv";

// Replays read fixture CSVs by relative path; resolution depends on the
// process cwd. When the bench is launched from the project root (as the
// README documents) the paths resolve; in ctest the cwd is build/ and
// the paths don't, but the test should *skip* rather than crash. This
// helper lets generate() short-circuit to an empty Scenario when any
// required input is missing.
bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

// Read an environment variable, falling back to `def` when unset or empty.
// Used to point a replay at an alternate fixture without recompiling; an
// unset/empty var reproduces the compiled-in default bit-identically.
std::string envOr(const char* var, const char* def) {
  const char* v = std::getenv(var);
  return (v != nullptr && *v != '\0') ? std::string(v) : std::string(def);
}

// Resolve a repo-relative fixture path against NAVTRACKER_FIXTURE_ROOT (W2.7).
// The test binary points that env var at the source tree so fixture paths
// resolve under ctest (cwd = build/) from any directory; an already-absolute
// path or an unset root is returned unchanged — bit-identical for the bench
// launched from the project root.
std::string fixturePath(const std::string& rel) {
  if (!rel.empty() && rel.front() == '/') return rel;  // already absolute
  const char* root = std::getenv("NAVTRACKER_FIXTURE_ROOT");
  if (root == nullptr || *root == '\0') return rel;
  std::string r(root);
  if (!r.empty() && r.back() == '/') r.pop_back();
  return r + "/" + rel;
}

// AIS Measurement → TruthSample. The AIS adapter emits Position2D in the
// working ENU frame; we reuse those positions as ground truth, mirroring
// the Philos OSPA test that scores tracks against AIS-as-truth.
TruthSample aisMeasurementToTruth(const Measurement& m) {
  TruthSample t;
  t.time = m.time;
  if (m.value.size() >= 2) t.position = Eigen::Vector2d(m.value(0), m.value(1));
  if (m.hints.mmsi.has_value())
    t.truth_id = static_cast<std::uint64_t>(*m.hints.mmsi);
  return t;
}

class PhilosScenarioRun : public ScenarioRun {
 public:
  enum class TruthSource { AisOnly, RadarOnly, Union };

  explicit PhilosScenarioRun(TruthSource truth_source = TruthSource::AisOnly)
      : truth_source_(truth_source) {}

  ScenarioDescriptor descriptor() const override {
    const std::string label =
        (truth_source_ == TruthSource::RadarOnly)
            ? "philos_radartruth"
            : (truth_source_ == TruthSource::Union ? "philos_union" : "philos");
    ScenarioDescriptor d{label, /*is_multi_seed=*/false,
                         /*seed_count=*/1};
    // Per-sensor detection table, calibrated against interpolated
    // AIS-as-truth on the ais_ferry_near fixture (30 m gate):
    //
    // - Radar plots arrive as ~10 narrow sub-scan events/s (the
    //   rotating sweep is split per azimuth burst), so the honest
    //   per-EVENT P_D for an in-coverage vessel is low: measured
    //   0.07 across 187 (vessel × event) opportunities. λ from the
    //   unmatched-plot rate over the plot bounding box: 9.8/event /
    //   3.6e6 m² ≈ 2.7e-6 m⁻²; max plot range ≈ 980 m carried as
    //   coverage. Boston-harbor caveat: most unmatched plots are
    //   persistent shore/moored-structure returns, not Poisson clutter
    //   (same lesson as the AutoFerry urban cameras — backlog §5).
    // - AIS is a per-vessel broadcast, not a surveillance sweep: any
    //   single AIS scan event "detects" exactly one of the ~23 vessels
    //   in range, so the per-event P_D for an arbitrary vessel is
    //   ≈ 1/23 ≈ 0.05, with essentially zero clutter.
    d.detection_table = {
        {SensorKind::ArpaTtm, MeasurementModel::Position2D,
         DetectionParams{0.07, 2.7e-6, /*max_range_m=*/1000.0}},
        {SensorKind::Ais, MeasurementModel::Position2D,
         DetectionParams{0.05, 1e-9}},
    };
    // Philos: Boston Harbor coastline GeoJSON for land-prior wiring.
    // Sweep checks existence before loading; missing file → no land model.
    d.coastline_geojson_path =
        fixturePath("tests/fixtures/philos/boston.geojson");
    return d;
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    // Clip selectable at runtime (PHILOS_CLIP); unset/empty ⇒ ais_ferry_near,
    // bit-identical to the historical hardcoded paths.
    const std::string dir =
        fixturePath(std::string("tests/fixtures/philos/out/") +
                    envOr("PHILOS_CLIP", kPhilosDefaultClip) + "/");
    const std::string ownship_csv = dir + "ownship.csv";
    const std::string ais_csv = dir + "ais.csv";
    const std::string plots_csv = dir + "radar_plots.csv";
    const std::string radar_truth_csv = dir + "radar_truth.csv";

    const bool need_ais = (truth_source_ == TruthSource::AisOnly ||
                           truth_source_ == TruthSource::Union);
    if (!fileExists(ownship_csv) || !fileExists(plots_csv)) {
      return {};  // fixtures absent — caller skips
    }
    if (need_ais && !fileExists(ais_csv)) return {};
    if ((truth_source_ == TruthSource::RadarOnly ||
         truth_source_ == TruthSource::Union) &&
        !fileExists(radar_truth_csv)) {
      return {};
    }

    // Prime an OwnShipProvider from the full ownship history so plot/AIS
    // adapters can look up pose at-or-before any time.
    const auto poses = navtracker::replay::loadOwnshipCsv(ownship_csv);
    if (poses.empty()) return {};
    OwnShipProvider provider(/*history_size=*/poses.size() + 1);
    navtracker::replay::feedOwnshipHistory(provider, poses);

    // Radar plots in body frame (Philos is a moving platform).
    const auto radar = navtracker::replay::loadPlotCsvBodyFrame(
        plots_csv, provider, SensorKind::ArpaTtm, "philos_radar");

    Scenario s;

    // Radar-only MEASUREMENT mode (PHILOS_RADAR_ONLY set/non-empty): exclude AIS
    // from the measurement set so the arm consumes radar alone. This is what
    // makes an HONEST accuracy number possible on a clip whose only truth is
    // AIS-derived — a radar-only arm scored against AIS(-projected) truth does
    // not consume the truth source, so it is not circular (see the circularity
    // rule in docs/superpowers/plans/2026-07-06-philos-farcross-measurement-ticket.md).
    // AIS truth is still built below regardless. Unset/empty ⇒ AIS fed as usual,
    // bit-identical to the historical behaviour.
    const bool radar_only = !envOr("PHILOS_RADAR_ONLY", "").empty();

    // Build measurement set — AIS included as measurements unless radar-only
    // (it provides vessel IDs / AIS tracks); AIS measurements are not the same
    // as AIS truth, so the truth build below is independent of this gate.
    // #20 pricing (mechanics only — philos truth is AIS-derived, so a velocity
    // arm scored against it is CIRCULAR; labelled as such in the eval-log).
    // Default OFF => byte-identical Position2D.
    const bool ais_velocity = !envOr("PHILOS_AIS_VELOCITY", "").empty();
    if (fileExists(ais_csv)) {
      const auto ais = navtracker::replay::loadAisCsv(
          ais_csv, provider.datum(), "ais", ais_velocity);
      s.measurements.reserve(radar.size() + (radar_only ? 0 : ais.size()));
      if (!radar_only)
        s.measurements.insert(s.measurements.end(), ais.begin(), ais.end());

      if (truth_source_ == TruthSource::AisOnly ||
          truth_source_ == TruthSource::Union) {
        // AIS as truth (built even in radar-only mode — that is the honest
        // radar-vs-AIS-truth scoring the whole pass exists to produce).
        s.truth.reserve(ais.size());
        for (const auto& m : ais) s.truth.push_back(aisMeasurementToTruth(m));
      }
    } else {
      s.measurements.reserve(radar.size());
    }
    s.measurements.insert(s.measurements.end(), radar.begin(), radar.end());
    std::sort(s.measurements.begin(), s.measurements.end(),
              [](const Measurement& a, const Measurement& b) {
                return a.time < b.time;
              });

    // Build truth set according to truth_source_.
    if (truth_source_ == TruthSource::RadarOnly ||
        truth_source_ == TruthSource::Union) {
      // radar_truth.csv is AIS positions analytically PROJECTED into the
      // radar's range/azimuth frame (build_truth.py, uid = MMSI). It is
      // therefore NOT independent of AIS — it is the same truth expressed in
      // radar coordinates, useful as a projection/datum consistency check and
      // for expressing truth in the sensor frame, but it does NOT break AIS
      // circularity. Scoring an AIS-consuming arm against it is still circular
      // (see docs/algorithms/evaluation-log.md, 2026-07-06 correction). Honest
      // uses: radar-only / camera-bearing arms scored against it.
      auto radar_truth = navtracker::replay::loadRadarTruthCsv(
          radar_truth_csv, provider);
      s.truth.insert(s.truth.end(), radar_truth.begin(), radar_truth.end());
    }

    std::sort(s.truth.begin(), s.truth.end(),
              [](const TruthSample& a, const TruthSample& b) {
                return a.time < b.time;
              });
    // Resample onto a shared 1 Hz evaluation clock so BenchRunner's
    // exact-time bucketing sees honest multi-target steps.
    // 30 s max gap comfortably bridges report intervals without bridging
    // real dropouts (AIS: 8–12 s; radar_truth: sub-second).
    s.truth = resampleTruthToClock(s.truth, /*period_s=*/1.0,
                                   /*max_gap_s=*/30.0);
    // Expose the ENU datum so Sweep can construct datum-aware components
    // (e.g. CoastlineModel) without re-parsing the own-ship history.
    // The datum is fixed for the whole bench run (feedOwnshipHistory
    // processes all history before any tracking, so no recentering occurs).
    s.datum = provider.datum();
    return s;
  }

 private:
  TruthSource truth_source_{TruthSource::AisOnly};
};

// Env override with a default, for the increment-8 HAXR occupancy A/B: point the
// scenario at a decimated per-station CSV without recompiling. Empty/unset env
// reproduces the original kattwyk_08_t40 fixture. Thin alias over envOr.
std::string haxrEnv(const char* var, const char* def) {
  return envOr(var, def);
}

class HaxrScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    // Label stays "haxr" (the bench --with-haxr gate and test_replay_scenario_run
    // key on it); per-station runs are distinguished by the bench --run-id.
    return {"haxr", /*is_multi_seed=*/false, /*seed_count=*/1};
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    const std::string plots_csv = fixturePath(haxrEnv("HAXR_PLOTS_CSV", kHaxrPlotsCsv));
    const std::string ais_csv = fixturePath(haxrEnv("HAXR_AIS_CSV", kHaxrAisCsv));
    const std::string stations_csv =
        fixturePath(haxrEnv("HAXR_STATIONS_CSV", kHaxrStationsCsv));
    const std::string station = haxrEnv("HAXR_STATION", "kattwyk");
    if (!fileExists(plots_csv.c_str()) || !fileExists(ais_csv.c_str()) ||
        !fileExists(stations_csv.c_str())) {
      return {};
    }
    const auto stations = navtracker::replay::loadStations(stations_csv);
    auto plots = navtracker::replay::loadPlotCsv(
        plots_csv, stations, SensorKind::ArpaTtm, station);
    auto truth = navtracker::replay::loadHaxrTruth(ais_csv, station, stations);

    Scenario s;
    s.measurements = std::move(plots);  // loadPlotCsv already sorts by time
    s.truth = std::move(truth);
    // loadHaxrTruth returns raw CSV order with no ordering guarantee, but
    // BenchRunner::groupTruth buckets on exact-time changes and requires truth
    // sorted by non-decreasing time (unsorted truth silently fragments into
    // duplicate groups). Sort here, matching the philos/autoferry replay paths.
    std::sort(s.truth.begin(), s.truth.end(),
              [](const TruthSample& a, const TruthSample& b) {
                return a.time < b.time;
              });
    // Nominal fixed anchor. HAXR is a LOCAL METRE frame (stations.csv is
    // x_m/y_m — no geodetic origin), so the datum's lat/lon is only a label; it
    // exists so the Stage-1b LiveOccupancyModel / land / obstacle wiring (gated
    // on scen.datum.has_value(), Sweep.cpp) activates on HAXR. Never used for
    // recenter (fixed datum per run). Without this the occupancy A/B is a no-op:
    // the ON arm would be bit-identical to OFF because the model is never wired.
    s.datum = geo::Datum(geo::Geodetic{53.53, 9.95, 0.0});  // Hamburg port, nominal

    // Increment-8 AIS THIRD ARM (HAXR_FEED_AIS set): additionally feed the AIS
    // vessel positions as SensorKind::Ais Position2D MEASUREMENTS (not just
    // truth), so the corroboration veto (observeVesselFix) fires on real AIS
    // traffic. CIRCULARITY (hard rule): AIS is both input AND truth here, so this
    // arm measures veto MECHANICS + phantom behaviour ONLY — never accuracy-vs-AIS
    // (that would be scoring against the same data we fed). Off by default → the
    // radar-only core A/B is unaffected.
    if (!haxrEnv("HAXR_FEED_AIS", "").empty()) {
      for (const auto& t : s.truth) {
        Measurement m;
        m.time = t.time;
        m.sensor = SensorKind::Ais;
        m.source_id = "ais_" + station;
        m.model = MeasurementModel::Position2D;
        m.value = t.position;
        m.covariance = Eigen::Matrix2d::Identity() * (30.0 * 30.0);  // AIS default σ
        m.hints.mmsi = static_cast<std::uint32_t>(t.truth_id & 0xFFFFFFFFu);
        s.measurements.push_back(std::move(m));
      }
      std::sort(s.measurements.begin(), s.measurements.end(),
                [](const Measurement& a, const Measurement& b) {
                  return a.time < b.time;
                });
    }
    return s;
  }
};

// One AutoFerry scenario folder under data/autoferry/<label>/. generate()
// returns empty (caller skips) when the JSON files are not reachable from
// cwd, mirroring the philos/haxr fixture-absent behaviour.
class AutoferryScenarioRun : public ScenarioRun {
 public:
  AutoferryScenarioRun(std::string label, bool inject_truth_anchor)
      : label_(std::move(label)),
        inject_truth_anchor_(inject_truth_anchor) {}

  ScenarioDescriptor descriptor() const override {
    const std::string suffix =
        inject_truth_anchor_ ? "_anchored" : "";
    ScenarioDescriptor d{"autoferry_" + label_ + suffix,
                         /*is_multi_seed=*/false,
                         /*seed_count=*/1};
    // Per-sensor detection table, calibrated against the published
    // ground truth (matching gate 15 m position / 0.15 rad bearing):
    //
    //   sensor  empirical P_D  λ_C (units)        coverage
    //   lidar   0.40–0.71      ≈ 5e-6 m⁻²         ≤135 m observed
    //   radar   0.71–0.91      ≈ 1e-5 m⁻²         (region-cropped)
    //   EO cam  0.62–0.87      see below, rad⁻¹
    //   IR cam  0.21–0.57      see below, rad⁻¹
    //
    // The lidar P_D is depressed by out-of-coverage opportunities in
    // the empirical count; with the 140 m range gate carried in
    // max_range_m, the in-coverage value is higher → 0.7.
    //
    // EO and IR share SensorKind::EoIr but are distinct physical
    // sensors, so they get source-keyed entries (backlog item 4):
    // measured per-camera across all nine ground-truthed scenarios
    // (0.15 rad gate), EO P_D ≈ 0.73 aggregate vs IR ≈ 0.46, split per
    // environment — open water (scenarios 2–6) EO 0.62–0.87 / IR
    // 0.34–0.57, urban channel (13/16/17/22) EO 0.78–0.82 / IR
    // 0.21–0.54.
    //
    // λ_C deliberately stays at the kind-wide 0.5 rad⁻¹ for all camera
    // entries. The measured unmatched-bearing rate splits hugely by
    // environment (open water 0.004–0.6 rad⁻¹, urban 1.0–4.9 rad⁻¹),
    // but the urban excess is persistent structured returns (shoreline,
    // moored vessels) — not uniform Poisson clutter. Feeding the
    // ML-fitted uniform λ into the score charges every camera hit
    // ~2 nats, including hits on true targets, and was measured
    // (2026-06-11_eoir_split_measured_lambda baseline) to collapse
    // urban lifetime 0.65→0.35 / 0.77→0.59 / 0.71→0.44. When the model
    // family is wrong, the honestly-fitted parameter is not the right
    // operating point; the spatial clutter map (backlog item 5) is the
    // vehicle for shoreline clutter, not a bigger uniform λ.
    const bool urban = (label_ == "scenario13" || label_ == "scenario16" ||
                        label_ == "scenario17" || label_ == "scenario22");
    const DetectionParams eo =
        urban ? DetectionParams{0.8, 0.5} : DetectionParams{0.7, 0.5};
    const DetectionParams ir =
        urban ? DetectionParams{0.4, 0.5} : DetectionParams{0.5, 0.5};
    d.detection_table = {
        {SensorKind::Lidar, MeasurementModel::Position2D,
         DetectionParams{0.7, 5e-6, /*max_range_m=*/140.0}},
        {SensorKind::ArpaTtm, MeasurementModel::Position2D,
         DetectionParams{0.8, 1e-5}},
        {SensorKind::EoIr, MeasurementModel::Bearing2D,
         DetectionParams{0.6, 0.5}},
        {SensorKind::EoIr, MeasurementModel::Bearing2D, eo, "autoferry_eo"},
        {SensorKind::EoIr, MeasurementModel::Bearing2D, ir, "autoferry_ir"},
    };
    // Real Trondheim inner-harbour coastline (OSM/Overpass, ODbL) about the
    // Piren datum set in generate(). Enables the land model on AutoFerry for the
    // use_land_model configs — the real-geometry reality check for the land
    // clutter prior and the land-aware PDA pool. Inert for non-land configs
    // (loaded only when config.use_land_model && scen.datum is set).
    d.coastline_geojson_path =
        fixturePath("tests/fixtures/autoferry/trondheim_harbor.geojson");
    return d;
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    // Full four-sensor fusion: radar + lidar (active → Position2D) plus
    // EO + IR (passive → Bearing2D). Bearings refine existing tracks but
    // never initiate one — the track-birth paths drop non-gating bearings
    // (canInitiateTrack), matching the paper's active-only initiation.
    navtracker::replay::AutoferryLoadOptions opts;
    opts.include_bearings = true;
    opts.inject_truth_anchor = inject_truth_anchor_;
    // Per-env R override (item 12 (a) refinement). The header defaults
    // match env 1; env 2's operational σ is meaningfully smaller, so
    // we override down for the urban-channel scenarios.
    const bool urban = (label_ == "scenario13" || label_ == "scenario16" ||
                        label_ == "scenario17" || label_ == "scenario22");
    if (urban) {
      opts.lidar_pos_std_m = 3.0;
      opts.radar_pos_std_m = 5.0;
      opts.bearing_std_rad = 0.0925;  // ~5.3°, slightly above env-2 σ.
      // Do NOT tighten below ~0.088 rad: 2026-06-19 (Cl-2 #4) measured
      // 0.0925 → 0.06 against the gated canonical and saw catastrophic
      // env-2 anchored regression (sc17 GOSPA +88%, sc16 +63%, sc22
      // +19%) and env-2 unanchored NEES p99 blow-up. Step 2's NIS α̂
      // recommendation was misleading: α̂ = innovation² / (HPH^T+R) is
      // small because the bias estimator removes systematic offset
      // post-debias, not because R is loose. The physical sensor noise
      // floor (~0.088 rad on env-2 empirical residual) bounds how
      // tight R can be. See eval-log entry "Cl-2 #4 close-out".
    }
    Scenario s = navtracker::replay::loadAutoferryScenario(
        fixturePath(std::string("data/autoferry/") + label_), label_, opts);
    // The AutoFerry loader frames everything in the Piren local tangent plane
    // (NED origin LLA [63.4389029083, 10.39908278, 39.923]) but leaves
    // Scenario::datum unset, so Sweep never wires a land model here (the reason
    // AutoFerry has been "chartless"). Set the true Piren datum so the real
    // Trondheim-harbour coastline (descriptor().coastline_geojson_path) lines up
    // with the ENU measurements. Inert for every config with use_land_model =
    // false (nothing else reads scen.datum), so all standing baselines are
    // byte-identical; it only activates the land model for the land configs.
    s.datum = geo::Datum(geo::Geodetic{63.4389029083, 10.39908278, 39.923});
    return s;
  }

  // Seed the bias estimator with the offline-calibrated EO/IR bearing
  // bias for env-2 scenarios. Numbers from tools/autoferry_r_calibration.py
  // (per-scenario report, 2026-06-16): env-2 IR mean ≈ 4.9°, EO mean ≈
  // 7.0° (pooled across sc13/16/17/22). Per-scenario refinement is
  // possible but the pooled prior is already tight enough that the
  // online refinement (anchor pairs) takes it the rest of the way in
  // tens of observations.
  //
  // Why seed at all: sc13's 14 ID switches keep resetting the
  // recent_contributions window, so the bearing estimator's online
  // path never accumulates enough effective pairs to publish (its
  // variance plateaus at σ ≈ 0.26°, just below the 0.3° publish
  // threshold, but only after enough convergence which doesn't happen).
  // A seeded prior with publish-immediately variance closes that
  // chicken-and-egg loop. setKnownBearingBias is the supported API.
  //
  // Env-1 scenarios deliberately skip the seed: their bias is small
  // (3-4°), the online path catches it without help, and a wrong-env
  // seed would actively distort the first few hundred observations.
  void seedSensorBiasEstimator(SensorBiasEstimator& est) const override {
    const bool urban = (label_ == "scenario13" || label_ == "scenario16" ||
                        label_ == "scenario17" || label_ == "scenario22");
    if (!urban) return;
    // 0.3° σ → variance 2.7e-5 rad², equal to the publish threshold:
    // the seed publishes immediately and online observations
    // refine. We do not over-tighten the prior — the offline
    // calibration is "right environment, wrong scenario in detail",
    // so the online path needs room to adjust.
    constexpr double kSigmaSeedRad = 5.2e-3;  // 0.3°
    constexpr double kVarSeedRad2 = kSigmaSeedRad * kSigmaSeedRad;
    constexpr double kEoBiasRad = 0.122;  // 7.0°
    constexpr double kIrBiasRad = 0.085;  // 4.9°
    est.setKnownBearingBias({SensorKind::EoIr, "autoferry_eo"},
                            kEoBiasRad, kVarSeedRad2);
    est.setKnownBearingBias({SensorKind::EoIr, "autoferry_ir"},
                            kIrBiasRad, kVarSeedRad2);
  }

 private:
  std::string label_;
  bool inject_truth_anchor_{false};
};

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  out.reserve(3);
  out.push_back(std::make_unique<PhilosScenarioRun>(
      PhilosScenarioRun::TruthSource::AisOnly));
  // philos_radartruth: AIS truth expressed in the radar's range/azimuth frame
  // (build_truth.py projects AIS lat/lon → range/azimuth, uid = MMSI). This is
  // NOT independent of AIS — same truth, different frame. It is a
  // projection/datum consistency check vs `philos`, and lets truth be scored in
  // the sensor frame; it does NOT de-circularize an AIS-consuming arm. (The old
  // "independent radar-derived truth — kills the AIS leak" claim here was false;
  // corrected 2026-07-06, see evaluation-log.)
  out.push_back(std::make_unique<PhilosScenarioRun>(
      PhilosScenarioRun::TruthSource::RadarOnly));
  out.push_back(std::make_unique<HaxrScenarioRun>());
  return out;
}

std::vector<std::unique_ptr<ScenarioRun>> defaultAutoferryScenarios() {
  // The nine published AutoFerry scenarios with ground-truth coverage.
  // Env 1 (open water, Gunnerus+Havfruen): 2–6. Env 2 (urban channel,
  // Jetboat+Havfruen): 13, 16, 17, 22.
  static const char* kLabels[] = {"scenario2",  "scenario3",  "scenario4",
                                   "scenario5",  "scenario6",  "scenario13",
                                   "scenario16", "scenario17", "scenario22"};
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (const char* label : kLabels)
    out.push_back(
        std::make_unique<AutoferryScenarioRun>(label,
                                                /*inject_truth_anchor=*/false));
  return out;
}

std::vector<std::unique_ptr<ScenarioRun>> defaultAutoferryScenariosAnchored() {
  // AutoFerry with synthetic AIS-style truth anchor enabled — the
  // dataset itself ships no AIS, so this is the path that lets the
  // SensorBiasEstimator actually converge. Used by the
  // imm_cv_ct_mht_biascal_anchored bench config (item 9 option 1).
  static const char* kLabels[] = {"scenario2",  "scenario3",  "scenario4",
                                   "scenario5",  "scenario6",  "scenario13",
                                   "scenario16", "scenario17", "scenario22"};
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (const char* label : kLabels)
    out.push_back(
        std::make_unique<AutoferryScenarioRun>(label,
                                                /*inject_truth_anchor=*/true));
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
