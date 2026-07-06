#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
#include "adapters/replay/CameraBearingCsvReader.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {
namespace benchmark {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

std::string envOr(const char* var, const char* def) {
  const char* v = std::getenv(var);
  return (v != nullptr && *v != '\0') ? std::string(v) : std::string(def);
}

// Whether a CSV has at least one data row beyond the header.
bool hasDataRows(const std::string& path) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  if (!std::getline(f, line)) return false;  // header
  while (std::getline(f, line)) {
    if (!line.empty()) return true;
  }
  return false;
}

// Load the sim truth CSV into TruthSamples WITH velocity. This is the piece the
// existing replay truth loaders can't provide: HaxrTruthLoader / RadarTruthCsv
// carry velocity=0 (range/az snapshots) and philos derives truth from AIS
// itself. Here truth is the generator's independent ground truth, so we carry
// its SOG/COG through to a real ENU velocity. Kept local to the bench adapter
// (the "bench/test infrastructure only" boundary — nothing enters the core or
// nmea library surface).
//
// Columns (see tests/fixtures/sim_multisensor/generator/writer.py TRUTH_COLS):
//   unix_time,truth_id,mmsi,lat,lon,sog_mps,cog_deg,heading_deg,nav_status
// COG is marine (deg, N=0 CW): v_east = sog*sin(cog), v_north = sog*cos(cog).
std::vector<TruthSample> loadSimTruthCsv(const std::string& path,
                                         const geo::Datum& datum) {
  std::vector<TruthSample> out;
  std::ifstream f(path);
  if (!f) return out;
  std::string line;
  if (!std::getline(f, line)) return out;  // header
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tod, id, mmsi, lat, lon, sog, cog, hdg, nav;
    if (!std::getline(ss, tod, ',')) continue;
    if (!std::getline(ss, id, ',')) continue;
    if (!std::getline(ss, mmsi, ',')) continue;
    if (!std::getline(ss, lat, ',')) continue;
    if (!std::getline(ss, lon, ',')) continue;
    if (!std::getline(ss, sog, ',')) continue;
    if (!std::getline(ss, cog, ',')) continue;
    TruthSample t;
    t.time = Timestamp::fromSeconds(std::strtod(tod.c_str(), nullptr));
    t.truth_id = static_cast<std::uint64_t>(std::strtoull(id.c_str(), nullptr, 10));
    const double lat_d = std::strtod(lat.c_str(), nullptr);
    const double lon_d = std::strtod(lon.c_str(), nullptr);
    const double sog_d = std::strtod(sog.c_str(), nullptr);
    const double cog_r = std::strtod(cog.c_str(), nullptr) * kDeg2Rad;
    const Eigen::Vector3d enu = datum.toEnu({lat_d, lon_d, 0.0});
    t.position = Eigen::Vector2d(enu.x(), enu.y());
    t.velocity = Eigen::Vector2d(sog_d * std::sin(cog_r), sog_d * std::cos(cog_r));
    out.push_back(std::move(t));
  }
  std::sort(out.begin(), out.end(), [](const TruthSample& a, const TruthSample& b) {
    if (a.time.seconds() != b.time.seconds()) return a.time < b.time;
    return a.truth_id < b.truth_id;
  });
  return out;
}

class SimMultisensorScenarioRun : public ScenarioRun {
 public:
  explicit SimMultisensorScenarioRun(std::string label) : label_(std::move(label)) {}

  ScenarioDescriptor descriptor() const override {
    ScenarioDescriptor d{label_, /*is_multi_seed=*/false, /*seed_count=*/1};
    // Per-sensor detection table (multi-sensor path). Unlike real data, the
    // sim has KNOWN P_D / λ_C, so these are the generator's true values — with
    // ONE deliberate exception: λ_C is left at the clean-Poisson baseline even
    // for sim_ms_clutter_burst, whose truth uses a compound-K field. That
    // mismatch is the point (ticket §5): a uniform-λ model should measurably
    // under-perform a spatially-varying one there.
    d.detection_table = {
        // radar: ~0.9 per-scan P_D for an in-coverage target; λ from the
        // generator's clutter intensity (2e-8 m^-2 per scan); 8 km coverage.
        {SensorKind::ArpaTtm, MeasurementModel::Position2D,
         DetectionParams{0.9, 2e-8, /*max_range_m=*/8000.0}},
        // AIS is a per-vessel broadcast, not a surveillance sweep; essentially
        // clutter-free.
        {SensorKind::Ais, MeasurementModel::Position2D,
         DetectionParams{0.5, 1e-9}},
        // camera bearing-only: corroborates, never births (canInitiateTrack ==
        // false in the loader), so this mostly informs miss modelling.
        {SensorKind::EoIr, MeasurementModel::Bearing2D,
         DetectionParams{0.85, 1e-9, 8000.0}},
    };
    return d;
  }

  Scenario generate(std::uint64_t seed) override {
    const std::string dir = envOr("SIMMS_DIR", "tests/fixtures/sim_multisensor") +
                            "/" + label_ + "_s" + std::to_string(seed) + "/";
    const std::string ownship_csv = dir + "ownship.csv";
    const std::string ais_csv = dir + "ais.csv";
    const std::string plots_csv = dir + "radar_plots.csv";
    const std::string camera_csv = dir + "camera_bearings.csv";
    const std::string truth_csv = dir + "truth.csv";

    // Radar + own-ship + truth are mandatory; AIS / camera optional per scenario.
    if (!fileExists(ownship_csv) || !fileExists(plots_csv) || !fileExists(truth_csv)) {
      return {};  // fixtures absent — caller skips
    }

    const auto poses = navtracker::replay::loadOwnshipCsv(ownship_csv);
    if (poses.empty()) return {};
    OwnShipProvider provider(/*history_size=*/poses.size() + 1);
    navtracker::replay::feedOwnshipHistory(provider, poses);

    // RADAR-ONLY arm (SIMMS_RADAR_ONLY set/non-empty): exclude the AIS and
    // camera measurements so the arm consumes radar alone. Truth is unchanged —
    // it is the generator's independent ground truth, consumed by neither arm —
    // so radar-only and radar+AIS are BOTH honest against it. The pair yields
    // the first controlled fusion-vs-single-sensor accuracy delta this project
    // has had. Unset/empty ⇒ full radar+AIS(+camera) fusion, the default.
    const bool radar_only = !envOr("SIMMS_RADAR_ONLY", "").empty();
    // #20 pricing: emit AIS SOG/COG velocity content (PositionVelocity2D) +
    // nav_status from the fixtures. Default OFF (env unset) => Position2D,
    // byte-identical to before. See docs/algorithms/evaluation-log.md.
    const bool ais_velocity = !envOr("SIMMS_AIS_VELOCITY", "").empty();

    Scenario s;
    // Radar plots in the moving own-ship body frame.
    const auto radar = navtracker::replay::loadPlotCsvBodyFrame(
        plots_csv, provider, SensorKind::ArpaTtm, "sim_radar");
    s.measurements.insert(s.measurements.end(), radar.begin(), radar.end());

    // AIS absolute-position fixes (carry MMSI hint -> R11 identity surfacing).
    if (!radar_only && hasDataRows(ais_csv)) {
      const auto ais = navtracker::replay::loadAisCsv(ais_csv, provider.datum(),
                                                      "sim_ais", ais_velocity);
      s.measurements.insert(s.measurements.end(), ais.begin(), ais.end());
    }

    // Camera bearing-only (corroboration; present only in some scenarios).
    if (!radar_only && hasDataRows(camera_csv)) {
      const auto cam = navtracker::replay::loadCameraBearingsCsv(
          camera_csv, provider, "sim_cam");
      s.measurements.insert(s.measurements.end(), cam.begin(), cam.end());
    }

    std::sort(s.measurements.begin(), s.measurements.end(),
              [](const Measurement& a, const Measurement& b) {
                return a.time < b.time;
              });

    // Independent complete truth (already a clean 1 Hz shared clock: every
    // vessel is sampled at the same integer-second ticks, so BenchRunner's
    // exact-time bucketing sees honest multi-target steps without resampling.
    // A vessel is present in truth throughout — even during its AIS-dropout
    // window — because it does not cease to exist when a sensor goes quiet.)
    s.truth = loadSimTruthCsv(truth_csv, provider.datum());

    // Expose the datum so Sweep wires datum-aware components (occupancy / land /
    // static). Fixed for the run (feedOwnshipHistory processes all history up
    // front, so no recenter fires).
    s.datum = provider.datum();
    return s;
  }

 private:
  std::string label_;
};

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultSimMultisensorScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (const char* label : {"sim_ms_crossing", "sim_ms_headon",
                            "sim_ms_overtaking", "sim_ms_ais_dropout",
                            "sim_ms_clutter_burst", "sim_ms_anchored_camera"}) {
    out.push_back(std::make_unique<SimMultisensorScenarioRun>(label));
  }
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
