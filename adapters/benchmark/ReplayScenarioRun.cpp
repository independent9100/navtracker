#include "adapters/benchmark/ReplayScenarioRun.hpp"

#include <algorithm>
#include <cstdint>
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
#include "core/scenario/Truth.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {
namespace benchmark {

namespace {

// Philos ais_ferry_near fixture paths. Mirrors the constants used in
// tests/replay/test_philos_ospa.cpp; relative paths resolved against the
// process working directory (project root when run from build/).
constexpr const char* kPhilosOwnshipCsv =
    "tests/fixtures/philos/out/ais_ferry_near/ownship.csv";
constexpr const char* kPhilosAisCsv =
    "tests/fixtures/philos/out/ais_ferry_near/ais.csv";
constexpr const char* kPhilosPlotsCsv =
    "tests/fixtures/philos/out/ais_ferry_near/radar_plots.csv";

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
bool fileExists(const char* path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
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
  ScenarioDescriptor descriptor() const override {
    return {"philos", /*is_multi_seed=*/false, /*seed_count=*/1};
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    if (!fileExists(kPhilosOwnshipCsv) || !fileExists(kPhilosAisCsv) ||
        !fileExists(kPhilosPlotsCsv)) {
      return {};  // fixtures absent — caller skips
    }
    // Prime an OwnShipProvider from the full ownship history so plot/AIS
    // adapters can look up pose at-or-before any time.
    const auto poses = navtracker::replay::loadOwnshipCsv(kPhilosOwnshipCsv);
    if (poses.empty()) return {};
    OwnShipProvider provider(/*history_size=*/poses.size() + 1);
    navtracker::replay::feedOwnshipHistory(provider, poses);

    // AIS as both a measurement source and the truth track.
    const auto ais = navtracker::replay::loadAisCsv(
        kPhilosAisCsv, provider.datum(), "ais");
    // Radar plots in body frame (Philos is a moving platform).
    const auto radar = navtracker::replay::loadPlotCsvBodyFrame(
        kPhilosPlotsCsv, provider, SensorKind::ArpaTtm, "philos_radar");

    Scenario s;
    s.measurements.reserve(radar.size() + ais.size());
    s.measurements.insert(s.measurements.end(), radar.begin(), radar.end());
    s.measurements.insert(s.measurements.end(), ais.begin(), ais.end());
    std::sort(s.measurements.begin(), s.measurements.end(),
              [](const Measurement& a, const Measurement& b) {
                return a.time < b.time;
              });

    s.truth.reserve(ais.size());
    for (const auto& m : ais) s.truth.push_back(aisMeasurementToTruth(m));
    return s;
  }
};

class HaxrScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"haxr", /*is_multi_seed=*/false, /*seed_count=*/1};
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    if (!fileExists(kHaxrPlotsCsv) || !fileExists(kHaxrAisCsv) ||
        !fileExists(kHaxrStationsCsv)) {
      return {};
    }
    const auto stations = navtracker::replay::loadStations(kHaxrStationsCsv);
    auto plots = navtracker::replay::loadPlotCsv(
        kHaxrPlotsCsv, stations, SensorKind::ArpaTtm, "kattwyk");
    auto truth = navtracker::replay::loadHaxrTruth(
        kHaxrAisCsv, "kattwyk", stations);

    Scenario s;
    s.measurements = std::move(plots);
    s.truth = std::move(truth);
    return s;
  }
};

// One AutoFerry scenario folder under data/autoferry/<label>/. generate()
// returns empty (caller skips) when the JSON files are not reachable from
// cwd, mirroring the philos/haxr fixture-absent behaviour.
class AutoferryScenarioRun : public ScenarioRun {
 public:
  explicit AutoferryScenarioRun(std::string label) : label_(std::move(label)) {}

  ScenarioDescriptor descriptor() const override {
    ScenarioDescriptor d{"autoferry_" + label_, /*is_multi_seed=*/false,
                         /*seed_count=*/1};
    // Per-sensor detection table, calibrated against the published
    // ground truth across scenarios 2/5/13/22 (open water + urban
    // channel; matching gate 15 m position / 0.15 rad bearing):
    //
    //   sensor  empirical P_D  clutter/scan  λ_C (units)        coverage
    //   lidar   0.40–0.71      0.1–0.5       ≈0.3 / (π·140²)    ≤135 m
    //                                        ≈ 5e-6 m⁻²         observed
    //   radar   0.71–0.91      0.9–4.4       ≈3 / ~3e5 m²
    //                                        ≈ 1e-5 m⁻²         (region-
    //                                                            cropped)
    //   EO+IR   0.24–0.87      0.4–12        ≈2 / 2π ≈ 0.3–0.6 rad⁻¹
    //
    // The lidar P_D is depressed by out-of-coverage opportunities in
    // the empirical count; with the 140 m range gate carried in
    // max_range_m, the in-coverage value is higher → 0.7. EO and IR
    // share a (SensorKind, MeasurementModel) key, so one combined
    // entry. Values are deliberately round mid-points: per-scenario
    // adaptive estimation (AdaptiveSensorDetectionModel) is the
    // follow-up.
    d.detection_table = {
        {SensorKind::Lidar, MeasurementModel::Position2D,
         DetectionParams{0.7, 5e-6, /*max_range_m=*/140.0}},
        {SensorKind::ArpaTtm, MeasurementModel::Position2D,
         DetectionParams{0.8, 1e-5}},
        {SensorKind::EoIr, MeasurementModel::Bearing2D,
         DetectionParams{0.6, 0.5}},
    };
    return d;
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    // Full four-sensor fusion: radar + lidar (active → Position2D) plus
    // EO + IR (passive → Bearing2D). Bearings refine existing tracks but
    // never initiate one — the track-birth paths drop non-gating bearings
    // (canInitiateTrack), matching the paper's active-only initiation.
    navtracker::replay::AutoferryLoadOptions opts;
    opts.include_bearings = true;
    return navtracker::replay::loadAutoferryScenario(
        std::string("data/autoferry/") + label_, label_, opts);
  }

 private:
  std::string label_;
};

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  out.reserve(2);
  out.push_back(std::make_unique<PhilosScenarioRun>());
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
    out.push_back(std::make_unique<AutoferryScenarioRun>(label));
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
