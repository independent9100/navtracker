#include "adapters/benchmark/ReplayScenarioRun.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/AisCsvReplayAdapter.hpp"
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
    // Prime an OwnShipProvider from the full ownship history so plot/AIS
    // adapters can look up pose at-or-before any time.
    const auto poses = navtracker::replay::loadOwnshipCsv(kPhilosOwnshipCsv);
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

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  out.reserve(2);
  out.push_back(std::make_unique<PhilosScenarioRun>());
  out.push_back(std::make_unique<HaxrScenarioRun>());
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
