#include "adapters/benchmark/RbadScenarioRun.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {
namespace benchmark {

namespace {

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return static_cast<bool>(f);
}

std::string envOr(const char* var, const char* def) {
  const char* v = std::getenv(var);
  return (v != nullptr && *v != '\0') ? std::string(v) : std::string(def);
}

// Nominal datum. R-BAD has NO geo-reference (body-frame sensor, no ego pose), so
// this anchors the local ENU frame for Sweep's datum-aware models only; it is
// arbitrary. Kalimnos harbour, nominal — the fixtures are metres-from-origin.
const geo::Datum kNominalDatum{geo::Geodetic{36.9500, 26.9800, 0.0}};

// Load reference_tracks.csv (tod,ref_id,east_m,north_m,dock_label) into
// TruthSamples. These are the authors' own reference-TRACKER trajectories (NOT
// ground truth) — used only for cross-tracker CONSISTENCY scoring. Positions are
// already local ENU metres (E=X starboard, N=Y forward), matching the plot
// projection from the origin station, so no datum conversion is needed. The
// dock_label column is carried in the fixture for reporting but is not a
// TruthSample field (we assert nothing on it). Velocity is derived per ref_id by
// finite difference so the reference trajectory is kinematically complete.
std::vector<TruthSample> loadReferenceTracksCsv(const std::string& path) {
  // Per ref_id: time-ordered (time, position) samples.
  std::map<std::uint64_t, std::vector<std::pair<double, Eigen::Vector2d>>> by_id;
  std::ifstream f(path);
  if (!f) return {};
  std::string line;
  if (!std::getline(f, line)) return {};  // header
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tod, id, east, north;  // dock_label (5th) intentionally ignored
    if (!std::getline(ss, tod, ',')) continue;
    if (!std::getline(ss, id, ',')) continue;
    if (!std::getline(ss, east, ',')) continue;
    if (!std::getline(ss, north, ',')) continue;
    const double t = std::strtod(tod.c_str(), nullptr);
    const auto rid = static_cast<std::uint64_t>(std::strtoull(id.c_str(), nullptr, 10));
    by_id[rid].emplace_back(t, Eigen::Vector2d(std::strtod(east.c_str(), nullptr),
                                               std::strtod(north.c_str(), nullptr)));
  }

  std::vector<TruthSample> out;
  for (auto& [rid, samples] : by_id) {
    std::sort(samples.begin(), samples.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    const std::size_t n = samples.size();
    for (std::size_t i = 0; i < n; ++i) {
      Eigen::Vector2d vel = Eigen::Vector2d::Zero();
      if (n >= 2) {
        // Central where possible, one-sided at the ends; skip zero dt.
        const std::size_t lo = (i == 0) ? 0 : i - 1;
        const std::size_t hi = (i == n - 1) ? i : i + 1;
        const double dt = samples[hi].first - samples[lo].first;
        if (dt > 0.0) vel = (samples[hi].second - samples[lo].second) / dt;
      }
      TruthSample s;
      s.time = Timestamp::fromSeconds(samples[i].first);
      s.truth_id = rid;
      s.position = samples[i].second;
      s.velocity = vel;
      out.push_back(std::move(s));
    }
  }
  std::sort(out.begin(), out.end(), [](const TruthSample& a, const TruthSample& b) {
    if (a.time.seconds() != b.time.seconds()) return a.time < b.time;
    return a.truth_id < b.truth_id;
  });
  return out;
}

class RbadScenarioRun : public ScenarioRun {
 public:
  explicit RbadScenarioRun(std::string label) : label_(std::move(label)) {}

  ScenarioDescriptor descriptor() const override {
    ScenarioDescriptor d{label_, /*is_multi_seed=*/false, /*seed_count=*/1};
    // Single radar detection table. These are UNTUNED nominal environment
    // values for an mmWave FMCW short-range sensor, NOT fitted to this dataset
    // (a reality-check arm): P_D 0.9 (dense point clouds usually yield a
    // cluster), a modest false-alarm intensity, and an 80 m coverage disc a
    // little beyond the observed ~56 m max range. The dataset's true clutter
    // statistics are unknown and different from marine X-band by construction;
    // any over-count is REPORTED, not tuned away.
    d.detection_table = {
        {SensorKind::ArpaTtm, MeasurementModel::Position2D,
         DetectionParams{0.9, 1e-6, /*max_range_m=*/80.0}},
    };
    return d;
  }

  Scenario generate(std::uint64_t /*seed*/) override {
    const std::string dir = envOr("RBAD_DIR", "tests/fixtures/rbad") + "/" + label_ + "/";
    const std::string plots_csv = dir + "radar_plots.csv";
    const std::string refs_csv = dir + "reference_tracks.csv";
    if (!fileExists(plots_csv) || !fileExists(refs_csv)) {
      return {};  // fixtures absent — caller skips
    }

    // Fixed body frame, no ego pose: a single station at the ENU origin. The
    // plot CSV's (range, azimuth marine) recovers ENU E=X (starboard), N=Y
    // (forward). Doppler/SNR trailing columns are ignored by loadPlotCsv.
    const replay::StationMap stations{{"rbad", Eigen::Vector2d(0.0, 0.0)}};

    Scenario s;
    s.measurements = replay::loadPlotCsv(plots_csv, stations, SensorKind::ArpaTtm,
                                         "rbad_radar");
    if (s.measurements.empty()) return {};
    s.truth = loadReferenceTracksCsv(refs_csv);
    s.datum = kNominalDatum;
    return s;
  }

 private:
  std::string label_;
};

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultRbadScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (const char* label : {"rbad_kalimnos_16", "rbad_kalimnos_17", "rbad_kalimnos_3",
                            "rbad_kos_11", "rbad_kos_16", "rbad_kos_5"}) {
    out.push_back(std::make_unique<RbadScenarioRun>(label));
  }
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
