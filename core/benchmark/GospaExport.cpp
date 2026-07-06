#include "core/benchmark/GospaExport.hpp"

#include <fstream>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "core/scenario/Gospa.hpp"

namespace navtracker {
namespace benchmark {

void writeBenchStatesCsv(const BenchResult& result, const std::string& path) {
  std::ofstream os(path);
  // Round-trip-exact doubles so the external scorer computes distances on the
  // identical positions — a truncated CSV would manufacture a spurious
  // metric disagreement.
  os.precision(std::numeric_limits<double>::max_digits10);
  os << "scan,time_s,kind,id,east_m,north_m\n";
  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& step = result.steps[k];
    const double t = step.time.seconds();
    for (const auto& tr : step.truth) {
      os << k << ',' << t << ",truth," << tr.truth_id << ',' << tr.position.x()
         << ',' << tr.position.y() << '\n';
    }
    for (const auto& es : step.tracks) {
      os << k << ',' << t << ",track," << es.id.value << ',' << es.position.x()
         << ',' << es.position.y() << '\n';
    }
  }
}

void writeOurGospaCsv(const BenchResult& result, double gospa_cutoff_m,
                      const std::string& path) {
  std::ofstream os(path);
  os.precision(std::numeric_limits<double>::max_digits10);
  os << "scan,time_s,gospa,localisation,missed,false,n_missed,n_false\n";
  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& step = result.steps[k];
    std::vector<Eigen::Vector2d> truth;
    truth.reserve(step.truth.size());
    for (const auto& tr : step.truth) truth.push_back(tr.position);
    std::vector<Eigen::Vector2d> est;
    est.reserve(step.tracks.size());
    for (const auto& es : step.tracks) est.push_back(es.position);

    // Same cutoff/p/alpha the harness uses (gospa_cutoff_m, 2, 2) — the
    // GospaComponents decomposition is pre-root power-p space, and the rooted
    // headline is pow(total, 1/p), mirroring Stone Soup's compute_gospa_metric.
    const GospaComponents g = gospaComponents(truth, est, gospa_cutoff_m);
    const double gospa = gospaGreedy(truth, est, gospa_cutoff_m);
    os << k << ',' << step.time.seconds() << ',' << gospa << ','
       << g.localization << ',' << g.missed << ',' << g.false_ << ','
       << g.n_missed << ',' << g.n_false << '\n';
  }
}

}  // namespace benchmark
}  // namespace navtracker
