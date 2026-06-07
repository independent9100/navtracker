#include "core/benchmark/CsvWriter.hpp"

// CsvWriter — long-format benchmark CSV emitter with provenance header.
//
// Math:        none.
// Assumptions: caller has populated CsvProvenance with the values to be
//              recorded; rows are already filtered/ordered as desired.
// Rationale:   long format (one row per (run_id, config, scenario, seed,
//              metric)) makes a new metric a new value of the `metric`
//              column rather than a schema change. The header block lets
//              a reader reproduce a baseline run from the SHA alone.
// Improve next: column-typed sub-emitters if downstream tooling really
//               wants a wide table; today, `compare.cpp` does that join
//               at read time.

#include <ostream>
#include <sstream>

namespace navtracker {
namespace benchmark {

namespace {
std::string seedListJson(const std::vector<std::uint32_t>& v) {
  std::ostringstream os;
  os << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) os << ',';
    os << v[i];
  }
  os << ']';
  return os.str();
}
}  // namespace

void writeCsv(std::ostream& os,
              const CsvProvenance& p,
              const std::vector<MetricRow>& rows) {
  os << "# run_id: " << p.run_id << "\n"
     << "# started_at: " << p.started_at_utc << "\n"
     << "# git_sha: " << p.git_sha << "\n"
     << "# build_type: " << p.build_type << "\n"
     << "# compiler: " << p.compiler << "\n"
     << "# host: " << p.host << "\n"
     << "# seeds: " << seedListJson(p.seeds) << "\n"
     << "# configs: " << p.config_count << "\n"
     << "# scenarios: " << p.scenario_count << "\n"
     << "# total_runs: " << p.total_runs << "\n"
     << "# elapsed_seconds: " << p.elapsed_seconds << "\n";
  os << "run_id,config,scenario,seed,metric,value,unit\n";
  for (const auto& r : rows) {
    os << r.run_id << ',' << r.config << ',' << r.scenario << ','
       << r.seed << ',' << r.metric << ',' << r.value << ',' << r.unit << '\n';
  }
}

}  // namespace benchmark
}  // namespace navtracker
