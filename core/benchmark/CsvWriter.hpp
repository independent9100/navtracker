#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Run-provenance header for a sweep CSV — the reproducibility metadata
 * (run id, timestamp, git SHA, build/compiler/host, seeds, run counts,
 * wall-clock) emitted as `#`-prefixed lines above the data section.
 */
struct CsvProvenance {
  std::string run_id;
  std::string started_at_utc;   // ISO 8601, e.g. "2026-06-07T10:14:22Z"
  std::string git_sha;          // includes "(clean)" or "(dirty)" suffix
  std::string build_type;       // "Release" / "Debug" / ...
  std::string compiler;         // "gcc 13.2.0"
  std::string host;             // "linux x86_64"
  std::vector<std::uint32_t> seeds;
  std::uint32_t config_count{0};
  std::uint32_t scenario_count{0};
  std::uint32_t total_runs{0};
  double elapsed_seconds{0.0};
};

/** Write the provenance header block and metric `rows` as CSV to `os`. */
void writeCsv(std::ostream& os,
              const CsvProvenance& prov,
              const std::vector<MetricRow>& rows);

}  // namespace benchmark
}  // namespace navtracker
