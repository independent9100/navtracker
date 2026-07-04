#pragma once

#include <iosfwd>
#include <vector>

#include "core/benchmark/CsvWriter.hpp"  // for CsvProvenance + MetricRow
#include "core/benchmark/Sweep.hpp"      // (transitively via CsvWriter.hpp)

namespace navtracker {
namespace benchmark {

/** Parsed contents of a sweep CSV: run provenance plus the metric rows. */
struct CsvDocument {
  CsvProvenance provenance;
  std::vector<MetricRow> rows;
};

/**
 * Parses a CSV written by writeCsv(). Provenance comes from the
 * `#`-prefixed header block; rows from the data section. Whitespace
 * in fields is preserved literally (no trim).
 */
CsvDocument readCsv(std::istream& is);

}  // namespace benchmark
}  // namespace navtracker
