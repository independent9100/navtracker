#include <fstream>
#include <iostream>
#include <vector>

#include "core/benchmark/Comparator.hpp"
#include "core/benchmark/CsvReader.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: navtracker_bench_compare <baseline.csv> <other.csv>...\n"
                 "Writes Markdown diff to stdout.\n";
    return 1;
  }
  std::vector<navtracker::benchmark::ComparisonInput> inputs;
  for (int i = 1; i < argc; ++i) {
    std::ifstream is(argv[i]);
    if (!is) {
      std::cerr << "Cannot open: " << argv[i] << "\n";
      return 1;
    }
    const auto doc = navtracker::benchmark::readCsv(is);
    inputs.push_back({doc.provenance, doc.rows});
  }
  navtracker::benchmark::renderComparison(std::cout, inputs);
  return 0;
}
