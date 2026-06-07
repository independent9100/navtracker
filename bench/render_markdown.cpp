#include <fstream>
#include <iostream>
#include <string>

#include "core/benchmark/CsvReader.hpp"
#include "core/benchmark/MarkdownRenderer.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: navtracker_bench_render <input.csv> [output.md]\n";
    return 1;
  }
  const std::string in_path = argv[1];
  std::string out_path;
  if (argc >= 3) {
    out_path = argv[2];
  } else {
    out_path = in_path;
    // replace ".csv" with ".md"
    if (out_path.size() >= 4 &&
        out_path.compare(out_path.size() - 4, 4, ".csv") == 0) {
      out_path.replace(out_path.size() - 4, 4, ".md");
    } else {
      out_path += ".md";
    }
  }
  std::ifstream is(in_path);
  if (!is) {
    std::cerr << "Cannot open input: " << in_path << "\n";
    return 1;
  }
  const auto doc = navtracker::benchmark::readCsv(is);
  std::ofstream os(out_path);
  if (!os) {
    std::cerr << "Cannot open output: " << out_path << "\n";
    return 1;
  }
  navtracker::benchmark::renderMarkdown(os, doc.provenance, doc.rows);
  std::cout << "Wrote " << out_path << "\n";
  return 0;
}
