#include "core/benchmark/CsvReader.hpp"

// CsvReader — inverse of CsvWriter; parses the provenance header block
// and the long-format rows.
//
// Math:        none.
// Assumptions: input was produced by writeCsv() (or matches its grammar):
//              `# key: value` lines, then a single column-header line, then
//              `run_id,config,scenario,seed,metric,value,unit` rows. No
//              quoting, no embedded commas in fields.
// Rationale:   keeping the writer and reader symmetric means a baseline
//              CSV is self-describing — a downstream tool (render, compare)
//              recovers the run's identity and seed list without an out-of-
//              band manifest.
// Improve next: tolerate trailing whitespace / CRLF; switch to a proper
//               CSV parser (with quoting) once any field can contain `,`.

#include <istream>
#include <sstream>
#include <string>

namespace navtracker {
namespace benchmark {

namespace {

// "  hello" -> "hello"; preserves internal whitespace.
std::string ltrim(const std::string& s) {
  std::size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
  return s.substr(i);
}

std::string stripCrlf(const std::string& s) {
  if (!s.empty() && s.back() == '\r') return s.substr(0, s.size() - 1);
  return s;
}

// Parse "[a,b,c]" (as emitted by seedListJson) into a vector<uint32_t>.
// Empty list "[]" is supported.
std::vector<std::uint32_t> parseSeedList(const std::string& v) {
  std::vector<std::uint32_t> out;
  std::string s = v;
  if (s.size() >= 2 && s.front() == '[' && s.back() == ']') {
    s = s.substr(1, s.size() - 2);
  }
  if (s.empty()) return out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty()) continue;
    out.push_back(static_cast<std::uint32_t>(std::stoul(tok)));
  }
  return out;
}

// Set the named provenance field from its (already-trimmed) value.
void applyProvenanceField(CsvProvenance& p,
                          const std::string& key,
                          const std::string& value) {
  if (key == "run_id") p.run_id = value;
  else if (key == "started_at") p.started_at_utc = value;
  else if (key == "git_sha") p.git_sha = value;
  else if (key == "build_type") p.build_type = value;
  else if (key == "compiler") p.compiler = value;
  else if (key == "host") p.host = value;
  else if (key == "seeds") p.seeds = parseSeedList(value);
  else if (key == "configs") p.config_count = static_cast<std::uint32_t>(std::stoul(value));
  else if (key == "scenarios") p.scenario_count = static_cast<std::uint32_t>(std::stoul(value));
  else if (key == "total_runs") p.total_runs = static_cast<std::uint32_t>(std::stoul(value));
  else if (key == "elapsed_seconds") p.elapsed_seconds = std::stod(value);
  // Unknown keys are silently ignored (forward-compat).
}

// Parse "# key: value" line. Returns false if not the expected shape.
bool parseHeaderLine(const std::string& line, CsvProvenance& p) {
  // Strip leading '#' and optional space.
  if (line.empty() || line[0] != '#') return false;
  std::string rest = line.substr(1);
  rest = ltrim(rest);
  const auto colon = rest.find(':');
  if (colon == std::string::npos) return false;
  const std::string key = rest.substr(0, colon);
  const std::string value = ltrim(rest.substr(colon + 1));
  applyProvenanceField(p, key, value);
  return true;
}

MetricRow parseDataLine(const std::string& line) {
  // run_id,config,scenario,seed,metric,value,unit
  std::vector<std::string> fields;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(std::move(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  fields.push_back(std::move(cur));
  MetricRow r;
  // If a row is malformed we let std::stoull / std::stod throw — caller
  // sees a structured failure rather than a silently-wrong row.
  r.run_id   = fields.size() > 0 ? fields[0] : std::string{};
  r.config   = fields.size() > 1 ? fields[1] : std::string{};
  r.scenario = fields.size() > 2 ? fields[2] : std::string{};
  r.seed     = fields.size() > 3 ? std::stoull(fields[3]) : 0ull;
  r.metric   = fields.size() > 4 ? fields[4] : std::string{};
  r.value    = fields.size() > 5 ? std::stod(fields[5]) : 0.0;
  r.unit     = fields.size() > 6 ? fields[6] : std::string{};
  return r;
}

}  // namespace

CsvDocument readCsv(std::istream& is) {
  CsvDocument doc;
  std::string line;
  bool past_column_header = false;
  while (std::getline(is, line)) {
    line = stripCrlf(line);
    if (line.empty()) continue;
    if (line[0] == '#') {
      parseHeaderLine(line, doc.provenance);
      continue;
    }
    if (!past_column_header) {
      // First non-comment, non-empty line is the column header.
      past_column_header = true;
      continue;
    }
    doc.rows.push_back(parseDataLine(line));
  }
  return doc;
}

}  // namespace benchmark
}  // namespace navtracker
