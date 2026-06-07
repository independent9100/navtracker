#include "core/benchmark/Comparator.hpp"

// Comparator — joins N long-format MetricRow streams on (scenario, config,
// metric) and renders a single Markdown diff table with per-cell direction
// indicators ("up" = improvement, "down" = regression, mid-dot = no change).
//
// Math:        per (input, scenario, config, metric) cell: mean = (1/N) sum
//              x_i across seeds. Delta vs. baseline = mean_input - mean_base.
//              Direction sign depends on the metric's "lower is better" flag:
//              lower-is-better => improvement when delta < 0; higher-is-
//              better => improvement when delta > 0. |delta| < 1e-9 is
//              reported as "no change" regardless of sign.
// Assumptions: inputs[0] is the baseline; all subsequent inputs are diffed
//              against it. Rows are already per-seed values (one row per
//              seed) — the comparator aggregates by mean across seeds. The
//              metric registry's "lower is better" map is intrinsic to the
//              metric name (e.g. ospa_mean is always lower-is-better).
// Rationale:   long format keeps the join generic — the comparator does
//              not need to know about the seed dimension, it just buckets
//              by (input, scenario, config, metric) and means at the end.
//              A static lower-is-better map (rather than a metric-registry
//              lookup) keeps the comparator a pure rendering component that
//              can be unit-tested without touching the runtime metric
//              machinery.
// Improve next: per-config relative percentage deltas (current cells show
//               absolute delta only); stat-sig markers (mean diff vs.
//               combined stddev); cluster scenarios by sensor family so
//               long sweeps stay readable; honor a CLI flag to invert the
//               baseline (e.g. compare against the worst run).

#include <algorithm>
#include <cmath>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace navtracker {
namespace benchmark {

namespace {

// Canonical metric column order. Mirrors MarkdownRenderer.cpp.
const std::vector<std::string>& canonicalMetrics() {
  static const std::vector<std::string> v = {
      "ospa_mean",       "ospa_p95",     "lifetime_ratio", "track_breaks",
      "id_switches",     "pos_rmse_m",   "sog_rmse_mps",   "cog_rmse_deg",
  };
  return v;
}

// Metric -> "lower is better". Anything not in this map is treated as
// lower-is-better (the safer default for error/breakage metrics).
bool isLowerBetter(const std::string& metric) {
  // Higher-is-better metrics.
  if (metric == "lifetime_ratio") return false;
  return true;
}

// Format with 3 significant figures (matches MarkdownRenderer.fmt3).
std::string fmt3(double x) {
  if (std::isnan(x)) return "nan";
  if (std::isinf(x)) return x < 0 ? "-inf" : "inf";
  if (x == 0.0) return "0";
  std::ostringstream os;
  os.precision(3);
  os << x;
  return os.str();
}

double meanOf(const std::vector<double>& v) {
  if (v.empty()) return std::nan("");
  double sum = 0.0;
  for (double x : v) sum += x;
  return sum / static_cast<double>(v.size());
}

// Short SHA: take everything up to the first space (the writer suffixes
// "(clean)"/"(dirty)") and clip to 12 chars max.
std::string shortSha(const std::string& git_sha) {
  std::string head = git_sha;
  auto sp = head.find(' ');
  if (sp != std::string::npos) head.resize(sp);
  if (head.size() > 12) head.resize(12);
  return head;
}

// "up" indicator (improvement) using Unicode BLACK UP-POINTING TRIANGLE.
const char* kIndUp = u8"▲";    // U+25B2
const char* kIndDown = u8"▼";  // U+25BC
const char* kIndSame = u8"·";  // middle dot

const char* indicatorFor(const std::string& metric, double delta) {
  if (std::abs(delta) < 1e-9) return kIndSame;
  const bool improved = isLowerBetter(metric) ? (delta < 0.0) : (delta > 0.0);
  return improved ? kIndUp : kIndDown;
}

// Formats the delta token: "+X" / "-X" / "0".
std::string fmtDelta(double delta) {
  if (std::abs(delta) < 1e-9) return std::string("0");
  std::ostringstream os;
  if (delta > 0.0) os << '+';
  os << fmt3(delta);
  return os.str();
}

void writeSummary(std::ostream& os,
                  const std::vector<ComparisonInput>& inputs) {
  os << "# Benchmark comparison\n\n";
  os << "Baseline: `" << inputs.front().prov.run_id << "` ("
     << shortSha(inputs.front().prov.git_sha) << ")\n\n";
  os << "Inputs:\n\n";
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& p = inputs[i].prov;
    os << (i == 0 ? "- baseline: " : "- input ")
       << (i == 0 ? "" : std::to_string(i) + ": ")
       << "`" << p.run_id << "` "
       << "(" << shortSha(p.git_sha) << ", build " << p.build_type << ")\n";
  }
  os << "\n";
}

}  // namespace

void renderComparison(std::ostream& os,
                      const std::vector<ComparisonInput>& inputs) {
  if (inputs.empty()) {
    os << "# Benchmark comparison\n\n_no inputs_\n";
    return;
  }

  writeSummary(os, inputs);

  // Aggregate values keyed by (input_index, scenario, config, metric).
  // Use std::map for deterministic iteration so the rendered output is
  // stable regardless of insertion order across inputs.
  using Key = std::tuple<std::size_t, std::string, std::string, std::string>;
  std::map<Key, std::vector<double>> cell;

  // First-seen order trackers (driven by the BASELINE input where possible,
  // then extended by later inputs so we don't drop rows that only appear
  // in non-baseline runs).
  std::vector<std::string> scenario_order;
  std::unordered_set<std::string> scenario_seen;
  std::unordered_map<std::string, std::vector<std::string>> configs_per_scen;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      configs_seen_per_scen;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      metrics_seen_per_scen;
  std::unordered_map<std::string, std::vector<std::string>>
      extra_metrics_per_scen;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      extra_metrics_seen_per_scen;

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    for (const auto& r : inputs[i].rows) {
      if (!scenario_seen.count(r.scenario)) {
        scenario_seen.insert(r.scenario);
        scenario_order.push_back(r.scenario);
      }
      auto& cfg_order = configs_per_scen[r.scenario];
      auto& cfg_seen = configs_seen_per_scen[r.scenario];
      if (!cfg_seen.count(r.config)) {
        cfg_seen.insert(r.config);
        cfg_order.push_back(r.config);
      }
      auto& met_seen = metrics_seen_per_scen[r.scenario];
      met_seen.insert(r.metric);
      // Track non-canonical metrics in first-seen order for tail columns.
      const auto& canon = canonicalMetrics();
      if (std::find(canon.begin(), canon.end(), r.metric) == canon.end()) {
        auto& extra_seen = extra_metrics_seen_per_scen[r.scenario];
        if (!extra_seen.count(r.metric)) {
          extra_seen.insert(r.metric);
          extra_metrics_per_scen[r.scenario].push_back(r.metric);
        }
      }
      cell[{i, r.scenario, r.config, r.metric}].push_back(r.value);
    }
  }

  const std::size_t n_inputs = inputs.size();

  for (const auto& scen : scenario_order) {
    const auto& cfgs = configs_per_scen[scen];
    const auto& met_seen = metrics_seen_per_scen[scen];

    // Column order: canonical metrics (filtered to those actually present),
    // then extra metrics in first-seen order.
    std::vector<std::string> columns;
    for (const auto& m : canonicalMetrics()) {
      if (met_seen.count(m)) columns.push_back(m);
    }
    for (const auto& m : extra_metrics_per_scen[scen]) columns.push_back(m);

    os << "## " << scen << "\n\n";

    if (columns.empty() || cfgs.empty()) {
      os << "_no data_\n\n";
      continue;
    }

    os << "| config |";
    for (const auto& c : columns) os << ' ' << c << " |";
    os << '\n';
    os << "| --- |";
    for (std::size_t i = 0; i < columns.size(); ++i) os << " --- |";
    os << '\n';

    for (const auto& cfg : cfgs) {
      os << "| " << cfg << " |";
      for (const auto& m : columns) {
        // Baseline mean.
        auto it_base = cell.find({0, scen, cfg, m});
        const bool has_base = it_base != cell.end() && !it_base->second.empty();
        const double base_mean =
            has_base ? meanOf(it_base->second) : std::nan("");

        if (n_inputs == 1) {
          // Single input: just the mean (or "-").
          os << ' ' << (has_base ? fmt3(base_mean) : std::string("-")) << " |";
          continue;
        }

        std::ostringstream cellbuf;
        cellbuf << (has_base ? fmt3(base_mean) : std::string("-"));

        if (n_inputs == 2) {
          // "<base> -> <new> (delta indicator)"
          auto it_new = cell.find({1, scen, cfg, m});
          const bool has_new = it_new != cell.end() && !it_new->second.empty();
          const double new_mean =
              has_new ? meanOf(it_new->second) : std::nan("");
          cellbuf << " -> " << (has_new ? fmt3(new_mean) : std::string("-"));
          if (has_base && has_new) {
            const double delta = new_mean - base_mean;
            cellbuf << " (" << fmtDelta(delta) << ' '
                    << indicatorFor(m, delta) << ")";
          }
        } else {
          // 3+ inputs: "<base> -> <i1> <ind1>  -> <i2> <ind2>  ..."
          for (std::size_t i = 1; i < n_inputs; ++i) {
            auto it_i = cell.find({i, scen, cfg, m});
            const bool has_i = it_i != cell.end() && !it_i->second.empty();
            const double mean_i =
                has_i ? meanOf(it_i->second) : std::nan("");
            cellbuf << "  -> " << (has_i ? fmt3(mean_i) : std::string("-"));
            if (has_base && has_i) {
              const double delta = mean_i - base_mean;
              cellbuf << ' ' << indicatorFor(m, delta);
            }
          }
        }
        os << ' ' << cellbuf.str() << " |";
      }
      os << '\n';
    }
    os << '\n';
  }
}

}  // namespace benchmark
}  // namespace navtracker
