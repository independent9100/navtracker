#include "core/benchmark/MarkdownRenderer.hpp"

// MarkdownRenderer — converts a long-format MetricRow stream + provenance
// header into a human-readable Markdown report.
//
// Math:        per cell: mean = (1/N) Σ x_i; sample stddev =
//              sqrt((1/(N-1)) Σ (x_i - mean)²) for N≥2 (Bessel-corrected);
//              omitted for N=1.
// Assumptions: rows are already grouped per (scenario, config, metric) by
//              seed — i.e. the per-seed values are the values to aggregate.
//              First-seen order in `rows` defines the section / row order.
// Rationale:   long format keeps the writer dumb; the renderer joins on
//              (scenario, config, metric) at read time. Canonical column
//              order means the report layout is stable even as new metrics
//              are added (extras land at the end).
// Improve next: percentile cells (p50/p95) for skewed metrics; per-config
//               relative columns vs. a designated baseline; emit
//               better-is-up/-down arrows from the metric registry.

#include <algorithm>
#include <cmath>
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace navtracker {
namespace benchmark {

namespace {

// Canonical metric column order. Anything not in this list is appended at
// the end in first-seen order so the report still shows the data, even if
// the canonical list lags behind the metric registry.
const std::vector<std::string>& canonicalMetrics() {
  static const std::vector<std::string> v = {
      "ospa_mean",       "ospa_p95",     "lifetime_ratio", "track_breaks",
      "id_switches",     "pos_rmse_m",   "sog_rmse_mps",   "cog_rmse_deg",
      // Filter consistency (backlog item 12). nees_mean is the headline:
      // β̂ ≈ 1 means honest position covariance; sc5 measured 39 before
      // R calibration. nees_p95 catches tail overconfidence. Per-source
      // nis_* rows arrive after the canonical block in first-seen order.
      "nees_mean",       "nees_p95",     "nees_coverage_95", "nees_beta_hat",
      "nees_n",
  };
  return v;
}

// Format `x` with 3 significant figures. Falls back to fixed-point with
// short tails for small numbers (e.g. 0.94, 0.020) so the table stays
// readable. NaN -> "nan".
std::string fmt3(double x) {
  if (std::isnan(x)) return "nan";
  if (std::isinf(x)) return x < 0 ? "-inf" : "inf";
  if (x == 0.0) return "0";
  std::ostringstream os;
  os.precision(3);
  // %g-style: 3 significant figures, trimming trailing zeros.
  os << x;
  // std::ostringstream with precision and default flags emits %g-like.
  return os.str();
}

std::string formatCell(const std::vector<double>& vals) {
  if (vals.empty()) return "-";
  if (vals.size() == 1) return fmt3(vals.front());
  double sum = 0.0;
  for (double v : vals) sum += v;
  const double mean = sum / static_cast<double>(vals.size());
  double sq = 0.0;
  for (double v : vals) sq += (v - mean) * (v - mean);
  const double stddev =
      std::sqrt(sq / static_cast<double>(vals.size() - 1));
  return fmt3(mean) + " ± " + fmt3(stddev);
}

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

void writeHeader(std::ostream& os, const CsvProvenance& p) {
  os << "# Baseline run: " << p.run_id << "\n\n";
  os << "- started: `" << p.started_at_utc << "`\n";
  os << "- git: `" << p.git_sha << "`\n";
  os << "- build: `" << p.build_type << "`\n";
  os << "- compiler: `" << p.compiler << "`\n";
  os << "- host: `" << p.host << "`\n";
  os << "- seeds: `" << seedListJson(p.seeds) << "`\n";
  os << "- configs: " << p.config_count << "\n";
  os << "- scenarios: " << p.scenario_count << "\n";
  os << "- total runs: " << p.total_runs << "\n";
  os << "- elapsed: " << p.elapsed_seconds << " s\n\n";
}

}  // namespace

void renderMarkdown(std::ostream& os,
                    const CsvProvenance& prov,
                    const std::vector<MetricRow>& rows) {
  writeHeader(os, prov);

  // First-seen order trackers.
  std::vector<std::string> scenario_order;
  std::unordered_set<std::string> scenario_seen;
  // Per-scenario: ordered configs.
  std::unordered_map<std::string, std::vector<std::string>> configs_per_scen;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      configs_seen_per_scen;
  // Per-scenario: which metrics actually appear, for fallback ordering.
  std::unordered_map<std::string, std::vector<std::string>> metrics_per_scen;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      metrics_seen_per_scen;
  // Per-scenario: set of seeds touched (for the "(N seeds)" caption).
  std::unordered_map<std::string, std::unordered_set<std::uint64_t>>
      seeds_per_scen;

  // (scenario, config, metric) -> values across seeds.
  std::map<std::tuple<std::string, std::string, std::string>,
           std::vector<double>>
      cell;

  for (const auto& r : rows) {
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
    auto& met_order = metrics_per_scen[r.scenario];
    auto& met_seen = metrics_seen_per_scen[r.scenario];
    if (!met_seen.count(r.metric)) {
      met_seen.insert(r.metric);
      met_order.push_back(r.metric);
    }
    seeds_per_scen[r.scenario].insert(r.seed);
    cell[{r.scenario, r.config, r.metric}].push_back(r.value);
  }

  for (const auto& scen : scenario_order) {
    const auto& cfgs = configs_per_scen[scen];
    const auto& met_seen = metrics_seen_per_scen[scen];
    const auto& met_order = metrics_per_scen[scen];

    // Column order: canonical metrics that appear in this scenario, then
    // any extra metrics (first-seen) tacked on the end.
    std::vector<std::string> columns;
    std::unordered_set<std::string> col_seen;
    for (const auto& m : canonicalMetrics()) {
      if (met_seen.count(m)) {
        columns.push_back(m);
        col_seen.insert(m);
      }
    }
    for (const auto& m : met_order) {
      if (!col_seen.count(m)) {
        columns.push_back(m);
        col_seen.insert(m);
      }
    }

    const auto n_seeds = seeds_per_scen[scen].size();
    os << "## " << scen << " (" << n_seeds << " seed"
       << (n_seeds == 1 ? "" : "s") << ")\n\n";

    if (columns.empty() || cfgs.empty()) {
      os << "_no data_\n\n";
      continue;
    }

    // Header row.
    os << "| config |";
    for (const auto& c : columns) os << ' ' << c << " |";
    os << '\n';
    os << "| --- |";
    for (std::size_t i = 0; i < columns.size(); ++i) os << " --- |";
    os << '\n';

    // Data rows.
    for (const auto& cfg : cfgs) {
      os << "| " << cfg << " |";
      for (const auto& m : columns) {
        auto it = cell.find({scen, cfg, m});
        const std::string c =
            it == cell.end() ? std::string("-") : formatCell(it->second);
        os << ' ' << c << " |";
      }
      os << '\n';
    }
    os << '\n';
  }
}

}  // namespace benchmark
}  // namespace navtracker
