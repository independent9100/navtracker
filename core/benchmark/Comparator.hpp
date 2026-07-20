#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

/** One CSV file's worth of results (provenance + rows) to compare. */
struct ComparisonInput {
  CsvProvenance prov;
  std::vector<MetricRow> rows;
};

/**
 * Sign convention:
 *   Lower-is-better: ospa_mean, ospa_p95, pos_rmse_m, sog_rmse_mps,
 *                    cog_rmse_deg, track_breaks, id_switches.
 *   Higher-is-better: lifetime_ratio.
 *   Neutral (signed/target — no directional claim): card_err_*, nees_*,
 *                    nis_* (#36 M23). A decrease is not an improvement without
 *                    a target the renderer does not know.
 * For each cell: emit "baseline_mean -> new_mean (delta indicator)" with
 * an up-arrow for improvement, down-arrow for regression, a middle dot when
 * |delta| < 1e-9, and "~" for a neutral (signed/target) metric.
 *
 * Inputs are joined on (scenario, config, metric). Rows of one config
 * are baseline (inputs[0]); rows of subsequent inputs are deltas.
 */
void renderComparison(std::ostream& os,
                      const std::vector<ComparisonInput>& inputs);

}  // namespace benchmark
}  // namespace navtracker
