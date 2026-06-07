#pragma once

#include <iosfwd>
#include <vector>

#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

// Writes:
// 1. A top-of-document summary block mirroring the CSV header.
// 2. One section per scenario. Each section is a Markdown table:
//    rows = configs (in first-seen order), columns = metrics in the
//    canonical order [ospa_mean, ospa_p95, lifetime_ratio, track_breaks,
//    id_switches, pos_rmse_m, sog_rmse_mps, cog_rmse_deg]. Cells are
//    "mean ± stddev" aggregated across seeds, 3 sig figs.
void renderMarkdown(std::ostream& os,
                    const CsvProvenance& prov,
                    const std::vector<MetricRow>& rows);

}  // namespace benchmark
}  // namespace navtracker
