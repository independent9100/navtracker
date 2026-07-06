#pragma once

#include <string>

#include "core/benchmark/BenchRunner.hpp"

namespace navtracker {
namespace benchmark {

// D2 — GOSPA metric cross-validation export (measurement integrity).
//
// Purpose: serialize one bench run's per-scan (truth, track) position sets and
// our own per-scan GOSPA to neutral CSV, so an independently-authored scorer
// (Stone Soup, tools/stonesoup_gospa_crosscheck.py) can score the SAME tracks
// against the SAME truth and confirm core/scenario/Gospa.hpp agrees. The
// harness has had two truth-fragmentation bugs (autoferry 2026-06-10, harbor
// 2026-07-02) silently corrupt metrics; an external agreement is the cheapest
// hedge against a metric bug. See docs/algorithms/gospa-crosscheck.md.
//
// The export re-uses the exact BenchResult the metrics consume — the tracks
// exported are byte-for-byte the tracks scored — so the cross-check is faithful
// by construction, not a reconstruction that could drift from the harness.

// States CSV: one row per (scan, object). Columns:
//   scan,time_s,kind,id,east_m,north_m
// kind is "truth" or "track"; positions are ENU metres (the frame GOSPA is
// computed in — see BenchRunner.hpp). scan is the 0-based BenchStep index.
void writeBenchStatesCsv(const BenchResult& result, const std::string& path);

// Our GOSPA CSV: one row per scan. Columns:
//   scan,time_s,gospa,localisation,missed,false,n_missed,n_false
// gospa is the rooted headline distance (gospaGreedy); localisation/missed/
// false are the pre-root power-p decomposition (GospaComponents) — the same
// space Stone Soup reports its decomposition in, so the two are directly
// comparable term-by-term. cutoff/p/alpha are the harness values (c, 2, 2).
void writeOurGospaCsv(const BenchResult& result, double gospa_cutoff_m,
                      const std::string& path);

}  // namespace benchmark
}  // namespace navtracker
