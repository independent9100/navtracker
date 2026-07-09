#pragma once

#include <fstream>
#include <limits>
#include <string>

#include "core/pmbm/PmbmDiagnostics.hpp"

namespace navtracker::benchmark {

/**
 * Streams PMBM per-scan diagnostics (backlog #25) to two CSV siblings of the
 * `--export-states-dir` output, one row-set per scan:
 *
 *   <base>.pmbmscan.csv  — one row per scan:
 *       scan,time_s,n_meas,n_hyp,n_bernoulli,n_ids,
 *       hyp_dropped_floor,hyp_dropped_cap,bernoulli_pruned_rmin
 *   <base>.pmbmbern.csv  — one row per (scan, aggregated identity):
 *       scan,time_s,id,agg_mass,r_best,hyp_count,claimed_meas,
 *       east_m,north_m,speed_mps,in_dominant,in_output,confirmed,
 *       innov_east_m,innov_north_m,innov_norm_m,imm_weights
 *   (#25 Phase 2b: innov_* = true applied-measurement position innovation of the
 *    dominant-hyp Bernoulli, norm −1 when misdetected/absent/born/bearing-only;
 *    imm_weights = '|'-joined per-mode IMM weights, empty when not IMM.)
 *
 * Wired by Sweep only when export_pmbm_diag_dir is set AND the tracker is
 * PMBM; the recorder is a plain IPmbmDiagnosticSink, so with no sink the
 * tracker is byte-identical. Doubles at max_digits10 (round-trip exact) so a
 * re-run reproduces the CSV bit-for-bit — the determinism check.
 */
class PmbmDiagRecorder : public pmbm::IPmbmDiagnosticSink {
 public:
  explicit PmbmDiagRecorder(const std::string& base)
      : scan_(base + ".pmbmscan.csv"), bern_(base + ".pmbmbern.csv") {
    scan_.precision(std::numeric_limits<double>::max_digits10);
    bern_.precision(std::numeric_limits<double>::max_digits10);
    scan_ << "scan,time_s,n_meas,n_hyp,n_bernoulli,n_ids,"
             "hyp_dropped_floor,hyp_dropped_cap,bernoulli_pruned_rmin\n";
    bern_ << "scan,time_s,id,agg_mass,r_best,hyp_count,claimed_meas,"
             "east_m,north_m,speed_mps,in_dominant,in_output,confirmed,"
             "innov_east_m,innov_north_m,innov_norm_m,imm_weights\n";
  }

  void onPmbmScan(const pmbm::PmbmScanDiag& d) override {
    scan_ << d.scan_index << ',' << d.time_s << ',' << d.n_measurements << ','
          << d.n_global_hypotheses << ',' << d.n_bernoulli_total << ','
          << d.n_ids << ',' << d.n_hyp_dropped_floor << ','
          << d.n_hyp_dropped_cap << ',' << d.n_bernoulli_pruned_rmin << '\n';
    for (const auto& b : d.bernoullis) {
      bern_ << d.scan_index << ',' << d.time_s << ',' << b.id << ','
            << b.agg_mass << ',' << b.existence_r_best << ',' << b.hyp_count
            << ',' << b.claimed_meas_index << ',' << b.east_m << ','
            << b.north_m << ',' << b.speed_mps << ',' << (b.in_dominant ? 1 : 0)
            << ',' << (b.in_output ? 1 : 0) << ',' << (b.confirmed ? 1 : 0)
            << ',' << b.innov_east_m << ',' << b.innov_north_m << ','
            << b.innov_norm_m << ',';
      for (std::size_t k = 0; k < b.imm_mode_weights.size(); ++k)
        bern_ << (k ? "|" : "") << b.imm_mode_weights[k];
      bern_ << '\n';
    }
  }

 private:
  std::ofstream scan_;
  std::ofstream bern_;
};

}  // namespace navtracker::benchmark
