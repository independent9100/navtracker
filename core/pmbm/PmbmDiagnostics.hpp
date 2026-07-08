#pragma once

#include <cstdint>
#include <vector>

/**
 * PMBM per-scan introspection sink — a DIAGNOSTIC-ONLY hook.
 *
 * Backlog #25 (PMBM close-pass track loss) needs to watch the quantities the
 * aggregated `PmbmTracker::tracks()` output collapses away: per-Bernoulli
 * existence r, the per-identity aggregated existence mass BEFORE the
 * output-existence floor, which measurement each track claimed this scan
 * (detected vs misdetected), the dominant-hypothesis state (position + speed,
 * the divergence signal), and the per-scan structural events (hypotheses
 * dropped by the weight floor / the global cap, Bernoullis erased by r_min).
 * `tracks()` exposes only the survivors above `output_existence_floor` with no
 * r, no association, no structural-event trace — so none of the H1/H2/H3
 * discriminators are observable from it.
 *
 * This interface is NOT a consumer integration port. It lives beside the
 * tracker (not in `ports/`) on purpose: it exposes MBM-internal state for
 * forensics, and a normal library consumer never wires it. `PmbmTracker`
 * holds a single nullable pointer to it (per-instance, no globals — the
 * no-global-toggles rule); when null the tracker is byte-identical to a build
 * without this header (every diagnostic computation is guarded).
 *
 * Determinism: emission is a pure read of the post-prune density at end of
 * `processBatch`; it never mutates tracking state. Same input → same records.
 */

namespace navtracker::pmbm {

/**
 * One aggregated identity's per-scan snapshot.
 *
 * `agg_mass` is the load-bearing quantity: it is Σ_j w^j · r^{j,id}, the exact
 * existence mass the output aggregation floors at `output_existence_floor` —
 * so a track disappears from `tracks()` / the states.csv export precisely when
 * this crosses the floor. Its trajectory across scans is the H1 (smooth decay)
 * vs H2 (one-scan cliff) discriminator. `existence_r_best` /
 * `claimed_meas_index` / `speed_mps` come from the dominant (max-weight) global
 * hypothesis and localise the mechanism (miss-starvation vs divergence).
 */
struct PmbmBernoulliDiag {
  std::uint64_t id{0};
  // Σ_j w^j·r^{j,id} across ALL global hypotheses (the OUTPUT existence mass,
  // pre-floor). Emitted from output <=> agg_mass >= output_existence_floor.
  double agg_mass{0.0};
  // r of this id's Bernoulli in the dominant (max-weight) hypothesis; 0 when
  // the id is absent from the dominant hypothesis (present only in weaker ones).
  double existence_r_best{0.0};
  // How many global hypotheses carry this id (mass-fragmentation signal).
  int hyp_count{0};
  // Measurement index the dominant-hyp Bernoulli claimed this scan:
  //   >= 0 detected (index into the scan's measurement vector),
  //   -1   misdetected (in the dominant hypothesis this scan),
  //   -2   the id is absent from the dominant hypothesis entirely.
  int claimed_meas_index{-1};
  // Aggregated moment-matched mean position (ENU metres) — matches the
  // states.csv `track` position for in-output ids; for sub-floor ids it is
  // where the (about-to-vanish) track sits.
  double east_m{0.0};
  double north_m{0.0};
  // sqrt(vx^2 + vy^2) of the dominant-hyp state (m/s); -1 when velocity is not
  // available (state < 4 dims). A value far above the target's true speed is
  // the direct state-divergence (H3) signature.
  double speed_mps{-1.0};
  bool in_dominant{false};  // present in the max-weight global hypothesis
  bool in_output{false};    // agg_mass >= output_existence_floor
  bool confirmed{false};    // agg_mass >= confirm_threshold
};

/** Per-scan PMBM diagnostic record (one per processBatch when a sink is set). */
struct PmbmScanDiag {
  long scan_index{0};        // 0-based processBatch counter (join key: time_s)
  double time_s{0.0};        // filter time after this scan's predict
  int n_measurements{0};     // returns in this scan (radar + AIS, post sort)
  int n_global_hypotheses{0};
  int n_bernoulli_total{0};  // summed across all global hypotheses
  int n_ids{0};              // distinct aggregated identities
  // Structural events accumulated over THIS scan's pruneAndNormalise call(s):
  int n_hyp_dropped_floor{0};      // global hypotheses removed by weight floor
  int n_hyp_dropped_cap{0};        // global hypotheses removed by the size cap
  int n_bernoulli_pruned_rmin{0};  // Bernoullis erased by r_min (across hyps)
  std::vector<PmbmBernoulliDiag> bernoullis;
};

/**
 * Nullable per-scan PMBM diagnostic sink. Null on the tracker (default) = no
 * emission, no overhead, byte-identical tracking. See file header.
 */
class IPmbmDiagnosticSink {
 public:
  virtual ~IPmbmDiagnosticSink() = default;
  virtual void onPmbmScan(const PmbmScanDiag& d) = 0;
};

}  // namespace navtracker::pmbm
