#include "core/benchmark/Metrics.hpp"

// Metrics — bench-side aggregates and helpers feeding MetricsResult.
//
// This file accumulates the metric implementations across Tasks 5–9
// (OSPA aggregates, per-step assignment, continuity, RMSE,
// computeMetrics bundle). Each function carries its own
// Math/Assumptions/Rationale/Improve-next block per CLAUDE.md.

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_set>

#include "core/association/Hungarian.hpp"
#include "core/scenario/Gospa.hpp"
#include "core/scenario/Ospa.hpp"

namespace navtracker {
namespace benchmark {

namespace {
constexpr double kPi = 3.14159265358979323846;
double wrapDeg(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}
double cogDeg(const Eigen::Vector2d& v) {
  // COG: clockwise from north (positive y).
  return wrapDeg(std::atan2(v.x(), v.y()) * 180.0 / kPi);
}
}  // namespace

// Math:        per-step OSPA via optimal (Hungarian) assignment.
// Assumptions: result.steps[i].truth and .tracks are valid;
//              cutoff_m > 0; positions in metres.
// Rationale:   reuse ospaGreedy() from core/scenario/Ospa.hpp (now backed
//              by min-cost assignment) so the metric matches the harness.
// Improve next: OSPA(2) window-based variant (penalises ID switches
//               directly).
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m) {
  std::vector<double> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    std::vector<Eigen::Vector2d> truth;
    truth.reserve(step.truth.size());
    for (const auto& t : step.truth) truth.push_back(t.position);
    std::vector<Eigen::Vector2d> est;
    est.reserve(step.tracks.size());
    for (const auto& tr : step.tracks) est.push_back(tr.position);
    out.push_back(ospaGreedy(truth, est, cutoff_m));
  }
  return out;
}

// Math:        per-step GOSPA at p=α=2, cutoff cutoff_m, via optimal
//              (Hungarian) assignment (gospaGreedy).
// Assumptions: same as computeOspaPerStep — positions in metres, ENU.
// Rationale:   GOSPA is the conventional metric for PMBM and the
//              autoferry literature (Helgesen 2022). Unlike OSPA it
//              does not divide by max(|X|, |Y|), so missed and false
//              tracks contribute c²/2 each — cardinality errors stop
//              hiding behind a saturated per-step cutoff.
// Improve next: T-GOSPA (trajectory-level, time-weighted) — directly
//               measures track fragmentation; needs trajectory-aware
//               step bundling, queued in the PMBM plan phase 4.
std::vector<double> computeGospaPerStep(const BenchResult& result,
                                        double cutoff_m) {
  std::vector<double> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    std::vector<Eigen::Vector2d> truth;
    truth.reserve(step.truth.size());
    for (const auto& t : step.truth) truth.push_back(t.position);
    std::vector<Eigen::Vector2d> est;
    est.reserve(step.tracks.size());
    for (const auto& tr : step.tracks) est.push_back(tr.position);
    out.push_back(gospaGreedy(truth, est, cutoff_m));
  }
  return out;
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

// Math:        linear-interpolated percentile;
//              idx = q * (n-1), result = v[floor(idx)]*(1-frac) +
//              v[ceil(idx)]*frac where frac = idx - floor(idx).
// Assumptions: q in [0,1]; v non-empty (returns 0 if empty).
// Rationale:   matches NumPy's "linear" interpolation method; standard
//              and predictable for downstream tooling.
// Improve next: support alternative methods (nearest, lower, higher)
//               if a future metric needs them.
double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = q * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  const double frac = idx - static_cast<double>(lo);
  return v[lo] * (1.0 - frac) + v[hi] * frac;
}

// Math:        per-step optimal (min-cost) assignment of truth → track.
//              Build an N×M cost matrix C(i,j) = ||tr_j - truth_i|| when
//              that distance is strictly below gate_m, else +∞ (forbidden);
//              solve with the Hungarian algorithm. Truth i is assigned the
//              track on its matched (finite-cost) cell, or left unassigned.
// Assumptions: truth and track positions are in the same ENU frame and
//              metres; gate_m is the exclusive ceiling. The cost matrix is a
//              deterministic function of the step, so the assignment (and
//              hence id-switch / continuity counts) is replay-deterministic.
// Rationale:   greedy NN (the previous impl) can flip pairings between
//              adjacent steps in close-crossing geometry, manufacturing
//              spurious id-switches and OSPA spikes that confound A/B
//              estimator comparisons. Optimal assignment reflects the
//              tracker, not assignment churn. Single shared assignment
//              function across continuity, id switches, and per-track RMSE
//              keeps the three metrics consistent. O(N·M·min(N,M)); N,M are
//              small (≤ ~10) in every bench scenario.
// Improve next: extend to GOSPA-style assignment that also scores
//               cardinality if continuity should penalise false tracks.
std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m) {
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<StepAssignment> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    StepAssignment a(step.truth.size(), std::nullopt);
    const int n = static_cast<int>(step.truth.size());
    const int m = static_cast<int>(step.tracks.size());
    if (n > 0 && m > 0) {
      Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(n, m, kInf);
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
          const double d =
              (step.tracks[j].position - step.truth[i].position).norm();
          if (d < gate_m) cost(i, j) = d;
        }
      }
      const std::vector<int> row_to_col = hungarianAssignment(cost);
      for (int i = 0; i < n; ++i) {
        const int j = row_to_col[i];
        if (j >= 0 && std::isfinite(cost(i, j))) a[i] = step.tracks[j].id;
      }
    }
    out.push_back(std::move(a));
  }
  return out;
}

// Math:        per truth id g (keyed by TruthStateSnapshot::truth_id),
//              walk the steps where g is present in step.truth:
//                - lifetime_ratio_g = (#present steps where g assigned)
//                                     / (#present steps)
//                - track_breaks_g  = # of maximal assigned -> unassigned
//                                    transitions while present
//                - id_switches_g   = # of adjacent assigned -> assigned
//                                    transitions where the TrackId changes
//              Steps where g is absent from truth reset the per-id walk
//              (prev id forgotten, no break counted): breaks and switches
//              are only scored within continuous-presence intervals, since
//              the tracker cannot be faulted while truth itself is gone.
//              Reported as plain means across truth ids.
// Assumptions: assigns is the per-step output of assignPerStep (parallel
//              to result.steps and to each step's truth slots). Truth
//              cardinality and slot order may vary per step. Bad inputs
//              return zeros.
// Rationale:   CLAUDE.md names ID stability as an architectural
//              guarantee; OSPA alone wouldn't catch silent ID churn or
//              brief drops that don't change cardinality. Keying by
//              truth_id (not slot index) is required for real-data
//              replays whose truth lists targets in varying order and
//              number — slot keying there counts file layout, not
//              tracker behaviour.
// Improve next: replace per-step assignment with a run-level longest
//               common subsequence over (truth, track_id) — better at
//               distinguishing brief swap-then-swap-back from real
//               permanent ID churn.
ContinuityCounts computeContinuity(const BenchResult& result,
                                   const std::vector<StepAssignment>& assigns) {
  if (result.steps.empty() || assigns.size() != result.steps.size())
    return {0, 0, 0};

  struct PerTruth {
    double present = 0.0;
    double assigned = 0.0;
    double breaks = 0.0;
    double switches = 0.0;
    bool in_gap = true;
    std::optional<TrackId> prev;
  };
  std::map<std::uint64_t, PerTruth> per;  // ordered: deterministic means

  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& truth = result.steps[k].truth;
    const auto& assign = assigns[k];
    std::unordered_set<std::uint64_t> seen;
    for (std::size_t i = 0; i < truth.size() && i < assign.size(); ++i) {
      seen.insert(truth[i].truth_id);
      PerTruth& p = per[truth[i].truth_id];
      p.present += 1.0;
      const auto& a = assign[i];
      if (a.has_value()) {
        p.assigned += 1.0;
        if (p.in_gap) p.in_gap = false;
        if (p.prev.has_value() && p.prev->value != a->value) {
          p.switches += 1.0;
        }
        p.prev = a;
      } else {
        if (!p.in_gap) {
          p.breaks += 1.0;  // exited an assigned interval
          p.in_gap = true;
        }
        p.prev = std::nullopt;
      }
    }
    // Truth ids absent this step: reset the walk without scoring — the
    // next presence interval starts fresh.
    for (auto& [id, p] : per) {
      if (seen.count(id)) continue;
      p.in_gap = true;
      p.prev = std::nullopt;
    }
  }
  if (per.empty()) return {0, 0, 0, {}};

  ContinuityCounts c{};
  for (const auto& [id, p] : per) {
    const double lr = (p.present > 0.0) ? p.assigned / p.present : 0.0;
    c.lifetime_ratio += lr;
    c.track_breaks += p.breaks;
    c.id_switches += p.switches;
    c.per_truth[id] = PerTruthContinuity{lr, p.breaks, p.switches};
  }
  const double n = static_cast<double>(per.size());
  c.lifetime_ratio /= n;
  c.track_breaks /= n;
  c.id_switches /= n;
  return c;
}

// Math:        for each truth id g, walk timesteps where g is present and
//              assigned to a track tid. For each such (truth, track)
//              pair, accumulate squared errors of position, SOG (= |v|),
//              and wrapped COG. Per truth id, take sqrt(mean). Final
//              result is the mean across truth ids that contributed at
//              least one sample.
//                pos_rmse_m   = mean_g sqrt(mean_k ||pos_diff_k||^2)
//                sog_rmse_mps = mean_g sqrt(mean_k (|v_g,k| - |v_truth,k|)^2)
//                cog_rmse_deg = mean_g sqrt(mean_k wrap(cog_diff_k)^2)
// Assumptions: assigns is the per-step output of assignPerStep so
//              estimate/truth pairs agree across all three metrics;
//              truth cardinality and slot order may vary per step;
//              track velocity is in metres/second; COG convention is
//              cog = atan2(vx, vy) (clockwise from north).
// Rationale:   decomposes OSPA so an improvement's source (position
//              fit / velocity / course) is visible. Keyed by truth_id
//              for the same reason as computeContinuity: slot keying
//              mixes different physical targets on real-data replays.
// Improve next: report NEES / NIS to check covariance calibration; add
//               a velocity-vector RMSE (single number capturing both
//               magnitude and direction) if downstream callers want it.
RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns) {
  if (result.steps.empty() || assigns.size() != result.steps.size())
    return {0, 0, 0};

  struct Accum {
    double pos_se = 0.0;
    double sog_se = 0.0;
    double cog_se = 0.0;
    std::size_t n = 0;
  };
  std::map<std::uint64_t, Accum> per;  // ordered: deterministic means

  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& step = result.steps[k];
    const auto& assign = assigns[k];
    for (std::size_t i = 0; i < std::min(step.truth.size(), assign.size());
         ++i) {
      if (!assign[i].has_value()) continue;
      const TrackId tid = *assign[i];
      auto it = std::find_if(step.tracks.begin(), step.tracks.end(),
                             [&](const TrackStateSnapshot& s) {
                               return s.id.value == tid.value;
                             });
      if (it == step.tracks.end()) continue;
      Accum& acc = per[step.truth[i].truth_id];
      const Eigen::Vector2d dp = it->position - step.truth[i].position;
      acc.pos_se += dp.squaredNorm();
      const double ds = it->velocity.norm() - step.truth[i].velocity.norm();
      acc.sog_se += ds * ds;
      const double dc =
          wrapDeg(cogDeg(it->velocity) - cogDeg(step.truth[i].velocity));
      acc.cog_se += dc * dc;
      acc.n += 1;
    }
  }

  RmseResult out{0, 0, 0, {}};
  std::size_t contributing = 0;
  for (const auto& [id, acc] : per) {
    if (acc.n == 0) continue;
    const double n = static_cast<double>(acc.n);
    const double pr = std::sqrt(acc.pos_se / n);
    const double sr = std::sqrt(acc.sog_se / n);
    const double cr = std::sqrt(acc.cog_se / n);
    out.pos_rmse_m += pr;
    out.sog_rmse_mps += sr;
    out.cog_rmse_deg += cr;
    out.per_truth[id] = PerTruthRmse{pr, sr, cr, acc.n};
    ++contributing;
  }
  if (contributing == 0) return {0, 0, 0, {}};
  out.pos_rmse_m /= static_cast<double>(contributing);
  out.sog_rmse_mps /= static_cast<double>(contributing);
  out.cog_rmse_deg /= static_cast<double>(contributing);
  return out;
}

// Math:        composition; runs each metric in turn from the same
//              BenchResult and the same per-step assignment so all
//              fields of MetricsResult agree on which track represents
//              which truth at each timestep.
// Assumptions: params.ospa_cutoff_m > 0; params.assoc_gate_m > 0.
//              Truth cardinality and slot order may vary per step; the
//              continuity and RMSE metrics key by truth_id.
// Rationale:   single entry point keeps the Sweep code in Task 13 trivial
//              ("run, compute, emit") and prevents drift between
//              individual metric callers.
// Improve next: OSPA(2) (window-based) to fold identity churn into the
//               headline metric instead of reporting it separately.
MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params) {
  MetricsResult m{};
  const auto per_step = computeOspaPerStep(result, params.ospa_cutoff_m);
  m.ospa_mean = mean(per_step);
  m.ospa_p95 = percentile(per_step, 0.95);
  const auto gospa_per_step =
      computeGospaPerStep(result, params.gospa_cutoff_m);
  m.gospa_mean = mean(gospa_per_step);
  m.gospa_p95 = percentile(gospa_per_step, 0.95);
  // RMS over per-step GOSPA: matches Helgesen 2022's aggregation.
  double sumsq = 0.0;
  for (double v : gospa_per_step) sumsq += v * v;
  m.gospa_rms = gospa_per_step.empty()
                    ? 0.0
                    : std::sqrt(sumsq / static_cast<double>(gospa_per_step.size()));
  const auto assigns = assignPerStep(result, params.assoc_gate_m);
  const auto cont = computeContinuity(result, assigns);
  m.lifetime_ratio = cont.lifetime_ratio;
  m.track_breaks = cont.track_breaks;
  m.id_switches = cont.id_switches;
  const auto rmse = computeRmse(result, assigns);
  m.pos_rmse_m = rmse.pos_rmse_m;
  m.sog_rmse_mps = rmse.sog_rmse_mps;
  m.cog_rmse_deg = rmse.cog_rmse_deg;
  // Per-truth-id breakdown: union the continuity and RMSE per-truth
  // maps. A truth id present in continuity but missing from RMSE means
  // it appeared in step.truth but no Confirmed track was assigned to
  // it within the gate (no kinematic samples) — we still report its
  // lifetime / breaks / switches; RMSE fields stay zero with n=0.
  for (const auto& [id, c] : cont.per_truth) {
    auto& pt = m.per_truth[id];
    pt.lifetime_ratio = c.lifetime_ratio;
    pt.track_breaks = c.track_breaks;
    pt.id_switches = c.id_switches;
  }
  for (const auto& [id, r] : rmse.per_truth) {
    auto& pt = m.per_truth[id];
    pt.pos_rmse_m = r.pos_rmse_m;
    pt.sog_rmse_mps = r.sog_rmse_mps;
    pt.cog_rmse_deg = r.cog_rmse_deg;
    pt.rmse_n = r.n;
  }
  return m;
}

}  // namespace benchmark
}  // namespace navtracker
