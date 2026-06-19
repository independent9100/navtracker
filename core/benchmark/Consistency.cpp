#include "core/benchmark/Consistency.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include <Eigen/Cholesky>

#include "core/benchmark/Metrics.hpp"

namespace navtracker {
namespace benchmark {

namespace {

constexpr std::size_t kMinSamples = 30;       // below → bands suppressed
constexpr double kMinPivot = 1e-12;           // LDLT pivot floor

// Inverse standard-normal CDFs. Two-sided 95% band on ε̄ uses ±kZ95Band.
// Per-sample χ² upper quantiles are ONE-sided (we ask "is ε ≤ upper
// 95%?"), so use kZ95Upper / kZ99Upper.
constexpr double kZ95Band  = 1.959963984540054;  // Φ⁻¹(0.975)
constexpr double kZ95Upper = 1.644853626951472;  // Φ⁻¹(0.95)
constexpr double kZ99Upper = 2.326347874040841;  // Φ⁻¹(0.99)

// Wilson-Hilferty approximation of upper χ²_k_{1-α} quantile.
// χ²_k_{1-α} ≈ k · (1 − 2/(9k) + z_{1-α} · sqrt(2/(9k)))^3
double chi2Quantile(std::size_t k, double z) {
  const double kd = static_cast<double>(k);
  const double a = 2.0 / (9.0 * kd);
  const double t = 1.0 - a + z * std::sqrt(a);
  return kd * t * t * t;
}

// Quadratic form x^T M^{-1} x via LDLT. Returns nullopt on
// non-PSD or near-singular pivots — caller drops the sample.
std::optional<double> quadForm(const Eigen::MatrixXd& M,
                               const Eigen::VectorXd& x) {
  Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
  if (ldlt.info() != Eigen::Success) return std::nullopt;
  // Vector D is the diagonal of the LDLᵀ factorisation. Any pivot below
  // the floor → effectively singular; reject.
  const Eigen::VectorXd D = ldlt.vectorD();
  for (int i = 0; i < D.size(); ++i) {
    if (std::abs(D(i)) < kMinPivot) return std::nullopt;
  }
  const Eigen::VectorXd y = ldlt.solve(x);
  if (!y.allFinite()) return std::nullopt;
  return x.dot(y);
}

double safeTrace(const Eigen::MatrixXd& M) {
  if (M.rows() == 0 || M.cols() == 0) return 0.0;
  const Eigen::Index n = std::min(M.rows(), M.cols());
  double t = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) t += M(i, i);
  return t;
}

}  // namespace

// === NisCollector ============================================================

// Math:
//   For each successful hard-match update the sink emits (ν, S, R).
//   εⁿⁱˢ = νᵀ S⁻¹ ν via LDLT solve. Per-key Welford mean + χ² coverage
//   counters; trace_ratio = tr(HPHᵀ)/tr(R) tracked alongside so a high
//   ε̄ can be attributed to R-mistuning (trace_ratio ≪ 1) vs P-
//   mistuning (trace_ratio ≫ 1) — see Consistency.hpp doc.
// Assumptions:
//   S = HPHᵀ + R as documented in IInnovationSink.hpp; ν is angle-
//   wrapped where applicable; R carries the matrix the update applied.
// Rationale:
//   LDLT (not explicit inverse): numerically robust on 1- and 2-d S;
//   the LDLT path also gives a free non-PSD detector that drops bad
//   samples (e.g. zero-R adapter bugs) without poisoning the running
//   mean. Welford keeps the mean stable over long replays.
// Improve next:
//   - Per-source variance (m2 field reserved) for outlier diagnostics.
//   - JPDA β-weighted variant once soft-update emission lands.
void NisCollector::onInnovation(const InnovationEvent& e) {
  ++total_samples_;
  const auto nis = quadForm(e.S, e.residual);
  Accum& a = per_key_[ConsistencySourceKey{e.sensor, e.model, e.source_id}];
  if (!nis.has_value() || !std::isfinite(*nis) || *nis < 0.0) {
    ++a.dropped_singular;
    ++total_dropped_;
    return;
  }
  ++a.n;
  // Welford running mean.
  const double delta = *nis - a.mean;
  a.mean += delta / static_cast<double>(a.n);
  a.dim = e.dim;
  // Per-sample coverage thresholds: ONE-sided upper χ²_m quantiles. For
  // small m (1, 2) the closed forms are exact and much better than
  // Wilson-Hilferty (WH overshoots by ~20% at m=2, biasing coverage
  // upward); use WH for m ≥ 3 where its accuracy is fine.
  auto chi2UpperOneSided = [](std::size_t m, double z) {
    if (m == 1) {
      // χ²_1 = N(0,1)². Φ⁻¹((1+p)/2) for upper-p; we pass z = Φ⁻¹(p).
      // The two relationships line up only when p = 0.5; the standard
      // identity is: χ²_1 upper-p quantile = (Φ⁻¹((1+p)/2))².
      const double zhalf = (z == kZ95Upper) ? 1.959963984540054
                          : (z == kZ99Upper) ? 2.575829303548901 : z;
      return zhalf * zhalf;
    }
    if (m == 2) {
      // χ²_2 = Exp(rate=1/2). Upper-p quantile is −2 ln(1 − p).
      const double p = (z == kZ95Upper) ? 0.95
                      : (z == kZ99Upper) ? 0.99 : 0.95;
      return -2.0 * std::log(1.0 - p);
    }
    return chi2Quantile(m, z);
  };
  const std::size_t m = std::max<std::size_t>(e.dim, 1);
  const double up95 = chi2UpperOneSided(m, kZ95Upper);
  const double up99 = chi2UpperOneSided(m, kZ99Upper);
  if (*nis <= up95) ++a.below_95;
  if (*nis <= up99) ++a.below_99;
  // R ≫ HPHᵀ regime check (mean trace ratio).
  const double tr_R = safeTrace(e.R);
  const double tr_HPH = std::max(0.0, safeTrace(e.S) - tr_R);
  if (tr_R > 0.0) {
    a.sum_trace_ratio += tr_HPH / tr_R;
  }
}

std::map<ConsistencySourceKey, NisStats> NisCollector::finalize() const {
  std::map<ConsistencySourceKey, NisStats> out;
  for (const auto& [key, a] : per_key_) {
    NisStats s;
    s.n = a.n;
    s.dropped_singular = a.dropped_singular;
    s.dim = a.dim;
    if (a.n == 0) {
      s.mean = 0.0;
      s.coverage_95 = 0.0;
      s.coverage_99 = 0.0;
      s.alpha_hat = 1.0;
      s.trace_ratio_HPH_over_R = 0.0;
      s.low_sample = true;
      s.band_lo = std::numeric_limits<double>::quiet_NaN();
      s.band_hi = std::numeric_limits<double>::quiet_NaN();
      out.emplace(key, s);
      continue;
    }
    s.mean = a.mean;
    s.coverage_95 = static_cast<double>(a.below_95) / static_cast<double>(a.n);
    s.coverage_99 = static_cast<double>(a.below_99) / static_cast<double>(a.n);
    s.alpha_hat = a.mean / std::max<std::size_t>(a.dim, 1);
    s.trace_ratio_HPH_over_R =
        a.sum_trace_ratio / static_cast<double>(a.n);
    // Wilson-Hilferty band on ε̄: N·ε̄/m ∼ χ²_{Nm}/(Nm), so the band on
    // ε̄ is [χ²_{Nm, 0.025}, χ²_{Nm, 0.975}] / N.
    if (a.n >= kMinSamples) {
      const std::size_t Nm = a.n * std::max<std::size_t>(a.dim, 1);
      const double lo_q = chi2Quantile(Nm, -kZ95Band);
      const double hi_q = chi2Quantile(Nm, kZ95Band);
      s.band_lo = lo_q / static_cast<double>(a.n);
      s.band_hi = hi_q / static_cast<double>(a.n);
      s.low_sample = false;
    } else {
      s.band_lo = std::numeric_limits<double>::quiet_NaN();
      s.band_hi = std::numeric_limits<double>::quiet_NaN();
      s.low_sample = true;
    }
    out.emplace(key, s);
  }
  return out;
}

// === NEES =================================================================

// Math:
//   At each BenchStep, assignPerStep maps each truth slot to at most one
//   track id (greedy under gate_m). For each assigned pair:
//     e = p_truth − p̂  (ENU, 2-d)
//     εⁿᵉᵉˢ = eᵀ P_xy⁻¹ e
//   ε̄ⁿᵉᵉˢ and p95 across all assigned samples; β̂ = ε̄ / 2.
// Assumptions:
//   BenchStep::tracks already filtered to Confirmed (see BenchRunner);
//   covariance carries the post-update P_xy at snapshot time. Tentative
//   tracks are not in the snapshot, so no Tentative/Confirmed split is
//   needed at this layer.
// Rationale:
//   Position-only NEES, not full-state: AutoFerry truth SOG is
//   derivative-quality (RMSE columns already flag the issue) and full-
//   state NEES would blame the velocity block for a position-cov bug.
// Improve next:
//   - Velocity-block NEES once a truth source with reliable SOG/COG
//     uncertainty is available.
//   - Per-(truth_id, step) timeseries dump for plotting.
NeesStats computeNees(const BenchResult& result, double gate_m) {
  NeesStats out;
  const auto assigns = assignPerStep(result, gate_m);
  std::vector<double> samples;
  samples.reserve(result.steps.size());
  std::size_t below_95 = 0;
  std::size_t dropped = 0;
  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& step = result.steps[k];
    const auto& a = assigns[k];
    for (std::size_t i = 0;
         i < std::min(step.truth.size(), a.size()); ++i) {
      if (!a[i].has_value()) continue;
      const TrackId tid = *a[i];
      auto it = std::find_if(step.tracks.begin(), step.tracks.end(),
                             [&](const TrackStateSnapshot& s) {
                               return s.id.value == tid.value;
                             });
      if (it == step.tracks.end()) continue;
      const Eigen::Vector2d e = step.truth[i].position - it->position;
      Eigen::MatrixXd P = it->pos_covariance;
      // A track with zero covariance has not been measured; skip
      // rather than divide by zero. This handles state.size() < 2
      // edge case + any sub-2x2 covariance defensively.
      if (P.rows() < 2 || P.cols() < 2) {
        ++dropped;
        continue;
      }
      const auto qf = quadForm(P, e);
      if (!qf.has_value() || !std::isfinite(*qf) || *qf < 0.0) {
        ++dropped;
        continue;
      }
      samples.push_back(*qf);
      // χ²_2 upper-95% = −2·ln(0.05) = 5.99146.
      if (*qf <= 5.991464547107983) ++below_95;
    }
  }
  out.n = samples.size();
  out.dropped_singular = dropped;
  if (out.n == 0) {
    out.mean = 0.0;
    out.p95 = 0.0;
    out.coverage_95 = 0.0;
    out.beta_hat = 1.0;
    out.low_sample = true;
    out.band_lo = std::numeric_limits<double>::quiet_NaN();
    out.band_hi = std::numeric_limits<double>::quiet_NaN();
    return out;
  }
  double sum = 0.0;
  for (double v : samples) sum += v;
  out.mean = sum / static_cast<double>(out.n);
  out.median = ::navtracker::benchmark::percentile(samples, 0.50);
  out.p95 = ::navtracker::benchmark::percentile(samples, 0.95);
  out.p99 = ::navtracker::benchmark::percentile(samples, 0.99);
  out.coverage_95 = static_cast<double>(below_95) / static_cast<double>(out.n);
  out.beta_hat = out.mean / 2.0;
  if (out.n >= kMinSamples) {
    const std::size_t Nm = out.n * 2;  // m = 2 (position only)
    const double lo_q = chi2Quantile(Nm, -kZ95Band);
    const double hi_q = chi2Quantile(Nm, kZ95Band);
    out.band_lo = lo_q / static_cast<double>(out.n);
    out.band_hi = hi_q / static_cast<double>(out.n);
    out.low_sample = false;
  } else {
    out.band_lo = std::numeric_limits<double>::quiet_NaN();
    out.band_hi = std::numeric_limits<double>::quiet_NaN();
    out.low_sample = true;
  }
  return out;
}

ConsistencyResult computeConsistency(const NisCollector& nis,
                                     const BenchResult& result,
                                     double assoc_gate_m) {
  ConsistencyResult c;
  c.per_source = nis.finalize();
  c.nees = computeNees(result, assoc_gate_m);
  return c;
}

}  // namespace benchmark
}  // namespace navtracker
