// Backlog #21 — robust guard against the imm_cv_ct_pmbm_adapt_k3 knife-edge on
// harbor_complete_truth.
//
// Diagnosis (Phase 0 of the clutter/birth campaign, 2026-07-06): the fragility
// #21 describes is NOT the min_new_bernoulli_existence gate (a fine sweep of it
// leaves harbor card_err bit-flat). It is the **birth intensity lambda_birth**.
// adapt_k3 ships lambda_birth = 1e-5, which sits exactly on a card_err CLIFF:
//
//     lambda_birth <= 9.7e-6  -> card_err 7.165  (wide flat plateau, 8 samples)
//     lambda_birth  = 1.0e-5  -> card_err 10.90  (ON the cliff face)   <-- ships here
//     lambda_birth >= 1.05e-5 -> card_err 11.82  (flat plateau)
//
// Because the default is wedged on the cliff, an epsilon-class change anywhere in
// the estimator hot path (e.g. the perf-round-3 2x2 kernels) shifts rho_target by
// ~1e-15, tips one borderline harbor birth across confirm_threshold, and flips
// card_err_mean / gospa_false by the small #21 amounts (~0.5 / ~95) with a cascade
// into the derived per-truth rows. It is isolated to this one non-KEEP config; all
// KEEP/land/dense_clutter configs stayed byte-identical under the same changes.
//
// There is NO numeric gtest pinning adapt_k3 harbor numbers to a point (test_config
// only checks the label exists); the flip surfaces only in the campaign's Class-A/B
// byte-compare pricing, where it re-raises a spurious "did the math change?" on every
// epsilon-class change — and the clutter/birth campaign runs harbor constantly.
//
// FIX (this test) = the freeze-rule "robust assertion (tolerance / cardinality band
// instead of a pinned point)". The band brackets the fp-tie flip with margin, so a
// harmless epsilon flip stays GREEN while a REAL change (a birth spiral pushing
// card_err past the band, or a collapse onto the 7.165 low plateau = births
// suppressed / real targets dropped) goes RED. During pricing, treat adapt_k3 harbor
// rows that move WITHIN this band as fp-tie noise, not drift.
//
// Alternative fix (NOT taken here; left to the arbiter): nudge lambda_birth off the
// cliff onto a plateau. The smallest, target-preserving nudge is UP to ~1.1e-5
// (card_err 10.90->11.82, gospa_missed 45.5->36.5, lifetime 0.9615->0.9725 — keeps
// MORE real targets, over-counts +0.9). DOWN to <=9.7e-6 lands on 7.165 but drops
// real anchored boats (gospa_missed 45.5->79) — a presence-over-classification
// regression, not recommended. A nudge changes adapt_k3 on ALL scenarios and needs a
// cross-scenario re-price, so it is a deliberate config decision, not a Phase-0
// mechanical defusing — hence the band guard instead. See docs/algorithms/
// improvement-backlog.md #21.
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
double meanMetric(const std::vector<MetricRow>& rows, const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.scenario == "harbor_complete_truth" && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

std::vector<std::unique_ptr<ScenarioRun>> harborOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_complete_truth")
      out.push_back(std::move(s));
  return out;
}
}  // namespace

TEST(AdaptK3HarborKnifeGuard, CardinalityStaysWithinTheKnifeEdgeBand) {
  const Config* cfg = nullptr;
  const auto all = defaultConfigs();
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm_adapt_k3") cfg = &c;
  ASSERT_NE(cfg, nullptr) << "imm_cv_ct_pmbm_adapt_k3 config missing";

  SweepParams params;
  params.run_id = "adapt_k3_harbor_knife_guard";
  params.synthetic_seeds = 5;  // fixed → deterministic band; keeps ctest cheap
  const auto rows = runSweep({*cfg}, harborOnly(), params);
  ASSERT_FALSE(rows.empty());

  const double card_err = meanMetric(rows, "card_err_mean");
  const double gospa_false = meanMetric(rows, "gospa_false");
  std::cout << "\n=== adapt_k3 harbor_complete_truth knife-edge guard (5 seeds) ===\n"
            << "  card_err_mean = " << card_err << "  (band [8.0, 14.0])\n"
            << "  gospa_false   = " << gospa_false << "  (band [1600, 2900])\n"
            << std::flush;

  // Band brackets the lambda_birth-cliff fp-tie (current ~10.74 / ~2188 at 5
  // seeds; historical ~9.4-9.9 / ~1910-2005) with margin. Lower bounds exclude the
  // 7.165 / 1512 low plateau (a real over-suppression / target-drop). Upper bounds
  // catch a birth spiral. A move WITHIN the band under an epsilon-class change is
  // the known fp-tie, not drift — do not investigate it as a math change.
  EXPECT_GE(card_err, 8.0)
      << "adapt_k3 harbor card_err collapsed below the knife-edge band — likely "
         "birth over-suppression / real targets dropped (see backlog #21).";
  EXPECT_LE(card_err, 14.0)
      << "adapt_k3 harbor card_err spiralled above the knife-edge band — likely a "
         "real birth/clutter regression (see backlog #21).";
  EXPECT_GE(gospa_false, 1600.0)
      << "adapt_k3 harbor gospa_false collapsed below the knife-edge band (see "
         "backlog #21).";
  EXPECT_LE(gospa_false, 2900.0)
      << "adapt_k3 harbor gospa_false spiralled above the knife-edge band (see "
         "backlog #21).";
}
