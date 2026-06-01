# Evaluation Log

Running record of measured comparisons between estimator / associator
alternatives and the baseline. Each entry records the scenarios run, the
numbers, and a one-line takeaway. Predictions go in the algorithm docs;
this file holds *observations* only.

Tracker configuration unless noted: `ConstantVelocity2D(q=0.1)`,
`GnnAssociator`, `TrackManager`, baseline thresholds from the scenario tests.

## 2026-06-01 — UKF vs EKF baseline (4 scenarios)

Filter swapped behind the `IEstimator` port; everything else identical
between the two runs. Source: `tests/scenario/test_filter_comparison.cpp`.

| Scenario | Filter | mean OSPA (m) | ID switches | Final tracks |
|----------|--------|---------------|-------------|--------------|
| SingleStraightLine (20 steps, σ=5 m) | EKF | 4.9904 | 0 | 1 |
| SingleStraightLine | UKF | 4.9904 | 0 | 1 |
| ParallelTargets (30 steps, σ=5 m, 800 m apart) | EKF | 4.1646 | 0 | 2 |
| ParallelTargets | UKF | 4.1646 | 0 | 2 |
| Crossing (40 steps, σ=8 m, 20 m offset at crossing) | EKF | 7.1620 | 0 | 2 |
| Crossing | UKF | 7.1620 | 0 | 2 |
| AisDropout (5 → 7 s gap → 9 steps, σ=5 m) | EKF | 5.5534 | 0 | 1 |
| AisDropout | UKF | 5.5534 | 0 | 1 |

**Takeaway.** Bit-identical to 4 decimals on every scenario. Reason: every
scenario uses Position2D measurements (linear `h`); on linear `h` the UKF
posterior equals the Kalman posterior by construction (Wan–van der Merwe).
The current scenario suite **cannot** differentiate kinematic-filter
choices. Distinguishing UKF (and later particle / IMM) from the EKF requires
a scenario where the measurement function is materially nonlinear:
short-range range/bearing, bearing-only, or rapid range-rate. That scenario
is not built yet.

**What this means for the next filter (particle).** Building a
short-range range/bearing scenario with appreciable prior position
uncertainty is a prerequisite for any meaningful comparison. Without it
every estimator we add will show identical numbers and we'll learn nothing.
Recommend adding that scenario to `core/scenario/Builders.hpp` before or
alongside the particle filter.

## 2026-06-01 — UKF vs EKF on range/bearing pass scenarios

New builder `buildRangeBearingPassScenario` (initial Position2D seed →
RangeBearing2D thereafter, sensor at ENU origin). Two configurations:

| Scenario | Geometry | Filter | mean OSPA (m) | Δ vs EKF |
|----------|----------|--------|---------------|----------|
| ShortRangePass | CPA ≈ 50 m, σ_r=10 m, σ_β=5° | EKF | 8.6976 | — |
| ShortRangePass | as above | UKF | 8.6308 | −0.068 m (−0.8%) |
| VeryShortRangePass | CPA ≈ 20 m, σ_r=20 m, σ_β=10° | EKF | 17.2779 | — |
| VeryShortRangePass | as above | UKF | 16.1210 | −1.157 m (−6.7%) |

**Takeaway.** UKF advantage is real and **scales with nonlinearity
intensity**, as theory predicts. The mild-nonlinearity case (CPA 50 m, small
noise) shows a ~1% improvement — near the noise floor of a single-seed run
and likely not worth the extra cost in production. The sharper case (CPA
20 m, large noise) shows ~7% — the EKF's first-order linearization across
the closest-approach geometry materially diverges from the unscented
treatment.

**Implication.** Quoting a single "UKF vs EKF" number is misleading; the
ratio depends entirely on how close to the sensor the geometry gets and how
much measurement and prior uncertainty there is. For realistic maritime
scenarios where vessels stay >1 km from sensors, the gap will be small. For
harbor-proximity, docking, or close passes, the gap matters.

**Methodology notes.** Single fixed seed per scenario, so the absolute
numbers carry single-realization noise. A proper comparison would average
over multiple seeds and report a confidence interval — that's a documented
next step. Two configurations is a thin sample; widening to a sweep of CPAs
and noise levels would let us draw the EKF→UKF transfer curve quantitatively.

## 2026-06-01 — PF vs EKF vs UKF on range/bearing pass scenarios

`ParticleFilterEstimator` with `N=1000`, `ess_fraction=0.5`,
`init_speed_std=10`, seed = scenario seed. Same scenarios, gates, and
thresholds as the previous entry.

| Scenario | Filter | mean OSPA (m) | Δ vs EKF |
|----------|--------|---------------|----------|
| ShortRangePass | EKF | 8.6976 | — |
| ShortRangePass | UKF | 8.6308 | −0.068 m (−0.8%) |
| ShortRangePass | PF  | 9.9828 | +1.285 m (+14.8%) |
| VeryShortRangePass | EKF | 17.2779 | — |
| VeryShortRangePass | UKF | 16.1210 | −1.157 m (−6.7%) |
| VeryShortRangePass | PF  | 16.4674 | −0.811 m (−4.7%) |

**Takeaway.** The PF lands *behind* both Kalman variants on the mild
nonlinearity scenario and *between* them on the sharper one. This is the
expected outcome for a bootstrap PF on a unimodal posterior: with N=1000
particles and a 4-D state the Monte-Carlo variance of the weighted mean is
non-negligible, and there is no offsetting structural advantage when the
true posterior is well-approximated by a Gaussian. The UKF's `2n+1 = 9`
sigma points capture the second-moment correction at a tiny fraction of the
runtime cost.

The PF's theoretical advantage — representing non-Gaussian or *multimodal*
posteriors — is not exercised by either of these scenarios. Both pass
geometries produce a posterior that converges to a single mode once range
information accumulates over a few updates. To see the PF win against the
UKF we need a scenario where the posterior is genuinely multimodal: a
**bearing-only** track, a target near closest approach with high prior
position uncertainty, or two targets whose individual posteriors overlap
significantly. Documented as the next scenario to build.

**Methodology notes.** Single seed per filter, N=1000, ESS threshold 0.5·N.
The PF runs to completion in tens of milliseconds for these scenarios, so
runtime is not a current concern. Multi-seed averaging is still the right
next step before quoting absolute numbers — single-seed deltas of <1 m are
within Monte-Carlo noise for N=1000. The 14.8% gap on ShortRangePass is
large enough that it would survive averaging, but the 4.7% gap on
VeryShortRangePass is borderline.

**Open follow-ups.** (1) Build a bearing-only or close-approach scenario
where the posterior is provably multimodal. (2) Sweep `N ∈ {200, 500, 1000,
2000, 5000}` to characterize the variance/cost trade. (3) Refactor
`MeasurementModels` so the PF update path does not allocate a throwaway
Jacobian `H` per particle (noted in the code-review of Task 5).
