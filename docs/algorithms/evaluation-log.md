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
