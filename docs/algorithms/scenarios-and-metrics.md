# Scenarios & Metrics

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec §9 (testing).

## 1. Scenario types and builders

**Math/Logic.** Constant-velocity straight-line truth: `pos(t) = start + v·t`.
Measurements: truth + Gaussian noise (zero mean, std `pos_noise_std_m`,
identity R). Parallel-targets variant emits two independent CV truths at the
same timestamps, interleaving their measurements.

**Assumptions.** Constant velocity per target; Gaussian zero-mean position
noise; deterministic `std::mt19937` seeded from `seed`; truth and measurements
share timestamps.

**Rationale.** Smallest deterministic scenarios that exercise (a) the EKF on a
single target and (b) multi-target counting / association.

**Ways to improve / test next.** Maneuvering trajectories (turn rate, change
of speed); crossing / overtaking encounters; AIS dropout, non-cooperative
target (no MMSI); per-sensor noise models; truth interpolation for arbitrary
query times.

## 2. OSPA (greedy, p=2)

**Math.** Given truth set X, estimate set Y, cutoff `c`, `n = max(|X|,|Y|)`.
Greedily pair the closest remaining (x,y); record `min(‖x−y‖, c)²`. Unmatched
elements contribute `c²` each. `OSPA = √(sum / n)`.

**Assumptions.** p = 2; cutoff units = meters; greedy assignment in place of
the optimal Hungarian.

**Rationale.** Standard MTT performance metric. Greedy mirrors the GNN data
associator and is cheap; gives a meaningful scalar score for scenario
comparisons.

**Ways to improve / test next.** Hungarian assignment for optimal OSPA;
OSPA² over time windows; OSPA decomposition (localization vs cardinality).

## 3. Harness (`runScenario`)

**Math/Logic.** Drives the supplied `Tracker` over the scenario's measurement
stream in chronological order. At each measurement time, samples truth
positions where `truth.time == z.time` and estimated positions from
`manager.tracks()` (first two state components = ENU position), computes
`ospaGreedy`, and aggregates the mean.

**Assumptions.** State layout begins with `[px, py]`; truth and measurements
share timestamps (true for the baseline builders).

**Rationale.** Single entry point that takes any composed tracker
configuration (estimator + associator + manager) and a scenario, returns a
scalar score — the foundation for comparing algorithm choices (UKF/IMM
estimator, JPDA/MHT associator, hint locking) against the baseline.

**Ways to improve / test next.** Truth interpolation; cardinality /
localization breakdown; ID-switch counting; multi-seed runs with variance.
