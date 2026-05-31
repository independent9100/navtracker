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
localization breakdown; multi-seed runs with variance.

## 4. Stress scenarios

### Crossing (`buildCrossingTargetsScenario`)

**Math/Logic.** Two CV targets on opposing courses with a configurable
lateral offset. They cross `x = 0` at a known time with `2·offset` meters of
y-separation. Stress comes from the data-association decision at the
crossing instant when measurement noise overlaps the lateral separation.

**Assumptions.** Targets emit truth in (A, B) order per step; one
measurement per target per step.

**Rationale.** The canonical GNN failure mode (target swap).

**Baseline result (GNN+EKF, 20 m offset, σ=8 m).** 0 ID switches —
20 m of lateral separation is enough to prevent the swap. Tightening the
offset or raising σ further will eventually stress the baseline; that
sharpening is intentionally left for the next iteration.

**Ways to improve / test next.** Variable crossing angle and offset;
multiple simultaneous crossings; mixed-rate sensors at the crossing.

### Overtaking (`buildOvertakingScenario`)

**Math/Logic.** Same-direction CV targets with a y-offset; the faster one
passes the slower one along x. With sufficient lateral separation tracks
should stay distinct.

**Assumptions.** Cross-track separation ≥ several measurement σ to keep the
gate uncluttered.

**Rationale.** Close-pass without crossing; tests that proximity alone
doesn't trigger an ID switch.

**Baseline result (GNN+EKF, 30 m offset, σ=5 m).** 0 ID switches, both
tracks confirmed throughout.

**Ways to improve / test next.** Varying lateral offset; multi-sensor
contributions during the pass.

### AIS dropout (reuses `buildStraightLineScenario` with sparse times)

**Math/Logic.** Single straight-line target with a gap in the measurement
stream; the EKF predicts across the gap. With `gap < miss_timeout`, no
maintenance miss fires while no measurements arrive, so the track coasts and
re-associates the first post-gap measurement.

**Assumptions.** Gap is shorter than `miss_timeout`; gate wide enough that
the predicted post-gap position still includes the resumed measurement.

**Rationale.** Tests track survival across cooperative-sensor outages
(common in real AIS feeds).

**Baseline result (gap=7 s, miss_timeout=15 s, gate=80 d²).** Track
survives with the same `TrackId` across the gap, 0 ID switches.

**Ways to improve / test next.** Multi-target dropout; cross-sensor
hand-off during the gap (AIS drops while ARPA keeps reporting);
gap > miss_timeout case (track gets deleted and re-initiated — should still
have a stable internal id, just a new one).

## 5. ID-switch metric (`countIdSwitches`)

**Math.** For each truth index i, maintain `last[i]` = last assigned
`TrackId.value`. At each step, find the nearest track within `cutoff` to
truth[i]; if `last[i] ≠ 0 ∧ now ≠ 0 ∧ now ≠ last[i]`, increment switches;
then update `last[i] = now` when `now ≠ 0`.

**Assumptions.** Truth ordering is index-stable across steps (true for the
builders); track-loss events (no track in gate) do NOT count as switches.

**Rationale.** Captures the canonical GNN failure (one track's id is
replaced by another's mid-scenario).

**Ways to improve / test next.** Hungarian-based truth-track matching for
sets where index correspondence is unclear; weighted by track-lifetime.
