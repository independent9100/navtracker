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

## 2026-06-01 — Multi-seed N-sweep on ShortRangePass

Same scenario as before, but each `(filter, seed)` cell rerun for 20 seeds
to convert single-realization deltas into mean ± standard deviation.

Source: `tests/scenario/test_filter_comparison.cpp` ::
`FilterComparison.ShortRangeMultiSeedSweep` (seeds 41–60).

| Filter / Config | mean OSPA (m) ± stddev |
|-----------------|------------------------|
| EKF             | 9.2929 ± 1.4251 |
| UKF             | 9.2467 ± 1.4377 |
| PF, N=200       | 16.5884 ± 10.2107 |
| PF, N=500       | 10.6783 ± 4.5291 |
| PF, N=1000      | 10.0169 ± 1.5601 |
| PF, N=2000      | 9.8517 ± 1.9292 |

**Takeaway, retraction.** The previous entry quoted a 0.8% UKF advantage on
ShortRangePass from a single seed. The 20-seed average **vacates** that
claim: UKF beats EKF by **0.05 m (≈ 0.5%)** which is well within
single-realization noise (1.4 m). On a unimodal Position+range/bearing
posterior at moderate range, EKF and UKF are statistically indistinguishable
on this scenario. The single-seed VeryShortRangePass UKF advantage (6.7%)
likely survives averaging but is not yet re-measured.

**Particle filter cost / accuracy frontier.** The PF requires roughly N=1000
to come within ~8% of the UKF and N=2000 to come within ~6%. At N=200 it is
catastrophically noisy (16.6 ± 10 m), meaning the bootstrap PF needs
sufficient particle count for *minimum* viability before the trade-off
discussion even starts. This is the expected story for a bootstrap PF on a
Gaussian-ish posterior: no structural advantage to redeem the Monte-Carlo
variance. Adaptive `N` or a more sophisticated PF variant (auxiliary,
marginalized) is the next thing to try if PF is to be competitive at lower
N.

## 2026-06-01 — Bearing-only pass scenario (stationary sensor)

New scenario builder `buildBearingOnlyScenario` + new measurement model
`MeasurementModel::Bearing2D` (scalar `β = atan2(py, px)`, 1×4 Jacobian).
Sensor at ENU origin, **stationary**. Initial Position2D seed with σ=80 m
(wide), then 60 s of bearing-only measurements with σ=3°.

| Filter | mean OSPA (m) (seed 71) |
|--------|--------------------------|
| EKF | 181.85 |
| UKF | 183.26 |
| PF, N=2000 | 183.66 |

**Takeaway.** All three filters are statistically indistinguishable on this
scenario. The expected PF advantage (representing a non-Gaussian
banana-shaped posterior) is **not realized** here, because from a stationary
sensor with no own-ship motion the **range channel is genuinely
unobservable** — there is no information in any bearing sequence that
recovers range. The posterior on range stays as wide as the prior allowed,
and OSPA is dominated by the along-bearing position error that no estimator
can fix. The PF correctly maintains the spread rather than artificially
collapsing it, which is the right behaviour but invisible in OSPA.

**Implication.** Bearing-only is *not* automatically a PF-favouring
scenario. The PF only beats a Kalman filter when the posterior is genuinely
non-Gaussian AND the data carries enough information to localize the true
mode. To exercise the PF's real advantage we need one of:
1. **Own-ship motion** — moving sensor → parallax → range becomes weakly
   observable; intermediate posteriors are banana-shaped.
2. **Crossed bearings** from a second sensor with known offset — produces a
   bimodal prior that collapses to one mode as more data arrives.
3. **Maneuvering target with known constraint** (e.g. confined to a
   channel) breaking the bearing-only symmetry.

All three are substantial scenario work and require a sensor-frame abstraction
the codebase does not yet have. Documented as the next scenario investment.

**Open follow-ups (carried forward).** (1) Build a scenario with own-ship
motion to make bearing-only range observable. (2) Sweep the PF on
VeryShortRangePass over 20 seeds to confirm whether the 6.7% UKF advantage
survives averaging. (3) Auxiliary or regularized PF variants to reduce the
N required for viability.

## 2026-06-01 — IMM (CV+CT, EKF backend) vs EKF/UKF/PF on maneuvering target

`ImmEstimator` with K=2 (`ConstantVelocity5State` + `CoordinatedTurn`), EKF
backend per mode, transition matrix `[[0.95, 0.05], [0.10, 0.90]]`,
initial mode probabilities `[0.5, 0.5]`, `q_a = 0.5`, `q_ω = 0.1` (CT) and
`0.01` (CV5). Scenario: target moves straight for 5 s, turns at 0.2 rad/s
for 5 s, straight for 5 s. Position2D measurements at 1 Hz, σ = 5 m.
Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.ManeuveringTarget`.

| Filter | mean OSPA (m) |
|--------|----------------|
| EKF (CV2D)        | 6.5871 |
| UKF (CV2D)        | 6.5871 |
| PF  (CV2D, N=1000)| 6.7230 |
| IMM (CV5 + CT)    | 6.5871 |

**Takeaway.** IMM ties EKF and UKF exactly to four decimals. This is **not
a measurement-noise-floor effect** — a sharper diagnostic scenario
(`ω = 0.5 rad/s`, σ = 1 m, dt = 0.5 s, 8 s turn) gave EKF=UKF=IMM=1.9767
with PF=41.0334 (collapsed). The IMM's CT-mode probability is observed to
**decline monotonically** from its initial 0.5 throughout the run, reaching
0.334 at the end — the CT mode is never activated, regardless of how
sharp the turn is.

**Diagnosis (not a bug).** With `Position2D`-only measurements the
linearized `H` has zero in the `ω` column, so `ω` is unobservable by the
EKF update. Both CV and CT modes converge their `ω_mean` to 0, making
their predicted positions essentially identical, so their likelihoods are
indistinguishable. The transition-matrix prior (CV self-loop 0.95 vs
CT self-loop 0.90) then drives the mode probability monotonically toward
CV. The IMM algorithm is correct — it's the position-only + EKF-backend
+ symmetric-2-mode configuration that has no observability path.

**Implication.** The current IMM is correctly built and unit-tested, but
it does not win against single-model CV on the position-only scenarios we
have. To see IMM win on position-only measurements, implement
**prescribed-rate three-mode IMM** (`CV + CT(+ω̂) + CT(−ω̂)`) — the
classic maritime configuration. This is captured as the next IMM step in
`docs/algorithms/estimation.md` § 6 "Known limitation".

**Methodology notes.** Single seed (91), single scenario, single
configuration. With IMM tied to the single-mode baseline, multi-seed
averaging does not change the conclusion; no sweep was run.

## 2026-06-01 — Three-mode IMM (CV + prescribed CT±) on maneuvering target

`PrescribedTurn(omega_const, q_a, q_omega)` motion model: fixed turn rate
at construction, otherwise identical to `CoordinatedTurn`. Three-mode IMM
configuration: `{CV5State(0.5, 0.001), PrescribedTurn(+0.2, 0.5, 0.001),
PrescribedTurn(-0.2, 0.5, 0.001)}`, transition matrix
`[[0.90,0.05,0.05],[0.10,0.85,0.05],[0.10,0.05,0.85]]`, initial mixture
`[0.34, 0.33, 0.33]`. Same maneuvering scenario as the previous IMM entry
(5 s straight + 5 s turn at +0.2 rad/s + 5 s straight; 1 Hz Position2D,
σ = 5 m, seed 91).

Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.Maneuvering3ModeIMM`.

| Filter | mean OSPA (m) | Δ vs EKF |
|--------|----------------|----------|
| EKF (CV2D)                          | 6.5871 | — |
| IMM-2 (CV5 + free-ω CT)             | 6.5871 | 0.0 (0%) |
| IMM-3 (CV5 + CT(+0.2) + CT(-0.2))   | **6.0973** | **−0.4898 (−7.4%)** |

**Takeaway.** First IMM configuration to actually beat the EKF baseline
on any scenario in this codebase. The mechanism is exactly the one
predicted in the prior entry's diagnosis: `CT(+0.2)` matches the true turn
rate, so during the 5-second turn its predicted positions track truth
while the CV mode's predicted positions diverge. The mode-probability
update shifts mass to `CT(+0.2)`, the mixture projection uses it more,
and OSPA drops. `CT(-0.2)` stays quiet (its predicted positions diverge
even more than CV's during a left turn).

The 7.4% number is bounded by the fact that the maneuver is only 5/15 of
the scenario duration. Restricting OSPA to just the turn-segment timesteps
would show a much larger gap; the cross-segment average is what we report
for direct comparability with the prior IMM-2 entry.

**Implication.** Prescribed-rate IMM is the right baseline for maritime
maneuver tracking with position-only AIS/Position2D inputs. The free-ω
single-CT IMM-2 should be considered a curiosity rather than a useful
configuration when measurements don't observe ω.

**Methodology notes.** Single seed (91). Multi-seed averaging would tighten
the 7.4% claim but the directional result (IMM-3 < EKF, IMM-2 ≈ EKF) is
robust by construction — the prescribed-rate CT has a structural advantage
the other modes cannot offer.

**Open follow-ups.** (1) Multi-seed sweep on this scenario. (2) Sweep over
maneuver rate ω_true with fixed prescribed ω̂ to characterize the
sensitivity (how close does ω̂ have to match ω_true for IMM-3 to win?).
(3) Wider mode bank, e.g. `CV + CT(±0.1) + CT(±0.2) + CT(±0.5)`.
(4) UKF backend per mode to let a single free-ω CT mode work via
sigma-point propagation through F (replaces prescribed rates).

## 2026-06-01 — JPDA vs GNN on clutter-crossing scenario

`JpdaAssociator(gate=20, P_D=0.9, λ_C=1e-4)` vs
`GnnAssociator(gate=20)`. Same backend (`EkfEstimator` with
`ConstantVelocity2D(0.1)`), same lifecycle thresholds. Scenario:
`buildClutterCrossingScenario` — two CV targets crossing at the origin
plus 4 uniform false alarms per scan in [−300, 300] × [−50, 50], target
measurement σ = 5 m, 30 scans, seed 31. Run via `runScenarioBatched`.

Source: `tests/scenario/test_jpda_comparison.cpp::JpdaComparison.ClutterCrossing`.

| Associator | mean OSPA (m) | ID switches | Final tracks |
|------------|----------------|-------------|---------------|
| GNN  | 47.3286 | **11** | **35** |
| JPDA | 45.9158 | **4**  | **14** |

**Takeaway.** JPDA's primary value is identity stability and clutter
rejection, not localization accuracy. ID switches drop **64%** (11 → 4)
and final track count drops **60%** (35 → 14). The OSPA improvement is
modest (~3%) because OSPA is clipped at the cutoff (50 m) and the GNN's
errors are largely identity errors (track-swap at crossing) rather than
position errors — the cutoff masks them. The right metric for this
comparison is ID switches, not OSPA.

The 35-track count for GNN reflects clutter contamination — each scan's
4 false alarms have nonzero probability of being the closest in-gate
measurement to some track, kicking GNN's hard assignment to a clutter
point and seeding a new track when the next scan's real measurement
fails to match the contaminated old track. JPDA's soft update spreads
mass across all in-gate measurements weighted by likelihood; clutter
measurements contribute near-zero weight to real tracks and the new
tracks they would seed get suppressed by the M-of-N confirmation policy.

**Methodology notes.** Single seed (31). Multi-seed sweep would
strengthen the OSPA number but the ID-switch reduction is structural
(it follows from JPDA's contamination resistance, not from one lucky
seed) and should survive averaging. The clutter density (~ 4 false
alarms per scan over a 600×100 m box) is moderate; sweeping density up
should widen JPDA's advantage further. Runtime: enumeration cost is
negligible at 2 tracks × ~6 gated measurements per scan.

**Open follow-ups.** (1) Multi-seed × multi-clutter-density sweep.
(2) JIPDA (Integrated PDA) — adds per-track existence probability,
ties into M-of-N. (3) K-best joint events for cluster sizes beyond
~6×6. (4) MHT, the natural next step in the hypothesis-deferment line.
