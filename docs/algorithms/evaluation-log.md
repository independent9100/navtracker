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

## 2026-06-01 — GNN / JPDA / MHT on crossing-with-dropout scenario

`MhtTracker` (P_D=0.9, λ_C=1e-4, gate=9.0, N_scan=3, K_max_leaves=5,
score_delete=−15.0) vs `JpdaAssociator(gate=9, P_D=0.9, λ_C=1e-4)` vs
`GnnAssociator(gate=9)`. Shared EKF backend
(`EkfEstimator + ConstantVelocity2D(0.1)`). Scenario:
`buildCrossingDropoutScenario(vx=4, y=1, noise=1, dropout=[13, 17), seed=113)`
— two targets cross with ~2 m closest approach, sensor blacks out for
4 consecutive scans across the crossing.

Source: `tests/scenario/test_mht_comparison.cpp::MhtComparison.CrossingWithDropout`.

| Associator | mean OSPA (m) | ID switches | Final tracks |
|------------|----------------|-------------|---------------|
| GNN  | 7.2684 | 3 | 3 (1 ghost) |
| JPDA | 7.2667 | 2 | 3 (1 ghost) |
| **MHT**  | **1.0501** | **0** | **2 (correct)** |

**Takeaway.** Largest single-scenario win in the codebase. MHT preserves
identity through the 4-scan dropout where both single-scan associators
commit too early at the crossing, swap identities, and leave ghost
tracks behind. The 7× OSPA gap is not a tuning artifact — it reflects
the *structural* limit of any per-scan decision rule against a problem
where the right answer is only knowable after seeing post-dropout
measurements. GNN and JPDA are nearly tied (7.27 vs 7.27): JPDA's soft
update doesn't help when both targets are equally likely under any
hypothesis until the dropout ends and trajectories disambiguate.

This is the first scenario where MHT's added complexity over JPDA is
clearly worth it. On the clutter-crossing scenario where the right
answer is locally available each scan (JPDA's 64% ID-switch reduction
already captures most of the value), MHT would not pay off comparably.

**Methodology notes.** Single seed (113). The dropout length (4 scans)
is matched to `N_scan = 3` so the MHT trunk extends exactly through the
gap. Lengthening the dropout beyond `N_scan` would force MHT to commit
during the blackout and erase its advantage. Sensitivity sweep documented
as future work.

**Open follow-ups.** (1) Multi-seed sweep on this scenario to tighten
the OSPA number (the 7× ratio is unlikely to budge much under
averaging — it's structural — but worth confirming). (2) Sensitivity
sweep over `(dropout_length, N_scan, closest_approach)` to characterize
the regime where MHT dominates. (3) K-best global non-conflict via
Murty's — the largest expected improvement to MHT itself. (4) IMM-backed
MHT for maneuvering targets across ambiguous gaps. (5) Murty + JIPDA
hybrid (track existence probability + hypothesis tree) as the
eventual high-end maritime tracker.

## 2026-06-01 — Bearing-only with moving sensor (parallax) — PF wins

`Measurement.sensor_position_enu` is now wired through every estimator and
associator's measurement-model call path. The new scenario builder
`buildBearingOnlyMovingSensorScenario` emits an initial wide-covariance
Position2D seed (σ = 300 m) followed by 60 s of `Bearing2D` measurements
(σ = 1.5°) from a sensor moving +y at 10 m/s **perpendicular to the
line-of-sight** to a stationary target at (1500, 0). Sensor sweeps from
(0, −300) to (0, +300), producing ~22° of bearing change against the
1.5° measurement noise (~15:1 parallax SNR). Wide initial range prior
keeps the posterior in the non-Gaussian regime during the first ~15 s
of convergence.

Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.BearingOnlyMovingSensor`.

| Filter | mean OSPA (m) | Δ vs EKF |
|--------|----------------|----------|
| EKF (CV)         | 181.6201 | — |
| UKF (CV)         | 185.4117 | +3.79 (+2.1%) |
| **PF (CV, N=2000)** | **123.1583** | **−58.46 (−32.2%)** |

**Takeaway.** First scenario in the codebase where the PF demonstrably
beats both Gaussian filters. The mechanism is exactly what theory
predicts: with proper parallax geometry (sensor motion perpendicular to
LOS) and a wide initial range prior, the posterior on `(px, py)` is
genuinely banana-shaped during the early convergence window — the
crescent of (bearing line) ∩ (broad range prior). EKF and UKF
moment-match this into a Gaussian ellipse and accumulate error; the PF
retains the actual non-Gaussian shape through the transient and gets
substantially better position estimates.

UKF is slightly worse than EKF here, consistent with sigma-point
sampling error mildly exceeding linearization error at this nonlinearity
level. Both Kalman variants sit at ~180 m OSPA because they collapse the
banana to an axis-aligned ellipse around the centroid, which is far from
the actual posterior mode early on.

The first-attempt geometry (sensor moves along LOS at (0, 0) toward
target at (1000, 100)) gave only ~2.4° of bearing sweep against 3° noise
and produced a PF *loss* (112.87 vs 100.12 for EKF). That null result
demonstrated the prerequisite: parallax SNR has to exceed measurement
noise by a meaningful margin for the non-Gaussian regime to manifest.
The retuned geometry above passes that bar comfortably.

**Methodology notes.** Single seed (137), N=2000 particles. Multi-seed
sweep is the straightforward next step. The PF win should survive
averaging because the geometry advantage is structural, not seed-dependent.

**Open follow-ups.** (1) Multi-seed sweep to tighten the 32% claim.
(2) Sweep over sensor velocity (slower = less parallax = PF should win
by more, until the parallax disappears entirely). (3) Slowly-moving
target variant. (4) Closer target ((500, 0) instead of (1500, 0)) —
geometry remains in the non-Gaussian regime longer, gap should grow.
(5) Use this scenario harness to test JPDA / MHT with bearing-only
measurements once the soft-update / branching paths support
non-position measurement models.

**Honest summary of the PF story.** Across three bearing-only attempts:
- Stationary sensor, position-only-seed prior: PF tied EKF/UKF
  (~182 m all, range unobservable from a stationary sensor — documented
  earlier).
- Moving sensor, sensor motion **along** LOS: PF *worst* by 12 m
  (parallax SNR too low — first attempt above).
- Moving sensor, sensor motion **perpendicular** to LOS, wide prior:
  PF wins by 32% — the textbook geometry, finally exercised.

The PF advantage was always conditional on geometry that lets the
non-Gaussian posterior actually form. The first two scenarios didn't
provide that; the third does.

## 2026-06-01 — Multi-seed sweep on the four "wins" (retraction + confirmations)

Re-ran the four winning comparison scenarios with 20 seeds each (seeds
201..220, same set across scenarios) to convert single-realization
deltas into mean ± stddev. Source:
`tests/scenario/test_multi_seed_sweep.cpp`.

### IMM-3 on Maneuvering — **confirmed**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| EKF (CV)   | 5.6713 ± 0.9093 | 0.00 |
| IMM-2      | 5.6713 ± 0.9093 | 0.00 |
| IMM-3      | **4.8148 ± 0.5916** | 0.00 |

Confidence intervals do not overlap. The 7.4% single-seed delta tightens
to ~15% in expectation (≈0.86 m), and IMM-3's stddev is also smaller
than EKF's, indicating the prescribed-CT modes reduce *variability* in
addition to mean error. EKF and IMM-2 are bit-identical to 4 decimals
because position-only measurements collapse the 5-state-CV and free-ω-CT
posteriors into the same predicted positions (no information channel to
distinguish ω modes — same observability gap documented earlier).

### JPDA on ClutterCrossing — **confirmed, very cleanly**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| GNN  | 47.3207 ± 0.1141 | 9.90 |
| JPDA | **45.3199 ± 0.4377** | **2.45** |

OSPA intervals barely overlap; ID-switch advantage is enormous and
robust (9.90 → 2.45 mean across 20 seeds = ~75% reduction, larger than
the original single-seed 64% claim). Strongest and most defensible win
in the codebase.

### MHT on CrossingDropout — **retracted**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| GNN  | 1.9659 ± 1.9014 | 0.70 |
| JPDA | 1.9656 ± 1.9017 | **0.20** |
| MHT  | 1.9656 ± 1.9010 | 0.90 |

**The 7× MHT win was a single-seed artifact.** Averaged over 20 seeds,
all three associators land at the same OSPA (1.966) with the same
±1.9 m noise floor. The stddev is ≈97% of the mean, indicating the
scenario is genuinely bimodal: some seeds the crossing resolves
cleanly for everyone, some seeds it doesn't resolve cleanly for anyone.
On ID switches, **JPDA actually slightly beats MHT** (0.20 vs 0.90 mean
across 20 seeds). The previous entry (seed 113) is left intact for
historical record but the "7× lower OSPA" headline is wrong in
expectation — it was a single favorable realization.

**Methodological lesson.** Scenarios designed to expose an algorithm's
structural advantage need multi-seed validation before any claim is
made. The dropout window in `buildCrossingDropoutScenario` interacts
with the seed-driven position noise in ways that make the crossing
genuinely ambiguous on some draws — and on those draws, deferred
commitment doesn't help because the right answer isn't recoverable
even with hindsight.

**What this changes.** MHT is no longer the codebase's "biggest win."
The infrastructure (TrackTree, N-scan pruning, K_local cap) is still
correctly implemented and architecturally useful, but the demonstrated
empirical advantage over JPDA on this specific scenario doesn't survive
averaging. A scenario where MHT *does* dominate over the multi-seed
average likely exists (e.g., longer dropout vs N_scan, more targets,
explicit identity-preservation metric) but hasn't been found yet.
Recorded as an open follow-up.

### PF on BearingOnlyMovingSensor — direction holds, intervals overlap

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| EKF (CV) | 212.8332 ± 124.6144 | 0.00 |
| UKF (CV) | 214.2199 ± 125.9745 | 0.00 |
| PF       | **180.3379 ± 124.5977** | 0.00 |

The 32% single-seed PF win narrows to ~15% in expectation (212.83 vs
180.34 = 32.49 m gap). The direction is consistent — PF beats both
Kalman variants on every aggregate — but the per-seed variance is so
large (±125 m, ≈70% of mean) that confidence intervals overlap
substantially. Individual seeds can favor either filter; the *average*
favors PF.

The large variance is inherent to bearing-only with a wide range prior:
on some seeds the bearing sequence converges range quickly, on others
it stays in the banana-shaped ambiguity zone for most of the run and
every filter does poorly. 20 seeds is insufficient to tighten this;
N ≥ 100 is the right next step to convert "directionally PF wins" into
"PF beats EKF with 95% confidence."

---

### Honest revised summary of the wins

| Component | Multi-seed status |
|-----------|---------------------|
| UKF | Tied with EKF on every scenario averaged |
| **PF** | **Wins on bearing-only-with-parallax, but ±125 m variance — need more seeds** |
| IMM-2 (free ω) | Tied with EKF (observability gap, expected) |
| **IMM-3 (prescribed ω)** | **Confirmed: 15% OSPA reduction with non-overlapping CIs** |
| **JPDA** | **Confirmed: 75% ID-switch reduction, tightest CIs of any win** |
| ~~MHT~~ | **Retracted: ties JPDA on this scenario; no demonstrated win** |

The codebase has **two confirmed wins** (JPDA, IMM-3), **one
directional win** (PF on parallax bearing-only), and **one retraction**
(MHT). Net: still useful, more honest, less impressive than the
single-seed numbers suggested. JPDA is the clear winner of the
association axis on the scenarios we have today; MHT's added complexity
over JPDA is not yet justified on demonstrated empirical grounds.

---

## Bus-driven confirmation pass (2026-06-02)

Re-ran the four winning comparisons through `SimulatedSensorBus` (full
sensor quartet: OwnShip + AIS + ARPA + EO/IR; ARPA clutter Poisson(5)
per rotation on JPDA and MHT scenarios). Metric: per-window OSPA
(1 s windows, mean of per-window means) + cumulative ID-switch count.
20 seeds (range 201..220, identical to the prior direct-Measurement
sweep). Heading bias deferred to §14.9.

### JPDA vs GNN — bus clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| GNN    | 49.8548 ± 0.0222    | 16.90            |
| JPDA   | 49.8452 ± 0.0274    | **18.55**        |

**Verdict: retracted (under bus).** Prior direct-Measurement win (45.32
vs 47.32 OSPA, 2.45 vs 9.90 ID-switches) does not meaningfully survive.
Both methods saturate near the 50 m OSPA cutoff; JPDA's OSPA edge
(~0.01 m) is well inside one stddev, and it actually **loses on
ID-switches** (18.55 vs 16.90). The prior JPDA advantage came from
clean clutter discrimination on direct Position2D measurements; under
the bus's EO/IR-dominated stream (10 Hz, ~600 measurements per 30 s),
the per-batch clutter exposure is too sparse for JPDA's soft-assignment
machinery to differentiate from GNN's hard nearest-neighbour.

### IMM-3 vs CV — bus maneuvering

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | 96.9531 ± 0.4587    | 4.00             |
| IMM-3  | **96.7948 ± 0.3768** | **3.85**        |

**Verdict: directionally preserved, materially diminished.** Prior
direct-Measurement IMM-3 win was 4.81 ± 0.59 vs 5.67 ± 0.91 OSPA
(~15% gap, non-overlapping CIs). Through the bus, IMM-3 still wins on
both metrics but the margin is <1σ (~0.16 m on a ~97 m baseline) — the
prior 15% advantage collapses. Both methods sit near the 100 m OSPA
cutoff, suggesting the 15 s scenario length and the bus's measurement
heterogeneity together prevent IMM from settling into the right mode
before the metric saturates. The direction of the effect is right;
the magnitude no longer matches prior claims.

### PF vs EKF — bus bearing-only moving sensor

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | 387.0256 ± 51.5514  | 0.00             |
| PF     | **380.4102 ± 53.6372** | 0.00          |

**Verdict: directional, unchanged from prior.** PF beats EKF on OSPA by
~6.6 m, but the per-seed stddev (~52 m) means CIs overlap heavily —
identical pattern to the prior direct-Measurement sweep (180 ± 125 vs
213 ± 125). The bus version operates at higher absolute OSPA (~380 m vs
~200 m) because the bus EO/IR bearing-only emission has the
projection-time own-ship pose attached via §14.1 — but the ratio is
similar. No new conclusion: PF directionally wins on the high-curvature
bearing-only posterior; N≥100 seeds needed to nail statistical significance.

### MHT vs JPDA — bus clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| JPDA   | 49.8452 ± 0.0274    | **18.55**        |
| MHT    | **49.5934 ± 0.0465** | 25.55           |

**Verdict: retraction re-confirmed.** Prior multi-seed sweep retracted
the MHT win (both ≈ 1.97 m, tied). Under the bus, MHT shows a tiny OSPA
edge (49.59 vs 49.85, ~0.5%) but loses ID-stability by 38% (25.55 vs
18.55). MHT's deferred branching pays for itself only when track
confusion under clutter can be unwound retroactively — here, the bus's
heavy non-ARPA measurement stream (~600 EO/IR detections per 30 s)
already gives JPDA enough info to track correctly without needing
N-scan hypotheses. **Neither dominates; the retraction stands.**

### Cross-cutting observation

Three of four scenarios (JPDA, IMM-3, MHT) show OSPA saturating near
the cutoff — under bus-realistic noise and 20 seeds, the metric loses
discriminative power between methods. This is itself a finding:

- The prior direct-Measurement scenarios (where each scan was 2 clean
  Position2D measurements) gave tracker-quality differences a clear
  signal pathway. The bus's EO/IR-dominated stream (~600 measurements
  per 30 s scenario) overwhelms the per-scan information advantage
  that JPDA's soft assignment / MHT's deferred branching were
  designed to exploit.
- Likely follow-ups: (a) revisit tracker init/delete and gate
  parameters for bus-regime measurement densities, (b) report
  per-window OSPA *conditioned on at-least-one-track-confirmed* so
  the cardinality-penalty saturation doesn't dominate the signal,
  (c) longer scenario durations for IMM-3 to give the mode-switching
  enough time to express.

The bus pass surfaces these limits honestly rather than burying them
behind tuned parameters chasing the direct-Measurement baselines.

### Methodology notes

- Per-window OSPA differs in scale from the prior per-measurement mean
  OSPA because the bus emits ~10× more measurements than direct-
  Measurement scenarios, and 1 s windows average each tick once. Direct
  comparison of *absolute numbers* between this table and the prior
  table is illustrative, not strict; the comparison that matters is
  between methods on the SAME row of each sub-table.
- Bus injects: 1 Hz OwnShip GPS (no heading bias yet), Class-A SOTDMA
  AIS, 3 s ARPA rotation (with optional Poisson clutter), 10 Hz EO/IR
  with bearing+range or bearing-only.
- Determinism: each seed produces a byte-identical Scenario; re-running
  this table yields the same numbers.
- Tests live at `tests/sim/test_bus_jpda_comparison.cpp`,
  `tests/sim/test_bus_imm3_comparison.cpp`,
  `tests/sim/test_bus_pf_comparison.cpp`,
  `tests/sim/test_bus_mht_comparison.cpp`.

## Post-metric-fix bus pass (2026-06-02)

The "Bus-driven confirmation pass" section above was contaminated by a metric
artifact: `runScenario` / `runScenarioBatched` / `runScenarioBatchedMht`
evaluated OSPA *per measurement* and matched truth by `==` on timestamps.
With truth at 1 Hz and EO/IR at 10 Hz, ~93% of evaluation points had empty
`truth_xy`, and `ospaGreedy([], est, cutoff)` returns exactly the cutoff for
any non-empty track set — pinning the reported mean near saturation. See
`docs/superpowers/plans/2026-06-02-truth-tick-ospa.md` for the fix (drive
OSPA evaluation on the truth-sample clock).

### Saturation evidence (seed=201)

| Scenario | Pre-fix empty-truth % | Pre-fix overall OSPA | Post-fix overall OSPA |
|---|---|---|---|
| JPDA clutter crossing (cutoff 50 m)   | 93.1% | 49.88 | 48.26 |
| IMM-3 maneuvering (cutoff 100 m)      | 92.4% | 98.00 | 81.33 |
| PF bearing-only (cutoff 500 m)        |  0.0% | 329.46 | 329.46 |

PF was untouched because that scenario already configures EO/IR at 1 Hz,
matching the truth sample rate.

### Re-run verdicts (20 seeds, post-fix metric)

#### JPDA vs GNN — clutter crossing (cutoff 50)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| GNN  | 48.27 ± 0.29 | 20.40 |
| JPDA | 48.15 ± 0.35 | 24.20 |

OSPA margin 0.12 m sits well within seed stddev (~0.3 m) — statistically a
tie. GNN wins ID-stability by ~4 switches/30 s on average. **The
direct-measurement JPDA win remains retracted under bus-realistic noise.**
The pre-fix verdict was correct in direction but masked the magnitude: the
metric was already at 49.85 (cutoff 50) so neither method had room to express
itself; now both are 1.5 m below cutoff with a real but tiny gap.

#### IMM-3 vs CV — maneuvering (cutoff 100)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| CV (EKF)   | 76.57 ± 3.47 | 5.55 |
| IMM-3      | 75.51 ± 3.14 | 5.05 |

Direction preserved but the 1.07-m margin is within 1σ. **The direct-measurement
IMM-3 win is meaningfully diminished**: at 15 s scenario length with the bus's
plentiful position fixes (AIS 2 s, ARPA 3 s, EO/IR 10 Hz), the CV-only
estimator stays close enough to truth that IMM's mode-switching advantage
doesn't dominate. Likely needs longer scenarios with sustained maneuvering to
re-express.

#### PF vs EKF — bearing-only moving sensor (cutoff 500)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| EKF | 387.03 ± 51.55 | 0.00 |
| PF  | 380.41 ± 53.64 | 0.00 |

**Numerically identical to pre-fix** (as predicted): this scenario configures
EO/IR at 1 Hz with truth at 1 Hz, so no cadence mismatch → no saturation. The
prior verdict stands: directional PF advantage, CIs overlap, PF is not a
clearly justified choice for bearing-only in this regime.

#### MHT vs JPDA — clutter crossing (cutoff 50)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| JPDA | 48.15 ± 0.35 | 24.20 |
| MHT  | 45.09 ± 0.60 | 32.60 |

This is the most interesting reveal: under the pre-fix saturated metric MHT
was tied with JPDA at the cutoff. Under the corrected metric MHT shows a real
OSPA margin (~3 m, outside seed stddev), but pays ~35% more ID switches.
**Verdict: trade-off, not a clear winner** — MHT's deferred branch resolution
yields better positional accuracy by re-binding measurements once enough
evidence accumulates, but the cost is more aggressive track ID churn.
Downstream consumers that care about identity continuity (CPA, sensor
hand-off) may still prefer JPDA; consumers that care about positional
accuracy may prefer MHT. The decision is application-dependent rather than
algorithmic.

### Cross-cutting

Three of four prior verdicts (JPDA, IMM-3, MHT) were either reversed or
materially diminished. The metric artifact is responsible for the three
retractions in the pre-fix table appearing more uniform than they should
have. With the corrected metric:

- One verdict held outright (PF — directional only).
- One was meaningfully weakened (IMM-3 — within 1σ).
- One was confirmed-retracted but for the right reason now (JPDA — GNN
  matches on OSPA and beats on ID-stability).
- One revealed a genuine accuracy-vs-stability trade-off (MHT — better OSPA
  at the cost of ID churn).

The general lesson: when evaluating a fusion stack, **the temporal alignment
between truth sampling and metric evaluation has to match** — otherwise
sensors that fire faster than truth ticks contribute cardinality-penalty
noise rather than signal. The truth-tick clock is the standard convention
in the OSPA literature and matches the cadence at which real ground-truth
(GPS) is typically available; we should not have used the per-measurement
clock in the first place.

## Heading error sweep (2026-06-02)

§14.9 wired end-to-end. Own-ship HDT now carries injected bias / drift /
white noise; `ArpaAdapter` and `EoIrAdapter` accept a `heading_std_deg`
that propagates through `projectRangeBearingToEnu` into the bearing
variance (combined in quadrature with the sensor's intrinsic σ).

Sweep: EKF + GNN, 20 seeds (201..220), σ_h ∈ {0°, 0.5°, 1°, 2°},
R-inflation off vs on. Three scenarios re-used from the bus comparison
helpers.

### ClutterCrossing (targets at ~200 m range)

```
[Bus Heading Sweep on ClutterCrossing, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 48.2740 +/- 0.2852 m | 20.40
        0.00  | on        | 48.2740 +/- 0.2852 m | 20.40
        0.50  | off       | 48.2445 +/- 0.2891 m | 20.60
        0.50  | on        | 48.1917 +/- 0.3046 m | 17.90
        1.00  | off       | 48.2469 +/- 0.3038 m | 20.60
        1.00  | on        | 48.1492 +/- 0.3313 m | 16.40
        2.00  | off       | 48.2896 +/- 0.2940 m | 19.60
        2.00  | on        | 48.1067 +/- 0.3379 m | 12.05
```

### BearingOnlyMoving (target at 1.5 km range — headline)

```
[Bus Heading Sweep on BearingOnlyMoving, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 387.5510 +/- 51.4886 m | 0.00
        0.00  | on        | 387.5510 +/- 51.4886 m | 0.00
        0.50  | off       | 419.0032 +/- 47.6722 m | 0.00
        0.50  | on        | 397.6000 +/- 51.9899 m | 0.00
        1.00  | off       | 457.0521 +/- 32.7441 m | 0.00
        1.00  | on        | 402.9367 +/- 52.3337 m | 0.00
        2.00  | off       | 482.6076 +/- 12.0074 m | 0.00
        2.00  | on        | 408.8005 +/- 47.1020 m | 0.00
```

### Maneuvering (single target, 15 s scenario)

```
[Bus Heading Sweep on Maneuvering, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 81.6523 +/- 2.0179 m | 6.15
        0.00  | on        | 81.6523 +/- 2.0179 m | 6.15
        0.50  | off       | 81.0586 +/- 2.2462 m | 6.55
        0.50  | on        | 79.5049 +/- 2.8731 m | 5.55
        1.00  | off       | 81.1884 +/- 2.4011 m | 6.60
        1.00  | on        | 77.5293 +/- 4.0335 m | 4.90
        2.00  | off       | 81.0962 +/- 2.3672 m | 6.55
        2.00  | on        | 74.5369 +/- 4.9032 m | 3.45
```

### Bias / drift propagation probe

Single-seed probe to confirm bias and drift propagate. Plan called for
1° bias / 0.01 deg/s drift; bumped to 3° / 0.03 deg/s after the smaller
magnitudes were too close to the single-seed noise floor on the
1.5 km bearing-only scenario.

```
[Bus Heading Probe: BearingOnlyMoving, seed=201]
  no error   : per-window OSPA mean = 329.4629 m
  bias 3 deg : per-window OSPA mean = 333.5720 m

[Bus Heading Probe: BearingOnlyMoving, seed=201]
  no error     : per-window OSPA mean = 329.4629 m
  drift 0.03/s : per-window OSPA mean = 329.7823 m
```

### Verdict

The BearingOnlyMoving scenario (1.5 km range) is the headline result and
shows the §14.9 failure mode sharply: with R-inflation off, per-window
OSPA climbs monotonically with σ_h (387.55 → 419.00 → 457.05 → 482.61 m),
while R-on stays nearly flat (387.55 → 397.60 → 402.94 → 408.80 m),
recovering roughly 74 m of the ~95 m saturation cliff at σ_h = 2°. This
is exactly the "tracker over-trusts long-range relative bearings when
heading is uncertain" pathology the spec predicted. Maneuvering shows a
smaller but consistent effect: OSPA R-off is flat at ~81 m across σ_h
while R-on drops to 74.5 m at σ_h = 2°, and the ID-switch signal is
cleaner still — R-off ~6.5 across the sweep, R-on falling 6.15 → 5.55 →
4.90 → 3.45 as σ_h grows, indicating R-inflation calms data-association
overreactions. ClutterCrossing (200 m range) shows the expected weak
OSPA response — differences sit below the ~0.2 m stddev noise floor —
but ID-switches still drop materially under R-on (20.40 → 17.90 → 16.40
→ 12.05), so even at short range a heading-aware R reshapes which
scan-to-track associations win. The single-seed bias/drift probe
confirms both error modes propagate end-to-end (drift contributes
~0.3 m of OSPA over a 60 s window at 0.03 deg/s, i.e. a 1.8° final
offset). The overall pattern matches the `range × σ_h` rule the spec
called out: R-inflation is essentially free at small σ_h (the increment
is dominated by intrinsic sensor noise) and progressively saves the
tracker as σ_h grows, with the dramatic gains at long range. Practical
implication: maritime trackers consuming relative bearings should
accept a `heading_std_deg` configuration and propagate it through their
projections — this work makes that path real in navtracker.

### Methodology notes

- Per-window OSPA at 1 s windows (truth-tick clock).
- Heading noise is white (per-tick i.i.d. Gaussian). No process model.
- Bias and drift held at 0 during the sweep; they get a separate probe.
- Sweep uses one canonical tracker per scenario (EKF + GNN). The
  comparison vs other estimators / associators is intentionally not
  re-run; the question here is the error model, not the algorithm.
- Determinism: each seed produces a byte-identical Scenario.
- Tests live at `tests/sim/test_bus_heading_sweep.cpp` and
  `tests/sim/test_bus_heading_bias_drift_probe.cpp`.

## Heading bias estimator (2026-06-03)

**Setup.** Re-runs the §14.9 heading sweep with a global scalar
heading-bias state that the tracker estimates from AIS-vs-ARPA position
residuals on fused tracks. ClutterCrossing's and Maneuvering's primary
target already carries AIS+ARPA+EOIR, so pair observations flow as soon
as a track is confirmed; BearingOnlyMoving has no AIS or ARPA in scene
(EOIR-only), so the estimator never publishes — that row directly tests
the graceful-fallback path. Three rows per σ_h cell: (R-off, no
estimator), (R-on, no estimator), (R-on + estimator). 20 seeds
(201..220), EKF + GNN, publish-variance threshold relaxed to (0.5°)²
so the estimator publishes within the short scenarios. SUCCEED-only
data capture.

### ClutterCrossing — 20 seeds

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 48.27 ± 0.29           | 20.40      |
| 0.0° | R-on       | 48.27 ± 0.29           | 20.40      |
| 0.0° | R-on + est | 47.88 ± 0.48           | **17.85**  |
| 0.5° | R-off      | 48.24 ± 0.29           | 20.60      |
| 0.5° | R-on       | 48.19 ± 0.30           | 17.90      |
| 0.5° | R-on + est | 47.76 ± 0.56           | **14.90**  |
| 1.0° | R-off      | 48.25 ± 0.30           | 20.60      |
| 1.0° | R-on       | 48.15 ± 0.33           | 16.40      |
| 1.0° | R-on + est | 47.65 ± 0.61           | **10.40**  |
| 2.0° | R-off      | 48.29 ± 0.29           | 19.60      |
| 2.0° | R-on       | 48.11 ± 0.34           | 12.05      |
| 2.0° | R-on + est | **47.57 ± 0.62**       | **7.65**   |

### BearingOnlyMoving — 20 seeds (no AIS / no ARPA in scene)

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 387.55 ± 51.49         | 0.00       |
| 0.0° | R-on       | 387.55 ± 51.49         | 0.00       |
| 0.0° | R-on + est | 387.55 ± 51.49         | 0.00       |
| 0.5° | R-off      | 419.00 ± 47.67         | 0.00       |
| 0.5° | R-on       | 397.60 ± 51.99         | 0.00       |
| 0.5° | R-on + est | 397.60 ± 51.99         | 0.00       |
| 1.0° | R-off      | 457.05 ± 32.74         | 0.00       |
| 1.0° | R-on       | 402.94 ± 52.33         | 0.00       |
| 1.0° | R-on + est | 402.94 ± 52.33         | 0.00       |
| 2.0° | R-off      | 482.61 ± 12.01         | 0.00       |
| 2.0° | R-on       | 408.80 ± 47.10         | 0.00       |
| 2.0° | R-on + est | 408.80 ± 47.10         | 0.00       |

R-on+est is byte-identical to R-on across the cell: no AIS+ARPA pairs
get extracted, so the estimator's variance never falls below the
publish threshold and gating stays closed. This is the designed
behavior, not a bug — the R-inflation budget continues to do all the
work in non-cooperative scenes.

### Maneuvering — 20 seeds

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 81.65 ± 2.02           | 6.15       |
| 0.0° | R-on       | 81.65 ± 2.02           | 6.15       |
| 0.0° | R-on + est | 81.66 ± 1.95           | 6.25       |
| 0.5° | R-off      | 81.06 ± 2.25           | 6.55       |
| 0.5° | R-on       | 79.50 ± 2.87           | 5.55       |
| 0.5° | R-on + est | 79.37 ± 2.90           | 5.30       |
| 1.0° | R-off      | 81.19 ± 2.40           | 6.60       |
| 1.0° | R-on       | 77.53 ± 4.03           | 4.90       |
| 1.0° | R-on + est | 77.54 ± 4.03           | 4.90       |
| 2.0° | R-off      | 81.10 ± 2.37           | 6.55       |
| 2.0° | R-on       | 74.54 ± 4.90           | 3.45       |
| 2.0° | R-on + est | 74.51 ± 4.92           | 3.60       |

### Anchor-loss scenario (single seed 401, 120 s)

ClutterCrossing-style scene with σ_h = 2°, R-inflation on, estimator
on. AIS broadcasts on target 1 for [0, 60) s and drops out at t = 60 s.

- `is_published` at t ∈ [30, 60): **true** (estimator converged on
  AIS+ARPA pairs; final variance ≈ (0.29°)²).
- `is_published` at t = 90 s (30 s after dropout): **false** (stale
  window closed; adapters revert to b̂ = 0 with R-inflation only).
- Pre-dropout mean per-window OSPA on [40, 60): 14.17 m.
- Post-dropout mean per-window OSPA on [60, 120): 30.59 m. Growth is
  dominated by target 1's increasing range (cross-track error scales
  with range) and is structurally unrelated to the dropout; the
  bounded-fallback assertion confirms OSPA stays well below the
  cutoff with no divergence.

### Verdict

The bias estimator delivers a clean, measurable ID-stability win in
the AIS-cooperative scene and reverts cleanly when the AIS anchor
disappears. The largest effect is on ClutterCrossing's `id_sw_mean`:
at σ_h = 2°, R-inflation already cut switches 19.6 → 12.05; the
estimator drops them further to **7.65** — a 60% total reduction vs
the no-mitigation baseline. OSPA improvement on the same cell is
smaller (48.11 → 47.57 m) because ClutterCrossing's targets sit at
~200 m where the `range × σ_h` penalty is modest — the ID benefit
comes from sharper, less-uncertain bearings making data-association
decisions more confident under clutter. Maneuvering and
BearingOnlyMoving see no closed-loop OSPA change: Maneuvering's 15 s
duration leaves the estimator barely above the publish threshold, and
BearingOnlyMoving has no AIS in scene so the estimator stays
unpublished by design. Anchor-loss confirms the gating contract — the
30 s stale window closes cleanly, behavior falls back to the §14.9
R-inflation path, and there is no accuracy cliff at the dropout
moment. Practical implication: AIS-vs-ARPA bias estimation is most
valuable in cluttered cooperative scenes where ID stability matters
most; the deferred multi-track bearing-innovation observer (spec
§11 #1) remains the right next step for non-cooperative scenes like
BearingOnlyMoving.

### Methodology notes

- Three sweep TESTs and one anchor-loss TEST: `tests/sim/test_bus_bias_estimator_sweep.cpp`, `tests/sim/test_bus_anchor_loss.cpp`.
- Bus driven via `sim::SimulatedSensorBus::stepOnce(...)` for the estimator-on rows so adapter projections see the latest published b̂ on the cycle after each AIS+ARPA pair is observed.
- Publish threshold (0.5°)² for the sweep; default (0.3°)² used elsewhere.
- AIS dropout in the anchor-loss test uses `sim::AisEmitterConfig::dropout_windows_s`.
- Default `AisArpaPairExtractorConfig` (cycle window 0.5 s, AIS σ fallback 10 m, ARPA bearing σ fallback 1°).
- The estimator is intentionally bias-agnostic during sim warmup — initial state b̂ = 0, variance (5°)². No precomputed calibration.
