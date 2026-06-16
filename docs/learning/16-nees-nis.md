# 16 — Filter consistency: NEES and NIS

> Prerequisites: [04 — Kalman filter](04-kalman-filter.md),
> [11 — Gating](11-gating-gnn-hungarian.md),
> [13 — Clutter and detection](13-clutter-and-detection.md).
> Next: [17 — Multi-sensor + bias](17-multi-sensor-and-bias.md).

Every filter we have discussed makes a self-statement at each
step: *"this is the mean, this is the covariance, this is how
confident I am"*. We have no way to know if those statements are
honest until we **compare them against ground truth**.

Two simple, classical metrics do this. They are the **NEES**
(Normalised Estimation Error Squared) and the **NIS**
(Normalised Innovation Squared). Both are dimensionless
chi-squared statistics. Both are essential for tuning `Q` and
`R`. Both are the first thing to check when a tracker
"behaves weirdly".

## 1. NIS — am I lying about my innovation covariance?

NIS lives on the *measurement side*. It does not need ground
truth. After every update:

```
ν  = z − h(x̂⁻)              (innovation, bearing-wrapped)
S  = H P⁻ Hᵀ + R              (predicted innovation covariance)
ε_NIS = νᵀ S⁻¹ ν              (NIS, dimensionless)
```

Under the Gaussian-noise hypothesis, `ε_NIS` is **chi-squared
distributed** with degrees of freedom equal to the measurement
dimension `m`. Its expected value is `m`.

### What NIS tells you

Average NIS over many updates:

- `ε̄_NIS ≈ m`: filter is consistent. Predicted `S` matches the
  actual surprise.
- `ε̄_NIS > m`: filter is **overconfident**. Real innovations are
  bigger than `S` says — either `R` is too small, `Q` is too
  small, or the motion model is wrong.
- `ε̄_NIS < m`: filter is **underconfident**. Real innovations
  are smaller than `S` expects — your `R` is too big.

### Picture

![Three NIS regimes: consistent, overconfident, underconfident](figures/16-nis-regimes.png)

Left: the sample mean of `ε_NIS` sits near `m = 2` — the filter
is consistent. Centre: the mean is well above `m`, the filter is
overconfident (`R` and/or `Q` too small). Right: the mean is well
below `m`, the filter is underconfident (`R` too big).

Now the same picture as a time series with the 95 % confidence
band for the running mean:

![NIS time series with running mean and 95 % band](figures/16-nis-timeseries.png)

Individual `ε_NIS` values bounce around — a single big sample is
fine (one outlier visible mid-run). The running mean is what
matters, and it stays inside the band around `m = 2`. A drift
upward over minutes would tell you `R` is creeping wrong.

### NIS is per (sensor, measurement-model, source) bucket

The codebase aggregates NIS per source key
`k = (SensorKind, MeasurementModel, source_id)`. Different
sensors have different `R`s and the consistency story is
per-sensor.

### The R-calibration loop

If NIS for ARPA at source X is `2× m`, you have evidence that
ARPA's published `R` is too small. The corrective `R` scaling
is roughly `R' = (NIS / m) · R`. Iterate until NIS settles near
`m`.

This is what `docs/algorithms/consistency.md` and
`docs/superpowers/specs/2026-06-13-nees-r-calibration-design.md`
describe in detail. The `IInnovationSink` port carries
innovations to the bench-side aggregator.

## 2. NEES — am I lying about my state covariance?

NEES lives on the *state side*. It needs ground truth `x_true`:

```
e = x_true − x̂
ε_NEES = eᵀ P⁻¹ e
```

Under the Gaussian hypothesis, `ε_NEES` is **chi-squared
distributed** with degrees of freedom equal to the state
dimension `n`. Expected value is `n`.

Average NEES over many updates:

- `ε̄_NEES ≈ n`: filter is consistent.
- `ε̄_NEES > n`: filter is **state-overconfident**. Real error
  is bigger than `P` says. Could be biased state, badly-tuned
  `Q`, or model mismatch.
- `ε̄_NEES < n`: filter is **state-underconfident**.

In production we cannot compute NEES directly — there is no
ground truth. We use NEES on **scenario tests** where synthetic
truth is known.

### Picture

The same shape as NIS but with `n` (state dim) instead of `m`.

## 3. The combined view

Both must look right. Two common failure modes:

```
  NIS high + NEES high   → Q too small. Filter is too stiff;
                           predict step does not allow
                           enough motion uncertainty.
  NIS high + NEES low    → R too small. Filter trusts the
                           sensor too much; updates are too
                           snappy.
  NIS low  + NEES high   → R too big. Filter ignores good
                           sensors; state drifts.
  NIS low  + NEES low    → Filter is too cautious overall.
                           Inflate confidence (rarely the bug).
```

A good filter tune lands both near their expected values, with
narrow spread. Drift across the run (e.g. NEES rises over
minutes) suggests a structural model problem rather than a
covariance tune.

## 4. The chi-squared confidence bounds

For a sample size `N` and DOF `m`, the 95 %-confidence interval
for the *average* NIS is roughly

```
[ m · (1 − 2·√(2m)/√N) , m · (1 + 2·√(2m)/√N) ]
```

(narrows as `N` grows). For `N ≈ 200` and `m = 2`, the
acceptable mean-NIS band is `[1.6, 2.4]`. Anything outside is
statistically meaningful.

The codebase computes per-bucket confidence intervals and flags
"out of band" buckets in the consistency report.

## 5. Where NEES/NIS go wrong (and don't conclude too fast)

A *single* high NIS does not prove anything. Bigger-than-expected
innovations happen for legitimate reasons:

- The vessel turned right between predict and update.
- A clutter measurement squeezed past the gate.
- An IMM mode mismatch (predict was on CV, target was in CT).

Always look at:

- The **average over many samples** (Welford-style running
  mean).
- The **spread** (variance of NIS samples).
- The **time series** (does it drift?).

The aggregator in `core/benchmark/Consistency.{hpp,cpp}` does
all three.

## 6. When the model is the problem

NEES/NIS are diagnostics for **covariance** mistuning. If the
motion model is *fundamentally* wrong (e.g. CV used for a
constantly-turning vessel), the covariance tune alone cannot
fix it: even with the right `Q`, the predicted mean is off, so
the predicted innovation is biased, so NIS is biased even with
perfectly-tuned `R`.

In that case the fix is **structural** — add a CT mode,
adopt IMM, widen the motion family. NEES/NIS told you *that*
the model is wrong; you have to diagnose *what* is wrong by
inspecting failure trajectories.

## 7. When NEES is wildly off — what actually breaks downstream

Sections 1-6 explained *what* NEES and NIS measure. This section is
about what happens to a *real* tracker when they are deeply broken —
specifically, the AutoFerry scenario 5 case where NEES is ~77 instead
of ~2. The mathematical statement is "the filter underestimates its
own uncertainty by 40×." The operational consequence is much more
dramatic.

### 7.1 The filter is lying about how confident it is

A consistent filter reports an *honest* covariance: when it says
"1-sigma error is 5 m", reality is roughly 5 m. An overconfident
filter says "5 m" while the real error is 40 m. It thinks it's
nailing the target; in practice it has no idea where the target is.

This isn't a quiet bookkeeping problem. The filter uses its own
covariance *to make decisions* — gating, association, track birth.
When the covariance is wrong, every one of those decisions is wrong.

### 7.2 The conveyor belt of duplicate tracks

Imagine a sc5 track that has been carried for several seconds on
nothing but EO/IR bearings (16 Hz). The filter's reported (east,
north) covariance has shrunk to ~1 m² — it claims to know the
target's position to a metre. In reality, bearings only constrain
the *direction* to the target. Range is essentially unknown. The
filter's stated position uncertainty is wildly optimistic.

Then a radar return comes in 0.6 Hz later, at the *actual* target
location. The radar return tries to enter the existing track's gate:

```
                gate  ┌─────────┐
                      │  track  │     ← gate centred on wrong (range)
                      │   ●     │       position, claimed 1-sigma 1 m
                      │         │
                      └─────────┘
                                       
                                  ●  ← actual radar return, 20 m away,
                                       OUTSIDE the gate
```

The gate is the small box because the filter thinks the track is
known to 1-sigma 1 m. The radar return is the dot 20 m away —
the real position. The math says "this measurement could not have
come from this track" (because the gate width × the sigma says so).
**The return is rejected.**

In the next step, an unassociated radar return births a *new* track.
That new track sits next to the existing track — now there are two
tracks for one truth. The new track grabs the next few radar returns;
the old one starves. After a few seconds, identity switches between
them.

Repeat. 91 ID switches on sc5 over 139 seconds for 2 targets.

It looks like an association bug. It is actually an *uncertainty*
bug. Fix the covariance and the association becomes trivial.

### 7.3 The three suspects

NEES says "the filter is overconfident" but not *why*. Three things
can produce the symptom:

#### (a) `R` is too small — the sensor noise is misreported

`R` is the parameter "how noisy is this sensor?". If the configured
`R` is smaller than reality, the filter eats every measurement as
if it were laser-accurate. The posterior shrinks far below what the
data actually justifies. NEES grows because the *real* error is
bigger than the filter's claimed sigma.

This is the easiest to test. Take a chunk of data where ground truth
is available. Compute `z − truth` per measurement, take the variance,
compare to the configured `R`. If `var(z − truth) > R(0,0)`, `R` is
too small by exactly that ratio.

#### (b) Bearing-only tracking (BOT) pathology — fake range certainty

When a sensor reports only a *bearing* (no range), the math has a
known failure mode. Each bearing tightens the angular constraint on
where the target is. Range stays unobserved by that measurement, so
*range certainty must not change*. The textbook is clear about this.

The EKF approximation can break the rule. The linearisation around
the current state computes a Jacobian `H` that points along the
bearing direction. If the EKF's update step has any numerical leakage
between the angle and range components — and it does — the range
covariance starts shrinking with every bearing. After 50 bearings at
16 Hz, the filter thinks it knows the range to a metre. It doesn't.
It only knows the angle.

Then a real position measurement at the actual range arrives and is
rejected because the gate is tight around a wrong range. Section 7.2
plays out.

The fix is a guard: **the range-direction component of the
covariance must never shrink during a Bearing2D update**. Compute
the range-direction unit vector `r̂` from the sensor to the predicted
target position. The variance in that direction is `r̂ᵀ P r̂`. After
the bearing update, require that this number is not smaller than it
was before. If it is, replace the post-update `P` with one that
preserves the pre-update range variance.

#### (c) `Q` (process noise) is too small — the filter expects the target to be more predictable than it is

`Q` is "how much randomness do I expect in the target's motion
between measurements". Synthetic scenarios use small `Q` because
their targets move smoothly. Real harbour targets manoeuvre —
slowing, turning, drifting in current. A small `Q` makes the filter
say "I expect this target to continue exactly as before"; the next
real measurement deviates from that prediction, so the innovation is
big, so NIS spikes. Average NEES drifts up because predicted state
covariance was tiny.

The fix is empirical: re-tune `Q` against observed manoeuvres on
real data, per motion-model mode in the IMM. Larger `Q` widens
gates and makes the filter humbler about its predictions.

### 7.4 Why this is upstream of almost everything

If the filter lies about its own uncertainty:

- **Gating** is wrong → real measurements get rejected → duplicates.
- **Association** is wrong → JPDA/MHT weights are biased → wrong
  hypothesis wins.
- **Track birth** fires too often → false tracks proliferate.
- **Track death** fires too rarely → stale tracks linger.
- **Per-sensor detection models** built from observed gated counts
  (`AdaptiveSensorDetectionModel`) see biased counts → wrong P_D, λ_C.
- **Bias estimators** that use the filter's covariance for outlier
  gates (item 9) see biased gates → wrong observation acceptance.
- **OSPA / GOSPA** numbers are dominated by cardinality errors caused
  by the duplicate-birth cycle.

This is why item 12 sits highest in the improvement backlog: the
operational symptoms in items 11 and 9 (and the cardinality gap to
Helgesen 2022 on env 1) are *downstream* of a filter that doesn't
tell the truth about its own confidence. Fix the truth-telling and
several other things get better at once.

### 7.5 The fix-it order

In test order:

1. **(a) R calibration first.** It is mechanical data work, not
   filter math — no design choices, no hypotheses to test. Once `R`
   is honest, NEES movement attributable to (a) is gone, and the
   remaining 77 → X reduction tells us how much of the symptom (a)
   was responsible for. Without this baseline, (b) and (c) are
   unfalsifiable.

2. **(b) BOT range-variance guard second.** Clean mathematical fix,
   surgical scope (one new branch in the Bearing2D update path),
   measurable effect (the per-track covariance no longer pretends to
   know range from bearings alone). Lands on top of honest `R` so
   the remaining NEES residual tells us how much was BOT and how
   much is (c).

3. **(c) `Q` retune last.** Subjective, never converges to one right
   answer, depends on clean `R` and a non-cheating filter. Tune on
   measured manoeuvre statistics; the IMM's CT mode in particular
   needs more `Q` for harbour-scale turns.

User-friendly version: **calibrate the sensors first, fix the math
bug second, tune the model third.**

## 8. Assumptions

| Assumption                                       | When it pinches                                  |
|--------------------------------------------------|--------------------------------------------------|
| Gaussian innovations                             | Heavy-tailed sensors bias NIS                    |
| `R` known per sensor                             | Wrong `R` is *what* NIS is detecting             |
| Ground truth available for NEES                  | Scenario tests only                              |
| Adequate sample size                             | Use confidence bands; don't over-interpret few   |
| Source-keyed buckets are stable                  | Re-keyed buckets reset the running average       |

## 9. Why we can use this here

NEES/NIS are universal Bayesian-filter diagnostics. They apply
to *any* of the filters in chapters 04–09. The codebase's
`IInnovationSink` design means the same port emits innovations
from EKF/UKF/IMM/MHT, and the same bench-side aggregator scores
all of them.

For us specifically:

- NIS catches per-sensor mis-calibration (ARPA `R` wrong, EO/IR
  `R` wrong, etc.).
- NEES on synthetic-truth scenarios catches structural model
  problems (CV used when CT is needed, etc.).
- The two together give us a tight feedback loop on tuning.

## 10. Where this lives in code

- `core/benchmark/Consistency.{hpp,cpp}` — running aggregator.
- `ports/IInnovationSink.hpp` — port for per-update innovations.
- `core/pipeline/Tracker.cpp` / `MhtTracker.cpp` — emit to the
  sink after each successful update.
- `docs/algorithms/consistency.md` — math, source keys, exact
  formulae.
- `docs/superpowers/specs/2026-06-13-nees-r-calibration-design.md`
  — the R-calibration plan.

## 11. What we did not pick, and why

- **Normalised Estimation Error Squared in NED** instead of in
  the filter's frame — different normalisation, no theoretical
  advantage. We use the filter's own `P` so the math closes.
- **Adaptive `R` tuning at runtime** — tempting, but dangerous
  feedback loop. Offline NEES/NIS + manual tune is safer.
- **Use NEES with the *posterior* `P` instead of the prior `P⁻`**
  — both have variants; the posterior version is more common
  in textbook coverage. We use the predicted moment-matched
  state at measurement time so the diagnostic catches the
  predict-side mis-tune separately from the update-side.

---

Previous: [15 — Track lifecycle](15-track-lifecycle.md)
Next: [17 — Multi-sensor + bias](17-multi-sensor-and-bias.md) →
