# PMBM — Algorithm Design Reference

Follows the project documentation standard:
Math / Assumptions / Rationale / Ways to improve.

Companion documents:

- High-level literature entry: [sota-roadmap.md §4](sota-roadmap.md).
- Phased engineering plan: [`docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md`](../superpowers/plans/2026-06-07-pmbm-integration-plan.md).
- Vertical-slice positioning across stacks: [learning §22](../learning/22-tracker-stack-alternatives.md).
- Plain-language introduction: [learning §23](../learning/23-pmbm.md).

This document is the *equation-level reference*; the plan is *how we build it*;
the learning chapter is *how a non-expert reads it*.

---

## 0. Where PMBM sits in the pipeline

Mapping to the §22 vertical slices:

| Slice | Today (Cl-2, post-2026-06-20) | Cl-3 PMBM |
|---|---|---|
| 1. Sensor R / clutter density λ_C / P_D | per-sensor σ + Schmidt bias correction | unchanged |
| 2. Motion model | IMM (CV5 + CT) | IMM (CV5 + CT), one IMM per Bernoulli |
| 3. Inner filter | UKF per IMM mode | UKF per IMM mode (unchanged inside Bernoulli) |
| **4. Gating** | Mahalanobis χ² | folded into RFS update (likelihood ratio + Poisson birth weight) |
| **5. Association** | TOMHT (`MhtTracker` + Murty K-best) | Multi-Bernoulli mixture (Murty K-best per global hypothesis) |
| **6. Lifecycle** | IPDA persistence + VIMM visibility + M-of-N | Bernoulli existence `r` + Poisson birth `λ^u` (continuous, no thresholds) |
| 7. Cross-sensor fusion / bias | `SensorBiasEstimator` Schmidt-KF fold | unchanged, composes in front of Bernoulli update |

Net: PMBM **replaces slices 4-6 as a block**. IMM + UKF live inside each
Bernoulli's spatial density. Bias correction is unchanged and operates on
measurements before they reach the PMBM update.

This is why the natural canonical config will be named `imm_cv_ct_pmbm`,
sibling to `imm_cv_ct_mht`.

---

## 1. State representation

**The full multi-target posterior is a Random Finite Set (RFS) density**
`f_{k|k}(X)` over the set `X` of currently existing target states. PMBM
factorises that density as the convolution of a Poisson density (undetected
targets) and a Multi-Bernoulli mixture (detected targets).

### 1.1 Poisson Point Process (PPP) — undetected targets

```
λ^u_k(x) = Σ_i  w^u_{k,i} · 𝒩(x; m^u_{k,i}, P^u_{k,i})
```

A Gaussian-mixture intensity over single-target state `x`. Carries the
probabilistic location of every target that *might* exist but has never been
detected. Birth intensity is added each predict step; missed-but-not-yet-
detected mass flows from Bernoulli → PPP via `(1 − P_D)` factors;
detected-for-the-first-time mass flows from PPP → new Bernoulli.

`w^u` is an unnormalised intensity, not a probability — units are
"expected number of targets per state-space volume".

### 1.2 Multi-Bernoulli (MB) — one global hypothesis over detected targets

A single multi-Bernoulli set of `n_j` Bernoulli components

```
{ b^{j,1}, …, b^{j,n_j} },
b^{j,i} = (r^{j,i}, f^{j,i}(x))
```

where `r^{j,i} ∈ [0,1]` is the existence probability of the i-th target in
the j-th hypothesis and `f^{j,i}(x)` is its single-target density (Gaussian,
or — once IMM is added — a Gaussian mixture indexed by motion mode).

Each Bernoulli has a stable internal identifier (the seed measurement that
spawned it from the PPP). Different global hypotheses may share that
identifier, allowing posterior aggregation per target across hypotheses.

### 1.3 Multi-Bernoulli Mixture (MBM) — uncertainty over hypotheses

```
f^d_k(X) = Σ_j  w^j_k · f^j_k(X)
Σ_j w^j_k = 1
```

A weighted mixture over global hypotheses. Each `j` is a complete
"who-saw-what" interpretation of the measurement history. Weight `w^j_k`
is the Bayesian posterior probability of that interpretation.

### 1.4 Full posterior

```
f_{k|k}(X) = (PPP with intensity λ^u_k) ⊛ (MBM Σ_j w^j_k f^j_k)
```

where `⊛` is the RFS convolution. In implementation: maintain the PPP and the
MBM separately, combine only at output time.

---

## 2. Math — predict step

Given `f_{k−1|k−1} = (λ^u_{k−1}, {w^j_{k−1}, b^{j,·}_{k−1}})`:

### 2.1 PPP predict

```
λ^u_{k|k−1}(x) = ∫ p_S(x') · π(x | x') · λ^u_{k−1}(x') dx'
                 +  λ^b_k(x)
```

- `p_S(x')` — survival probability (constant per scan, typ. 0.99).
- `π(x | x')` — motion model transition density. For IMM, this is the
  mode-mixed posterior.
- `λ^b_k(x)` — birth intensity. Per-scan additive. Configurable per sensor
  region (radar coverage edge, AIS reception zone, off-shipping-lane area).

For Gaussian-mixture PPP under linear/EKF motion this collapses to a
weight-scaled, propagated Gaussian per component.

### 2.2 MBM predict

For each global hypothesis `j`, for each Bernoulli `b^{j,i}`:

```
r^{j,i}_{k|k−1}  = p_S · r^{j,i}_{k−1|k−1}
f^{j,i}_{k|k−1}  = ∫ π(x | x') f^{j,i}_{k−1|k−1}(x') dx'
w^j_{k|k−1}      = w^j_{k−1|k−1}            (mixture weights unchanged)
```

`f^{j,i}` predict is delegated to the per-Bernoulli `IEstimator::predict`
(`ImmEstimator` with UKF inner filter). Bias-correction state is held
elsewhere (`SensorBiasEstimator`) and applies to *measurements* before the
update — predict is unaffected.

---

## 3. Math — update step

Given a scan with measurements `z_1, …, z_m` at time `k`:

### 3.1 Per-Bernoulli single-target updates

For each Bernoulli `b^{j,i}` and each measurement `z_m`:

```
ℓ^{j,i}_m       = ∫ g(z_m | x) · f^{j,i}_{k|k−1}(x) dx
f^{j,i}_{m}(x)  = g(z_m | x) · f^{j,i}_{k|k−1}(x) / ℓ^{j,i}_m
r^{j,i}_{m}     = 1                                        (existence given detection)
```

`g(z|x)` is the measurement likelihood — for IMM-UKF, evaluated as the
mode-weighted sum of per-mode innovation Gaussians (this is the
`IEstimator::logLikelihood` API). Bias-corrected `z_m` and bias-inflated
`R` are what reach `g`.

For the *misdetection* of Bernoulli `b^{j,i}`:

```
r^{j,i}_{miss}   = (1 − p_D) · r^{j,i}_{k|k−1}  /  (1 − r^{j,i}_{k|k−1} + (1 − p_D) · r^{j,i}_{k|k−1})
f^{j,i}_{miss}   = f^{j,i}_{k|k−1}                                          (no measurement update)
```

`p_D` here is the *effective* detection probability for Bernoulli `i` given
the current scan — see below.

#### 3.1.1 Effective miss-P_D and per-scan dedup (`dedup_miss_pd`)

The standard formula assumes one measurement per target per sensor per scan.
When a scan contains `N` returns from the same radar sweep, the naive loop
multiplies the miss contribution `N` times:

```
survive_legacy = ∏_{l=1}^{N} (1 − p_D)  =  (1 − p_D)^N   [WRONG for a single sweep]
```

This drives effective `p_D → 1` regardless of the configured value (N=50
radar returns → effective `p_D ≈ 1 − 0.9^50 ≈ 1.0`). The tracker was
relying on this over-penalty as a cardinality brake.

**Textbook fix** (`dedup_miss_pd = true`): dedup by distinct `(sensor, model,
source_id)` channel — one detection opportunity per sensor per scan:

```
effective p_D = 1 − ∏_{c ∈ unique channels in scan} (1 − p_D^c · coverage^c)
```

For a single radar sweep with N returns: effective `p_D = p_D^radar` (one
opportunity). For a simultaneous AIS+radar scan: two independent opportunities
(one per sensor type). This is the correct textbook model and makes the
detect-branch `p_D` and the miss-branch `p_D` consistent.

**Off by default** (bit-identical to Phase 8/9 baseline). Config flag:
`dedup_miss_pd = false` → legacy per-measurement product.

#### 3.1.2 Source-aware misdetection gate (`source_aware_misdetection`)

AIS broadcasts are per-vessel: vessel A's broadcast contains no information
about vessel B's existence. With `source_aware_misdetection = true`, a
Bernoulli skips the misdetection recursion when none of its
contributing `source_id`s appear in the current scan. Fresh Bernoullis with
no contribution history are not protected (they decay normally).

#### 3.1.3 Per-vessel identity for the AIS gate (`source_aware_identity`)

All AIS vessels share `source_id = "ais"`. With only channel-level gating
(`source_aware_identity = false`), vessel A's broadcast protects vessel B
from misdetection — wrong. With `source_aware_identity = true`, a
`SourceTouch` entry with a `vessel_id` (from `Measurement::hints.mmsi`) uses
the vessel-level key instead of the channel key:

```
should_misdetect(id) =
  ∀ touch ∈ contribution_history[id]:
    if touch.vessel_id set → touch.vessel_id ∈ scan_vessel_ids
    else                   → touch.source_id ∈ scan_source_ids   (channel fallback)
```

**Off by default** — bit-identical to `source_aware_misdetection` alone.

Note: `source_id = "ais"` is intentionally left unchanged. MHT's miss-dedup
(`TrackTree.cpp`) collapses all `"ais"` to one detection opportunity by
channel; per-vessel `source_id` would silently break MHT. Per-vessel
separation goes through `SourceTouch::vessel_id` only.

### 3.2 New-target Bernoulli from PPP

For each measurement `z_m`:

```
ρ_m        = ∫ p_D · g(z_m | x) · λ^u_{k|k−1}(x) dx
           +  λ^FA(z_m)                                    (clutter intensity)
r^new_m    = ρ_m_target / (ρ_m_target + λ^FA(z_m))
f^new_m(x) = p_D · g(z_m | x) · λ^u_{k|k−1}(x) / ρ_m_target
```

where `ρ_m_target` is the first integral alone. Gaussian-mixture math gives
this in closed form.

#### 3.2.1 Adaptive Birth (Reuter 2014) — `adaptive_birth = true`

Standard measurement-driven PPP birth contaminates `ρ_m_target` because
every clutter return injects a fresh `PoissonComponent` centred on itself
(see `measurement_driven_birth`). The contaminated value makes `r_new ≈ 1`
for every measurement, including clutter. Reuter 2014 decouples the spatial
density from the existence prior:

```
r^new_m   = λ_birth / (λ_birth + λ^FA(z_m))
f^new_m(x) = estimator.initiate(z_m)         (Gaussian centred at z_m)
```

`λ_birth` is the expected new-target birth rate per scan per
measurement-space volume unit (same units as `λ^FA`). The PPP injection is
skipped; the spatial mean comes from `IEstimator::initiate` directly.

#### 3.2.2 Clutter-invariant birth existence (`birth_existence_target > 0`)

The problem with a fixed absolute `λ_birth`: it was tuned when `λ^FA ≈ 1e-4`.
On a different scenario/sensor where `λ^FA` is very different, `r_new` drifts
far from the intended design point:

| Scenario / sensor  | λ^FA     | r_new (λ_birth=1e-5) |
|--------------------|----------|----------------------|
| autoferry radar    | 1e-4     | 0.091 (≈ intended)   |
| philos radar       | 2.7e-6   | 0.79 (over-confident)|
| philos AIS         | 1e-9     | ≈ 1.0 (certain!)     |

The fix: set `birth_existence_target = r*` and derive `λ_birth` per
measurement from the live `λ^FA(z_m)`:

```
λ_birth  = (r* / (1 − r*)) · λ^FA(z_m)
r_new    = λ_birth / (λ_birth + λ^FA(z_m))
         = (r*·λ^FA) / (r*·λ^FA + (1−r*)·λ^FA)  [substituting]
         = r*                                      (independent of λ^FA)
```

This is **clutter-invariant**: autoferry `λ^FA=1e-4`, philos radar
`λ^FA=2.7e-6`, and philos AIS `λ^FA=1e-9` all yield `r_new = r*` without
per-scenario or per-sensor retuning of `λ_birth`.

Config knobs:
- `birth_existence_target = 0.0` — legacy (use absolute `lambda_birth`).
  Bit-identical to pre-Task-1 behaviour.
- `birth_existence_target = 0.1` — bench probe value (Task 1, 2026-06-24):
  philos `gospa_mean` dropped from 82.63 to 48.50 (−41 %).

### 3.3 PPP update — undetected mass loss

After the scan, undetected mass decays:

```
λ^u_k(x) = (1 − p_D) · λ^u_{k|k−1}(x)
```

(`p_D` may be position-dependent — sensor coverage. Constant simplifies the
prototype.)

### 3.4 Global hypothesis enumeration

This is the combinatorial step. For each *prior* global hypothesis `j` with
`n_j` Bernoullis and `m` new measurements, build the assignment cost matrix
`C^j ∈ ℝ^{(n_j + m) × m}`:

```
C^j[i, l]            = − log( ℓ^{j,i}_l · r^{j,i}_{k|k−1} )      for i = 1..n_j   (Bernoulli i ↔ z_l)
C^j[n_j + l', l]     = − log( ρ_l · 𝟙[l = l'] )                  for l' = 1..m    (z_l is new target / clutter)
```

A column-perfect assignment of `C^j` is a *child global hypothesis* `j'`.
Its weight is

```
w^{j'} ∝ w^j · exp(−cost(assignment))
```

normalised across all children of all prior `j`. Exhaustive enumeration is
`O((n_j + m)! / (n_j!))` per prior; we truncate with **Murty K-best**
(Phase 0, already shipped in `core/association/Murty.{hpp,cpp}`), keeping
the K cheapest assignments per prior. The same shared truncator services
the MHT global-non-conflict step.

### 3.5 Mixture pruning + Bernoulli merging

Three pruning operations after enumeration:

1. **Hypothesis weight pruning.** Drop global hypotheses with
   `w^{j'} < w_min` (e.g. 1e-4); cap total hypothesis count at `K_max`.
2. **Bernoulli existence pruning.** Within a hypothesis, drop Bernoullis
   with `r^{j,i} < r_min` (e.g. 1e-3).
3. **Bernoulli merging.** Across hypotheses sharing the same seed
   identifier, merge similar Bernoullis by Bhattacharyya distance — same
   primitive as `mergeBranches` used by MHT, generalised from
   "within tree" to "across hypotheses for the same target".

PPP components below `w_min` are also dropped; remaining components may be
merged by moment-matching when close in mean and covariance.

### 3.6 Output extraction

For each unique Bernoulli identifier `i`, aggregate across hypotheses:

```
P(target i exists) = Σ_{j: i ∈ j}  w^j · r^{j,i}
mean(target i)     = Σ_{j: i ∈ j}  (w^j · r^{j,i} / P(exists)) · μ^{j,i}
cov(target i)      = moment-matched covariance about that mean
```

Emit as a `TrackOutput` with `status = Confirmed` iff
`P(exists) ≥ confirm_threshold` (default ~0.5), else `Tentative`. The
aggregated `P(exists)` is *strictly more information* than today's M-of-N
state and replaces it on the output.

---

## 4. Assumptions

The conjugacy result relies on the **standard point-target model**:

1. **One-measurement-per-target-per-scan.** Each target generates at most one
   measurement per sensor scan; multi-detection (extended targets) is not in
   the prototype.
2. **Conditional independence** of measurement generation given the
   association.
3. **Poisson birth** with intensity `λ^b(x)` per scan. Tunable per sensor
   region. For the first cut, uniform over the local ENU patch.
   With `birth_existence_target > 0` (§3.2.2), `λ^b` is derived per
   measurement from `λ^FA` so `r_new` is scenario-invariant; this assumption
   changes: the birth intensity is now a *function of* clutter density, not
   an independent parameter.
4. **Poisson clutter** with intensity `λ^FA(z)` per measurement space.
   Constant per sensor matches our current per-sensor `clutter_density`.
5. **Bernoulli survival** with constant `p_S` per scan. Sensor-conditional
   later if needed (ships entering/exiting radar coverage).
6. **Linear / EKF / UKF tractability of single-target densities.** PMBM is
   conjugate under any single-target filter that gives `predict` and
   `update`; our `IEstimator` already covers EKF, UKF, IMM.

When these are violated — extended targets, AIS bursts as multi-detection,
multipath — the recursion is approximate. The trajectory-PMBM variant
relaxes (1) and (2) for trajectory association but keeps the point-target
generation model.

**Additional assumptions for `source_aware_identity = true`:**
- `Measurement::hints.mmsi` is populated for all AIS measurements. If MMSI
  is absent on a measurement, the gate falls back to channel-level keying.
- The `SourceTouch::vessel_id` carried in `contribution_history_` is set at
  detection time from the matched measurement's `hints.mmsi`. Bernoullis born
  before this flag was enabled carry no `vessel_id` and use the channel
  fallback (benign degradation).

**Additional assumption for `dedup_miss_pd = true`:**
- Returns from the same radar sweep share identical `(sensor, model,
  source_id)`. If two physically separate sensors happen to share the same
  triple (misconfigured source_id), they will be deduplicated as one
  opportunity — an operator configuration error, not a filter bug.

---

## 5. Rationale

**Why PMBM as the Cl-3 endgame.** The four-month Cl-2 program has
exhausted the cheap upgrades inside IMM+TOMHT (see
[evaluation-log.md](evaluation-log.md)):

- Cl-2 #1 (sc13 NEES residual): metric artefact, not filter — closed.
- Cl-2 #2 (re-confirmation over-confidence): **not reachable** from
  lifecycle thresholds or init-cov priors. The rejection bench
  (`cl25_life_20260620.csv`) shows cardinality bloat at +17 % anchored
  GOSPA when we widen — the over-confidence lives in the **joint
  existence + association coupling at re-acquisition**, which our
  TOMHT + IPDA layers handle as two separate concerns with no shared
  Bayesian recursion.
- Cl-2 #3 (UKF inside IMM): shipped, real wins on autoferry/philos.
- Cl-2 #4 (EO/IR R tightening): rejected, physical noise floor bounds R.

PMBM addresses Cl-2 #2 **natively**: existence and association are coupled
in the same RFS update. The Bernoulli's posterior covariance after a
miss-and-recover *automatically* reflects the gap, because `r` and `f` are
updated by the same equations. There is no separate IPDA layer doing the
existence side without telling the kinematic side.

**Why PMBM over JIPDA (Cl-1) at endgame.** JIPDA *does* couple existence
with association — that's its point relative to JPDA. But JIPDA still
commits per scan (no deferred-decision mixture across multiple scans), so
it can't recover from a wrong commit the way MHT can. PMBM gives both
the joint existence-association coupling (JIPDA's strength) and the
deferred mixture over hypotheses (MHT's strength) inside one recursion.

**Why PMBM over δ-GLMB.** Both RFS, both with existence probabilities. PMBM
has a simpler propagation step (no labelled GLMB component algebra), and the
trajectory-PMBM variant emits labelled trajectories directly — matching our
`TrackOutput` lifecycle natively (`onTrackInitiated/Confirmed/Updated/Deleted`).
Maritime evals (*Sci. Rep.* 2025, *Signal Processing* 2024) show PMBM
competitive with δ-GLMB at lower implementation cost.

**Why not GLMB/PHD/CPHD.** PHD/CPHD discard target identity (no labels) — a
non-starter under our stable-`track_id` invariant. GLMB carries labels but
adds the labelled-component algebra. PMBM (with trajectory variant) gets
the labels we need without the algebra.

---

## 6. Implementation strategy — port shape

The current `IDataAssociator` port returns assignments; lifecycle is a
separate `TrackManager` concern. PMBM does both jointly, so it does **not**
fit `IDataAssociator` cleanly. Two options:

**Option A (recommended).** New `PmbmTracker` sibling to `MhtTracker`,
implementing the existing `ITracker` interface (or whatever umbrella shape
MHT exposes). The PMBM update internally owns its mixture, Murty K-best,
existence updates, and per-Bernoulli `IEstimator` use. `IDataAssociator`
stays as-is — PMBM is parallel to it, not behind it.

This matches the existing engineering plan ([Phase 1](../superpowers/plans/2026-06-07-pmbm-integration-plan.md#phase-1-gm-pmbm-gaussian-per-bernoulli-no-imm-yet--3-weeks)).

**Option B.** Generalise `IDataAssociator` to return per-pair existence
mass and lifecycle decisions, with `TrackManager` accepting that richer
output. Higher refactor cost; pollutes the MHT path with PMBM-shaped
outputs. Rejected.

### 6.1 Composition with existing components

| Existing component | Role under PMBM |
|---|---|
| `SensorBiasEstimator` | unchanged; corrects measurements before they reach `PmbmTracker::update` |
| `ImmEstimator` (UKF inner) | per-Bernoulli single-target density; called via `IEstimator::predict/update/logLikelihood` |
| `core/association/Murty.{hpp,cpp}` | Phase 0 — already shipped; reused per prior hypothesis to enumerate K cheapest children |
| `mergeBranches` | generalised across hypotheses for Bernoulli merging (§3.5) |
| `OwnShipProvider` + datum shift | unchanged; PPP intensity shifts with datum just like Bernoulli means |
| `TrackOutput` aggregation | new output extractor that walks the mixture and emits aggregated tracks |

### 6.2 What needs `IEstimator` to expose

Beyond the current `predict`/`update`:

- `logLikelihood(measurement)` — needed for `ℓ^{j,i}_l` (§3.1) and for the
  PPP→new-Bernoulli weight (§3.2). For IMM, the mode-weighted mixture
  likelihood. *Status: Task #1 in the todo list — prerequisite per the
  2026-06-07 plan; verify state before Phase 1.*
- `gate(measurement)` — cheap pre-filter for `g(z|x) > 0`. May reuse the
  existing Mahalanobis gate.

---

## 7. Ways to improve / what to test next

In order of expected payoff:

1. **Bench `imm_cv_ct_pmbm` (GM, no IMM-per-Bernoulli yet) vs
   `imm_cv_ct_mht`** on the full 29-scenario matrix — Phase 1 deliverable.
   Headline metric: GOSPA on autoferry-anchored sc3/sc4 (where Cl-2 #2
   over-confidence is biggest) and autoferry-unanchored sc17/sc22 (where
   Cl-2 #3 UKF gave the largest wins). If PMBM beats MHT on the
   over-confidence-driven anchored cases, that closes Cl-2 #2 as a
   structural fix.
2. **Add IMM per Bernoulli** (Phase 2). Compare `imm_cv_ct_pmbm`
   (IMM-per-Bernoulli) vs the GM PMBM from step 1, on the same matrix.
   Validates that IMM benefits compose with PMBM the way they did with MHT.
3. **GOSPA + T-GOSPA metrics** (Phase 4 — promote before TPMBM). Without
   GOSPA we cannot reproduce published comparisons or quantify
   fragmentation. OSPA penalises only assignment cost; GOSPA penalises
   missed/false detections explicitly. T-GOSPA penalises trajectory
   fragmentation — the exact thing PMBM is supposed to win on.
4. **TPMBM** (Phase 3 — trajectory-augmented Bernoullis). Replaces the
   current `MhtTracker` *structurally* once it beats TOMHT-IMM by ≥10 %
   GOSPA across the bus scenarios.
5. **Clutter-invariant birth existence tuning** (Task 1, 2026-06-24, §3.2.2).
   `birth_existence_target = 0.1` is the first probe value; sweep
   `{0.05, 0.1, 0.2, 0.3}` against philos + autoferry to find the optimal
   target. The design intent is that real targets earn confidence over 2-3
   detections (posterior `r` ramps from `r*` → 1.0), so `r* < 0.5` is
   appropriate; keep it high enough that the phantom-birth gate
   (`min_new_bernoulli_existence`) can still suppress the near-zero-r clutter
   births (set `min_new_bernoulli_existence = r*/2`). The task-1 probe
   `gospa_mean` on philos: 48.50 m vs 82.63 m baseline (−41 %).
5a. **Misdetection dedup + cardinality control bundle** (Task 2, 2026-06-24,
   §3.1.1–3.1.3). `source_aware_identity = true` and `dedup_miss_pd = true`
   are now behind config flags (default off = bit-identical). Task 2c
   (2026-06-24) ran the full bundle (`dedup_miss_pd=true`,
   `source_aware_identity=true`, `birth_existence_target∈{0.1,0.15,0.2}`,
   `output_existence_floor∈{0.1,0.3}`, `min_new_bernoulli_existence=0.1`):
   best philos result was gospa_mean=112.0 (target=0.1, floor=0.1,
   card_err=+46.3) — significantly worse than the adapt baseline (82.63) and
   the task-1 birthtarget result (48.5). The bottleneck is structural:
   `dedup_miss_pd=true` reduces the miss penalty for phantom Bernoullis born
   from radar clutter so they accumulate to r>0.3 before idle-decay prunes
   them; the `output_existence_floor` has negligible effect on them (they
   exceed it). Autoferry guard is *unexpectedly better* (bundle mean 8.70 vs
   adapt 10.50, −18.3%, 16 of 18 scenarios improved); the dedup+identity
   combination helps autoferry where radar detections are dense and per-vessel
   gating is clean. The bundle ships as `imm_cv_ct_pmbm_bundle` in
   `Config.cpp` for ablation use. To make `dedup_miss_pd` viable on philos,
   the next candidate fix is a tighter clutter-conditional r_min (raise
   `r_min` per-sensor when λ_C is sparse-AIS, not uniform 1e-5), or a
   PPP-coverage birth gate that suppresses births in AIS-only regions.
6. **Poisson birth intensity spatial tuning.** First cut is uniform over
   local ENU patch. Real wins come from sensor-region-conditional birth —
   radar coverage edges, AIS broadcast zones, named ports. Pin a synthetic
   "fixed-birth-region" scenario to A/B uniform vs tuned.
7. **Sensor-conditional `p_S`.** Ships pop in and out of radar coverage; a
   single `p_S` mis-handles entry/exit. Likely small effect; revisit only
   if a scenario shows track loss attributable to coastal radar geometry.
8. **OOSM-PMBM** (Phase 5, optional). Out-of-sequence measurements
   (late-arriving AIS) handled by retrodiction. Our `ReorderBuffer` is the
   pragmatic substitute today; revisit only if buffer latency becomes a
   problem.
9. **Cluster decomposition for Murty cost.** PMBM Murty cost matrices have
   the same block-sparsity that MHT cost matrices have (a target with no
   in-gate measurement is independent of the rest). Reusing the planned
   cluster decomposition for `MhtTracker` benefits both.

---

## 9. Coverage / Visibility Channel (`ISensorActivity`)

*Shipped: Task 4, 2026-06-29. Port: `ports/ISensorActivity.hpp`,
`ports/IStaleSignalSink.hpp`. Provider: `core/sensor_activity/DeclaredSensorActivity.{hpp,cpp}`.
Wired in `core/pmbm/PmbmTracker.cpp` (surveillance miss + cooperative stale).
Plain-English introduction: [learning §24](../learning/24-coverage-visibility-channel.md).*

---

### 9.1 Math

The Bernoulli existence recursion after a missed detection is:

```
r⁺ = (1 − p_D) · r / (1 − r · p_D)
```

The whole point of the coverage channel is to feed that recursion the **right
`p_D`, charged at the right time**. The rule:

**Charge at most one miss per sensor duty cycle, not per timestamp-batch.**

For each Bernoulli and each channel that `ISensorActivity` knows:

1. **Surveillance channel** (`ChannelKind::Surveillance` — radar, EO/IR, lidar).
   The sensor sweeps an area on a known duty cycle (e.g. 2.5 s per radar
   rotation). The miss penalty fires *only* when all three hold simultaneously:
   - the sensor is active,
   - the track's predicted ENU position is inside the declared coverage
     (`max_range_m`, azimuth sector), and
   - a full sweep has completed since `last_activity_check_` for this Bernoulli.

   When all three hold and no associated return arrived → `MissOpportunity
   {surveillance_miss=true, p_D=<channel p_D>}` → one miss is charged.
   Otherwise `p_D = 0` → the recursion simplifies to `r⁺ = r` (unchanged).

2. **Cooperative-announce channel** (`ChannelKind::Cooperative` — AIS, fleet
   link). Does **not** enter the existence recursion at all (decision spec §9c).
   If the track's own-identity report is overdue (elapsed since last contact
   exceeds `expected_report_interval_sec`), `MissOpportunity
   {cooperative_overdue=true}` → `IStaleSignalSink::onTrackStale` is raised and
   `r` is left unchanged. A cooperative-only track is retired *only* by an
   explicit `cooperative_stale_timeout_sec` (configured, operator-tunable), never
   by the miss-existence recursion.

3. **Between sweeps / outside coverage / sensor off** → no opportunity → `r`
   unchanged. This replaces `idle_halflife_sec`: existence bleeds only from a
   genuine surveillance miss, never from wall-clock time alone.

**Snapshot + deferred write** (`last_activity_check_` / `staged_activity_check_`
in `PmbmTracker`). The per-Bernoulli activity-check timestamp is read as a
frozen pre-scan snapshot before the hypothesis loop begins, and writes are
staged in `staged_activity_check_`, then applied *once* after all hypotheses are
enumerated. This makes the activity-check window hypothesis-order-independent:
every parent hypothesis sees the same `last_checked` for a given Bernoulli id,
regardless of which hypothesis the loop processes first.

---

### 9.2 Assumptions

1. **Known surveillance coverage and cadence.** Each surveillance sensor has a
   declared `max_range_m`, azimuth sector, `duty_cycle_sec`, and `p_D` in a
   `DeclaredSensorActivity::ChannelProfile`. An adaptive learned provider is a
   planned later implementation behind the same `ISensorActivity` interface (spec
   roadmap §13.1).
2. **Known cooperative cadence.** Each cooperative source has a declared
   `expected_report_interval_sec` per channel profile.
3. **Identity stable enough to key cadence.** The per-Bernoulli
   `last_activity_check_` lookup uses the Bernoulli's track id. Cooperative
   overdue detection uses `hints.mmsi` or `hints.platform_id` — these are hints
   (not the fusion key), but must be populated and stable enough for the cadence
   window to be meaningful.
4. **No re-feeding of stale measurements.** The consumer supplies
   coverage/cadence state via the `ISensorActivity` port, not by reinjecting
   old positions as fresh measurements. `ISensorActivity::evaluate` is a pure
   function of declared profiles and its timestamp arguments: no wall-clock, no
   RNG.
5. **Sensor at ENU origin in the declared profile.** The range/sector coverage
   check uses the track's ENU position relative to origin. A sensor mounted off-
   centre requires either a translated profile or an adapter.

---

### 9.3 Rationale

**Why per-duty-cycle, not per-timestamp-batch.** Before this change, a "scan"
was whatever measurements happened to share the same timestamp — accidental
batching, not a physical sweep boundary. That made the miss-penalty accidental:
a sensor only counted as "looked and found nothing" if it happened to emit some
other measurement at the exact same instant. One radar rotation was one
*opportunity* regardless of how many blips arrived in that rotation. The
per-duty-cycle rule matches that physical model, and matches what MHT already
approximated with its IPDA/VIMM visibility channel.

**Why AIS/Cooperative channels do not touch existence.** Cooperative-announce
sources are asymmetric: a report is strong evidence (the target exists, here,
with an identity), but silence is weak evidence (transponder congestion, link
drop, range, switched off). Penalising `r` on a quiet fleet member would wrongly
kill cooperative-only tracks on a comms drop. The architecture separates the two
signals: a stale cooperative link raises a human-visible flag (`IStaleSignalSink`
→ operator display) while the filter continues to coast on whatever surveillance
evidence it has. Retirement on cooperative silence happens only through a long,
explicitly-tunable timeout, or after a surveillance channel that covers the
position confirms absence.

**What three crutches this retires.**
- *Wrong per-blip `compute_miss_pD`*: charged once per measurement not per
  sweep. Was the only thing suppressing phantom Bernoullis on philos (a
  load-bearing crutch that happened to work by accident).
- *`idle_halflife_sec` fade-out*: a global wall-clock decay applied uniformly to
  all tracks regardless of whether any sensor had a real chance to observe them.
- *`source_id="ais"` patch (Task 2a)*: ad-hoc AIS identity gate, folded into the
  unified identity gate (`mmsi` else `platform_id`) and the cooperative stale
  path.

**Port is nullable.** `PmbmTracker::setSensorActivity(nullptr)` → bit-identical
legacy behaviour. Every new config defaults to `use_sensor_activity=false`; the
old code paths are not removed. The interface is exchangeable: `DeclaredSensorActivity`
is implementation #1; an adaptive provider with learned cadence/coverage is
implementation #2, behind the same `ISensorActivity` port (spec roadmap §13.1).

---

### 9.4 Ways to improve / what to test next

**Measured result (2026-06-29; see [evaluation-log.md](evaluation-log.md) §"Task 4").**

Coverage channel is **best-in-class on autoferry** (high p_D 0.6–0.8,
open-water surveillance-dominated):

| scenario | coverage gospa | bundle gospa | adapt gospa |
|---|---|---|---|
| scen2 | **11.33** | 12.88 | 17.28 |
| scen22 | **15.28** | 15.74 | 21.39 |
| scen2 card_err | **+0.15** | −0.55 | +0.39 |
| scen2 id_switches | **0–1.5** | 0–5.5 | 5–18 |

Wins on accuracy, cardinality control, and identity stability, with *fewer
knobs* (no `idle_halflife`, no wrong-math `dedup_miss_pd`).

Coverage **regresses badly on philos** (coastal radar, p_D=0.07):

| config | gospa_mean | card_err |
|---|---|---|
| birthtarget (Task 1) | **48.5** | −7.8 |
| adapt | 82.6 | +17.5 |
| bundle (Task 2) | 112.0 | +46.3 |
| **coverage (Task 4)** | **153.6** | **+107.9** |

Two compounding causes:
1. **AIS immortality (plumbing gap).** The cooperative retirement timer
   (`last_cooperative_touch_`) fires only for `SensorKind::Cooperative` — philos
   AIS is `SensorKind::Ais`, so the timer never starts and AIS tracks are never
   retired. Confirmed by isolation: changing `cooperative_stale_timeout_sec` from
   120 s to 15 s changes the result by zero.
2. **Honest radar miss is too weak at p_D=0.07.** A single missed sweep moves
   existence only marginally (`r⁺ ≈ 0.93 · r`). Persistent shore returns are
   re-detected every rotation so they never miss; removing the wrong-math and
   idle_halflife removed the only thing suppressing those phantoms. Same lesson as
   Task 2c (correct math worse on philos) and Task 3 (clutter map inert): **philos
   over-count is a spatial clutter problem, not a temporal one.**

**Decision (Task 8).** Keep `imm_cv_ct_pmbm_coverage` as an opt-in ablation
config, not the canonical default. It is the recommended PMBM choice for
high-p_D surveillance-dominated deployments (autoferry-class); it must not be
used for low-p_D coastal workloads (philos-class).

**Follow-up candidates in expected payoff order:**

1. **Coastline / land-mask clutter suppression at birth** (next candidate for
   philos). A spatial prior over the surveillance area suppresses shore-echo
   Bernoullis at birth — a clutter-prior at the PPP birth step, or an occlusion
   mask in the coverage query. Addresses the root cause: once phantom births are
   prevented, the temporal miss model no longer has to compensate.
2. **Timer key on `ChannelKind`, not `SensorKind`.** The cooperative stale/
   retirement path should key on `ChannelKind::Cooperative` from the activity
   profile, not on `SensorKind`. This fixes the AIS immortality gap (cause #1)
   and makes the coverage model viable for AIS-heavy deployments. Deferred
   because it would not change the philos verdict (radar phantoms, cause #2,
   dominate), but needed before promoting `coverage` to any AIS-heavy scene.
3. **Adaptive cadence provider** (spec roadmap §13.1). Replace the declared
   static profile with a learned model that infers each source's cadence and
   coverage from observed report gaps, behind the same `ISensorActivity`
   interface. No tracker changes; a drop-in replacement.
4. **Birth confidence by sensor kind** (spec §9b). Timid surveillance births /
   confident cooperative births / most-confident fleet-link births. Compounds
   with Task 1; pick up once the coverage model is stable.
5. **Target-dependent p_D** (RCS / size). Small vessels are genuinely harder to
   detect; today p_D is per-sensor not per-target. A surveillance-side term so
   faint intermittent targets are not over-penalised on a miss.

---

## 10. Land / Coastline Clutter Prior

> Plain-language introduction: [learning §25 — Suppressing tracks on land: the coastline clutter prior](../learning/25-land-clutter-prior.md).

### 10.1 Math

**The spatial clutter prior.** At every new-target birth position `p` (ENU metres), the tracker
queries `c = ILandModel::clutterPrior(enu_xy) ∈ [0, 1]`. The concrete implementation
(`CoastlineGeometry`) computes a *signed-distance shoreline ramp*:

Let `d` be the signed distance from `p` (projected to geodetic) to the nearest shore edge of the
consumer-supplied land polygons (`d < 0` inland, `d > 0` offshore). Then:

```
c(d) = clamp( (W_off − d) / (W_off + W_in), 0, 1 )
```

where `W_in` = inland half-width (default 50 m) and `W_off` = offshore half-width (default 50 m).
Consequently:

- `c = 1.0` for `d ≤ −W_in` — well inside land (the plateau / hard-gate region).
- `c ≈ 0.5` at `d = 0` — exactly at the waterline (when `W_in = W_off`).
- `c = 0.0` for `d ≥ +W_off` — open water (no suppression).

Positions outside the loaded polygon coverage return `c = 0.0` (no suppression — unknown is
treated as open water).

**Birth suppression rule** (applied in both `buildAdaptiveBirthCandidates` and
`buildNewTargetCandidates`):

```
const double land_scale = landBirthScale(cand.mean);
if (land_scale < 0)             // c > land_birth_hard_gate (default 0.95) → inland hard gate
    lambda_birth = 0.0;         // birth candidate dropped entirely
else
    lambda_birth *= land_scale; // soft suppression: lambda_birth *= (1 − c)
// rho_target = lambda_birth; r_new = rho_target / rho_total
```

**Why this must act on birth intensity, not λ_C.** The obvious route — raising clutter intensity
λ_C near shore — is silently defeated by Task 1's `birth_existence_target`. In adaptive-birth
mode the code sets `λ_birth = (r*/(1−r*)) · λ_C(z)`, so:

```
r_new = λ_birth / (λ_birth + λ_C(z))
      = (r*/(1−r*)) · λ_C(z) / ( (r*/(1−r*)) · λ_C(z) + λ_C(z) )
      = r*       [λ_C(z) cancels]
```

`r_new` is pinned to the configured target `r*` and is independent of `λ_C`. Raising `λ_C` just
scales `λ_birth` proportionally, leaving `r_new` unchanged. The land prior must therefore scale
`λ_birth` (i.e. `rho_target`) directly.

**ENU → geodetic query.** `clutterPrior` accepts ENU metres; the adapter converts to geodetic via
`datum.toGeodetic(enu_xy)` and evaluates the stored polygons. Geometry is stored in its native
geodetic frame (WGS84 lat/lon degrees) and is never reprojected to ENU.

### 10.2 Assumptions

1. **Consumer-supplied coastline.** The application provides GeoJSON (WGS84, Polygon /
   MultiPolygon features) covering a reasonable radius around own-ship. The tracker does not fetch
   or discover coastline data itself (hexagonal boundary: I/O lives in the adapter, not the core).

2. **Soft band absorbs waterline imprecision.** Coastline GeoJSON is typically administrative- or
   survey-grade, not tide-corrected. The `W_in`/`W_off` margin band (~50 m default) accommodates
   this: positions that fall on the wrong side of the coarse waterline get soft suppression
   (c ≈ 0.5), not hard rejection.

3. **Datum recenters are observed.** When `OwnShipProvider` recenters the datum, the
   `CoastlineModel` receives `IDatumChangeSink::onDatumRecentered(old, new)` and swaps the query
   datum. Because geometry is stored in geodetic coordinates, no polygon reprojection occurs.
   Out-of-coverage positions (new area not yet in the loaded GeoJSON) return `c = 0.0` — no
   suppression, never invented land — providing graceful stale-until-fresh degradation.

4. **Async coastline swaps at deterministic stream points.** Fresh GeoJSON arrives asynchronously
   from a consumer task. The swap (`CoastlineModel::setCoastline(...)`) must be applied at a
   deterministic, timestamped point in the measurement stream — never from a wall-clock callback
   mid-scan. This preserves CLAUDE.md invariant 4 (same input ordering → identical output).

5. **Real targets inside the hard-gate region are rare and accepted.** The hard gate fires only at
   `c > 0.95`, which requires `d < −W_in + ε` — the inland plateau, not the waterline. A vessel
   moored against a pier finer than the polygon resolution might be hard-gated at birth; this is an
   accepted residual risk bounded by keeping the gate strictly inland-only (design spec §9a).

### 10.3 Rationale

**Philos over-count is a spatial problem.** The Boston inner-harbor (philos) replay had card_err
+107.9, traceable to 185 fixed, stationary radar returns at shore positions that no AIS vessel ever
occupies. A pre-check (2026-06-29) found that 86% of all philos radar plots fall on land (69%) or
within 50 m of shore (17%), vs 13.8% open water. The Task 4 coverage/visibility channel cannot
remove these: at philos radar p_D = 0.07 a missed sweep barely moves existence (`r⁺ ≈ 0.93·r`),
and persistent shore returns are re-detected every antenna rotation. The over-count is spatial —
the only lever is spatial.

**Suppress births, not λ_C.** As shown in §10.1 the algebra, with `birth_existence_target` active
`r_new` is independent of λ_C. Births must be attacked by scaling the birth intensity directly.

**Geodetic storage for trivial recenter.** The ENU datum auto-recenters as own-ship moves. Storing
polygons in geodetic lat/lon makes recenter trivial: swap one datum pointer, no geometry
recomputation. Stale-until-fresh behaviour (out-of-coverage → 0) follows for free.

**Inland-only hard gate protects anchored near-shore vessels.** The shoreline ramp reaches c = 1.0
only well inland (`d ≤ −W_in`). The hard gate (c > 0.95) therefore never fires at the waterline.
Vessels moored at the waterline sit in the mid-ramp (c ≈ 0.5) and can still accumulate posterior
evidence over repeated detections and confirm a track. A "re-detected every scan → override gate"
rule is deliberately not used: shore returns are also re-detected every scan.

### 10.4 Ways to improve / what to test next

**Measured results (2026-06-30; see [evaluation-log.md](evaluation-log.md) §"Task A — PMBM
land/coastline clutter-prior").**

| config | gospa | card_err | gospa_false |
|---|---|---|---|
| birthtarget (Task 1; wrong-math brake) | **48.5** | −7.8 | 390 |
| coverage (honest, no land) | 153.6 | +107.9 | 23750 |
| **coverage + land** | **73.1** | **+6.9** | **3550** |
| adapt | 82.6 | +17.5 | 5150 |
| bundle | 112.0 | +46.3 | 11420 |
| MHT canonical (historical) | ~69.4 | — | — |

The land model works decisively: card_err collapses from +107.9 to +6.9 (~94% gone), gospa_false
from 23750 to 3550 (−85%), gospa from 153.6 to 73.1 (−52%). `coverage+land` beats `adapt` and
`bundle` and is near MHT — the first honest, no-crutch PMBM config competitive on philos.

Autoferry guard: `coverage+land` is byte-identical to `coverage` on all four autoferry scenes —
the land model is correctly inert where no coastline fixture exists. No regression.

`birthtarget + land` is byte-identical to `birthtarget` (48.5 / −7.8 / 390): the wrong-math
miss-pD already over-suppresses the on-land phantoms, so the land mask is redundant on top of it.
The land model is the *honest substitute* for the wrong-math crutch, not an addition.

**Remaining gap and next steps.** The dishonest `birthtarget` (gospa 48.5) still edges
`coverage+land` (73.1) because its over-aggressive wrong-math also kills the residual *water /
near-shore* clutter that the land mask does not cover (gospa_false 390 vs 3550). Closing this gap
requires spatial work on the water side:

1. **Tighter offshore margin / near-shore handling.** Reduce `W_off` or add a graduated near-shore
   zone (e.g. 0 < d < 100 m) with moderate soft suppression to damp waterline-adjacent clutter.
2. **Finer / higher-resolution charts.** Administrative GeoJSON has ~30–100 m waterline error.
   Survey-grade or nautical-chart polygons would push the soft-band needs toward zero.
3. **Online clutter-field learning.** Couple with the existing `ClutterMapDetectionModel` (Task 3):
   learn the spatial clutter density from observed false tracks and feed it into the birth prior.
   Complements the static coastline — catches water-side clutter the land mask cannot reach.
4. **Coverage-occlusion (Task 4 coupling, §9).** Land between the sensor and the track should
   suppress the surveillance miss penalty for the occluded sector, not just births. A
   land-occlusion query at miss-detection time would further close the near-shore gap and couple
   this module with the coverage/visibility channel.

---

## 11. PDA soft detected-branch update (`use_pda_soft_detected_branch`)

Opt-in refinement of the per-Bernoulli detection update (§3.1). Cross-ref:
[learning ch.12 — JPDA](../learning/12-jpda.md) for the plain-English intro;
`PmbmTracker::Config::use_pda_soft_detected_branch`, the detected branch in
`enumerateChildren` (`core/pmbm/PmbmTracker.cpp`).

### 11.1 Math

Under K=1 the assignment gives each detected Bernoulli *i* a single winner
measurement *l* = `bernoulli_to_meas[i]`, and the Bernoulli's posterior is the
Kalman update against *l* alone (§3.1). The soft update instead forms a
probabilistic-data-association posterior over a **pool** P(i):

```
P(i) = { l } ∪ { gated j : C(i,j) finite AND j not claimed by another existing Bernoulli }
β_j  = p_D(j)·ℓ_{i,j} / Σ_{k∈P(i)} p_D(k)·ℓ_{i,k}          (detections-only; Σβ_j = 1)
x̂    = Σ_{j∈P(i)} β_j · x̂_{i,j}                            (β-weighted per-cell means)
P̂    = Σ_{j∈P(i)} β_j · ( P_{i,j} + (x̂_{i,j} − x̂)(x̂_{i,j} − x̂)ᵀ )   (moment match + spread)
```

`x̂_{i,j}`, `P_{i,j}` and `ℓ_{i,j}` are the per-cell updated state / covariance /
likelihood **already pre-computed** during cost-matrix construction (§3.4), so the
soft update adds no extra `estimator.update` calls — it re-weights cached results.
For IMM each mode's mean/cov/probability is β-combined the same way. Only the
**state** changes; the child hypothesis weight still uses the winner *l*
(`log r + log p_D(l) + log ℓ_{i,l}`), so the mixture / Murty / birth structure is
untouched. When `|P(i)| = 1` this is byte-identical to the hard update.

### 11.2 Assumptions

1. **The gate captures the target.** β only spreads over gated measurements; a
   target return outside the χ² gate cannot rescue the state.
2. **Unclaimed / birth-claimed gated returns are the softening set.** A return
   claimed by *another established* Bernoulli is excluded (it belongs to that
   track); returns that are clutter or fresh births may enter the pool.
3. **Detections-only weighting** (no β₀ miss term): the missed-detection
   hypothesis stays the separate misdetection branch (§3.1); the soft averaging
   over the detection pool is what damps the clutter pull, and keeping β₀ out of
   the detected branch preserves the PMBM branch structure.
4. **Winner-weighted hypothesis mass.** The soft update is a *state-estimation*
   refinement inside the winning hypothesis, not a re-weighting of hypotheses.

### 11.3 Rationale

The K=1 GNN winner-take-all mis-assigns a real open-sea track to a gate-**closer**
clutter return and pulls the state fully onto it, so the real return leaves the
gate next scan and the track dies (`imm_cv_ct_pmbm_land` dense_clutter lifetime
0.823 vs MHT 0.925). Raising K in the flat representation regressed anchored
(phantom-birth Bernoullis in alternative hypotheses). The soft update fixes the
state pull *without* enumerating alternatives: it never adds a Bernoulli, never
changes K, never touches Murty. The **unclaimed-only** pool is the load-bearing
scoping decision — it lets isolated-target clutter soften the update (open-sea)
while excluding measurements owned by competing tracks, so dense multi-target
scenes (philos) see pool ≈ {winner} and stay hard/byte-identical, avoiding the
over-count that has repeatedly bitten this codebase.

### 11.4 Ways to improve / what to test next

- **β₀ miss term.** Add the clutter/missed-detection weight to further damp the
  pull; measure whether it helps beyond detections-only without leaking miss mass
  into the detected branch.
- **Promotion to default.** A/B measured open-sea ↑, extended-target over-count
  ↓, philos/anchored flat (eval-log 2026-07-02). Before promoting past opt-in,
  run the autoferry replay A/B + real-data error bars.
- **`pda_soft_detected_branch_on_confirmed_only`.** Restricting to confirmed
  tracks is wired but unmeasured — test whether softening young births helps or
  hurts phantom control.
- **Joint (JPDA) marginals.** The single-target β ignores cross-track competition
  fully modelled only by joint association; a JPDAF marginal β would be the
  principled extension if the unclaimed-only heuristic proves too coarse.

---

## 8. References

Same as [sota-roadmap.md §4](sota-roadmap.md#4-trajectory-pmbm-as-a-third-idataassociator).
Primary: García-Fernández, Williams, Granström, Svensson 2018 (PMBM
filter — direct derivation) and 2020 (trajectory variant). Bridge: Williams
2015 (RFS derivation of MHT/JIPDA/MeMBer — read before porting).
Reference implementation: [Agarciafernandez/MTT](https://github.com/Agarciafernandez/MTT)
(MATLAB, BSD-2; we port, not link).
