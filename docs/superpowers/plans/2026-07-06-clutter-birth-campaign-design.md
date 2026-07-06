# Clutter/birth-model campaign — design note (Phase A → ARBITER CHECKPOINT 1)

**Status: awaiting arbiter approval. No implementation (Phase B) until this is signed off.**
Date: 2026-07-06 · Branch: `clutter-birth-campaign` (worktree `../navtracker-clutter`, off master `7cbfe9d`)
Ticket: `docs/superpowers/plans/2026-07-06-clutter-birth-campaign-ticket.md`
Phase 0 (backlog #21) is committed separately ahead of this note.

Grounding: a 7-agent code+doc research sweep (all claims file:line-verified) plus a
direct `lambda_birth` sweep on `harbor_complete_truth`. The load-bearing citations
are collected in the Appendix.

---

## 0. TL;DR

**Recommendation — build TWO candidates, both on the same non-cancelled channel, sequenced fail-fast:**

- **Candidate A (primary): a learned *spatial* clutter birth-suppression prior.** An
  ENU-anchored grid that learns, from a **claim-agnostic** persistence×dispersion
  signal on raw returns, a per-cell suppression `c_clut(x) ∈ [0,1]`, folded into the
  existing `birthScale` as a third multiplicative factor `(1 − c_land)(1 − c_static)(1 − c_clut)`
  — the exact template of the shipped static-obstacle birth prior. Attacks the
  *persistent burst* that dominates `sim_ms_clutter_burst` and any spatially-structured
  HAXR clutter.

- **Candidate B (paired): a compound-K / negative-binomial count-adaptive birth
  discount.** A statistical (not spatial) `c_od ∈ [0,1]` derived online from the
  over-dispersion of the per-scan unassociated-return count, folded into the same
  `birthScale`. Attacks the *over-dispersed diffuse* clutter (the compound-K background
  and, we hope, the diffuse HAXR over-count). This is the honest test of the learning
  doc's own §10 "not picked" bullet (non-homogeneous cluster process, "estimation
  fragile").

Both are **default-off (byte-identical), position-domain only, claim-agnostic, and act
on `birthScale` — never on `λ_C`.** That last point is the whole design: see §1.1.

**The single most important research finding — the ticket's premise needs a correction.**
On the campaign's actual deployed PMBM config (`imm_cv_ct_pmbm_coverage_land`,
`birth_existence_target = 0.1`), the birth existence is `r_new = λ_birth/(λ_birth+λ_C)`
with `λ_birth = (r*/(1−r*))·λ_C`, so **`r_new = r*` exactly and `λ_C` cancels.** A
spatially-varying `λ_C` (candidate a as literally worded), a per-sensor `λ_C`
(candidate d, already shipped), and a count-adaptive `λ_C` (candidate b as an intensity)
are **all inert on the deployed config** — raising `λ_C` changes nothing in the born
existence. This is why the shipped clutter map is inert and why this note routes every
candidate through `birthScale` instead of `λ_C`.

An honest "built, measured, not better" remains an acceptable close for this redirect —
especially on HAXR, which the increment-8 verdict already characterised as **diffuse**
clutter (a persistence/occupancy suppressor cut <3%), and which may resist a spatial
prior by construction.

---

## 1. What the research overturned (read before the candidates)

### 1.1 `λ_C` is *cancelled* in the deployed birth existence — the load-bearing coupling

Under `adaptive_birth = true` (every real config), birth existence is
`r_new = λ_birth / (λ_birth + λ_C(z))` (`PmbmTracker.cpp:1082-1083`, `:500-507`).

- On **`_land` / `_adapt`** (fixed `λ_birth = 1e-5`, `birth_existence_target = 0`), a
  larger `λ_C` **does** lower `r_new`.
- On **`_coverage_land` / `_bundle*`** (`birth_existence_target = 0.1`),
  `λ_birth = (r*/(1−r*))·λ_C`, so `r_new = r* = 0.1` **independent of `λ_C`**
  (`PmbmTracker.cpp:506-507`; algebra spelled out in `pmbm-design.md:791-803`).

`imm_cv_ct_pmbm_coverage_land` is the config the sim battery and HAXR baselines were
measured on (`docs/baselines/2026-07-06_sim_multisensor_battery.md`,
`2026-07-05_pmbm_runtime_frontier.md`). **So on the headline config, no `λ_C` lever
touches births.** The only place `λ_C` still acts there is the birth-cell assignment
cost `−log(λ_birth+λ_C)` (`PmbmTracker.cpp:806-812`) — and that acts in the **wrong
direction** (higher `λ_C` → cheaper birth cell → Murty *prefers* birthing).

**Consequence for design:** any birth-suppression must ride a channel that is *not*
`λ_C`. The clean one is `birthScale`, the multiplicative existence discount the land and
static-obstacle priors already use (`PmbmTracker.hpp:981-1000`,
`static-obstacle-birth-prior.md:64-91`). Both candidates target `birthScale`.

### 1.2 The PPP intensity is *empty* in deployment → the parked PPP-birth ideas are inert

No bench PMBM config wires a `BirthModelFn`, and `adaptive_birth` skips the
measurement-driven PPP injection (`PmbmTracker.cpp:1384` guard `!adaptive_birth`;
`:1437-1439`). So `density_.ppp` is permanently empty. The whole PPP-intensity machinery
(`buildNewTargetCandidates`, `smart_birth_skip_existing_ppp` = the parked "coverage
check" formulation 1) is dead code in deployment (`Config.cpp:976-985`; the `_cmap`
config sets `use_clutter_map` but never `feed_clutter_map`, so it is byte-identical to
`_adapt`). The parked "prior-mass gate" (formulation 2) was never implemented.
**Candidate (c) as parked is inert.** Its *spirit* (clutter-aware birth weighting) is
what A/B implement, on the live `birthScale` channel rather than the dead PPP channel.

### 1.3 The discriminator conflates TWO clutter mechanisms that reward DIFFERENT models

`sim_ms_clutter_burst` (truth = 2 CV vessels; `scenarios.py:77-87`) mixes:

1. **Compound-K background** — 24 azimuth wedges, per-wedge `Gamma(ν=0.6)` texture ×
   `Poisson(λ̄·A·g)`, marginal **negative-binomial**, mean ≈ 12 plots/scan
   (`clutter.py:48-67`). The texture is **re-drawn every scan**, so hot wedges *move*.
   A learned spatial map **cannot lock onto it**; a non-Poisson count model **can**.
2. **A localized burst** — 25 plots/scan uniformly in a **150 m disk at fixed world
   ENU (1200, 900)** for `t ∈ [120,240] s` (49 scans, `clutter.py:70-82`). Spatially
   *and* temporally persistent → this is what births the persistent phantoms that
   dominate `card_err`. A learned spatial map **can** lock onto it; a count model
   **cannot** localise it.

The tracker is deliberately told the clean-Poisson baseline `λ_C = 2e-8` even here,
while the truth mean is `6e-8` — a **3× mean mismatch on top of the structure mismatch**
(`SimMultisensorScenarioRun.cpp:105-115`). **This mandates a control arm:** declare a
correct *uniform* `λ_C = 6e-8` to separate "right mean" credit from "right structure"
credit — otherwise the discriminator does not prove what the ticket claims.

Because the two mechanisms reward different models, we build one candidate for each (A
for the burst, B for the compound-K background) and let the discriminator tell us which
drives the +3.48.

### 1.4 Three traps that killed the last spatial-clutter attempts (must be designed around)

- **The "channel-reach" wall.** A birth suppressor only gates *new* births; it cannot
  remove an *already-confirmed* phantom (increment-8 finding; `card_err` counts
  Confirmed tracks = aggregated mass ≥ `confirm_threshold = 0.5`,
  `BenchRunner.cpp:246-262`, `Metrics.cpp:128-130`). So A/B can only *prevent* phantoms,
  and only if the suppression is in place *before* the phantom confirms. This is a
  measurable race (§5.0) and a real ceiling on birth-side fixes.
- **The `1−r` death spiral.** The shipped clutter map learns from returns weighted by
  `1−r` of the claiming Bernoulli. On uniform clutter a low-`r` *real* target's own
  returns raise `λ_C` at its own cell → suppress its updates → `r` drops → spiral
  (`test_philos_cluttermap_ab.cpp`: dense_clutter lifetime 0.90 → 0.26; R2 proved this
  is the `1−r` *weight*, orthogonal to the labelling method). **A/B must NOT use `1−r`.**
  They use a *claim-agnostic* signal (raw-return persistence×dispersion, or count
  over-dispersion) that uniform clutter and real point-targets do not cross.
- **The bearing spiral + over-delete.** Bearing-domain maps self-reinforce (bearings
  can't initiate; `enable_bearing_map=false` is load-bearing) → **position-domain only**.
  And the fed cmap *over-deletes* on philos (`card_err` flips to −3.2, deleting non-AIS
  objects AIS-only truth can't score) → violates ADR-0002. **A/B are gated against
  complete-truth scenarios (harbor / sim / dense_clutter), not just the philos
  AIS-truth number.**

### 1.5 Which miss-P_D brake is live depends on the config (minefield #1, precisely)

- `_land` / `_adapt` / `_bundle*` run the **legacy non-dedup `compute_miss_pD`**
  (`1−(1−P_D)^N`, `PmbmTracker.cpp:649-655`) — the "load-bearing brake" on philos
  over-count. `dedup_miss_pd` / `use_sensor_activity` remove it and blow philos up
  (card_err +17.5 → +48). **Do not touch the miss math.**
- **`_coverage_land`** (the headline config) uses `use_sensor_activity = true`, which
  **bypasses `compute_miss_pD` entirely** (`PmbmTracker.cpp:684, 1006`); `dedup_miss_pd`
  is dead code there. Its over-count brake is the coverage/activity model.

So on the headline config A/B do **not** interact with the wrong-math brake; on `_land`
they do. The design A/B's on `_coverage_land` (deployed) and reports `_land` as a
fixed-`λ_birth` control (§5).

---

## 2. The design surface, as coded (where a birth suppressor can attach)

| Channel | Expression (code) | Cancelled by `birth_existence_target`? | Direction | Verdict |
|---|---|---|---|---|
| Birth existence `r_new` via `λ_C` | `λ_birth/(λ_birth+λ_C)` (`:1082`) | **YES** on `_coverage_land` | higher λ_C → lower r_new | dead lever on deployed config |
| Birth-cell assignment cost | `−log(λ_birth+λ_C)` (`:806-812`) | no | higher λ_C → **cheaper birth** | wrong direction; unusable |
| **`birthScale` factor** | `(1−c_land)(1−c_static)` (`:981-1000`) | **no** (applied after the ratio) | higher c → lower r_new | **the channel A/B use** |
| Birth **hard-veto** | drop candidate if `c > gate` (land uses 0.95) | no | binary | optional strong-clutter veto |
| Existing-Bernoulli existence update | `r ← (1−P_D)r/…`, `r·P_D·ℓ/…` — **no `λ_C`** (`:984-986,1048`) | — | — | out of reach of any λ_C model (channel-reach wall) |

The `birthScale` third-factor path is a drop-in: `applyBirthPriors` already multiplies
land and static factors; A/B add `(1 − c_clut)`. A hard-veto mirrors
`land_birth_hard_gate`.

---

## 3. Candidate evaluations (four-part each + joint coupling)

### Candidate A — Spatial clutter birth-suppression prior  ★ recommend build

**Math.**
An ENU-anchored coarse grid (cell ~50 m, matching `ClutterMapParams`). Each cell `k`
holds a time-decayed estimate of a *claim-agnostic clutter feature*, updated post-scan:
```
   s_k ← (1 − w)·s_k + w·f_k ,   w = 1 − exp(−Δt/τ)          (τ ~ 20 s)
```
where `f_k` is **not** a `1−r`-weighted count. Two feature options to A/B in Phase B:
  (A1) *area-rate of unclaimed-and-unresolved returns* — count of returns in cell `k`
       that neither a confirmed track nor a fresh tight cluster explains, per cell area;
  (A2) *dispersion excess* — returns persistently present in `k` but **not resolving to
       a sub-cell point** (spatial spread ≫ radar resolution). A real point-target (even
       anchored) produces a tight cluster → low dispersion; the burst (25 pts over a
       150 m disk) and compound-K clumps produce high dispersion.
Suppression is a saturating map `c_clut(x) = clamp(g·(s(x) − s0), 0, c_max)` with a
soft cap `c_max ~ 0.9` (never a hard 1 by default) and an optional hard-veto at a high
gate. `birthScale ← (1−c_land)(1−c_static)(1−c_clut)`; born existence
`r_new ← r_new_config · (1−c_clut)`.

**Assumptions.**
- Clutter that causes *persistent* over-count is spatially concentrated and ENU-fixed
  (true for the burst and for shore/pier structure; **false** for uniform-diffuse
  clutter — see failure modes).
- A real vessel's returns are spatially tighter than a clutter clump at the same
  persistence — i.e. dispersion (A2) or claim-status (A1) separates them. If a real
  anchored vessel produces a returns cloud as diffuse as clutter, A over-deletes it
  (ADR-0002 risk).
- The map warms (τ, first-touch weight) fast enough to raise `c_clut` at the burst
  **before** the phantom confirms (§5.0 race).

**Rationale vs alternatives.**
- vs the shipped `ClutterMapSensorDetectionModel`: that one (i) is never fed
  (`feed_clutter_map=false` everywhere → inert), (ii) modulates `λ_C` (cancelled on
  `_coverage_land`), (iii) learns from `1−r` (death spiral), (iv) can run in the bearing
  domain (spiral). A fixes all four: ships its write path **on** (per-instance), acts on
  **`birthScale`** not `λ_C`, uses a **claim-agnostic** feature, **position-only**.
- vs a global `λ_C` retune: cannot win the discriminator by construction (spatial-by-
  design) and would suppress the clean sim scenarios + philos KEEP.

**What to test.** §5. Key numbers: `sim_ms_clutter_burst` card_err +3.48 → target
≤ +2.51 (beat MHT), ideally ≈ +0.9 (radar-only). Guard: harbor/dense_clutter no
regression, philos byte-identical default-off, HAXR card_err down with
lifetime/gospa_missed flat.

**Joint (λ_C, miss-P_D, λ_birth) analysis.** A does **not** touch `λ_C` (avoids the
cancellation and the wrong-direction cost) or the miss path (avoids the philos brake).
It multiplies the *existing* `r_new` — so it composes with both `birth_existence_target`
(coverage_land) and fixed `λ_birth` (`_land`). Risk vector is ADR-0002 over-delete, not
the miss coupling.

### Candidate B — Compound-K / negative-binomial count-adaptive birth discount  ★ recommend build

**Math.**
Model the per-scan *unassociated* return count `N` as gamma-modulated Poisson
(negative-binomial), matching the compound-K generator (`13-clutter-and-detection.md`
§9). Estimate the two moments online (EWMA of `N` and of `N²` → mean `μ̂`, variance
`σ̂²`; over-dispersion `φ̂ = σ̂²/μ̂ ≥ 1`). On a scan whose count is high *relative to what
the NB predicts is normal clutter*, discount births:
```
   c_od = 1 − min(1, P_Poisson(≥N | μ̂) / P_NB(≥N | μ̂, φ̂))          (clamped)
```
i.e. when the NB tail says "this many returns is unremarkable for clutter" but the
Poisson tail says "surprising", suppress the surprise-driven births. Fold into the same
`birthScale`. Optionally regionalise (per azimuth wedge) to match the generator's 24
wedges, staying claim-agnostic.

**Assumptions.**
- The damaging over-count on clumpy scans comes from the tracker treating an
  over-dispersed clutter count as evidence of new targets (the compound-K failure mode,
  `13-...md:244-251`).
- Over-dispersion is estimable online without the fragility that got the cluster process
  shelved in §10 — the campaign's job is to *test* this, honestly.
- Diffuse HAXR clutter is *over-dispersed* (not merely uniformly dense). If HAXR clutter
  is high-but-Poisson, B does nothing on HAXR (honest limitation).

**Rationale vs alternatives.**
- vs candidate A: B localises nothing but catches the *moving* compound-K clumps A
  cannot; it is the direct answer to the discriminator's mechanism (1).
- vs re-deriving the PMBM update with NB clutter cardinality: that is a deep hot-path
  change threatening KEEP byte-identity and determinism. B keeps the Poisson update and
  expresses the NB insight as a *birth discount* — far smaller blast radius, still on
  the non-cancelled channel.

**What to test.** §5. B's headline is the diffuse arm: HAXR card_err ~48.8 delta, and
the compound-K background component of `sim_ms_clutter_burst` (isolated by turning the
burst off in a Phase-B fixture variant, or by the control arm in §1.3).

**Joint analysis.** B, expressed as a `birthScale` factor, is **not** cancelled and does
**not** touch `λ_C` or the miss path. If instead expressed as an effective `λ_C(scan)`
(the naive form) it would be cancelled on `_coverage_land` — so B is specified as a
`birthScale` discount, not a `λ_C` modulation. This is the crux that keeps B alive on
the deployed config.

### Candidate C — clutter-aware PPP-birth (the parked formulations)  ✗ do not build as parked

Formulation 1 (coverage check) is shipped-off, marginal (philos −4.4 / sc4_anchored
+2.3), and **dead code under `adaptive_birth`** (touches the empty PPP). Formulation 2
(prior-mass gate) was never built and also operates on the empty PPP. Both are inert on
deployment (§1.2). Their intent — discount births where clutter already explains the
return — is **subsumed by A/B on the live `birthScale` channel.** Recommendation:
retire C as a distinct candidate; note the subsumption in `pmbm-design.md`.

### Candidate D — per-sensor `λ_C`  ✗ already shipped; used as substrate, not a build

Backlog #8 (DONE 2026-06-13): PMBM already reads per-`(sensor,model)` `λ_C` via
`paramsFor(z)` (`PmbmTracker.cpp:264-269, 437-439`), live on both sim and philos. It is
dimensionally correct across mixed sensors (§Appendix). But on `_coverage_land` it is
cancelled for births (§1.1). So D is not a fresh lever — it is the *substrate* A/B build
on (A's grid is a `paramsFor`-style spatial resolver; both stay position-domain where
`λ_C` units are `m⁻²`). No new build; ensure any experiment keeps a per-sensor detection
table wired (PMBM ignores the scalar `desc.clutter_density` — a wiring asymmetry to
avoid).

---

## 4. The pick, and why

**Build A (primary) + B (paired), both as default-off `birthScale` factors, position-
domain, claim-agnostic; sequence A first.**

- A and B attack the two *distinct* mechanisms the discriminator was purpose-built to
  separate (§1.3): A the persistent burst, B the over-dispersed background. Picking one
  would leave half the instrument unaddressed.
- They share the delivery channel (`birthScale`), so B's marginal cost given A is small
  (one more factor + an online moment estimator), fitting the ~2-day Phase B.
- Sequencing A first is fail-fast: A alone is the likelier discriminator win (the burst
  dominates card_err); if A caps out at the channel-reach ceiling (§5.0) or resists HAXR
  (diffuse), B's orthogonal mechanism is the measured fallback. If A already wins both
  sim and HAXR, B can be deferred — an acceptable "one candidate sufficed" outcome.
- C and D are excluded with reasons above (inert / already shipped).

Config surface (Phase B, all default-off ⇒ byte-identical): one new `PmbmTracker::Config`
knob group (e.g. `use_clutter_birth_prior`, a `ClutterBirthPriorParams` struct with
`cell_size_m`, `tau_s`, `c_max`, `hard_gate`, and a `signal` selector for A1/A2/B).
Note: a field on `PmbmTracker::Config` **escapes** the integration-guide drift-guard
(it only matches `struct <Name>Config`), but a new `ClutterBirthPriorParams` struct does
**not** trip it either (it matches `…Config` only) — so the integration-guide entry is a
manual obligation either way (§6).

---

## 5. Experiment matrix (Phase C order — fail-fast on the cheap gates)

Prereq (both fixtures are git-ignored, regenerate + verify checksums BEFORE trusting any
baseline): sim_multisensor via `python -m generator.generate` (venv, `trafficgen==0.9.0`,
`numpy==2.5.1`; verify `sim_ms_clutter_burst` sha256 `a4ecaba3/…`), and HAXR
`kattwyk_08_dec50_w285.csv` (md5 `304cdeb8e81f03cbddb52d629fab22a9`). Diff with
`tools/bench_diff.py --all-metrics` (the default metric set omits card_err/gospa_false/
gospa_missed/lifetime — they never appear without `--all-metrics`).

**5.0 Phase-B entry probe (before the full build): the birth-vs-confirm race.**
On `sim_ms_clutter_burst`, instrument how many burst phantoms cross `confirm_threshold`
in the first ~3 scans after `t=120` (before any birth prior could have learned). This
bounds the *ceiling* of a birth-side fix. If most phantoms confirm in ≤2 scans,
birth-suppression alone cannot reach the +0.9 target and we report that up front (the
honest "birth channel insufficient" finding) rather than after building.

**5.1 The discriminator (cheapest, first).**
| arm | metric | baseline | success |
|---|---|---|---|
| `sim_ms_clutter_burst`, `_coverage_land` + A | card_err | **+3.48** | **≤ +2.51** (beat MHT); good ≈ **+0.9** |
| same, real vessels | lifetime / breaks | 0.995 / 0.0 | **unchanged** (no real-track damage) |
| **control**: declare uniform λ_C=6e-8 | card_err | +3.48 | isolates mean-credit from A's structure-credit |
| other 5 sim (`crossing/headon/overtaking/ais_dropout/anchored_camera`) | card_err, ospa | table in battery doc | **no regression** |

If A does **not** separate from uniform-λ here, that is a gate finding → stop-and-report
(the instrument failed to reward the mechanism), per the ticket.

**5.2 harbor_complete_truth + dense_clutter_datum (standing clutter gates).**
- `harbor_complete_truth` (all configs): card_err/gospa_false **unchanged-or-better**;
  adapt_k3 within the Phase-0 band guard.
- `dense_clutter_datum`: the **no-death-spiral** gate
  (`test_occupancy_detector_gates.cpp::DenseClutterDatumNoDeathSpiral`:
  `life ≥ life_land − 0.05`, `gospa ≤ gospa_land·1.15 + 1`). This is where the `1−r`
  spiral would surface — A/B's claim-agnostic signal must pass it.
- KEEP-over-deletion gates (`test_synthetic_clutter_ab.cpp`,
  `test_harbor_gate_scenarios.cpp`): shore over-count down **without** dropping real/
  near-shore targets.

**5.3 philos KEEP (safety ABSOLUTE).**
- Default-off: **byte-identical** on `_land` + `_coverage_land` (188 rows, 0 moved) —
  Class-A. Any movement beyond fp-noise = stop-and-report.
- Default-on: `tracks_on_keep` flat (`test_philos_sunset_labels` /
  `_close_approach_labels`), no over-delete (card_err must **not** flip negative — the
  fed-cmap failure). The philos bench GOSPA is AIS-as-truth (circular) → the
  un-gameable check is `tracks_on_keep` + `lifetime_ratio` byte-identity, not GOSPA.

**5.4 HAXR decimated (+ raw if cheap) — the real-data payoff.**
- `kattwyk_08_dec50_w285`, `_coverage_land`: card_err ~**48.8** delta, reported beside
  **lifetime_ratio** (must not fall) and **gospa_missed** (must not rise) — the "real
  vessels untouched" guards. Honest expectation: **B** is the candidate with a chance
  here (diffuse, over-dispersed); **A** likely inert if the HAXR over-count is not
  spatially concentrated (increment-8: diffuse, occupancy cut <3%). A null on HAXR is a
  documented, acceptable outcome.

**5.5 Cost (the perf arc must not be undone).**
- Wall + RSS + per-scan `scan_proc_ms_{p99,max}` deltas on the HAXR workload. A grid
  update per scan + a per-cell query per measurement must stay well inside the 148 ms
  scan interval (current worst raw scan 76.7 ms). A map that doubles scan cost must say
  so. Single-run > ~5 min = stop-and-report.

**Predicted failure modes (stated up front).**
1. A hits the **channel-reach ceiling** — suppresses new burst births but the early-
   confirmed ones cap the win above +0.9. (Probe 5.0 quantifies this before building.)
2. A **over-deletes** on complete-truth (dispersion/claim signal too weak to separate a
   real anchored vessel from a clump) → fails 5.2/5.3. Mitigation: soft cap + dispersion
   feature + gating against complete-truth, not philos.
3. B's **NB estimation is fragile** (the §10 reason it was shelved) — noisy `φ̂` →
   erratic discount. Mitigation: heavy EWMA smoothing, clamp, default-off.
4. **HAXR resists both** (diffuse + Poisson-ish) → honest "not better on HAXR".
5. A `1−r`-flavoured signal creeps back in and reopens the **dense_clutter spiral** →
   caught by 5.2 run early.

---

## 6. Docs plan (keep-in-sync obligations, per CLAUDE.md)

- **Learning `docs/learning/13-clutter-and-detection.md`:** extend §3.3 (spatial map)
  with the claim-agnostic, `birthScale`-not-`λ_C` design; if B ships, **flip §10's
  "non-homogeneous cluster process — not picked"** to "picked" with measured evidence;
  add a new `fig_*()` in `figures/generate.py` (never hand-edit PNGs). **Glossary
  `19-glossary.md` gap:** add `compound-K`, `negative-binomial`, `over-dispersion`
  (currently absent).
- **Algorithm docs:** new `docs/algorithms/clutter-birth-prior.md` (copy the four-part
  structure of `static-obstacle-birth-prior.md`); cross-update `association.md §6`
  (clutter map) and `pmbm-design.md §3.2/§9` (the `λ_C`-cancels algebra + the new
  `birthScale` factor); add the A/B result + control arm to `synthetic-clutter-bench.md
  §4` and the battery baselines doc.
- **Integration guide:** manual entry in the §10 appendix (Table B for a
  `PmbmTracker::Config` flag, Table C for `ClutterBirthPriorParams`) — the drift-guard
  will **not** remind us (it only fires on new `struct <Name>Config`), so this is a
  self-discipline obligation.

---

## 7. Questions for the arbiter (Checkpoint 1)

1. **Scope:** approve building **A + B** (sequenced), or **A only** first (defer B until
   A's discriminator + HAXR results are known)? A-only is the smaller, safer bite.
2. **Signal for A:** prefer **A1** (unclaimed-return area-rate) or **A2** (dispersion
   excess) as the first feature, or build A2 (the stronger real-vs-clutter discriminator)
   directly? A2 is more work but is the principled answer to the ADR-0002 over-delete
   risk.
3. **HAXR expectation:** accept up front that HAXR may null (diffuse clutter), so the
   discriminator + harbor + KEEP are the primary success gates and HAXR is a "report,
   don't require" arm? Or is a HAXR card_err win a hard requirement (which would raise
   the risk of a null-campaign close)?
4. **Hard-veto:** allow candidate A a hard birth-veto (like `land_birth_hard_gate`) for
   strong-clutter cells, or soft-suppression only (lower over-delete risk, weaker on the
   burst)?
5. **Phase-0 nudge:** leave `adapt_k3` on the `lambda_birth` cliff behind the band guard
   (as shipped), or additionally nudge `lambda_birth` to 1.1e-5 (the target-preserving
   plateau)? (Band guard alone is sufficient; the nudge is optional.)

---

## Appendix — load-bearing citations (all file:line-verified)

- `λ_C` cancellation: `PmbmTracker.cpp:500-507, 1082-1083`; `pmbm-design.md:791-803`.
- `birthScale` third-factor template: `PmbmTracker.hpp:981-1000`;
  `static-obstacle-birth-prior.md:64-91`.
- PPP empty under adaptive: `PmbmTracker.cpp:1384, 1437-1439, 202-233`.
- Cmap inert = never fed: `PmbmTracker.cpp:1673-1674, 1783`; `PmbmTracker.hpp:562`;
  `Config.cpp:976-985`.
- `1−r` death spiral: `test_philos_cluttermap_ab.cpp` (dense_clutter 0.90→0.26);
  eval-log `1915-1958`, `7226-7263`.
- Discriminator structure: `clutter.py:48-82`; `scenarios.py:77-87`;
  `SimMultisensorScenarioRun.cpp:105-124`. Baselines: `2026-07-06_sim_multisensor_battery.md`
  (MHT +2.51 / PMBM +3.48 / radar-only +0.93).
- card_err = Confirmed count (mass ≥ 0.5): `BenchRunner.cpp:246-262`;
  `Metrics.cpp:128-130`.
- Miss-P_D brake by config: `PmbmTracker.cpp:649-655` (legacy) vs `:684,1006`
  (sensor-activity bypass); `Config.cpp:1170-1176` (coverage_land).
- KEEP set + gates: `Config.cpp:652, 1142`; `test_philos_sunset_labels.cpp`;
  `test_occupancy_detector_gates.cpp::DenseClutterDatumNoDeathSpiral`.
- HAXR fixture: `tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv`
  (md5 `304cdeb8…`), card_err ≈ 48.76, lifetime 0.104 (`2026-07-05_pmbm_runtime_frontier.md`).
- Doc landing spots: `13-clutter-and-detection.md` §3.3/§9/§10; `association.md §6`;
  `pmbm-design.md §3.2/§9`; `integration-guide.md` §10 Tables A/B/C; drift-guard
  `tests/docs/test_integration_guide_config_coverage.cpp:54-61,103`.
