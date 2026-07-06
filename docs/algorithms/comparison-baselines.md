# Comparison baselines — what we are proving

This document pins what navtracker is being compared *against*, what we
*claim*, and what evidence each claim still needs. It is the **north
star for prioritisation**: every "what should we work on next?"
question is answered here, against the three claims below, not against
whichever eval-log entry happens to be the most recent.

This is a **living document**. Update it when a claim closes, a new
baseline emerges, or the priority order changes. New eval-log entries
should tag the claim they advance (e.g. `[Cl-2]`) so a reader can
trace which measurements served which claim without re-reading the
whole log.

Related:
- `sota-roadmap.md` — enumerates *techniques* drawn from literature.
  This doc enumerates *what we are comparing against and what we
  claim*. They cross-reference.
- `evaluation-log.md` — observations, with claim tags going forward.

---

## The two SOTA axes (frames the three claims)

Multi-target tracking has two distinct notions of "state of the art",
and they don't move at the same pace:

- **Deployment-SOTA** — what is actually fielded in commercial /
  defence systems (naval surveillance, ATC, ground-based radar). As
  of 2024–2025 this is dominantly **IMM + TOMHT** (track-oriented
  multi-hypothesis tracking with interactive multiple-model motion
  inside), with the inner filter being EKF, UKF, or cubature KF as
  an implementation detail driven by nonlinearity severity. Reference
  bibles: Bar-Shalom *Estimation with Applications to Tracking and
  Navigation* (2001), *Tracking and Data Fusion* (2011); Blackman
  *Multiple Hypothesis Tracking* (1999, 2004 update). Public
  references for Lockheed (AEGIS/CEC), Saab, Thales cite TOMHT
  variants.

- **Academic-SOTA** — what the model-based-tracking literature
  considers the frontier. As of 2024–2025 this is the **Random Finite
  Set (RFS) family** (PHD / CPHD / LMB / GLMB / **PMBM**), with
  trajectory-PMBM (García-Fernández et al. 2020) the leading variant
  for trajectory estimation. Williams 2015's "Marginal multi-Bernoulli
  filters" paper shows MHT and JIPDA are *special cases* of marginal
  multi-Bernoulli filters within RFS — so PMBM does not replace
  MHT/JIPDA, it **strictly generalises** them. Commercial adoption of
  RFS-class trackers is rare as of 2025.

The three claims sit on these axes as follows:

- Cl-1 (Helgesen 2022) is a specific *recent academic baseline* in
  the JIPDA-class — neither the deployment-SOTA nor the academic
  frontier, but a published, peer-reviewed reference on the dataset
  we care about.
- Cl-2 (IMM+MHT canonical) is **navtracker's position on the
  deployment-SOTA axis**.
- Cl-3 (PMBM) is **navtracker's position on the academic-SOTA axis**.

---

## Cl-1 — Beat or match the published baseline: Helgesen 2022

**Their tracker.** "Visibility Interactive Multiple Model JIPDA
(VIMM-JIPDA)" from Brekke et al. 2021, simplified in this paper to a
**single constant-velocity mode** (their §4.1; the multi-mode VIMM
formulation is the full version, deferred). Filter: measurements
converted to Cartesian to bypass the EKF nonlinearity, then linear
Kalman update — **functionally EKF-equivalent**. Per-sensor R; per-
sensor and range-dependent (P_D, λ_C) (their §4.3). No mention of
PMBM / RFS in their problem framing — they treat JIPDA as the modern
endpoint of the JPDA family.

**Reference.** Helgesen, Vasstein, Brekke, Stahl, *Heterogeneous
multi-sensor tracking for an autonomous surface vehicle in a littoral
environment*, Ocean Engineering 252 (2022) 111168. Dataset:
AutoFerry, published alongside. Metrics: GOSPA (c=20m, p=α=2),
posRMSE, Break.L (mean break length), ANEES.

**The claim.** navtracker either matches or beats Helgesen 2022 on
their own benchmark and metrics.

**Apples-to-apples honesty.** Two axes of difference:

1. **Association class.** Their JIPDA-class vs our MHT-class. A
   strict apples-to-apples test requires us to run a JIPDA
   configuration; we don't have one. Building it = SJPDA + JIPDA on
   our existing JPDA branch (sota-roadmap §2). Optional, deferred.
2. **Motion model.** Their single CV mode vs our CV+CT IMM.
   This is a **navtracker advantage**, not a gap — richer motion
   model is strictly more capable for maneuvering targets. Call it
   out in any comparison summary; do not hide it.

The cleanest publishable comparison position is: "We compare
navtracker (MHT-class, CV+CT IMM) against Helgesen 2022's published
VIMM-JIPDA on the same dataset and metrics. The result tells us
whether the MHT-class advantage outweighs the JIPDA-class advantage
on AutoFerry data, and whether CV+CT IMM materially helps. This is
*not* class-controlled — for that, we would need our own SJPDA+JIPDA
implementation (deferred)."

**Where we stand today.** From `evaluation-log.md`:

| Run | env-1 GOSPA RMS | env-2 GOSPA RMS | Verdict |
|---|---:|---:|---|
| Paper (VIMM-JIPDA) | **20.4** | **31.0** | reference |
| navtracker canonical (no AIS) | 43.4 | 33.9 | env-1 ×2.1 worse; env-2 +3m worse |
| navtracker canonical (truth-AIS injected) | 20.6 | 7.1 | env-1 even, env-2 ×4.4 better |
| navtracker `_biascal` (truth-AIS) | **19.6** | **7.2** | env-1 marginally better, env-2 ×4.3 better |

**Read.** The truth-AIS injection is the apples-to-apples calibration
condition Helgesen used (they had RTK-GNSS truth available; ours is
synthesized at σ=5m, *less precise* than their RTK so the comparison
is not biased in our favour). Under that condition the canonical
*beats* the paper on env-2 substantially and is even on env-1.
Without the anchor — deployment without AIS-quality cooperative
target data — env-2 is roughly even but env-1 is far worse.

**Status.**
- "Beat under apples-to-apples calibration condition" ✅ (both envs).
- "Beat in cold deployment without anchor" ❌ (env-1).

**Work that closes the unanchored env-1 gap.** Identical to Cl-2's
open work below — env-1 BOT pathology and ID stability.

**Class-controlled extension (deferred).** Build SJPDA + JIPDA on our
JPDA branch; bench against the paper as a class-matched comparison.
Estimated half-day + 2–3 days. Defer until the headline-claim deltas
are settled. Result not load-bearing for the headline claim (the
canonical already beats the paper under truth anchor) — it answers
the orthogonal question "is the association class load-bearing?".

---

## Cl-2 — Beat or match the deployment-SOTA: IMM + MHT (TOMHT)

**The reference class.** What is actually deployed in commercial /
defence multi-target tracking as of 2024–2025: **IMM with CV/CT (and
sometimes CA) modes wrapped in TOMHT** with N-scan pruning and Murty
k-best hypothesis enumeration. Inner filter is EKF, UKF, or cubature
KF — choice is an implementation detail driven by sensor nonlinearity
severity, not a settled "standard". Public references: Bar-Shalom
2001/2011, Blackman 1999/2004; commercial systems documented include
Lockheed AEGIS/CEC, Saab tracking systems, Thales BlueTracker. Active
2025 research extending the class: Q-IMM-MHT 2025 (Q-learning
adaptive model switching), hybrid track-before-detect for coastal
radar (Trondheim Fjord 2025).

**navtracker's position.** Our canonical `imm_cv_ct_mht` is **in
this class**: IMM (CV + CT modes) + TOMHT (IPDA+VIMM lifecycle since
2026-06-11, Murty k-best, N-scan pruning, Bhattacharyya cross-tree
merging, etc. — see `core/pipeline/MhtTracker.cpp` for the full
configuration surface). Inner filter is EKF. So we are **a
representative deployment-SOTA tracker today**, not a missing
implementation.

**Two open sub-questions about whether we're at deployment-SOTA
quality:**

- (a) Inner filter: EKF vs UKF vs cubature KF. We're EKF. Build
  `ukf_cv_ct_mht` to measure; if EKF and UKF differ by ≤1% on
  AutoFerry GOSPA / NEES, declare EKF the right choice (simpler,
  faster); else ship UKF as canonical inner filter. Sota-roadmap §3.
- (b) Lifecycle: our IPDA+VIMM existence/visibility (2026-06-11
  canonical) vs the older score-based (LLR/SPRT) lifecycle. Already
  ablated as `_mofn`; existence-based wins on misses/clutter,
  bit-identical on clean synthetics. **This question is closed**;
  IPDA+VIMM is our settled choice.

**The claim.** The deployment-SOTA class (IMM+MHT, represented by
our canonical) is either competitive with or better than the
published baseline (Cl-1), *and* we can say *when, where, and why*
one beats the other. The "when/where/why" is the load-bearing
scientific contribution beyond "we got a smaller number".

**Where we stand today.** Cl-1's table already answers the "better"
question for the headline. The "when/where/why" reading:

- **Where MHT-class wins**: env-2 anchored (×4.3 better GOSPA RMS).
  Mechanism: MHT's hypothesis tree handles close-spaced parallel
  targets without coalescence, which is the dominant env-2 failure
  mode in JPDA-class trackers. (To strictly attribute this to MHT
  vs to our richer IMM, we need the deferred SJPDA+JIPDA from Cl-1.)
- **Where MHT-class loses**: env-1 unanchored — sc5/sc6 BOT range
  collapse (Jacobian-rank problem the canonical estimator can't fix;
  see eval-log 2026-06-16 per-target diagnosis). The earlier
  "sc13_anchored NEES = 69 residual" turned out to be a
  metric-reporting artefact rather than a filter bug — see eval-log
  2026-06-19 (later 2): median NEES is 0.37, p95 is 7.71,
  coverage_95 is 0.94; the mean was dragged by ~1% extreme samples
  from metric-side ID-switch reassignment events. Closed as no bug.
- **Cooperative-fleet deployment (step 5 in plan)**: cooperative
  GNSS is an *additional* anchor source — it sits alongside AIS,
  not in place of it. Real deployments will commonly have both
  (community AIS for non-cooperative traffic, cooperative GNSS
  from fleet partners). Under either-or-both, Cl-2 inherits the
  truth-AIS column from Cl-1's table, i.e. cleanly beats the paper.

**Open Cl-2 work (ranked):**

1. ~~sc13_anchored NEES = 69 residual.~~ **CLOSED 2026-06-19 as no
   filter bug** — was a metric-reporting artefact (mean dragged by
   ~1% extreme samples from ID-switch reassignment events; median
   0.37, p95 7.71, cov95 0.94 are all healthy). Bench harness now
   emits `nees_median` + `nees_p99` alongside `nees_mean`; eval-log
   headline convention is `(median, p95, cov95)` first. See
   2026-06-19 (later 2).
2. **env-1 unanchored gap to Helgesen** (formerly framed as "env-2
   BOT pathology"; re-scoped 2026-06-19). The honest mechanism is
   **filter over-confidence on re-confirmed tracks after brief
   misses**, not BOT range collapse — median NEES on sc3 unanchored
   is 20 (expected ~1.4 for χ²₂), p95 is 417. `_bearguard` was
   measured and gives only 0.6–6.8% GOSPA improvement and no
   anchored effect — *small but no regression, not load-bearing*.
   `_recapture` gives 10–36% GOSPA wins but at catastrophic
   lifetime cost (sc17 0.90 → 0.39) — *not shippable as canonical*.
   The real fix candidates *inside* Cl-2's stack were (a) IPDA+VIMM
   lifecycle re-tuning (looser demote, longer ever-confirmed memory)
   and (b) track-spawn init-covariance prior widening. **Both
   measured and REJECTED 2026-06-20** (bench
   `cl25_life_20260620.csv`): cardinality bloat broadly regressed
   autoferry-unanchored GOSPA +4.3% mean and autoferry-anchored
   GOSPA +17.1% mean (sc3_anchored +56%). The over-confidence
   mechanism lives in the joint existence + association coupling,
   not in lifecycle thresholds or init priors — a standalone
   parameter tweak cannot reach it. A third path sometimes confused
   with a Cl-2 fix is "JIPDA-class lifecycle", but that is **not** a
   fix inside our stack: JIPDA replaces TOMHT at slice 5
   (`docs/learning/22-tracker-stack-alternatives.md`), so building
   JIPDA is the Cl-1 sibling pipeline (GNN/JPDA branch), not a
   stackable change to Cl-2. **Cl-2 #2 is deferred indefinitely**
   in favour of Cl-3 (PMBM), which collapses slices 4-6 into one
   RFS recursion and makes the over-confidence question moot.
3. ~~UKF / cubature KF inside IMM~~ **Closed 2026-06-20 as
   SHIPPED.** Measured the UKF inner filter against EKF on
   cl23_ukf_full_20260619.csv (29 scenarios × seed 0). Autoferry
   unanchored — the Cl-2 #2 regime — gave **9/9 GOSPA wins, mean
   −12.3%** (sc17 −20.5%, sc22 −21.7%, sc3/4/6 all −14 to −16%);
   philos −4.6%. Anchored flat (no regression). Synthetic
   regressed +5.7% mean (linear-CV scenarios where EKF is exact;
   bounded ≤+13%). Promoted: `makeImmCvCt` now uses
   `use_ukf=true`; the EKF stack is preserved as
   `imm_cv_ct_mht_ekf` ablation. Sub-question (a) of Cl-2 closed
   in UKF's favour. See eval-log "Cl-2 #3 close-out".
4. ~~EO/IR R tightening from step-2 NIS finding~~ **Closed 2026-06-19
   as REJECTED.** Measured 0.0925 → 0.06 against the gated canonical;
   env-2 anchored GOSPA regressed +18-88% (sc16/17/22), RMSE up to
   +88%, env-2 unanchored NEES p99 catastrophic. Mechanism: bias
   estimator shrinks innovations on anchored runs → α̂ looks "loose"
   even when R matches the physical sensor noise floor (~0.088 rad
   empirical residual). α̂-driven R tightening is not safe on stacks
   with online bias correction without a `nobias` cross-check.
   See eval-log "Cl-2 #4 close-out".

---

## Cl-3 — Land PMBM as the academic-SOTA endgame

**The reference.** PMBM (Poisson Multi-Bernoulli Mixture; Williams +
García-Fernández + Granström, ~2015) and trajectory-PMBM
(García-Fernández et al. 2020) are the leading model-based
multi-target tracker in the academic literature as of 2024–2025.
Active research stream: multiple-model trajectory PMBM 2025, GGIW
PMBM for extended targets 2025, trajectory-measurement PMBM 2025,
PMBM smoother variants. Reference for direct head-to-head
comparison: "Systematic Analysis of the PMBM, PHD, JPDA, and GNN
Filters" 2022 — PMBM is consistently the top performer.

**Crucial lineage (Williams 2015).** "Marginal multi-Bernoulli
filters: RFS derivation of MHT, JIPDA, and association-based MeMBer"
shows MHT and JIPDA are *special cases* of marginal multi-Bernoulli
filters within the RFS framework. So PMBM does not *replace* Cl-1
and Cl-2 — it **strictly generalises** them. Landing PMBM in
navtracker therefore subsumes the JIPDA-class and the MHT-class
question at once.

**The claim.** PMBM, dropped in as a third `IDataAssociator` or as a
parallel tracker pipeline, **beats or matches both Cl-1 (Helgesen)
and Cl-2 (our IMM+MHT canonical) on autoferry GOSPA RMS at
acceptable runtime cost.** If PMBM is *not* meaningfully better
than Cl-2, that is also a publishable finding: "our IMM+TOMHT
implementation is competitive with state-of-the-art RFS on
autoferry-class problems; here is the data."

**Why this is a real contribution even if Cl-2 already beats Cl-1.**
PMBM is rare in deployed systems. Most published PMBM work is in
academic codebases (MATLAB, Python). Landing a tested, deterministic,
C++ PMBM in a hexagonal-architecture multi-sensor tracker, benched
against both a published baseline (Cl-1) and a deployment-class
tracker (Cl-2) on a real maritime dataset, is the kind of result
that has not been done. It is the bridge between the academic
frontier and a deployable system.

**Where we stand today.** Greenfield. Nothing in `defaultConfigs()`,
nothing in `core/association/`. Estimated effort: large
(weeks-to-months). Treat as a multi-phase milestone:

1. **PMBM as `IDataAssociator` for the JPDA-style pipeline.** Read
   García-Fernández et al. 2018 + 2020 trajectory-PMBM papers; build
   a simulation-only first version against synthetic crossing /
   parallel_targets / dense_clutter scenarios. Validate against
   Williams 2015's MHT-as-special-case derivation for sanity.
2. **PMBM applied to autoferry.** Bench on the full matrix; compare
   GOSPA RMS to Cl-1 and Cl-2.
3. **PMBM as canonical, if (1) and (2) prove the gain holds at
   acceptable runtime.** Otherwise document as "evaluated, not load-
   bearing for our deployment, here is why".

**Prerequisites.** Cl-2 #1 and #2 stable (so the comparison floor is
honest — comparing a PMBM-stripped-of-bugs against an MHT-canonical
that still has documented NEES regressions is not a fair fight).
Cl-1's SJPDA+JIPDA branch is optional — useful as a fourth tracker
class in the comparison, not required for Cl-3 itself.

---

## How the three claims sit together

```
                           Cl-3 PMBM (academic-SOTA endgame, RFS family)
                              │
                              │ strictly generalises (Williams 2015):
                              ▼
              ┌───────────────┴───────────────┐
              ▼                               ▼
     Cl-2 IMM + MHT                Cl-1 Helgesen 2022
     (deployment-SOTA,             (JIPDA-class published baseline)
      our canonical today)

  Claim 1: Cl-2 beats Cl-1 on AutoFerry, here when/where/why.
  Claim 2: Cl-2 is at deployment-SOTA quality (open sub-questions:
           inner-filter EKF/UKF; lifecycle answered).
  Claim 3: Cl-3 beats both, with acceptable runtime — or, if it
           does not, we publish why.
```

---

## Decision rule for prioritisation

Every proposed task answers three questions before it gets queued:

1. **Which claim does it advance?** (Cl-1, Cl-2, or Cl-3.)
2. **By how much?** Quantitative if measurable today against an
   existing bench scenario; qualitative-with-rationale if not.
3. **At what cost?** Rough engineering time. If the task is
   open-ended (research / spec / paper review), say so.

Tasks that don't tag to a claim are **side quests**. They may still
be the right thing to do — bug fixes, build-system work, refactors
that unblock something tagged — but they're labelled as such, not
quietly promoted to "the next step".

When a session ends without progressing any tagged claim, the eval-
log entry says so explicitly. Random walks happen; calling them out
when they do is the prevention mechanism.

---

## Current open work, mapped

| Item | Claim | Expected delta | Cost | Status |
|---|---|---|---|---|
| ~~sc13_anchored MHT NEES = 69 residual~~ | Cl-2 #1 | Was misdiagnosed; filter is fine, mean was tail-dragged. Closed as no-bug 2026-06-19; bench now emits nees_median + nees_p99. | n/a (done) | **closed** |
| Cl-2 #2 scoping: bearguard small, recapture not shippable (lifetime cost) | Cl-2 #2 partial | Scoped 2026-06-19; mechanism is filter over-confidence post-miss, not BOT. Real fix is lifecycle / init-cov; deferred. | scoping done | partial / deferred |
| ~~Cl-2 #2 (a)+(b): lifecycle re-tune + init-cov widening~~ | Cl-2 #2 | Measured 2026-06-20: cardinality bloat regressed autoferry-unanchored mean GOSPA +4.3% and anchored +17.1% (sc3_anchored +56%). Over-confidence is in joint existence+association coupling — not reachable by lifecycle/init-cov tweaks. Cl-2 #2 deferred indefinitely; PMBM (Cl-3) makes it moot. | n/a (rejected) | **closed** |
| ~~UKF / cubature KF inside IMM~~ | Cl-2 #3 | **Shipped 2026-06-20.** Autoferry unanchored mean GOSPA −12.3% (9/9 wins); philos −4.6%; anchored flat; synthetic +5.7% bounded (linear-CV regime). Promoted to canonical; EKF preserved as `imm_cv_ct_mht_ekf` ablation. | n/a (done) | **shipped** |
| ~~EO/IR R tightening (step-2 finding)~~ | Cl-2 #4 | Measured 2026-06-19: env-2 anchored GOSPA +18-88%, RMSE catastrophic. α̂ analysis was misleading on stacks with online bias correction. | n/a (rejected) | **closed** |
| ~~Step 5 — Cooperative GNSS as additional anchor (alongside AIS)~~ | Cl-2 (deployment) | Wiring shipped 2026-06-19 (new `SensorKind::Cooperative`, anchor extractors + AIS-ARPA pair extractor recognise it). No bench delta — no scenario emits Cooperative measurements yet. Synthetic sweep filed as next-step. | n/a (done) | **shipped** |
| SJPDA on JPDA branch | Cl-1 (class-controlled extension) | Half of "is the association class load-bearing?" answer. | half-day (permutation collapse) | optional / deferred |
| JIPDA proper on JPDA branch | Cl-1 (class-controlled extension) | Other half. | 2–3 days | optional / deferred |
| PMBM as `IDataAssociator` (sim-only first) | Cl-3 #1 | First-cut PMBM; sanity-check against Williams 2015 MHT-as-special-case. | 1–2 weeks (literature + build + sim tests) | **shipped** (Phase 1–9, through 2026-06-23) |
| PMBM applied to autoferry | Cl-3 #2 | Headline measurement for Cl-3. | 1 week after Cl-3 #1 | **shipped** — `pmbm_adapt`: autoferry_unanchored GOSPA −42% vs MHT, anchored ≈ tied (+5%); philos regresses +19%. |
| ~~PMBM runtime + canonical-promotion call~~ | Cl-3 #3 | **Decided 2026-06-24.** `pmbm_adapt` is 7–16x faster than MHT on autoferry, 1.4x on philos. K=3+xparent is 10.7x slower on philos — drop from `defaultConfigs()`. MHT stays canonical (philos regression); document `pmbm_adapt` as the autoferry-class choice. | 1 day | **closed** |
| ~~PMBM coverage/visibility channel (Task 4)~~ | Cl-3 | **Measured 2026-06-29.** Honest per-duty-cycle surveillance miss + existence-neutral cooperative stale signal (`ISensorActivity`/`IStaleSignalSink`), replacing wrong per-blip miss + `idle_halflife`. Autoferry best-in-class (gospa 11.3/15.3, near-zero card-err, fewest id-switch, fewer knobs); philos regresses (gospa 153.6, card-err +108) — low p_D + re-detected shore returns = spatial, not temporal, problem. `imm_cv_ct_pmbm_coverage` opt-in, NOT canonical. Next: coastline/land-mask clutter suppression at birth. | done | **shipped (opt-in)** |
| ~~PMBM land/coastline clutter-prior (Task A)~~ | Cl-3 | **Measured 2026-06-30.** GeoJSON land-mask (`ILandModel`) suppresses births on/near shore (signed shoreline ramp; inland-only hard gate; scales birth intensity, not λ_C). On philos collapses the coverage over-count: card_err +107.9→+6.9, gospa_false 23750→3550, gospa 153.6→**73.1** — first honest/no-crutch PMBM config beating adapt (82.6) / bundle (112), near MHT (69.4). Autoferry inert (no coastline → byte-identical, no regression). birthtarget (48.5, wrong-math) still edges it on philos gospa (residual water clutter the mask misses). `imm_cv_ct_pmbm_coverage_land` opt-in. | done | **shipped (opt-in)** |
| Synthetic geometry breadth bench (Project E, Tasks 1–6) | Cl-2 / Cl-3 | **Measured 2026-06-30.** 5 geometry scenarios + 2 shore-clutter scenarios, perfect truth. PMBM beats MHT on `parallel_lanes_dense` (GOSPA −20%, id_switches 3.4→0, lifetime 0.883→0.975); equivalent on crossing_60/90 and convoy; slight PMBM edge on crossing_30 (GOSPA −7%). No geometry regression. Shore-clutter A/B: land ON collapses card_err +29→~0 and gospa_false 5811→≤1 on BOTH scenarios, real targets intact (lifetime 0.975) — clean perfect-truth confirmation of the philos land-model result. The near-shore validator quantified the model's boundary: under `coverage_land` the soft offshore band (`offshore_halfwidth_m`=50 m) is a **no-birth zone** (gate==target ⇒ any c>0 drops r_new below the phantom-birth gate), so a vessel within 50 m of shore does not initiate. Decoupling the gate (0.1→0.05) revives near-shore births but regresses philos (gospa 73.1→100.0, card_err +6.9→+36.2) by re-admitting water clutter — **rejected**; 0.1 kept, limitation accepted (near-land ops rare), validator reframed to a vessel 60 m offshore (no collateral suppression — passes). Principled near-shore protection without re-admitting water clutter remains open. [Cl-2: geometry breadth] [Cl-3: clutter realism / synthetic validation] | done | **shipped; <50 m no-birth zone documented** |
| PMBM `bundle_land` = correct-math + land prior (no coverage) | Cl-3 | **Measured 2026-06-30.** Adding the land prior to `imm_cv_ct_pmbm_bundle` (which runs the CORRECT misdetection math `dedup_miss_pd=true` and regressed philos to 112): philos gospa **112.0→59.5**, card_err +46.3→**−2.95**, gospa_false 11420→1580. Autoferry byte-identical to bundle (no coastline → land inert). **Best HONEST philos result to date** (correct math, no wrong-math crutch, no coverage machinery) — beats `coverage_land` (73.1) and **MHT (69.4)**; only the dishonest `birthtarget` (48.5) is lower. Land is the principled *replacement* brake for the one correct-math removes (works because the legacy path keeps `dedup_miss_pd` live, unlike `coverage_land` where it is inert). Shipped `imm_cv_ct_pmbm_bundle_land`. **Gate-1 (17 synthetic scenarios, measured):** dominates on shore clutter (gospa 7–10 vs 74–76), best on parallel_lanes_dense, ≈ MHT on clean geometry — but **REGRESSES on dense uniform clutter** (gospa 16.7 vs MHT 12.4, lifetime 0.64 vs 0.93). NOTE: the regression is a combination of bundle_land's flags; an isolation (flip only `dedup_miss_pd`) shows the miss-math's own dense_clutter effect is modest (gospa +1.3, lifetime *improves*) and the land prior simply doesn't address uniform clutter — see eval-log "Gate 1 … CORRECTION". **Verdict: workload-specific, NOT a universal default** — recommended config for **coastal / near-shore** deployments; general-purpose PMBM default stays `adapt`. Remaining caveat: single-seed real-data margins (Gate 2 / error bars still open). | shipped | **recommended for coastal; not universal default** |
| PMBM `imm_cv_ct_pmbm_land` = adapt + land prior ONLY (root-cause fix) | Cl-3 | **Measured 2026-07-01.** Root-caused the open-sea missed-target regression that disqualified `bundle_land` as a universal config. Single-knob isolation on `dense_clutter` (10 seeds): the drop is **`birth_existence_target=0.1` ALONE** (lifetime 0.823→0.590) — it pins every birth to r_new=0.1 regardless of λ_C, sinking real re-acquisitions to the emit floor so one miss kills them. The other two non-dedup bundle knobs are byte-identical/inert; `dedup_miss_pd` *helps* open-sea (0.874) but **explodes philos over-count** (card +17.5→+48 with land, +112 without) — the legacy per-return miss penalty is the load-bearing philos brake, so a universal config must keep it. **Fix = adapt + land model, no bundle knobs.** Results: shore win fully preserved by the land model alone (shore_open card 0.000 / gospa 9.77 == bundle_land); open-sea restored to adapt (lifetime 0.823, fixing bundle_land's 0.639); philos repaired (lifetime 0.030→**0.369**, gospa **63.1** — best honest, beats MHT 69.4/adapt 82.6; card +3.95). SAFE BY CONSTRUCTION: land inert without a coastline → byte-identical to `adapt` on all non-shore scenarios. Residual: open-sea 0.823 still < MHT 0.925 — STRUCTURAL K=1 GNN commitment (present in plain adapt), needs a PDA-style soft detected-branch update, not a knob. Shipped `imm_cv_ct_pmbm_land`, **supersedes `bundle_land` as the recommended general/coastal PMBM config**; bundle_land kept as the birth-brake ablation. Clutter map NOT used (persistent-spatial only; inert on uniform noise + inert under PMBM as wired). See eval-log 2026-07-01. | shipped | **recommended general/coastal default** |
| Close the residual open-sea K=1 gap (PDA soft detected-branch update) | Cl-3 | Open. `imm_cv_ct_pmbm_land` open-sea lifetime 0.823 trails MHT 0.925 due to per-scan winner-take-all (K=1) mis-assigning the real target to a gate-closer clutter return. PDA/marginal-association soft update on the DETECTED branch keeps K=1 (no flat-rep phantom leak, no anchored regression, no philos over-count) while defeating the state-pull-onto-clutter step. Raising K in the flat rep is NOT a safe drop-in (regresses anchored). **SHIPPED opt-in 2026-07-02** (`imm_cv_ct_pmbm_land_pda`, commit 68c845e): detected branch β-weights the winner with gated-but-unclaimed returns (moment-matched + spread; per-mode IMM); state-only, K/Murty/births untouched; unclaimed-only pool = no philos over-count. A/B (10 seeds sim + philos replay): dense_clutter lifetime **0.823→0.847**, extended-target/anchored over-count DROPS (harbor card_err −2..−4, gospa_false −330..−874), philos 63.13→63.08 / philos_radartruth 67.08→67.04 (flat), single-return unchanged, flag-off byte-identical (891 tests). Docs: pmbm-design §11 + learning ch.12 §12. **Promotion A/B measured 2026-07-02 on AutoFerry (18 replays) → NOT promoted; result is regime-split.** Open-water (env 1, n=5, the target regime) = mild win (gospa_missed −3.5, pos_rmse 13.51→12.74 all-5-better, id_switches 7.4→6.3); anchored (n=9) = flat (the one hard gate — no anchored regression); urban channel (env 2, n=4) = mild regression (gospa_mean +0.70, pos_rmse +3.2/+20 % all-4-worse, gospa_false +9.4) as unclaimed shore/dock clutter enters the β pool. Net canonical ≈ wash / slightly negative on accuracy. Caveat: AutoFerry has no coastline ⇒ land inert ⇒ this measures PDA in isolation (pessimistic for charted coastal). Real replay caught sim optimism (the harbor over-count *drop* was a hull-return artefact, doesn't generalise to shore clutter). **Stays opt-in.** **Land-aware PDA pool BUILT 2026-07-02** (`pda_pool_excludes_land`, config `imm_cv_ct_pmbm_land_pda_wateronly`): drop non-winner shore returns (ILandModel clutterPrior > gate) from the β pool so PDA softens vs water only. Unit-proven (`PmbmPdaLandAwarePool`), safe-by-construction (byte-identical off / no coastline). Initially bench-inert (A/B byte-identical on all 42 pre-existing scenarios; gate=0.0 diagnostic ruled out "gate too high") — root-caused: no fixture had a gated+unclaimed+shore pool member. **Sim-validated 2026-07-03** on `shore_clutter_transit` (parallel-to-quay, on-land clutter c=0.75): `_land_pda` pos_rmse 17.0 → `_wateronly` 8.6 (paired 10/10 seeds). **Real-data promotion gate MEASURED 2026-07-03 → HOLD (not promoted).** Sourced the REAL Trondheim harbour coastline from OSM (Kanalen/Ravnkloløpet canals, Piren datum, ODbL; `tests/fixtures/autoferry/trondheim_harbor.geojson`), wired it + the Piren datum onto AutoFerry (the loader never set `Scenario::datum` — the real reason it was chartless). Candidate `_wateronly` vs default `_land`: open-water win retained (pos_rmse −0.77), anchored flat, philos flat (63.08 vs 63.13) — but **urban regression NOT closed** (pos_rmse 15.67→17.77, +2.10; land-aware pool recovers only ~⅓). Root cause: real urban clutter is largely **in-water** (moored vessels / floating structures, clutterPrior<0.5) that a land mask can't flag. Sim proved the mechanism for on-land clutter; real geometry shows on-land is only part of the problem. **Stays opt-in; `_land` remains default.** Residual is an association/existence problem (β₀ miss term / confirmed-only softening / static-occupancy Stage 1b), not a land-mask one. See `docs/baselines/2026-07-03_promotion_decision.md` + eval-log 2026-07-03. | done | **shipped (opt-in); real-data HOLD — not promoted (in-water urban clutter)** |
| Static-obstacle branch — **Stage 1a: charted input** (ADR 0002) | Cl-3 | `StaticObstacle` chart input (ENC/S-101-aligned: CATOBS/WATLEV/VALSOU/depth/lit/virtual-AtoN + keep-clear buffer) as a vessel-birth prior *and* hazard output (`StaticHazardOutput`). Soft ramp + hard footprint core. Keep-clear proximity alarm (`StaticHazardEvaluator`). Safe-by-construction: model off → bit-identical. Field-validated (ADR 0002 prior-art). Plan: `docs/superpowers/plans/2026-07-01-static-obstacle-stage1.md`. **A/B measured 2026-07-02 (R5)** on `harbor_charted_pier`: `imm_cv_ct_pmbm_static` cuts card_err 11.64→7.43 and gospa_false 2362→1518 while real-target lifetime holds at 0.975 (uncharted uniform clutter is the residual — charts are a ~⅓ partial lever). | n/a (done) | **shipped 2026-07-01; measured 2026-07-02** |
| Static-obstacle branch — **Stage 1b: clutter-map→hazard reframe** (ADR 0002) | Cl-3 | Reframe the parked clutter-map primitive into an honest "persistent-unclaimed-return → static-occupancy" layer wired into PMBM (dominant-hypothesis 1−r labeling) that is **output as a hazard** (`StaticHazardOutput`, `is_charted=false`), not a hidden λ_C tweak. **MUST incorporate review tickets R2 (label from the true assignment, per-sensor, birth-channel-only influence) and R3 (extent is an interim discriminator; failure-mode gate scenarios) from `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md`. NOTE (measured 2026-07-02): R2's fix is correct but does NOT cure the dense_clutter regression (0.26 unchanged) — the spiral is the 1−r weighting of correctly-claimed returns under depressed existence; the cure is this stage's persistence+extent discriminator itself.** **INCREMENT-8 VERDICT (2026-07-05, eval-log 41344fc): persistence-based suppression is NEAR-INERT on real dense-harbor radar — <3% phantom cut across 9 HAXR station-hours (once slightly negative), effect survives the decimation confound (undecimated over-count 2x larger, detector still cuts <0.5%); smoking gun kattwyk_09: 10,041 suppression hits, card_err -1.18 of ~50. Confirms philos R4 on a 2nd geography: the classifier catches persistent structure, but the real over-count is DIFFUSE clutter elsewhere. Safety intact (lifetime_ratio + gospa_missed identical, real vessels untouched). The layer's earned value = hazard/presence channel + corroboration substrate (chart/camera/AIS/coop, all live), NOT cardinality. Redirect: (1) front-end plot extraction is the biggest measured lever (clustering decimation alone halves the over-count 113->51); (2) diffuse residual -> clutter/birth-model investigation (spatially-varying lambda_C under PMBM — design call, cross-scenario gates mandatory); (3) Doppler if the deployment radar exposes it (HAXR is amplitude-only) — kills moving/stationary clutter at the front end but CANNOT separate anchored vessel from pier (corroboration stays).** | ~1 week | **persistence suppression CLOSED (negative, 2 geographies); corroboration+hazard channel live; clutter-model redirect open** |
| Static-obstacle branch — **Stage 2**: evidential occupancy grid + stationary mode | Cl-3 | Full Dempster-Shafer / DOGMa occupancy grid (free/occupied/dynamic/**unknown**; same RFS family as PMBM) for uncharted-static detection — the honest fix for the ADR-0001 no-birth cliff (carry "unknown", don't blanket-suppress). Plus a **stationary IMM mode** (low-PSD / zero-velocity pseudo-measurement) so moored tracks stay tight and CPA gets a clean stationary-hazard flag + getting-underway transition. | multi-week subsystem | open (after Stage 1) |
| Anchored-vessel safety: sensor/chart-aware near-shore birth + stop→go test | Cl-3 | Close the ADR 0001/0002 near-shore hole: make land suppression **sensor/chart-aware** (radar-only + chart-coincident → suppress; camera/AIS or compact watch-circle return → birth a vessel; ADR 0001 A3). Add the missing regression test: target **starts anchored (v≈0) → holds → gets underway**, asserting it initiates, is not suppressed, and keeps a **stable track_id** through the transition. **Extended by the ADR 0002 amendment (2026-07-03, presence-over-classification): also require the misclassified case — a boat represented as a static hazard gets underway → promoted to a confirmed track within bounded latency; and the conservation invariant — birth suppression at a location is legal only if that location is emitted as a static hazard (suppression into nothing is the forbidden failure).** `imm_cv_ct_pmbm_land` already relaxes the birth cliff; this finishes it. Needs EO/IR or AIS near shore. **First step is review ticket R1 (pre-suppression existence floor = ADR 0001 A2): today land×obstacle soft suppression composed with `min_new_bernoulli_existence` silently re-creates a hard no-birth zone — see `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md`.** | ~1 week (test) + spike | open |
| Track-before-detect (TBD) channel for weak targets | Cl-3 | **Recorded 2026-07-01 (Herrmann et al. 2025 delta).** Add a track-before-detect estimator (à la Herrmann et al. 2025 IE-PHPMHT, arXiv:2508.16169) running on **raw, unthresholded** radar to catch weak / low-SNR / small non-cooperative targets that never cross the detection threshold and are therefore invisible to today's detection-only PMBM pipeline (`extract_radar.py` thresholds intensity ≥64 → DBSCAN → plots, discarding sub-threshold energy — we throw the weak-target information away *before* the tracker). Orthogonal to ADR 0002 (weak-target *detection* vs static-environment *mapping*); fits the hexagonal design as an additional estimator branch feeding the same track picture. Big: needs a new **raw-radar input path** (the production contract is parsed `Measurement`s / plots, not raw energy) + a second tracker, so it is a subsystem, not a knob. The closest published system (Herrmann 2025) pairs exactly this TBD channel with a PMBM point tracker — i.e. it is the one axis where they go beyond us. | multi-week subsystem | open (future candidate) |
| Static-branch review fixes R1–R7 (2026-07-02 deep review) | Cl-3 | Ticket set from the full review of the static-object approach (docs + code + bench + literature): R1 pre-suppression birth floor (anchored-vessel safety), R2 true-assignment clutter-feed labeling, R3 extent-is-interim gates + Dalhaug 2025 reference, R4 phantom-cluster attribution reconciliation + commit field-check charts, R5 measure Stage 1a (charted-pier A/B — closes the "no measurement" gap above), R6 boat-near-pier gate scenario, R7 housekeeping (soft_max clamp, GeoJSON validation, hazard-id hysteresis keying, datum-sink docs, ADR 0001 amendment). Prerequisite for R5/R6 RESOLVED: harbor truth-sort fix + re-baseline landed 2026-07-02 (3ee491f+3aa9c58; corrected baseline card_err +11.64 / lifetime 0.974). Full tickets: `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md`. **SHIPPED 2026-07-02**: all R1–R7 landed, plus a follow-up code-review round (10 findings — 4 real bugs fixed TDD incl. R1-floor obstacle-scoping [#3] + R2 merge-claim scoping [#2], 6 documented contracts; commits 780615e/3d52e44). A/B over all 11 pmbm configs × 22 sim scenarios = 0 focus-cell change (inert on default configs); 886 tests green; merged to master. | R7+R2 days; R1 ~1-2 d; R5/R6 ~1-2 d; R4 ~1 d | **shipped** |
| R8 — video-derived existence labels: fixture + label-aware philos metrics + binary gates (2026-07-03) | Cl-3 + measurement integrity | Output of the R4 video pass on `sunset_cruise` (zero AIS in clip; ≥4 real movers; the two strongest "phantom" clusters were real vessels — a ferry with a natural stop→go at t≈90 s, and a loiterer whose returns cease at t≈94 s while still in radar view). R8.1 label fixture (`tests/fixtures/philos/labels/*.csv`, existence/region labels — NEVER converted to `TruthSample`s: video truth is existence+window, not kinematics; fabricating positions would be circular and corrupt GOSPA). R8.2 label-aware philos decomposition: raw `gospa_false` unchanged, new columns `false_on_suppress` / `tracks_on_keep` (must not shrink — shrinking = deleting real vessels) / `false_unlabeled` — makes philos un-gameable. R8.3 binary canary gates + real-data stop→go regression fixture (t≈60–120 s, stable track_id through the transition; the ADR-0002-amendment rule-3 gate on real data). R8.4 detector input: the **observed-empty discriminator** (occupancy ceasing while in radar coverage = vessel; structure never goes quiet while observable) — pull Stage-2's unknown-vs-observed-empty distinction forward into 1b-ii as coverage-aware decay. Full ticket: `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md` §R8. Philos = partial truth from here on: AIS positions + existence labels, decomposed reporting, canaries binding; same label format applies to HAXR when labeled. **STATUS 2026-07-03: R8.1–R8.3 SHIPPED (9fde7ad — label loader + sunset_cruise labels + label-scored zero-AIS replay: tracks_on_keep=1633/false_on_suppress=3070/false_unlabeled=18295; all 4 KEEP canaries covered; stop→go ferry holds stable id, 2.89 m/s late). Detector bench gates shipped (cad424d): presence hard-gate + mover lifetime gate + classification-quality-reported (synthetic-clutter-bench §5.6); dense_clutter_datum death-spiral guard (no spiral); stop→go scenario. NEW §R8.6 (open): `close_approach` pass — KEEP_MIXED label semantics (dock/mooring regions: presence-gated, departures→tracks), real-data CPA/collision fixture (dinghy contact t≈61 s, radar to 17 m), KEEP-stress label-scored replay, R4 ceiling correction, coverage-descriptor decision (sector-capable from day one, disc = degenerate case; decay → AIS veto → chart corroboration order).** | R8.6/8.7 ~1–2 days | **R8.1–8.3 shipped; R8.6 items 1/4/5 SHIPPED 2026-07-03 (1d55d00): KEEP_MIXED label class + close_approach fixture + shared PhilosLabelReplay harness (sunset bit-identical: 1633/3070/18295) + close_approach KEEP-stress baseline under `imm_cv_ct_pmbm_land` (tracks_on_keep=5570 / false_on_suppress=0 / false_unlabeled=15182; both KEEP_MIXED canaries covered 0.14/1.40 m) + R4 ceiling correction in eval-log. R8.6 items 2 (CPA fixture) + 3 (sensor-doc note) deferred as independent task. R8.4 (coverage-aware decay, sector-capable descriptor on ScanObservation, plumbing scouted) + R8.7 (`ais_ferry_near` labels: berthed own-ship beside marina — KEEP_MIXED marina + 3 SUPPRESS pier bands; makes historical gospa honest retroactively; own-ship-stationary decay sanity case) + R8.8 (`car_carrier_near` DATA-INTEGRITY: 2020 bag topic mismatch → 26-row/constant-heading-0 fixture → all its world projections rotated; extractor fallbacks (`/filter/positionlla`, `/filter/quaternion` yaw) + fail-loud guard (>1 distinct heading, ≥1 Hz) + re-extract + redo this clip's R4 slice + THEN the occlusion video pass — the shadowing test for the decay sector model. Also: 2020/2021 campaigns have NO AIS → AIS-veto real-data validation moves to HAXR) open — feed the 1b-ii detector.** |
| R9 — cooperative+radar readiness (pre-real-test review 2026-07-04) | Cl-3 guard + first real deployment test | Cooperative core verified INTACT after the coverage-decay/camera/pose changes (28 tests green: platform_id identity gate, cooperative-only retirement, AIS-as-cooperative, bus determinism). Three items before cooperative+radar runs for real: (1) LATENT occupancy leak — cooperative fixes are Position2D/canInitiateTrack so they feed the clutter/occupancy layer (full-weight when unclaimed) AND, with `estimate_coverage_sector` on, the cooperative bundle self-estimates a meaningless coverage wedge unioned into the decay footprint (over-claiming, unsafe; same family as the multi-cluster bug); fix = exclude non-scanning SensorKinds from `cov_sensor` + key the corroboration veto on Cooperative as well as AIS. Exposure only when the occupancy detector is ON (default OFF). (2) Scenario-test gap: NO end-to-end test fuses cooperative + radar on one vessel — add one asserting single track, platform_id carried, ID stable through cooperative dropout while radar corroborates (retirement verified to sit in the no-surveillance-opportunity branch, PmbmTracker ~L668–695 — pin it), retirement only when both channels silent past timeout. (3) Integration-guide cooperative section (retirement behavior needs `use_sensor_activity` + DeclaredSensorActivity profiles for BOTH channels; legacy fallback differs). Ticket: `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md` §R9. | (1)+(2) ~1 day; (3) rides the guide follow-up | **items 1a/1b/2 SHIPPED 2026-07-04** (veto observeVesselFix, AIS+Cooperative via isNonScanningSource; cov_sensor exclusion — RemoteTrack joins at R10; coop+radar scenario: one track, platform_id, no retirement while radar corroborates, retire when both silent; bonus self-heal gate: wrong eviction of a present object re-emerges ≤5 scans). Item 3 SHIPPED 2026-07-05 with R11 (54c20cb): guide ISensorActivity section gained the cooperative+radar deployment recipe. R9 fully closed. FINDING 2026-07-04 (post-R10 review): the veto mechanism is UNWIRED in production — `observeVesselFix` has no caller outside unit tests (`ILiveOccupancyFeed` carries only `observe(bundle)`; the PmbmTracker producer never routes AIS/Cooperative/RemoteTrack fixes). WIRED 2026-07-05 (0472eae): fixes routed through the production feed, through-the-tracker RED->GREEN test, header comment fixed. Veto mechanically live on real HAXR AIS (740 fixes/site shift suppression + classification, no cardinality damage); isolating its lift needs a veto-ON/OFF toggle holding AIS constant (approved follow-up). GOTCHA recorded: `source_aware_misdetection` + `use_sensor_activity` do NOT compose (identity gate short-circuits empty scans before the activity model → source-aware cooperative track never retires); deployment recipe uses use_sensor_activity alone; config-validation guard recommended. |
| R10 — remote-track ingestion (shore/VTS tracks as pseudo-measurements) | Cl-3 guard + target deployment suite | Target suite: remote-station TRACKS + cooperative + camera(+distance) + radar + AIS — all but the first supported today (camera+distance = existing range/bearing path). Remote tracks enter as PSEUDO-measurements (spec §13 stance, same as ARPA): new `SensorKind::RemoteTrack`, adapter with R-inflation (×2–3 default) + per-track rate thinning, `sensor_track_id` per station source_id, MMSI passthrough; excluded from `cov_sensor` coverage estimation like Cooperative/AIS (R9 item-1 family); circular-AIS warning when raw AIS + AIS-fusing feed both carry one MMSI; DeclaredSensorActivity surveillance profile; registration-bias per source_id (verify, exists). One fusion scenario shaped like the real deployment (remote+radar+AIS+coop, single track, ID stable through remote dropout/id swap). NOT in scope: covariance intersection — deferred with measured trigger (NEES on the fusion scenario detects when R-inflation stops being enough). Ticket: `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md` §R10. | ~1 day | **SHIPPED 2026-07-04.** `SensorKind::RemoteTrack` + `isNonScanningSource`; `RemoteTrackAdapter` (R-inflation ×3 default on stated cov / 50 m pessimistic default when none; rate thinning 1/2 s per (source_id, track_id); velocity opt-in OFF; `sensor_track_id`+`mmsi` hints; `circularAisMmsis()` guard); 12 adapter unit tests. Fusion scenario `PmbmRemoteTrackFusion` (remote+radar+AIS+coop → ONE track, remote fused in, ID stable through remote id-swap AND dropout). **NEES = 1.79** (2 DOF, E=2) → R-inflation consistent, the covariance-intersection trigger stays un-tripped. Latent SkewProfile OOB fixed fail-loud (array 8→9 + bounds-checked `.at()`). DeclaredSensorActivity surveillance profile wired in the scenario (RemoteTrack Surveillance channel); registration-bias is per-source_id (generic, verified — not rebuilt). Guide §3 + learning §17.3.1 + eval-log. Defaults chosen BLIND — replace with deployment numbers when known. |
| PMBM runtime probe + Murty K=1 fix | Cl-3 (dev velocity + deployment realtime gate) | Probe (merged d90d979): 85% of PMBM wall = Hungarian solves from murtyKBest child generation AFTER the K-th accepted assignment (~82 wasted LSAP solves + full cost-matrix copies per scan at K=1, ~98.8% of all solves); cost-matrix construction ~2.7%, occupancy <0.1%; config-knob frontier FLAT (only r_min=1e-2 moves, −6%, byte-identical accuracy incl. gate suite); bench harness self-taxes ~10.5% (own GOSPA scoring). Fix (45a504d): one guarded break after the K-th accepted assignment — bit-identical at every K (existing tests pin K=1≡Hungarian + full K=6 enumeration). Measured: 515 s → 41.6 s (12.4×) on decimated kattwyk_08 285 s, accuracy identical to 6 decimals; decimated feed now ~6.8× FASTER than realtime → realtime gate (≥5× at 60–100 plots/scan) MET on this workload. Follow-ups open non-blocking (re-profile first — old profile obsolete): sparse/gated LSAP; --fast-metrics bench stride. RAW-density check 2026-07-06: undecimated 169-plots/scan window = 141 s / 285 s → 2.0× faster than realtime (was 20× slower) but FAILS the ≥5× margin gate → front-end extraction stays deployment-MANDATORY (margin-short, not hopeless); extraction stage needs its own realtime budget; per-scan max/p99 latency still unmeasured (both regimes). ROUND 2 (2026-07-06, merged fa0ca25): fresh profile post-fix = no dominant safe-class lever left (IMM per-mode measurement update spread across Eigen ops; murtyKBest down to ~2k calls/a few %); shipped per-scan latency cols (scan_proc_ms mean/p95/p99/max — determinism guard excludes wall-derived rows) + --fast-metrics (raw −10.3%, default-off byte-identical). PER-SCAN ANSWER: max scan 41 ms (dec) / 102 ms (raw) vs 148 ms interval — the tracker keeps up scan-by-scan even at raw density (1.45× worst-case margin). ARBITER RULING on the gate nuance: extraction stays deployment-MANDATORY — 1.45× worst-case on a quiet dev box does not clear the margin the 5× gate exists to provide (weaker deployment hardware, CPU sharing, density spikes), and raw density doubles the phantom count regardless of speed. The per-scan result is recorded as a RESILIENCE property: if the extraction stage fails live, the tracker degrades (thin margin + over-count) instead of collapsing. Priced findings parked (perf off the critical path): Position2D H-elision (bit-identical ~5-8%, blast radius all estimators — own ticket if ever needed); coarse gate prefilter (result-affecting); sparse LSAP deprioritised. docs/baselines/2026-07-05_pmbm_runtime_frontier.{md,csv} + eval-log 2026-07-05/06. | probe 1 day + fix same-day | **closed 2026-07-06 (fix shipped, gate met)** |
| Pre-water window (interim selection while the real test waits) | Cl-3 tail + deployment readiness | Research arc closed (corroboration complete, steady-state measured, detector frozen at 0.6, held-out scored 3H/1M/4P); water test delayed → one SELECTION doc picks from the existing queues (no new tracking system): Tier 1 = R11 SHIPPED 2026-07-05 (54c20cb: platform_id on TrackAttributes/output; PMBM refreshAggregatedTracks adopts identity from highest-mass carrying hypothesis, persisted on the Bernoulli last-write-wins so ids survive feed drops; fusion test surfaces both mmsi+platform_id; closes R9 item 3 too) + backlog #20 target-reported kinematics (AIS nav-status = the veto's missing data path) + #17 bearing-wedge + #18 fact-free half + #16; Tier 2 = D2 Stone Soup cross-check (never done), D7 MOANA (non-AIS truth + Doppler-capable radar), D8 R-BAD, R8.8 re-extraction (still open), ais_ferry_far/almost_cross measurement pass (held-out duty discharged; ais_ferry_far = only philos clip with honest accuracy truth); Tier 3 = at most ONE of clutter-model investigation / Cl-1 cold-start gap / D10+D3. Doc: `docs/superpowers/plans/2026-07-05-pre-water-window.md`. | tiers 1+2 ≈ 1 wk | **open — Tier 1 nearly done 2026-07-06: #20 (3 increments, anchored-veto data path live), #17 (BearingWedgeModel), #18 fact-free half all SHIPPED; #16 remains; Tier 2: D2 Stone Soup DONE 2026-07-06 (b5b3ea5 — GOSPA kernel == Stone Soup to 1.4e-14 on sim+real, prior incidents were upstream truth-grouping, kernel externally validated); next = #16, then D7/D8 feasibility** |
| Data-expansion TODOs D1–D12 (unused `data/` sources + external sweep) | Cl-3 + measurement integrity | Bench today wires only philos + autoferry + one HAXR station-hour; ~2.3 GB of fetched data unused. D1 HAXR multi-station/hour (second real harbor cross-check for Stage 1b; also hosts increment-8 steady-state A/B + AIS-veto validation since labelled philos clips are zero-AIS), D2 Stone Soup GOSPA cross-validation (hedge after two truth-fragmentation harness bugs), D3 MarineCadastre/DMA anchored-vessel mining (R3/1b-ii corroboration input) + density soak, D4 Pohang moving-platform occupancy (parked for Stage 2), D5 SMD EO bearings (parked for ADR 0001 A3), D6 Kystverket soak (unscheduled). External sweep 2026-07-03 (evaluate-before-extract): **D7 MOANA** (real X-band radar with camera/lidar — NON-AIS — truth + anchored large vessels; first real source where "false track" means false), **D8 R-BAD** (69 h berthing radar + synced video: hour-scale steady state + philos-style label passes on a second geography), D9 DLR extended-target overlap check (~1 h), D10 GFW anchorage DB (pre-computed D3 stats + candidate KEEP-side chart-corroboration prior), D11 Reeds (Stage-2 moving-platform, evaluate before D4), D12 WaterScenes (logic cross-check only, wrong radar physics). Tickets: `docs/superpowers/plans/2026-07-02-data-expansion-todos.md`. | D1+D2 ~2 days; D7/D8 feasibility ~half day each | **open** (D1/D2 next; then D7/D8 feasibility) |
| Camera bearing-only corroboration channel | Cl-3 (1b-ii KEEP-guard, third corroboration leg) | Input side SHIPPED 2026-07-03 (branch camera-bearing-extraction): YOLO detections → hull-relative bearing fixtures, AIS-RANSAC calibration median 0.45°/p90 1.32° held-out; `ais_ferry_near` (validated) + `sunset_cruise` (same-vessel transfer, σ≈3.5°); `close_approach` REFUSED (different vessel, no intrinsics, zero AIS) so KEEP_MIXED there stays radar+labels; C++ `Bearing2D` loader + no-initiation proof green. Consumer side NOT started — sequenced AFTER chart corroboration (2026-07-03 decision). First target: the sunset loiterer (6c measured its cell 0/283-scans-observable after t≈94 — departure is a camera fact, the exact departed-vs-unobserved discriminator radar lacks). Rules: detection presence = KEEP/decay evidence only, absence NEVER suppresses; philos labels share the video source ⇒ mechanism demos on philos, promotion gates on synthetic truth only. | consumer side ~2–3 days when picked up | **input shipped / consumer open** (after chart corroboration) |
| Refresh / promotion of `_bearguard` | Cl-2 #2 | TBD; re-measure against the gated canonical first. | half-day measurement | open after Cl-2 #1 |

**Implicit ordering this produces (until you say otherwise):**

1. ~~Cl-2 #1~~ — **closed 2026-06-19** as metric-artefact, not bug.
2. ~~Cl-2 #2 (env-2 BOT framed)~~ — **partially scoped 2026-06-19**;
   `_bearguard` small, `_recapture` not shippable, real fix is
   lifecycle/init-cov work. Deeper investigation deferred behind
   cheaper wins below.
3. ~~Step 5 (Cooperative GNSS)~~ — **shipped 2026-06-19** as
   wiring-only (no bench delta until a scenario emits Cooperative
   measurements). Synthetic anchor-substitution sweep filed as
   follow-up.
4. ~~Cl-2 #3 (UKF inside IMM)~~ — **shipped 2026-06-20**
   (autoferry unanchored mean GOSPA −12.3%). ~~Cl-2 #4 (EO/IR R
   tightening)~~ — **rejected 2026-06-19** (env-2 anchored
   regression). ~~Cl-2 #2 (a)+(b) (lifecycle re-tune /
   init-cov widening)~~ — **rejected 2026-06-20** (cardinality
   bloat). With Cl-2 #2 deferred indefinitely, the Cl-2 stack is
   stable enough to move on.
5. *(Cl-2 #2 has no remaining cheap candidates inside the IMM+MHT
   stack — the joint-existence+association coupling is the real
   over-confidence locus, and reaching it means PMBM or the Cl-1
   sibling pipeline.)*
6. **Then Cl-3 (PMBM)** — the academic-frontier milestone. Begins
   only after Cl-2 stack is stable so the comparison floor is honest.
7. Cl-1 SJPDA/JIPDA — defer unless we explicitly want the
   class-controlled comparison published.

---

## Source bibliography (load-bearing references)

- **Cl-1 paper.** Helgesen, Vasstein, Brekke, Stahl, *Heterogeneous
  multi-sensor tracking for an autonomous surface vehicle in a
  littoral environment*, Ocean Engineering 252 (2022) 111168. Local
  PDF: `docs/references/S0029801822005753-helgesen-2022-heterogeneous-multisensor-littoral.pdf`. Dataset:
  https://autoferry.github.io/sensor_fusion_dataset/.
- **Cl-2 deployment references.** Bar-Shalom, *Estimation with
  Applications to Tracking and Navigation* (2001), *Tracking and Data
  Fusion* (2011). Blackman, *Multiple Hypothesis Tracking* (1999;
  2004 update). Q-IMM-MHT 2025 (PubMed 40006287) — recent active
  research in the class. Hybrid TBD coastal maritime radar (arXiv
  2508.16169, 2025) — recent maritime deployment-class work.
- **Cl-3 academic-SOTA references.** Williams 2015, "Marginal
  multi-Bernoulli filters: RFS derivation of MHT, JIPDA, and
  association-based MeMBer" — the lineage paper. García-Fernández et
  al. 2020 — trajectory-PMBM. "Systematic Analysis of the PMBM, PHD,
  JPDA and GNN Filters" 2022 — direct head-to-head. Multiple-Model
  Trajectory PMBM, Scientific Reports 2025 — current research
  frontier.

---

## Glossary of identifiers

| Symbol | Meaning |
|---|---|
| Cl-1 | Claim 1: Helgesen 2022 published baseline (JIPDA-class) |
| Cl-2 | Claim 2: IMM+MHT (TOMHT) deployment-SOTA — our canonical |
| Cl-3 | Claim 3: PMBM academic-SOTA endgame (RFS family) |
| env-1 / env-2 | AutoFerry environments 1 (open water, sc2–6) and 2 (urban channel, sc13/16/17/22) |
| canonical | `imm_cv_ct_mht` per `core/benchmark/Config.cpp::defaultConfigs()` |
| anchored / unanchored | Whether the AutoFerry replay injects a truth-AIS Position2D measurement (the paper's calibration condition) |
| deployment-SOTA | What is actually fielded in commercial/defence systems today (IMM+TOMHT class) |
| academic-SOTA | What the model-based-tracking literature considers the frontier (RFS family, PMBM-dominant as of 2024–2025) |
