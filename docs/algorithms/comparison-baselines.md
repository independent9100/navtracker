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
| PMBM as `IDataAssociator` (sim-only first) | Cl-3 #1 | First-cut PMBM; sanity-check against Williams 2015 MHT-as-special-case. | 1–2 weeks (literature + build + sim tests) | not started |
| PMBM applied to autoferry | Cl-3 #2 | Headline measurement for Cl-3. | 1 week after Cl-3 #1 | not started |
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
  PDF: `1-s2.0-S0029801822005753-main.pdf`. Dataset:
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
