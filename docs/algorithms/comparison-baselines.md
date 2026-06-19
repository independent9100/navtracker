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
- **Where MHT-class loses**: env-1 unanchored. Mechanism:
  sc13_anchored NEES = 69 (track-tree pair-stream regression from 14
  ID switches; documented step-0 residual 2026-06-19); sc5/sc6 BOT
  range collapse (Jacobian-rank problem the canonical estimator
  can't fix; see eval-log 2026-06-16 per-target diagnosis).
- **Cooperative-fleet deployment (step 5 in plan)**: cooperative
  GNSS acts as an AIS-replacement anchor; under it, Cl-2 inherits
  the truth-AIS column from Cl-1's table, i.e. cleanly beats the
  paper.

**Open Cl-2 work (ranked):**

1. **sc13_anchored NEES = 69 residual.** MHT-internal: 14 ID
   switches reset `recent_contributions`, biasing the AIS-bearing
   pair stream toward the better-tracked target; corrected bearings
   are then systematically wrong on the swap-prone target. Fix lives
   in `MhtTracker` / `TrackTree` ID-switch handling, not in JPDA or
   the bias estimator.
2. **sc5 / sc6 / sc22 env-2 BOT pathology.** Modified-polar EKF or
   bearing-bearing triangulation pre-init. `BearingRangeGuard`
   (`_bearguard`) already implements a post-update LOS clamp that
   helps measurably; promote to canonical once Cl-2#1 is closed and
   re-measurable cleanly.
3. **UKF / cubature KF inside IMM (sub-question (a), sota-roadmap
   §3).** Build a `ukf_cv_ct_mht` config and measure. If the answer
   is "≤1% different from EKF", we declare the inner-filter question
   closed in EKF's favour (simpler, faster, no loss). If meaningful,
   ship UKF as canonical inner filter.
4. **EO/IR R tightening from step-2 NIS finding (2026-06-19).** Small,
   safe-direction change; documented and not blocking.

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
| sc13_anchored MHT NEES = 69 residual | Cl-2 #1 | Closes the only remaining anchored-env-2 NEES catastrophe; ≈ −45 NEES on sc13. | 1–2 days investigation + fix | open |
| sc5/sc6/sc22 env-2 BOT pathology | Cl-2 #2 | Closes the unanchored env-1 GOSPA gap to Helgesen (Cl-1 cold-deployment claim) by an unknown amount; likely partial. | 3–5 days (mod-polar EKF or triangulation init) | open |
| UKF / cubature KF inside IMM | Cl-2 #3 | Answers Cl-2 sub-question (a); either ships UKF or formally closes the inner-filter question in EKF's favour. | 1–2 days build + bench | open |
| EO/IR R tightening (step-2 finding) | Cl-2 #4 | Small NEES improvement on anchored env-2; safe direction. | half-day | open, low-priority |
| Step 5 — Cooperative GNSS as anchor | Cl-2 (deployment) | Inherits truth-AIS-column performance under deployments with fleet GNSS. | half-day to 1 day | queued |
| SJPDA on JPDA branch | Cl-1 (class-controlled extension) | Half of "is the association class load-bearing?" answer. | half-day (permutation collapse) | optional / deferred |
| JIPDA proper on JPDA branch | Cl-1 (class-controlled extension) | Other half. | 2–3 days | optional / deferred |
| PMBM as `IDataAssociator` (sim-only first) | Cl-3 #1 | First-cut PMBM; sanity-check against Williams 2015 MHT-as-special-case. | 1–2 weeks (literature + build + sim tests) | not started |
| PMBM applied to autoferry | Cl-3 #2 | Headline measurement for Cl-3. | 1 week after Cl-3 #1 | not started |
| Refresh / promotion of `_bearguard` | Cl-2 #2 | TBD; re-measure against the gated canonical first. | half-day measurement | open after Cl-2 #1 |

**Implicit ordering this produces (until you say otherwise):**

1. Cl-2 #1 (sc13_anchored ID-switch residual) — directly improves the
   tracker we ship; concretely measurable; closes the only catastrophe
   left on the Cl-2 headline.
2. Cl-2 #2 (env-2 BOT) — same; biggest remaining MHT-class gap and
   directly improves Cl-1's unanchored env-1 number too.
3. Step 5 (cooperative GNSS) — deployment-relevant, cheap, unlocks
   the truth-AIS-column performance for real deployments without AIS.
4. Cl-2 #3 (UKF) and #4 (EO/IR R) — small, safe.
5. **Then Cl-3 (PMBM)** — the academic-frontier milestone. Begins
   only after Cl-2 stack is stable so the comparison floor is honest.
6. Cl-1 SJPDA/JIPDA — defer unless we explicitly want the
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
