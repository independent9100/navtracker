# PMBM Integration Plan

Concrete phased plan to introduce Poisson Multi-Bernoulli Mixture
filtering as a third tracker pipeline alongside `Tracker`
(GNN/JPDA-driven) and `MhtTracker` (TOMHT). Companion to the high-level
literature entry in [sota-roadmap.md §4](../../algorithms/sota-roadmap.md)
and the equation-level reference in
[pmbm-design.md](../../algorithms/pmbm-design.md).

**Status refresh 2026-06-20 (post Cl-2 close-out).** Original plan dated
2026-06-07 stands; the upgrades since then *strengthen* the case but do
not change the phasing:

- Cl-2 #3 SHIPPED — UKF is the canonical inner filter. Phase 2
  (`IMM-per-Bernoulli`) inherits UKF for free; no extra work.
- Cl-2 #2 (a)+(b) REJECTED — the re-confirmation over-confidence is **not
  reachable** from lifecycle thresholds or init-cov priors
  (`cl25_life_20260620.csv` regressed +17 % anchored GOSPA). PMBM's joint
  existence + association recursion is now the **principled remaining
  fix** for that failure mode, sharpening the rationale below.
- Cross-sensor bias correction promoted to canonical with anchor-gated
  publish; composes in front of PMBM update unchanged.
- Step 5 cooperative GNSS anchor wired; PMBM treats `Cooperative` as
  another sensor in the PPP / measurement-likelihood machinery.
- Murty K-best (Phase 0) shipped and in production use under
  `MhtTracker`. Same code path will service PMBM Phase 1.

Equation-level math, port shape, and composition with existing
components: see [pmbm-design.md](../../algorithms/pmbm-design.md). This
document remains the *engineering execution* reference; that document is
the *algorithm* reference.

**Context.** The current stack — EKF/UKF/IMM behind `IEstimator`,
TOMHT via `MhtTracker`, GNN/JPDA via `IDataAssociator` — is the
*classical* gold-standard for point-target multi-target tracking
(Bar-Shalom / Blackman lineage). PMBM (García-Fernández, Williams,
Granström, Svensson 2018; arXiv:1703.04264) is the *modern*
RFS-based competitor — Bayes-optimal conjugate under the standard
multi-object model with Poisson birth + Bernoulli survival, with
native existence probabilities replacing M-of-N / score-threshold
heuristics. Trajectory-PMBM (TPMBM, arXiv:1912.08718) extends to
per-target trajectories — the structural replacement for `MhtTracker`.

**Reference implementation.** [`Agarciafernandez/MTT`](https://github.com/Agarciafernandez/MTT)
(BSD-2). MATLAB. We port, not link. Includes TPMBM, TPMB, TMBM, TMB,
PHD/CPHD, continuous-discrete, non-linear, OOSM variants. Uses GOSPA /
T-GOSPA as canonical metrics.

## Why this comes after task #1 (clean `gate`/`logLikelihood` interface)

Phase 1 below assumes the per-target estimator exposes `gate(z)` and
`logLikelihood(z)` as first-class `IEstimator` methods, with IMM
returning the proper mode-weighted mixture likelihood. Task #1 in
`todo.md` adds exactly that. The cleanup is small (~2 weeks) and is a
prerequisite for PMBM's hypothesis enumeration. **Do task #1 first.**

## Phased plan

### Phase 0: Murty K-best assignment solver — ✅ COMPLETE (2026-06-08)

**Deliverable.** `core/association/Murty.{hpp,cpp}` — K-best assignment
on top of the existing `Hungarian` solver (Murty 1968).

**Status:** Solver implemented and wired into `MhtTracker`. 8 unit
tests including all-permutations exhaustive check, K=1 ≡ Hungarian
on randomised batches, ranked-cost monotonicity, +∞ honoring. See
[`docs/superpowers/specs/2026-06-08-murty-k-best-design.md`](../specs/2026-06-08-murty-k-best-design.md)
and [its impl plan](2026-06-08-murty-k-best-impl.md). Ready for PMBM
Phase 1 to reuse.

**Reuse.** Internally calls `hungarianAssignment` K times with
partition constraints (forbid / require edges per iteration).

**Why now.** Same algorithm has two consumers:

1. Closes the K>1 deferred item in `MhtTracker` (turn the K=1
   Hungarian global hypothesis into proper deferred K-best TOMHT —
   the textbook).
2. PMBM uses Murty K-best in the update step to truncate the
   exponential-in-measurements hypothesis enumeration. Same code,
   different caller.

**Tests.**
- `KBestMatchesHungarianForK1`.
- `KBestEnumeratesAllOnSmallProblem` (3×3 ground truth).
- `KBestRespectsForbiddenEdges`.
- `KBestStableUnderTies`.

### Phase 1: GM-PMBM (Gaussian per Bernoulli, no IMM yet) — ~3 weeks

**Deliverable.** `core/pipeline/PmbmTracker.{hpp,cpp}` alongside
`MhtTracker`. Single-mode Gaussian per-Bernoulli density.

**Data structures.**
- `PoissonComponent { weight, mean, covariance }` — one entry per
  spatial region where new targets may appear. Maintained as a vector.
- `Bernoulli { existence_probability, mean, covariance }` — one per
  detected target hypothesis.
- `GlobalHypothesis { vector<Bernoulli>, weight }` — a complete
  scan-consistent assignment. Maintained as a vector of mixture
  components.
- `PmbmDensity { vector<PoissonComponent> ppp; vector<GlobalHypothesis> mbm; }`.

**Per-scan update.**
1. **Predict.** Each Poisson component and each Bernoulli predicts via
   the existing `IEstimator::predict` (the per-target filter is
   reused — this is the entire point of doing task #1 first).
   `existence_probability *= survival_probability`. Poisson birth
   intensity added to the Poisson part.
2. **Update — per global hypothesis.** For each
   `GlobalHypothesis` in the MBM:
   - For each scan measurement m and each Bernoulli b in the
     hypothesis, compute the cost matrix entry as
     `−log( r_b · estimator.logLikelihood(b, m) + log P_D )` (cells
     for unassociated measurements get the Poisson-likelihood +
     P_D term; cells for unassociated Bernoullis get the
     "missed detection" weight).
   - Solve **Murty K-best** on this matrix; each of the K solutions
     becomes a child `GlobalHypothesis` with updated Bernoullis
     (per `IEstimator::update`) and updated weights.
   - Unassigned measurements may spawn new Bernoullis from the
     Poisson part (a Poisson component + measurement → Bernoulli
     with non-trivial `existence_probability`).
3. **Prune.**
   - Drop hypotheses with weight below threshold.
   - Cap the number of global hypotheses (top-K by weight).
   - Within a hypothesis, drop Bernoullis with `existence_probability`
     below threshold.
   - Merge Bernoullis (across hypotheses) by Bhattacharyya
     distance — reuse our `mergeBranches` primitive.

**Output mapping.** For each Bernoulli, weight by `existence_probability
× hypothesis_weight` and sum across hypotheses; report as `Track` with
`status = Confirmed` iff aggregated `P(exists) ≥ confirm_threshold`,
otherwise `Tentative`. The aggregated `P(exists)` is strictly more
information than M-of-N — expose it on `Track` as a new field.

**Tests.**
- `PmbmTracker.SingleTargetCleanlyTracked` — clean detections, P(exists)
  monotonically increases to ~1.
- `PmbmTracker.ExistenceProbabilityDecaysOnMissedDetections`.
- `PmbmTracker.BirthFromPoissonOnFirstDetection`.
- `BusComparison.GmPmbmVsMht` — head-to-head OSPA on bus scenarios.

### Phase 2: GM-PMBM with IMM per Bernoulli — ~1 week

**Deliverable.** Bernoulli's single-target density is now an IMM
mixture (per the existing `ImmEstimator`). No structural change to
PMBM — just `estimator_ = imm_cv_ct` instead of `ekf_cv` at
construction time. This is why the `IEstimator` interface had to be
clean (task #1).

**Test.** `BusComparison.ImmPmbmVsImmJpdaVsImmMht`.

### Phase 3: TPMBM (Trajectory PMBM) — ~1–2 weeks

**Deliverable.** Bernoullis carry per-target **trajectories**
(`vector<TrajectoryPoint>`) instead of just the current state.
Birth time, death time, and the per-scan state history become part
of the filter state.

**Why this matters for navtracker.** `ITrackSink::onTrackInitiated/Confirmed/
Updated/Deleted` and `TrackOutput`'s expected lifecycle map *natively*
onto TPMBM transitions:
- Birth: `existence_probability` crosses `confirm_threshold` → emit
  `onTrackConfirmed`.
- Death: `existence_probability` falls below `delete_threshold` AND
  Bernoulli gets pruned → emit `onTrackDeleted` with the full
  trajectory.
- Update: every scan → `onTrackUpdated`.

This is the **structural replacement** for `MhtTracker`. Once TPMBM
beats TOMHT on the bus scenarios end-to-end, MHT can be deprecated
(but kept around for comparison and as a sanity backstop).

**Test.** `TPmbmTracker.TrajectoryLifecycleEmitsExpectedSinkEvents`.

### Phase 4: GOSPA / T-GOSPA metrics — ~3 days

**Deliverable.** Add `core/scenario/Gospa.hpp` alongside `Ospa.hpp`.
Wire into `BenchSink` so reports include both OSPA and GOSPA.

**Why.** The PMBM literature uses GOSPA exclusively (penalizes
missed/false detections explicitly, OSPA only via assignment cost).
T-GOSPA (time-weighted) penalizes trajectory fragmentation — directly
measures the things PMBM is supposed to be better at. Switching the
metric is necessary to reproduce published comparisons.

**Reference.** Rahmathullah, García-Fernández, Svensson (2017),
"Generalized optimal sub-pattern assignment metric"
(arXiv:1601.05585).

### Phase 5 (optional): OOSM-PMBM — ~1 week

**Deliverable.** PMBM variant that handles out-of-sequence
measurements (late-arriving AIS, in particular) by augmenting the
state with retrodiction.

**Why optional.** We have `ReorderBuffer` which delays inputs to
enforce monotone timestamps — a pragmatic substitute. OOSM-PMBM is
the principled fix; the reference repo includes a `cd_pmbm_oos`
variant we could port if `ReorderBuffer` latency becomes a problem.

## Things to watch when porting

- **Hypothesis pruning is the cost driver.** PMBM's exact form has
  factorially many global hypotheses; the reference MATLAB uses
  Murty K-best + per-Bernoulli existence-probability pruning +
  Bhattacharyya merging across components. Skip the merging and the
  mixture explodes. Our `mergeBranches` primitive is the right
  reuse point — just generalize it from "within tree" to "across
  Bernoullis within a hypothesis."
- **Poisson birth intensity needs domain knowledge.** "Where do new
  ships appear?" — radar coverage edges, AIS-broadcast positions
  not yet associated, named ports, off-shipping-lane regions. The
  Poisson density is parameterized and must be tuned per deployment.
  This is where maritime-specific knowledge actually pays. For the
  first cut, use a uniform Poisson intensity over the local ENU
  patch and tune later.
- **Survival probability `P_S`.** Decay constant for Bernoulli
  existence between scans. Typical 0.99 per scan; for ships that
  pop in and out of radar coverage, may need to be sensor-conditional.
- **Reference MATLAB is well-cited but MATLAB-idiomatic.** Lots of
  cell arrays, structs, in-line linear algebra. A clean C++ port is
  probably 1.5×–2× the LoC of the MATLAB, but the math doesn't
  change. Read Williams 2015 *"Marginal multi-Bernoulli filters:
  RFS derivation of MHT, JIPDA and association-based MeMBer"*
  before porting — it shows how JIPDA, MHT, and PMBM all derive from
  RFS and explains which approximations the MATLAB takes.
- **License.** Repo is BSD-2-Clause. Compatible with our use.
  Attribution belongs in the ported source file.
- **No C++ port exists upstream.** We are the C++ port. Worth
  publishing back to the community in due course.

## Decision points to revisit

- **PMBM vs δ-GLMB.** Both are RFS conjugate priors with native
  existence probabilities. δ-GLMB has explicit target labels baked
  into the filter (some applications prefer that). PMBM has a
  simpler propagation step (no labelled GLMB component algebra) and
  the TPMBM variant produces labelled trajectory output directly. We
  favor PMBM; revisit only if a published comparison on maritime
  data shows δ-GLMB clearly winning.
- **When to deprecate `MhtTracker`.** Not before TPMBM with IMM
  beats TOMHT-IMM by ≥10% on OSPA across the bus scenarios. Keep
  `MhtTracker` around as the classical baseline indefinitely.

## Effort summary

| Phase | Deliverable | Effort |
|---|---|---|
| 0 | Murty K-best | ~1 week |
| 1 | GM-PMBM (Gaussian, no IMM) | ~3 weeks |
| 2 | GM-PMBM + IMM per Bernoulli | ~1 week |
| 3 | TPMBM (trajectory) | ~1–2 weeks |
| 4 | GOSPA / T-GOSPA metrics | ~3 days |
| 5 | OOSM-PMBM (optional) | ~1 week |
| **Total (phases 0–4)** | — | **~6–8 weeks** |

## Sources

- García-Fernández, Williams, Granström, Svensson (2018), *Poisson
  Multi-Bernoulli Mixture Filter: Direct Derivation and
  Implementation*, IEEE TAES 54(4). arXiv:1703.04264. **Primary reference.**
- García-Fernández, Williams, Granström, Svensson (2020), *Poisson
  Multi-Bernoulli Mixtures for Sets of Trajectories*.
  arXiv:1912.08718. **Trajectory variant.**
- Williams (2015), *Marginal multi-Bernoulli filters: RFS derivation
  of MHT, JIPDA and association-based MeMBer*, IEEE TAES 51(3).
  **Bridge text — read before porting.**
- Mahler (2007/2014), *Statistical Multisource-Multitarget
  Information Fusion*, Artech House. RFS reference text.
- Rahmathullah, García-Fernández, Svensson (2017), *Generalized
  optimal sub-pattern assignment metric*. arXiv:1601.05585.
  **GOSPA reference.**
- Reference repo: https://github.com/Agarciafernandez/MTT
  (BSD-2-Clause).
- Maritime evaluations:
  - *Multiple-Model Trajectory PMBM for Maneuvering Objects*,
    Sci. Rep. 2025.
  - *Novel MTT Based on PMBM Filter for High-Clutter Maritime
    Communications*, 2025.

## Cross-references

- [docs/algorithms/sota-roadmap.md §4](../../algorithms/sota-roadmap.md) —
  high-level entry; this plan is the execution detail.
- [docs/algorithms/algorithm-review-2026-06-07.md](../../algorithms/algorithm-review-2026-06-07.md) —
  current state of the stack; PMBM phases assume the fixes there are landed.
- [todo.md Task #1](../../../todo.md) — task #1 (`gate`/`logLikelihood`
  interface cleanup) is a prerequisite for phase 1.
- [todo.md Task #3](../../../todo.md) — the original PMBM ask.
