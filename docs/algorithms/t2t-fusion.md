# Track-to-track fusion — pedigree-aware covariance intersection (`navtracker_t2t`)

*New deployment capability on the Cl-3 line: a "tracker of trackers" that fuses
navtracker's own output with other trackers' tracks into one authoritative
picture, when the other trackers' input sensors are OFTEN UNKNOWN. Origin: user
decision 2026-07-09 (`docs/superpowers/plans/2026-07-09-t2t-fusion-ticket.md`).
Intuitive introduction (plain English, with the shared-newspaper metaphor):
`docs/learning/29-track-to-track-fusion.md` (added with the learning-docs
update). This document is the precise reference; the four-section template
below is the house standard.*

*Status: Milestone 1 (contracts + math). The engine (associator, fuser,
lifecycle) is Milestone 2; the math it will implement is specified in full
here. Sections marked (M2) describe behavior whose implementation lands with
the engine — the math is fixed now so the header-level API can be reviewed
against it.*

## 0. Where T2T sits in the pipeline

navtracker fuses raw sensor **measurements** into tracks. T2T sits one level
up: its inputs are already-formed **tracks** from N trackers (navtracker itself
plus external ones), and its output is a single fused set of tracks.

```
  sensors → [ navtracker Tracker/PMBM ] ─┐
                                          ├─→ ExternalTrack ─┐
  other tracker A ──────────────────────┘                   │
  other tracker B (may have fused AIS already) ─ ExternalTrack ┼→ [ T2tFuser ] → FusedTrackOutput
  shore/VTS feed ─────────────────────────────── ExternalTrack ┘
```

The one hazard that shapes the whole design: two of those trackers may have
consumed **the same sensor** (e.g. both fused the same AIS stream). Their
errors are then correlated. Fusing two correlated estimates as if independent
makes the fused covariance **too small** — the system becomes more confident
than any input justifies. That silent overconfidence (the classic
"double-counting" / rumor-propagation failure) is the failure this design makes
impossible by default, by fusing with **covariance intersection** and by
carrying an explicit **pedigree** with a first-class "I don't know" state.

## 1. Math

State convention throughout: the core kinematic state `x = [px, py, vx, vy]` in
the ENU local tangent plane (m, m, m/s, m/s), covariance `P` (4×4), same as
`core/types/Track`. Fusion operates on the position block (2×2) always, and on
the velocity block only when both inputs supply a valid velocity (§1.4). Time
is message-timestamp driven, never wall-clock (architecture invariant 4).

### 1.1 Time alignment

All fusion happens at the engine's current time `t`. Each stored source track,
last reported at `t_k`, is predicted forward to `t` with the constant-velocity
model (reusing `core/estimation/ConstantVelocity2D`, `dt = t − t_k`):

    x' = F(dt) x,     P' = F(dt) P F(dt)ᵀ + Q(dt)

with the white-noise-acceleration process noise

    F(dt) = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]

    Q(dt) = q · blockdiag over each axis of [[dt³/3, dt²/2],[dt²/2, dt]]

and `q = process_noise_accel_psd_m2_s3` (default 0.1 m²/s³, matching the
core CV model). A source track with no report newer than `max_report_age_s`
stops contributing — stale inputs are never extrapolated forever; the fused
track coasts and eventually demotes (§1.5). Prediction with `dt ≤ 0` is a no-op
(no backward prediction), matching `EkfEstimator::predict` — this keeps replay
deterministic.

### 1.2 Track-to-track association (M2)

Which of tracker B's tracks is the same object as this fused track? For each
candidate pair (fused prediction `x₁,P₁`; source track `x₂,P₂`), both at time
`t`, the squared statistical distance on the position block is

    d² = (x₁ − x₂)ᵀ (P₁ + P₂)⁻¹ (x₁ − x₂)          [position only]

Using `P₁ + P₂` ignores the cross-covariance between the two estimates. Under
positive correlation the true innovation covariance is smaller than `P₁ + P₂`,
so this makes the gate **conservatively wide** (it never wrongly rejects a true
pair for being too close) — an acceptable, safe bias, documented here per the
docs standard. Pairs with `d² > gate_chi2_position` (default 9.21 = χ²₂ at
p=0.99) are gated out.

Assignment reuses the core Hungarian solver (`hungarianAssignment`) on the
gated cost matrix. The cost is `d²` plus two **soft** attribute terms:

    cost = d² − shared_mmsi_cost_bonus·[same MMSI] + conflicting_mmsi_cost_penalty·[different MMSI]

Both terms are soft (magnitudes are config fields, defaults 2.0 and 6.0, both
below the gate) so a strong kinematic match still wins when identity evidence
disagrees — identity is evidence, never the key (invariant 5). Gated-out cells
are `+∞`; after solving, each assignment is re-checked for a finite original
cost before it is trusted (the Hungarian may return a forced-forbidden pick when
no feasible assignment exists).

**Pairing hysteresis (anti-flicker).** A (fused track, source track) pairing
forms after `pair_confirm_hits` consistent assignments and survives
`pair_break_misses` missed/failed ones. While paired, that source track flows
into its fused track unless the gate rejects it hard (then the pairing begins to
break). This mirrors the M-of-N spirit used elsewhere in the repo.

### 1.3 Covariance intersection (the fusion rule)

Two estimates `(x₁,P₁)`, `(x₂,P₂)` of the same state are fused by

    P_f⁻¹ = ω P₁⁻¹ + (1 − ω) P₂⁻¹
    x_f   = P_f ( ω P₁⁻¹ x₁ + (1 − ω) P₂⁻¹ x₂ ),    ω ∈ [0, 1]

**Weight.** ω is chosen to minimize `trace(P_f)`. `trace(P_f(ω))` is convex on
[0,1], so a 1-D line search suffices; we use a fixed-iteration golden-section
search (`ci_omega_iterations`, default 40). Fixed iterations — not a
convergence-dependent step count — so replay is bit-for-bit reproducible. When
`P₁ = P₂` the objective is flat and the minimizer is 0.5 by symmetry (returned
directly). trace is minimized rather than determinant because the position
trace is the operationally meaningful spread; det is the classical alternative,
noted in §4.

**Consistency property (the entire justification).** If each input is
consistent — `E[(x̂ − x)(x̂ − x)ᵀ] ⪯ P` — then the CI fusion is consistent for
**any** cross-correlation between the two inputs. This is why CI is the default
under an Unknown pedigree: no correctness assumption rides on knowing the
correlation. In plain words: naive fusion of two reports that secretly share a
source is like counting one newspaper article as two independent confirmations
because two friends both read it to you; CI refuses to become more certain than
the more careful reading of the two, unless genuine independence is proven.

The overconfident control — naive independent fusion `P_f⁻¹ = P₁⁻¹ + P₂⁻¹` — is
implemented only inside the bench (`naiveIndependentFuse`) as the scientific
baseline that shows *why* CI. It always satisfies
`trace(P_CI) ≥ trace(P_naive)`: the naive covariance is the dangerously small
one.

**More than two sources.** Fuse sequentially in canonical order (source tracker
ids sorted lexicographically): `CI(CI(CI(e₀,e₁),e₂),…)`. The order dependence
is real but small; batch CI (a joint weight vector on the simplex) is listed in
§4.

### 1.4 State-dimension handling

Fuse the position block (2×2) always. Fuse velocity only when **both** inputs
carry a valid velocity; otherwise adopt the valid side's velocity with its own
covariance (flagged), and never invent a velocity for a position-only input.

### 1.5 Pedigree and what it changes (v1)

A `SourcePedigree` declares, per named sensor stream, one of `{Used, NotUsed,
Unknown}`, with a default for unlisted streams. The pairwise rule
(`independenceOfPair`, pure and exhaustively unit-tested) is: A and B are
**ProvablyIndependent** iff for every stream — every id listed in either
pedigree, and the unlisted tail governed by the two defaults — at least one side
is `NotUsed`; otherwise **PossiblyCorrelated**. Equivalently, overlap on a
stream is possible iff *neither* side is `NotUsed` (Used/Used certain;
Used/Unknown and Unknown/Unknown possible; anything with NotUsed safe). An
absent pedigree is treated identically to an all-`Unknown` pedigree.

What the verdict changes in v1:

- **Fusion math: nothing.** CI is used for every pair, including provably
  independent ones (there CI is merely conservative). This is deliberate — no
  correctness risk rides on a pedigree declaration being right.
- **Output:** `independence_class` per fused track (SingleSource /
  ProvablyIndependent / PossiblyCorrelated), operator/diagnostic value.
- **Attributes:** if two inputs both declare `ais:* = Used` and carry the same
  MMSI, the shared MMSI is association corroboration but not counted as
  independent identity evidence twice; different non-empty MMSIs → the soft
  conflicting-MMSI penalty of §1.2.
- **Future hook:** `ports/IFusionRule.hpp` — CI is the only concrete
  implementation. A tighter rule for provably-independent pairs is a later,
  measured, opt-in change behind this port (§4).

### 1.6 Fused identity and lifecycle (M2; presence over classification, one level up)

- `fused_track_id`: minted by the fuser, monotonically increasing, never reused
  (invariant 5).
- Birth: an unassociated source track births a fused track once seen
  `fused_confirm_m` of `fused_confirm_n` reports. Single-source fused tracks are
  legitimate — one tracker seeing something is presence; ADR 0002's spirit
  applies at this level: never let "only one tracker sees it" suppress an object
  into nothing.
- Confirm: immediately if a source reports `Confirmed` and `trust_source_status`
  (default true); else the fused track's own M-of-N.
- Coast/delete: when no contributing source is fresher than `max_report_age_s`,
  the fused track coasts with inflating covariance (§1.1) and is deleted after
  `fused_delete_age_s`. A pairing breaking does not delete the fused track while
  another source still sustains it — continuity through single-input dropout is
  a headline win to measure.

## 2. Assumptions

- **Same object, same state, same frame.** Fusion assumes the associated
  inputs describe one physical object, in the shared ENU datum, aligned to time
  `t`. Association (§1.2) and time alignment (§1.1) establish this; mis-
  association violates it (mitigated by the conservative gate and hysteresis).
- **Each input is individually consistent** (`MSE ⪯ P`). CI inherits its
  guarantee from this. A source that is biased and overconfident (states a
  covariance far smaller than its true error) breaks the premise; CI limits but
  does not erase the damage (characterized in the bench conflict scenario, and
  the seed for the input-de-weighting entry in §4).
- **Stream identifiers are exact-string and globally agreed.** The pedigree
  rule is only as honest as the ids: the same physical stream must carry the
  same id across sources. A typo makes two views of one stream look independent.
- **Input covariances are positive-definite** after edge defaults. Non-PSD or
  non-finite inputs are rejected at the edge (`validateExternalTrack`); CI
  assumes invertibility and does not re-check.
- **Per-source clocks are independent.** Out-of-order detection is per
  `source_tracker_id` (each source has its own latency); staleness is never
  judged across sources.

## 3. Rationale

**Why covariance intersection by default.** The deployment reality is that we
often do not know what an upstream tracker fused. The only fusion rule that is
provably safe without that knowledge is CI: consistent for any unknown
cross-correlation. Choosing it by default means the worst outcome of an unknown
or mis-declared pedigree is a *looser* fused covariance, never an overconfident
one. Anything cleverer (Bar-Shalom / Campo, information decorrelation) requires
knowing or modeling the correlation and is therefore gated behind proof and a
port (§4), never the default.

**Why a three-valued pedigree.** Collapsing "I don't know" into either Used or
NotUsed would be a lie with opposite failure modes: Unknown→NotUsed silently
permits overconfident fusion; Unknown→Used forbids every future optimization.
Keeping ignorance explicit lets the classification be conservative without
pretending to knowledge we lack, and gives the future tighter rule a correct
signal to key on.

**Why the gate uses P₁+P₂.** It is the correct innovation covariance only when
the two estimates are uncorrelated; under correlation it over-estimates the
spread, widening the gate. A wide gate errs toward *considering* a pair, which
association hysteresis then stabilizes — the safe direction for a fusion layer
whose job is continuity of presence.

**Why trace, fixed-iteration search, sequential N-way.** trace targets the
operationally meaningful position spread; fixed iterations buy determinism
cheaply (ω is scalar); sequential folding is simple and deterministic given a
canonical order. Each choice trades a small, documented sub-optimality for
determinism and simplicity, with the better alternative recorded in §4.

## 4. Ways to improve / what to test next

- **Independence-exploiting fusion behind `IFusionRule`.** For a
  ProvablyIndependent pair, Bar-Shalom / Campo fusion with modeled
  cross-covariance, or information decorrelation from the full pedigree, is
  tighter than CI. Add it as a second `IFusionRule` concrete, keyed on the
  ProvablyIndependent verdict, and ship it only with measured proof (fused NEES
  stays consistent, GOSPA improves) on the bench scenarios. Experiment: A/B the
  new rule vs CI on `t2t_disjoint` (should tighten) and confirm no regression on
  `t2t_shared_ais` (must stay CI-safe).
- **Batch CI** (a single joint ω vector on the simplex over all N inputs)
  instead of sequential pairwise folding — removes the small order dependence.
  Experiment: compare fused trace and NEES on ≥3-source scenarios.
- **Determinant (or information) objective** instead of trace for the ω search.
  Experiment: measure fused-covariance shape (not just trace) vs truth.
- **Input plausibility de-weighting.** The conflict scenario (biased,
  overconfident source) quantifies how much CI limits the damage; its numbers
  seed an edge rule that inflates a source's stated covariance when its recent
  innovations are implausibly large. Experiment: sweep an inflation factor and
  plot fused NEES vs the factor on the conflict scenario.
- **Fused-level feedback** into navtracker (explicitly deferred): the fused
  estimate is not fed back into the base tracker in this design, to avoid a new
  double-counting loop one level down.
