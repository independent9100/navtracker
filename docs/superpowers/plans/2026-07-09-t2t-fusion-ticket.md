# Implementer ticket — track-to-track fusion (`navtracker_t2t`): fuse other trackers' tracks with pedigree-aware covariance intersection

Status: ready to hand off. Paste everything below the line. This is a
MULTI-SESSION build ticket (~2–4 weeks, 5 milestones, 2 checkpoints). Origin:
user decision 2026-07-09 — a "tracker of trackers" layer is wanted: navtracker's
output plus other trackers' tracks fused into one authoritative picture, where
the other trackers' input sensors are OFTEN UNKNOWN (they may have fused AIS
already, or run a fully independent chain). North-star tag: new deployment
capability on the Cl-3 line (the fused layer consumes the PMBM/MHT output).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` FIRST — every
rule there applies: hexagonal architecture, per-instance config (never
global/static), determinism, stable IDs, validate-at-edges, docs standards).
Worktree: `git worktree add ../navtracker-t2t -b t2t-fusion`, own build dir.
Fixtures from the MAIN tree; skip lists named BY NAME in every handoff; note
the ctest-from-build-dir trap (wire `build/tests` + `build/data` symlinks or
run the binary from the worktree root). No new third-party dependency is
needed or wanted (Eigen + gtest via existing Conan setup).

Because this spans multiple sessions: END EVERY SESSION with (a) all work
committed on the branch, (b) a short `HANDOFF.md` note in the worktree root
(NOT committed) stating milestone state, next step, and any open question for
the arbiter. Never leave uncommitted work overnight.

# 0. The problem and the one hazard that shapes everything

We receive tracks from N trackers (navtracker itself + external ones). Two
trackers may have consumed THE SAME sensor (e.g. both fused the same AIS
stream). Their errors are then CORRELATED. Fusing two correlated estimates as
if independent (naive Kalman/convex fusion) produces a fused covariance that
is TOO SMALL — the fused system is more confident than any input justifies.
That silent overconfidence is the classic "double counting" / rumor-propagation
failure, and it is the failure this design must make impossible BY DEFAULT.

Consequences baked into this ticket:

- The default fusion rule is **covariance intersection (CI)** — consistent for
  ANY unknown cross-correlation.
- Every input carries a **pedigree**: which sensor streams the source tracker
  used — with an explicit first-class "I don't know" state. Unknown NEVER
  degrades safety; it only forfeits future (tighter-fusion) optimizations.
- Anything cleverer than CI enters later behind a port, only with measured
  proof, never in this ticket.

# 1. Scope and non-goals

IN: the `navtracker_t2t` library target — input contract, pedigree model, CI
math, time alignment, track-to-track association, fused identity + lifecycle,
fused output, a navtracker→ExternalTrack self-adapter, sim-based two-tracker
evaluation harness, tests, docs (learning chapter + figures, integration
guide, algorithm doc, README/CLAUDE.md target lists).

OUT (record in the algorithm doc §4 "ways to improve", do not build):
Bar-Shalom–Campo fusion with modeled cross-covariance; information
decorrelation from full pedigree; input-tracker bias estimation; feedback from
the fused level back into navtracker; distributed/async network transport
(I/O is the consumer's problem — the contract is a struct, like Measurement).

# 2. Contracts (Milestone 1)

## 2.1 `ExternalTrack` — the input (analog of `Measurement`)

`core/t2t/ExternalTrack.hpp`. One struct per (source tracker, source track,
report time). Fields:

- `std::string source_tracker_id` — REQUIRED, non-empty. Identifies the
  reporting tracker instance (agreed at integration time, e.g. `"navtracker"`,
  `"bridge_arpa_1"`, `"shore_feed"`).
- `std::string source_track_id` — REQUIRED. The tracker-local track id,
  stable within that source. (source_tracker_id, source_track_id) is the
  source-track key. NEVER the fused key (invariant 5).
- `Timestamp time` — REQUIRED.
- Position: ENU meters against the shared datum (constructed via the same
  `OwnShipProvider`/datum machinery as Measurement; provide
  `makeExternalTrackFromGeodetic(lat, lon, ..., provider)` so consumers with
  lat/lon inputs never touch the datum by hand).
- `Eigen::Matrix2d pos_cov` — position covariance, m², ENU. May be zero-size/
  unset → apply `pessimisticExternalDefaults()` at the edge (same pattern as
  `applyDefaultsIfEmpty`).
- Velocity: optional (`bool vel_valid`); ENU m/s + 2×2 covariance when valid.
- `TrackStatus source_status` — optional lifecycle hint from the source
  (tentative/confirmed/coasted), `Unknown` allowed.
- `attributes` — mmsi / name / platform_id hints, same semantics as today:
  evidence, never keys.
- `SourcePedigree pedigree` — see §2.2. An ABSENT pedigree must behave
  IDENTICALLY to an all-Unknown pedigree (tested invariant).

Edge validation (validate-at-edges rule): reject NaN/inf, non-PSD covariance,
empty required ids, timestamps violating per-source monotonicity (same
stale-drop pattern as the Tracker high-water mark — per SOURCE, since sources
have independent clocks/latencies; count drops, expose a counter).

## 2.2 `SourcePedigree` — "which sensors did you use?" with honest ignorance

`core/t2t/Pedigree.hpp`:

```cpp
enum class SensorUsage { Used, NotUsed, Unknown };

struct SourcePedigree {
  // Sensor-stream identifiers are free-form strings agreed at integration
  // time; matching is EXACT-STRING. Same physical stream => same id
  // (e.g. "ais:region_feed", "radar:own_xband", "eoir:port_cam").
  std::map<std::string, SensorUsage> sensors;
  SensorUsage default_usage = SensorUsage::Unknown;  // anything not listed
};
```

Design rules:

- Three-valued on purpose. `Unknown` is the honest default everywhere; the
  user explicitly required "I don't know what has been used" as a first-class
  state.
- Pedigree is registered PER SOURCE TRACKER (a
  `T2tFuser::registerSource(source_tracker_id, SourcePedigree)` call at
  wiring time), with optional per-ExternalTrack override for sources whose
  usage varies per track (navtracker itself: fill from the track's
  `contributing_sources` — that is the self-adapter's job, §5).
- **The pairwise independence rule** (pure function, exhaustively unit-tested):
  two sources are `ProvablyIndependent` iff for EVERY sensor id appearing in
  either pedigree (and considering both defaults) it is impossible that both
  used the same stream — i.e. no sensor id resolves to Used/Used, and no
  Unknown on either side could overlap a Used or Unknown on the other.
  Anything else → `PossiblyCorrelated`. In v1 this classification changes
  DIAGNOSTICS and attribute handling, NOT the fusion math (CI regardless);
  it is the hook the future tighter rules key on, and it must be correct now.
- Attribute double-counting: if two inputs both declare `ais:* = Used` and
  carry the same MMSI, the shared MMSI is association corroboration but NOT
  independent identity evidence twice. If they carry DIFFERENT non-empty MMSIs
  → strong soft penalty against associating them (soft, not hard — invariant
  5: kinematics win when evidence disagrees).

## 2.3 Fused output

Reuse the existing output philosophy: a `FusedTrackOutput` that embeds/extends
`TrackOutput` (stable fused `track_id`, lat/lon + NED covariance, SOG/COG with
validity, lifecycle, last_update) plus:

- `contributing_trackers`: list of (source_tracker_id, source_track_id,
  last_seen) currently associated.
- `independence_class`: `ProvablyIndependent` / `PossiblyCorrelated` /
  `SingleSource` — the pedigree verdict for what is currently fused.
- `fusion_rule`: `"CI"` (future-proofing string, constant in v1).
- `covariance_is_pessimistic_default` flag (same diagnostic spirit as
  `covariance_is_default`).

Push output through the existing `ITrackSink` shape (a `IFusedTrackSink` with
the same four lifecycle events) so consumers get the identical push/pull dual
they already know. Pull = `fuser.fusedTracks()`.

# 3. The math (Milestone 1 for CI, Milestone 2 for the engine around it)

Document all of this in `docs/algorithms/t2t-fusion.md` with the four
mandatory sections (math / assumptions / rationale / ways to improve).

## 3.1 Time alignment

All fusion happens at the engine's current time `t` (time-driven, message
timestamps, NEVER wall clock — architecture invariant 4). Each stored source
track is predicted from its last report time to `t` with a CV model:

    x' = F(dt) x,   P' = F(dt) P F(dt)ᵀ + Q_t2t(dt)

`Q_t2t` = white-acceleration CV process noise with per-instance
`t2t_process_noise_accel` (default documented; same form as the core CV
model — reuse it, don't re-implement). A source track older than
`max_report_age_s` stops contributing (its fused track coasts / demotes per
§3.5) — never extrapolate stale inputs forever.

## 3.2 Track-to-track association (which of B's tracks is my track?)

Pairwise gate between fused-track prediction and each candidate source track
(both at time `t`):

    d² = (x₁ − x₂)ᵀ (P₁ + P₂)⁻¹ (x₁ − x₂)   [position block only]

Note in the doc: using P₁+P₂ ignores cross-correlation, which makes the gate
CONSERVATIVE (too wide) under positive correlation — acceptable; document it.
Gate at χ²₂ `t2t_gate_prob` (default 0.99). Assignment: reuse the existing
Hungarian on the gated cost matrix (cost = d² plus attribute terms: shared
MMSI bonus, conflicting-MMSI penalty — both soft, magnitudes config fields).

**Pairing hysteresis** (anti-flicker): a (fused, source-track) pairing forms
after `pair_m_of_n` consistent assignments and survives `pair_break_misses`
missed/failed ones. While paired, that source track bypasses global assignment
into its fused track unless the gate REJECTS it hard (then the pairing starts
breaking). This mirrors the M-of-N spirit used elsewhere in the repo.

## 3.3 Covariance intersection (the core, pure and unit-testable)

`core/t2t/CovarianceIntersection.hpp`, free functions, no engine dependency.

Two estimates (x₁,P₁), (x₂,P₂) of the same state:

    P_f⁻¹ = ω P₁⁻¹ + (1−ω) P₂⁻¹
    x_f   = P_f ( ω P₁⁻¹ x₁ + (1−ω) P₂⁻¹ x₂ ),   ω ∈ [0,1]

Choose ω minimizing trace(P_f) (document why trace over det: position trace is
the operationally meaningful spread; det is the classical alternative — note
it in "ways to improve"). Optimize by golden-section on [0,1] with fixed
iteration count (DETERMINISM: fixed iterations, no convergence-dependent
step counts) — ω is scalar, this is cheap and exact enough.

Consistency property (state it, test it): if each input is consistent
(E[(x̂−x)(x̂−x)ᵀ] ⪯ P), the CI fusion is consistent for ANY cross-correlation.
That is the entire justification for CI-by-default under Unknown pedigree.

Plain-words version for the learning chapter: naive fusion of two reports that
secretly share a source is like counting one newspaper article as two
independent confirmations because two friends both read it to you. CI is the
rule that refuses to become more certain than the more careful reading of the
two, unless genuine independence is PROVEN.

>2 sources: fuse SEQUENTIALLY in canonical order (sort source_tracker_id
lexicographically) — deterministic and simple; the order-dependence is real
but small, document it and list batch-CI (joint ω vector, weights simplex) in
"ways to improve".

State-dimension mismatch: fuse the position block always; fuse velocity only
when BOTH sides have `vel_valid` (else adopt the valid one's velocity with its
own covariance, flagged). Never invent velocity.

## 3.4 What pedigree changes in v1 (and what it does NOT)

- Fusion math: NOTHING. CI for every pair, including ProvablyIndependent ones
  (CI is merely conservative there). This is deliberate: no correctness risk
  rides on pedigree declarations being right.
- Output: `independence_class` per fused track (operator/diagnostic value).
- Attributes: the double-counting rule of §2.2.
- Future: `ports/IFusionRule.hpp` — interface with the CI implementation as
  the only concrete. A tighter rule for ProvablyIndependent pairs is a later,
  measured, opt-in change behind this port. Build the port, ship only CI.

## 3.5 Fused identity and lifecycle (presence over classification, one level up)

- `fused_track_id`: minted by the fuser, monotonically increasing, NEVER
  reused (invariant 5 verbatim).
- Birth: an unassociated source track births a fused track once seen
  `fused_confirm_m` of `fused_confirm_n` reports (single-source fused tracks
  are legitimate — one tracker seeing something is presence; ADR 0002's
  spirit applies at this level: never let "only one tracker sees it" suppress
  an object into nothing).
- Confirmed: immediately if the source reports `Confirmed` status and
  `trust_source_status` (config, default true); else own M-of-N.
- Coast/demote: no contributing source fresher than `max_report_age_s` →
  coast with inflating covariance; delete after `fused_delete_age_s`.
- A pairing breaking (§3.2) does not delete the fused track while another
  source sustains it — continuity through single-input dropout is one of the
  headline wins to measure.

# 4. Structure (where things live)

```
core/t2t/ExternalTrack.hpp          input type + builders + edge validation
core/t2t/Pedigree.hpp               SensorUsage/SourcePedigree + independence rule
core/t2t/CovarianceIntersection.hpp pure CI math
core/t2t/T2tAssociator.hpp/.cpp     gate + Hungarian reuse + pairing hysteresis
core/t2t/T2tFuser.hpp/.cpp          engine: ingest, align, associate, fuse, ids
core/t2t/T2tConfig.hpp              ALL knobs, per-instance, documented defaults
core/t2t/FusedTrackOutput.hpp       output type + toFusedTrackOutput(...)
ports/IFusionRule.hpp               CI as sole impl
ports/IFusedTrackSink.hpp           push events (nullable, null = no overhead)
adapters/t2t/NavtrackerSource.hpp   ITrackSink→ExternalTrack self-adapter (§5)
app/example_t2t.cpp                 end-to-end wiring example
tests/t2t/...                       unit tests
tests/integration/test_t2t_full_stack.cpp
tests/benchmark/test_t2t_scenario_run.cpp (sim-gated)
```

CMake: new consumer target `navtracker_t2t` (links `navtracker_core`);
`T2tFuser` must implement `IDatumChangeSink` (it caches ENU state — the
auto-recenter rule from CLAUDE.md applies; add it to the CLAUDE.md datum-sink
list and the integration guide when you wire it).

# 5. The navtracker self-adapter

`adapters/t2t/NavtrackerSource.hpp`: an `ITrackSink` implementation that
converts each navtracker track event into an `ExternalTrack` with
`source_tracker_id` from ctor, `source_track_id` = the navtracker track_id,
covariance from the track state, and pedigree AUTO-FILLED per track from
`contributing_sources` (navtracker is the one source whose pedigree we know
exactly — per-track, `default_usage = NotUsed`). This gives every consumer a
zero-effort first input and gives our tests a realistic tracker-in-the-loop.

# 6. Evaluation harness, scenarios, metrics (the heart of the ticket)

## 6.1 Harness

Extend the sim-multisensor generator (`tests/fixtures/sim_multisensor/`) with
a **two-tracker-view** mode: from one truth scenario, produce per-tracker
measurement streams per an arm spec, run N independent tracker instances
(reuse the bench configs), convert each output via the §5 adapter (pedigree
per arm spec), feed the fuser, score the FUSED output against truth with the
EXISTING metrics harness (GOSPA c=20 like Cl-1, plus NEES of fused estimates
vs truth, id_switches, track_breaks, lifetime). Deterministic: seeded,
regeneration byte-identical (spot-check like the imazu suite).

Also run a **naive-fusion baseline** (same engine, fusion rule = precision-
weighted Kalman merge assuming independence) — implemented ONLY inside the
bench (a test-only IFusionRule impl, never a shipped config) as the
scientific control that shows WHY CI.

## 6.2 Scenarios (each = fixture + test + row in the results doc)

1. `t2t_disjoint` — tracker A: radar-only; tracker B: AIS-only; pedigrees
   declared, ProvablyIndependent. EXPECT: fused GOSPA ≤ best single input
   (banded); fused NEES within χ² band; independence_class correct.
2. `t2t_shared_ais` — A: radar+AIS, B: AIS-only, SAME ais stream id, declared
   Used/Used → PossiblyCorrelated. THE DOUBLE-COUNTING GATE: CI-fused NEES
   stays within band; the naive baseline's NEES violates it by a large factor
   (assert the RATIO with a generous band — #24 rules, no exact pins).
3. `t2t_unknown` — same data as (2), pedigrees all-Unknown (and one arm with
   pedigree entirely absent). EXPECT: identical fusion math/NEES as (2)
   (CI regardless), independence_class = PossiblyCorrelated, and
   absent-pedigree behaves byte-identically to explicit-Unknown.
4. `t2t_dropout` — B goes silent for 60 s mid-scenario, resumes. EXPECT:
   fused track continuity (no fused id change), covariance inflates then
   recovers, no spurious second fused track on B's return (pairing
   re-forms). Latency skew sub-case: B reports 2 s late throughout.
5. `t2t_conflict` — B is deliberately biased (+150 m) and overconfident
   (covariance claims 5 m). EXPECT (document, banded): CI limits the damage
   vs naive; fused NEES degradation quantified; this scenario's numbers seed
   the "input validation / de-weighting" entry in ways-to-improve. No
   pass/fail heroics — it is a characterization row.
6. `t2t_cross` — two targets crossing (imazu_17-style geometry, two-tracker
   views). EXPECT: no cross-pairing (fused id_switches banded low), the
   MMSI-conflict penalty measurably reduces wrong pairings in an A/B.
7. Determinism: any scenario replayed twice → byte-identical fused output
   stream (the repo's standing determinism test, at the fused level).

## 6.3 Unit-test cases (Milestone 1–2, before any scenario)

- CI math: hand-computed 2×2 cases (equal covs → ω=0.5 by symmetry; one
  input infinitely uncertain → passthrough; ω endpoints); property tests
  over randomized-but-seeded PSD matrices: P_f consistent bound holds,
  trace(P_f) ≤ min(trace P₁, trace P₂) never violated in the wrong
  direction (CI never SMALLER than the honest floor), fusion of a track
  with itself is (numerically) that track.
- Pedigree independence rule: full truth table incl. defaults —
  Used/Used same id → correlated; Used/NotUsed all ids + NotUsed defaults →
  independent; ANY Unknown reachable overlap → correlated; absent pedigree ≡
  all-Unknown.
- Edge validation: NaN, non-PSD, empty ids, per-source stale timestamps.
- Association: gate math vs hand χ² values; MMSI conflict is soft (a huge
  kinematic match still wins — construct it); pairing hysteresis M-of-N
  state machine.
- Lifecycle: birth/confirm/coast/delete transitions on scripted sequences;
  id never reused.

All assertions: banded/structural (#24 discipline — and after case (5),
NO cross-config comparisons on marginal outcomes, no exact pins on
association results).

# 7. Milestones and checkpoints

- **M1 — contracts + math (≈2 sessions).** ExternalTrack, Pedigree +
  independence rule, CI functions, edge validation. Unit tests of §6.3
  (first three bullets) green. CHECKPOINT 1 (cheap, design): post the
  header-level API + the algorithm-doc math section to the arbiter before
  building the engine.
- **M2 — engine (≈2–3 sessions).** T2tFuser + associator + lifecycle +
  outputs + sinks + self-adapter. All §6.3 unit tests green. Determinism
  test green with scripted fake sources.
- **M3 — harness + core scenarios (≈2 sessions).** Two-tracker-view
  generator mode, scenarios 1–3 + 7, metrics wired, results doc
  `docs/baselines/<date>_t2t_gates.md` started. **CHECKPOINT 2 (measured):
  bring the scenario-2 NEES table (CI vs naive) to the arbiter. This is the
  ticket's load-bearing number — do not proceed while it fails.**
- **M4 — robustness + integration (≈2 sessions).** Scenarios 4–6;
  `test_t2t_full_stack.cpp` (synthetic streams → two live tracker instances
  → adapter → fuser → recording fused sink; asserts fused lifecycle events,
  continuity through dropout, independence_class); example app.
- **M5 — docs + closeout (≈1–2 sessions).** All of §8; full suite green,
  skips named; final handoff with the results table.

# 8. Documentation set (same-PR requirements, per CLAUDE.md)

1. `docs/algorithms/t2t-fusion.md` — the four mandatory sections. "Ways to
   improve": Bar-Shalom–Campo / decorrelation behind IFusionRule (keyed on
   ProvablyIndependent), batch-CI ω vector, det-vs-trace criterion, input
   plausibility de-weighting (from scenario 5's numbers), fused-level
   feedback (explicitly deferred).
2. `docs/learning/` — NEW numbered chapter "Fusing trackers: track-to-track
   fusion and covariance intersection". Plain English: the shared-newspaper
   metaphor for double counting; why "I don't know" must be a first-class
   answer; CI as "never claim to know more than your most careful friend".
   Figures via `docs/learning/figures/generate.py` (`fig_t2t_*`): the
   ellipse triptych — P₁, P₂, naive-fused (dangerously small) vs CI-fused
   (honest) — and a two-tracker double-counting cartoon. Update
   `00-index.md` + glossary (CI, T2T, pedigree, double counting).
3. `docs/integration-guide.md` — new section "You have tracks from other
   trackers" (wiring: registerSource + pedigree how-to incl. the Unknown
   default; the self-adapter; datum-sink registration; output fields), plus
   config-appendix rows for `T2tConfig` (the drift-guard test enforces the
   mention — extend its scan to `core/t2t/` if it doesn't already cover it).
4. `README` + `CLAUDE.md` — add `navtracker_t2t` to the consumer-target
   lists; add `T2tFuser` to the datum-sink registration list; one-paragraph
   module-layout entry for `core/t2t/`.

# 9. Acceptance (final handoff)

1. All milestones done; suite green in your worktree with fixtures wired,
   skips named by name; determinism tests (core + t2t) green.
2. Scenario results doc committed with all 7 rows + the CI-vs-naive NEES
   table; every number reproducible (commands + checksums in a dated
   eval-log entry per milestone that shipped numbers).
3. Both checkpoints were honored (paper trail in the handoffs).
4. Zero changes to existing tracker behavior: `navtracker_core` +
   `navtracker_nmea` byte-identical for non-t2t consumers (prove: full
   existing suite untouched-green before your docs/test additions land, and
   no existing header modified except additive CMake/docs — if you find you
   need to modify core, STOP and report with the reason).
5. Docs set of §8 complete — a PR missing the learning chapter or the
   integration-guide/config-appendix entries is INCOMPLETE by house rule.
6. Stop-and-report triggers: CI cannot pass the scenario-2 gate (would be a
   math/implementation bug — do not tune around it); the two-tracker-view
   harness needs generator changes that would break existing fixtures'
   byte-identical regeneration; any need to touch the existing Tracker/PMBM
   update paths.
