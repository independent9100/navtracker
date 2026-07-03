# Implementer prompt — consumer integration guide + keep-in-sync rule

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: 2026-07-03 discussion — the repo's documentation is deep but
organized by *algorithm* (for designers) and by *concept* (for learners);
there is no document organized by *integration surface* (for a library
consumer). Evidence: ~17 `Config` structs scattered across headers with no
index; the TTM/TLL pitfall and the `heading_std_deg{0.0}` "perfect gyro"
default both lived in code until an integrator asked.

---

You are working in the navtracker repo (C++17 maritime multi-sensor fusion
tracker; read `CLAUDE.md` first). Your task has three deliverables:

1. **`docs/integration-guide.md`** — a consumer-facing guide organized by
   "what you have / what you want", indexing every knob and wiring option.
2. **A keep-in-sync rule in `CLAUDE.md`** — mirroring the existing
   `docs/learning/` rule, so future consumer-facing features MUST update
   the guide.
3. **A drift-guard test** — mechanically enforcing that every consumer
   config struct is mentioned in the guide.
4. **Per-directory orientation READMEs** — one short `README.md` in each
   top-level module directory (`core/`, `ports/`, `adapters/`, `app/`,
   `tests/`): 5–15 lines saying what lives here, the two or three entry
   points a reader should start from, and links to the relevant deep docs
   and to the integration guide. NOT per-subdirectory (that drifts), and
   no content of its own — orientation and links only. The same Step-1
   inventory feeds these.

## Ground rules

- **Index, don't duplicate.** Deep documentation already exists
  (`docs/algorithms/`, `docs/sensors/sensor-reference.md`,
  `docs/output-contract.md`, `docs/learning/`, ADRs). A guide entry is:
  two-to-four sentences of plain English, the config struct with the
  defaults worth changing, a short wiring snippet, and links into the deep
  docs. If you find yourself writing more than ~15 lines of prose for one
  entry, you are duplicating a deep doc — link it instead. Single source
  of truth stays in the deep docs.
- **Every factual claim is read from HEAD, not from memory or from this
  prompt.** Every default you cite must come from the header at the time
  you write it. Every snippet must be copied from working code
  (`app/example.cpp`, `tests/integration/test_full_stack_pipeline.cpp`,
  other tests) or verified to compile; cite the source file next to each
  snippet.
- **Consumer surface only.** The boundary is the CMake targets: what a
  consumer of `navtracker_core` + `navtracker_nmea` touches is IN
  (Measurement builders, OwnShipProvider, Tracker/TrackManager wiring,
  ports/sinks, adapters and their configs, output types). Bench/sim
  internals are OUT (`navtracker_sim`, `core/benchmark/` Sweep knobs,
  scenario builders, replay test harnesses). State this boundary in the
  guide's intro so future authors know what belongs.
- **Plain English** (the repo's learning-docs tone rules apply: short
  sentences, define jargon on first use, no wall of text). The reader is a
  competent C++ engineer who knows ships, not this codebase.
- Do not restructure or rewrite any existing document. Small pointer
  additions to existing docs are fine.

## Step 1 — inventory (do this before writing anything)

Build the complete list of consumer-facing surface at HEAD:

- `grep -rn "struct.*Config" core/ ports/ adapters/ --include=*.hpp`
  (exclude tests, exclude `core/benchmark/`) — every hit is either IN the
  guide or explicitly out-of-scope with a reason.
- All interfaces in `ports/` (`ISensorAdapter`, `ITrackSink`,
  `ICollisionRiskSink`, `IStaticHazardSink`, `IDatumChangeSink`,
  `IHeadingBiasProvider`, `IBearingInnovationSink`, estimator/associator
  ports, …).
- All Measurement builders (`core/types/MeasurementBuilders.hpp`) and the
  four `MeasurementModel` kinds with their value/covariance layouts.
- All output types (`core/output/TrackOutput.hpp`,
  `core/output/StaticHazardOutput.hpp`, `core/collision/` evaluators).
- The adapters (`adapters/ais|arpa|eoir|own_ship|sinks`).
- CLAUDE.md's "Library use" section and `docs/sensors/sensor-reference.md`
  — the guide supersedes neither; it links both.

Record the inventory as a checklist in your working notes; the guide is
done when every item is either covered or listed as out-of-scope.

## Step 2 — the guide

`docs/integration-guide.md`, suggested skeleton (adjust if the inventory
argues otherwise, but keep the by-situation organization):

1. **Minimum viable integration** — the shortest path: construct
   `OwnShipProvider`, push a pose, build Measurements, `Tracker::process`,
   drain via `toTrackOutput`. One compact snippet (from `app/example.cpp`).
   Name the two CMake targets and when you need which.
2. **Own-ship, datum, and moving far** — auto-datum, the 30 km
   auto-recenter, `IDatumChangeSink`, and the register-your-caching-
   components gotcha (obstacle model, coastline, occupancy). State
   plainly: the datum is bookkeeping, not a detection window; nothing is
   dropped for being far away. MUST also answer (asked by a real
   integrator 2026-07-03, verify each at HEAD):
   - *Pose cadence:* you do NOT need an own-ship pose every tick.
     `poseAtOrBefore(t)` is a zero-order hold on the pose history — each
     measurement is projected with the latest fix at-or-before its own
     timestamp. Requirement: at least one pose before the first
     relative/projected measurement (adapters drop or return empty
     measurements otherwise — name the exact behavior per adapter).
     Out-of-order pushes are handled since 2026-07-03 (sorted insert;
     equal timestamps → last push wins) — verify at HEAD and say so.
   - *Multiple poses per tick:* push them ALL, in time order — more
     fixes mean better time alignment per measurement; the provider
     keeps a history, not a single latest value.
   - *Stale-pose caveat (pitfall list too):* there is currently NO
     staleness gate — if nav data drops out, relative measurements keep
     projecting with the last pose however old it is, and the growing
     error is absorbed silently. Say so honestly.
3. **Feeding sensors, by what your sensor gives you:**
   - absolute position (AIS-style) → `makeMeasurementFromEnuPosition`;
   - range + bearing (radar/EO-IR/sonar) → relative vs true builder,
     covariance composition, cross-range growth with range;
   - bearing-only → `Bearing2D` semantics: updates/corroborates only,
     `canInitiateTrack == false`, wedge-not-ellipse intuition;
   - no uncertainty info → `applyDefaultsIfEmpty` +
     `pessimisticSensorDefaults`, and what `covariance_is_default` means.
   Per-measurement covariance: R is per measurement, so range-dependent σ
   is the adapter's job and fully supported.
4. **NMEA path** — the adapters as one optional implementation; the
   TTM-vs-TLL rule (link `docs/sensors/sensor-reference.md` §2 — do NOT
   duplicate the table); `OwnShipNmeaAdapter` talkers.
5. **Heading and bias** — where heading comes from (`OwnShipPose`), the
   `heading_std_deg{0.0}` pitfall, `HeadingBiasEstimator` with its five
   observation kinds and which sources feed which (the CLAUDE.md table can
   move here or be linked — prefer moving it and leaving a pointer, so
   CLAUDE.md shrinks). MUST also explain the heading-σ split (integrator
   question 2026-07-03): the primary `heading_true_deg` has NO per-fix σ
   field — its random σ is configured statically per consuming adapter
   (`ArpaAdapterConfig.heading_std_deg` etc.) and its systematic part is
   the bias estimator's job; the v3 fields (`gps_true_heading_deg`,
   `magnetic_heading_deg`) DO carry per-fix σ because they feed bias
   observations. Also state what the v3 pose fields are actually consumed
   by at HEAD (as of writing: the NMEA adapter dispatches bias
   observations directly; the pose copies are informational) — verify
   before writing.
6. **Association hints** — `AssociationHints` scope rules: MMSI/platform_id
   consumed by the trackers; `sensor_track_id` is per-sensor (see its
   header comment), not consumed by association today. One short entry so
   integrators know what hints buy them.
7. **Getting results out** — pull (`mgr.tracks()` + `toTrackOutput`) vs
   push (`ITrackSink`, `ICollisionRiskSink`); CPA evaluation; static
   hazards (`IStaticHazardSink`, `StaticHazardEvaluator`,
   `toStaticHazardOutput`). Link `docs/output-contract.md`.
8. **Static environment inputs** — `StaticObstacle` (charted hazards),
   coastline/land model, and the live occupancy detector. Mark maturity
   honestly (e.g. occupancy detector: in active development, config may
   change — check the Stage 1b-ii plan).
9. **Choosing strategies** — estimator/associator ports are swappable;
   what the shipped defaults are; where the named configs live. Keep
   short; deep comparison lives in `docs/algorithms/`.
10. **Pitfall checklist** — one-liners with links: TLL when TTM exists;
   `heading_std_deg` left at 0; datum sink not registered; no pose pushed
   before first measurement; empty covariance never defaulted; expecting
   a bearing-only sensor to create tracks; treating MMSI/ARPA id as the
   fusion key.
11. **Config reference (appendix)** — a table: struct → header path → what
    it controls → the two or three fields most worth reviewing. Every
    consumer config struct from the Step-1 inventory appears here.

## Step 3 — the keep-in-sync rule (CLAUDE.md)

Add a section to `CLAUDE.md`, placed next to the existing
"Learning / foundations docs (REQUIRED to keep in sync)" section and
mirroring its force:

- Title: `## Integration guide (REQUIRED to keep in sync)`.
- Rule: whenever a change touches the consumer surface — a new or renamed
  config field or changed default, a new port/sink/builder/adapter, a new
  output field, a new named strategy config — the same PR must update
  `docs/integration-guide.md` (new entry or corrected entry, including the
  config-reference table). A PR that changes the consumer surface without
  a guide update is incomplete — same standard as the learning-docs rule.
- One sentence on persona so future authors keep the split: learning docs
  teach concepts to team members; the integration guide serves a consumer
  wiring the library; algorithm docs justify design decisions. New
  material goes to whichever persona it serves — usually not all three.
- Also: in the existing "Library use" section of CLAUDE.md, add a pointer
  line to the guide near the top. If you moved the heading-bias table
  (Step 2 §5), leave a pointer where it was.

## Step 4 — drift-guard test

Add a test that mechanically enforces the rule for the config surface
(the part that drifts silently):

- A gtest that, at test time, scans `core/`, `ports/`, `adapters/`
  headers (excluding tests and `core/benchmark/`) for
  `struct <Name>Config`, and asserts each found name appears as a
  substring in `docs/integration-guide.md`. Out-of-scope structs are
  allowed via an explicit exclusion list *inside the test* with a one-line
  reason each — so exclusion is a visible, reviewed act.
- This makes "added a Config, forgot the guide" a red test, not a review
  hope. Follow existing patterns for locating the source tree from test
  code (the fixture-loading tests do this); if source-tree access from the
  test runner proves genuinely awkward, ship the guide without the test
  and say so explicitly in your handoff summary.

## Acceptance

1. `docs/integration-guide.md` exists; every item from the Step-1
   inventory is covered or explicitly out-of-scope; every default and
   snippet verified against HEAD with source cited; by-situation
   organization; plain English.
2. CLAUDE.md carries the keep-in-sync section next to the learning-docs
   rule, plus the pointer in "Library use".
3. Drift-guard test green in the full suite (or explicitly waived with
   reason); full suite green; no behavior changes anywhere — this ticket
   is docs + one test only.
4. Handoff summary lists: inventory count (configs/ports/builders/outputs
   covered), anything declared out-of-scope and why, and any place where
   code and existing docs disagreed (report, don't silently pick one —
   those are bugs in one or the other).
