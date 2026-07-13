# Implementer prompt — fix wave, wave 3: the heading/sensor-bias chain (one repair project) + the input angle-convention audit

Status: ready to hand off. Paste everything below the line. Origin: the
pre-release review synthesis (`docs/reviews/2026-07-09-prerelease-open-points.md`
§B Theme 3) — four independently-confirmed defects in the heading/bias
estimation chain that the synthesis says to "treat as one repair project."
All four are correctness bugs in the same subsystem, so they are fixed and
tested together. Also folds in the user's 2026-07-12 input angle-convention
rider (this subsystem is where the five inconsistent angle conventions
live). Budget ~2–3 days. TDD throughout; teeth proofs per the #24 standard.
Read each finding's verifier evidence in
`docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md` first.

**Runs in PARALLEL with wave 2** (disjoint files: wave 2 = geo/adapters/
datum; wave 3 = bias estimators + pair extractor). Branch from current
master; the arbiter merges whichever lands first and rebase-verifies the
second. **This wave UNBLOCKS the F2 provenance cycle** (`2026-07-12-f2-
provenance-cycle-ticket.md`), which must be measured on a correct bias
chain — say so in the handoff.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-fw3
-b fixwave-wave3`, own build dir, fixtures inner-level symlinked, 0-skip
runs; commit on your branch, never merge/push master).

## W3.1 — Closed-loop heading-bias double-subtraction (HIGH)

`HeadingBiasEstimator.cpp:64`: the estimator provably converges to HALF the
true bias — the correction is applied to an observation that has already
had the current estimate subtracted, so the loop double-counts. Fix the
feedback so the innovation is formed against the RAW observation (per the
finding's mechanism). Test: inject a known constant gyro bias `b` into a
synthetic stream; assert the estimate converges to `b` (banded, e.g.
within 5% after N updates), NOT `b/2`. Teeth: the pre-fix code converges to
`b/2` → the convergence test fails on it → the fix makes it pass.

## W3.2 — Per-sensor bias loop, same feedback defect (HIGH)

`SensorBiasEstimator.cpp:99`: the same double-subtraction shape in the
per-sensor bias loop. Fix identically; same convergence-to-`b`-not-`b/2`
test for a per-sensor injected bias.

## W3.3 — v1 AIS/ARPA pair extractor: bearings about the datum origin, not own-ship (HIGH)

`AisArpaPairExtractor.cpp:48`: bearings are computed about the ENU datum
origin instead of own-ship position, so every innovation is geometrically
wrong whenever own-ship is away from the datum. Fix to use own-ship pose.
Test: own-ship displaced ~10 km from the datum, a target at a known
relative bearing → the extracted pair's bearing matches the true own-ship-
relative bearing (banded), and is WRONG (fails) under the datum-origin
formula. (This overlaps wave 2's datum theme conceptually but is a distinct
file — coordinate with the wave-2 implementer only if git says you touch a
shared line, which you should not.)

## W3.4 — Sign convention inconsistent across the five observation kinds (HIGH)

`HeadingBiasEstimator.cpp:46`: the five observation kinds mix a marine-
compass frame (N=0, clockwise-positive) with the ENU-math frame (E=0,
counter-clockwise-positive) inconsistently, so some observation kinds feed
a sign-flipped innovation. Establish ONE internal convention (document it),
convert each observation kind at its boundary, and add a per-kind test that
each of the five, fed a known bias, moves the estimate in the CORRECT
direction. This is the load-bearing fix — W3.1/W3.2 convergence is only
meaningful once every kind agrees on sign.

## W3.5 — the input angle-convention audit (user rider 2026-07-12)

Same principle as F3's dual-API naming, applied to inputs. This subsystem
is where angle ambiguity concentrates, so the audit lives here:

1. Every angle field this chain consumes (`Measurement`, `OwnShipPose`, the
   five observation kinds, the pair extractor's I/O) gets a doc comment
   naming its **zero reference and turn direction** (e.g. "bearing: radians,
   0 = true north, clockwise-positive (marine)" vs "…, 0 = east,
   counter-clockwise (ENU math)").
2. Audit `Measurement`/`OwnShipPose` for any field whose unit/frame is not
   self-evident from its name/type; report the list (fixing beyond comments
   is out of scope unless it's a one-line rename with no call-site churn —
   flag larger renames for a follow-up, do not sprawl this wave).

## Docs (same PR — house rule)

- **`docs/learning/`** heading-bias chapter: update the five-kinds table and
  the sign convention (easy English); if a figure shows the loop, note the
  corrected feedback. Regenerate figures via `figures/generate.py` (never
  hand-edit).
- **`docs/integration-guide.md` §5** ("Heading and bias") — correct any
  wiring/convention statement the fixes touch; add the angle-convention
  naming from W3.5.
- **`docs/algorithms/`** bias doc: math/assumptions/rationale/ways-to-improve
  updated for the corrected feedback and the unified sign convention.

## Acceptance

1. TDD paper trail per finding; convergence-to-`b` (not `b/2`) tests for
   W3.1/W3.2; per-kind direction tests for W3.4; the off-datum bearing test
   for W3.3 — all teeth-proven against the pre-fix code.
2. **A/B on the deployable configs:** the bias chain feeds tracking on any
   config using bias correction — report whether the autoferry/HAXR/philos
   numbers move (they may: this is a correctness fix on a live input). A
   material move on the Cl-4 candidate is a FINDING for the arbiter to
   reconcile against the just-frozen Cl-4 gauntlet — report the delta, do
   not re-freeze.
3. Full suite green at 0 skips; adversarial review before handoff (core
   estimator math).
4. Write-up `docs/baselines/2026-07-13_fixwave_wave3.md` + eval-log entry;
   note in the write-up that this unblocks the F2 provenance cycle
   (correct bias chain now available for its garbage-bias×broken-chain
   attribution question).
5. Commit on your branch; do not merge or push master.
