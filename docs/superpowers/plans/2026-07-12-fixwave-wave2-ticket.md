# Implementer prompt — fix wave, wave 2: the datum/own-ship cluster + sanitizer fixes + the fixture-skip guard

Status: ready to hand off. Paste everything below the line. Origin: the
pre-release review synthesis (`docs/reviews/2026-07-09-prerelease-open-points.md`
§B Theme 1, §C) — the review's dominant cluster: datum / own-ship position
handling at the edges. Several independently-confirmed findings point at
the same seam, so they get fixed as one project with the matching test
holes closed in the same PR. Budget ~2–3 days. TDD throughout; teeth
proofs per the #24 standard. Read each finding's verifier evidence in
`docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md` first.

**Base your branch on master AFTER the wave-1 merge lands** (the arbiter
will confirm; wave-1 touches OwnShipNmeaAdapter and the Foxglove call
sites — adjacent to this wave's files).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-fw2
-b fixwave-wave2`, own build dir, fixtures inner-level symlinked, 0-skip
runs; commit on your branch, never merge/push master). Keep the
`fixwave-wave1` BRANCH alive (it holds the held F2 commit for the F2
cycle); the fw1 worktree may be removed once the arbiter confirms the
wave-1 merge.

## W2.1 — Adapters cache a stale Datum across auto-recenter (HIGH)

`AisAdapter.cpp:24` (+ ARPA, EO-IR, RemoteTrack): each holds a private
`Datum` copy fixed at construction, never updated on auto-recenter → after
a recenter every non-cooperative measurement projects in the OLD frame.
Fix design is yours (query the datum source per-call, or register the
adapters as `IDatumChangeSink`s — pick one, justify in the design note;
per-instance either way, hexagonal direction preserved: adapters may
depend on core, never the reverse). Test: an end-to-end recenter scenario
— adapter-built measurements land in the NEW frame after a >30 km
own-ship move (this is the missing Section-D coverage for the whole
cluster). CLAUDE.md's datum-sink documentation gets the adapters added if
you go the sink route.

## W2.2 + W2.3 — `datumAxisRotation`: antimeridian wrap (HIGH) + wrong sign (MED)

`AxisRotation.cpp:15`: Δlongitude is not wrapped — a recenter across the
antimeridian applies a wildly wrong rotation. `AxisRotation.cpp:17`: the
rotation is applied as +γ where the correct transform is −γ (velocity /
covariance / IMM means / particles — worse than not rotating). **The unit
test `test_datum_shift.cpp:57` PINS the wrong sign — the suite encodes the
bug.** Fix both; correct the test with a comment stating the convention
and a worked numeric example; add an antimeridian-crossing test. Teeth:
re-flip the sign → both tests red → revert.

## W2.4 — `DeclaredSensorActivity`: coverage from the ENU origin + identity-blind retirement (HIGH ×2)

`DeclaredSensorActivity.cpp:12`: coverage is evaluated from the ENU origin,
not own-ship — wrong whenever own-ship is away from the datum.
`:51`: cooperative-overdue retirement is identity-blind and hard-deletes
radar-only tracks. Fix both. **A/B REQUIRED and reported prominently:** the
deployable config (`..._ivgate`, `use_sensor_activity = true`) may shift on
real replays — sims with own-ship at the origin are expected inert, HAXR/
philos may move. If the Cl-4 candidate's numbers move, that is a FINDING
for the arbiter to reconcile with the (possibly just-frozen) Cl-4 gauntlet
— report the delta, do not re-freeze anything yourself.

## W2.5 + W2.6 — sanitizer fixes (small, mechanical)

F-BUILD-1: `test_own_ship_nmea.cpp:275` binds `const auto&` to a
dereferenced temporary optional → use-after-scope; the test asserts on
garbage. One-line copy fix; confirm ASan-clean.
F-BUILD-2: UBSan null-pointer `memcpy` in the mcap writer via our foxglove
path (14 tests) — guard the adapter against nullptr/size-0 writes.

## W2.7 — the fixture-skip guard (closes the standing process hole)

The review confirmed our known trap (F-BUILD-3): ~36 real-data tests never
run under ctest (cwd-relative fixture paths + the partially-tracked-dir
symlink trap), and NOTHING catches a fixture-gated test that skipped when
it should have run (the 2026-07-08 red-master cause, still open). Build
the guard:

1. Anchor the cwd-gated tests with the `srcAbs(NAVTRACKER_SOURCE_DIR)`
   pattern their siblings already use (`test_los_guard_haxr_ab.cpp:44` is
   the template) so they run under ctest from any directory.
2. Add an opt-in strict mode — env `NAVTRACKER_REQUIRE_FIXTURES=1` turns
   every fixture-gate `GTEST_SKIP` into a FAILURE (a tiny helper macro
   wrapping the existing skip calls; default behavior unchanged). The
   verification ceremonies set it; casual runs don't.
3. Per-test ctest TIMEOUT properties (a hang currently has no watchdog).

## Acceptance

1. TDD paper trail per finding; teeth proofs for W2.2/2.3.
2. W2.4's A/B table (all bench workloads, deployable config called out);
   any candidate-config delta flagged to the arbiter as a finding.
3. The recenter end-to-end test, the antimeridian test, and the corrected
   sign test land with the fixes (Section-D backfill rides along).
4. Full suite green at 0 skips — and once W2.7 lands, re-run with
   `NAVTRACKER_REQUIRE_FIXTURES=1` to prove the guard passes on a fully
   wired tree and fails (teeth) when you hide one fixture dir.
5. Adversarial review before handoff (this touches core geo + adapters).
6. Write-up `docs/baselines/2026-07-12_fixwave_wave2.md` + eval-log entry.
7. Commit on your branch; do not merge or push master.
