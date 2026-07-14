# Pre-release fix wave — wave 2 (2026-07-12/13)

Branch `fixwave-wave2` off master `f9ad004` (after the wave-1 merge `fa4db84`).
Origin: the 2026-07-09 pre-release review synthesis
(`docs/reviews/2026-07-09-prerelease-open-points.md` §B Theme 1, §C), implementer
ticket `docs/superpowers/plans/2026-07-12-fixwave-wave2-ticket.md`. The dominant
review cluster — datum / own-ship handling at the edges — plus the two sanitizer
fixes and the standing fixture-skip process hole. TDD throughout: failing test
first (or teeth-proof after), then fix, then green. Commits are separated per
finding on this branch. `fixwave-wave1` branch is preserved (holds the held F2).

## Findings shipped

| ID | Sev | What | Commit |
|----|-----|------|--------|
| W2.1 | HIGH | Adapters cache a stale Datum across auto-recenter | `19c24d5` |
| W2.2 | HIGH | `datumAxisRotation` doesn't wrap Δlongitude (antimeridian) | `2a76402` |
| W2.3 | MED | `datumAxisRotation` applies +γ where −γ is correct | `2a76402` |
| W2.4a | HIGH | Sensor-activity coverage measured from ENU origin, not own-ship | `59254e0` |
| W2.4b | HIGH | Cooperative-overdue retirement is identity-blind (hard-deletes radar-only tracks) | `59254e0` |
| W2.5 | build | `test_own_ship_nmea.cpp` dangling reference (ASan) | `45f5d38` |
| W2.6 | build | mcap writer null-pointer `memcpy` via empty schema (UBSan) | `45f5d38` |
| W2.7 | process | Fixture-skip guard (anchoring + strict mode + timeouts) | (this branch) |

## W2.1 — adapters adopt the new datum + reproject on auto-recenter

**Finding.** `AisAdapter`, `ArpaAdapter`, `EoIrAdapter`, `RemoteTrackAdapter` each
held a private `Datum` copy fixed at construction and never updated it on an
`OwnShipProvider` auto-recenter. After a >30 km own-ship move every
non-cooperative measurement projected in the OLD ENU frame — silent position
corruption for every non-cooperative sensor whenever auto-recenter is enabled.

**Fix (design chosen: uniform `IDatumChangeSink`).** Each adapter now implements
`IDatumChangeSink`; on recenter it (1) swaps in the new datum for subsequent
projections and (2) re-expresses any already-buffered measurements into the new
frame (`adapters/util/DatumReproject.hpp` — position round-trip + velocity /
covariance rotation, mirroring `DatumShift`), so a single `poll()` after a
recenter never mixes old- and new-frame measurements. `IDatumChangeSink` was
extracted to its own header (`core/own_ship/IDatumChangeSink.hpp`, re-included by
`OwnShipProvider.hpp`) so AIS/RemoteTrack depend on the tiny sink interface, not
the whole provider. *Why the sink route over per-call datum query:* uniformity
across all four adapters, back-compatible constructors, and consistency with the
established pattern (`CoastlineModel` / `StaticObstacleModel` /
`LiveOccupancyModel` / `T2tFuser` are all sinks). Consumers MUST register each
adapter as a datum sink when auto-recenter is on — documented in CLAUDE.md and
integration-guide §2.

**Tests** (`tests/adapters/test_adapter_datum_recenter.cpp`, 5): all four adapters
land in the NEW frame after a recenter; a measurement buffered BEFORE the
recenter is reprojected. TDD: RED (registerDatumSink didn't compile) → GREEN.
Teeth: neuter AIS `onDatumRecentered` → the two AIS tests RED → restored.

## W2.2 + W2.3 — `datumAxisRotation` antimeridian wrap + correct −γ sign

**Findings.** (W2.2) Δlongitude was not wrapped, so a recenter crossing the ±180°
antimeridian fabricated a ~±360° rotation. (W2.3) The rotation re-expressing a
vector from the old datum's ENU axes into the new datum's is by **−γ** (the E,N
block of `R_new·R_oldᵀ`, verified numerically against the `Datum` ecef→enu map:
east unit (1,0) → (cos γ, −sin γ)); the code applied +γ — worse than not
rotating. `test_datum_shift.cpp:57` **pinned the wrong sign** — the suite encoded
the bug.

**Fix** (`core/geo/AxisRotation.cpp`). `std::remainder(Δlon, 360)` for the wrap;
`γ = −Δλ·sin(φ_mean)`. One function, three consumers (`DatumShift`, `T2tFuser`,
`TrackOutput::toGeodeticWithCov`) all corrected together. Convention documented
in `AxisRotation.hpp`.

**Tests.** Corrected `RotatesVelocityByConvergenceAngle` (−γ, with worked numeric
example); added `WrapsDeltaLongitudeAcrossAntimeridian` (crossing == equivalent
non-crossing +1° step); flipped the high-latitude
`TrackOutputTest.CovarianceRotatedAtHighLatitude` off-diagonal to −γ (the sign
flows through `toGeodeticWithCov`, per the cross-wave note). TDD: all three RED
against the buggy code, GREEN after. **Teeth:** re-flip the sign only → all three
RED → reverted → GREEN. No T2T test pins the sign (T2tFuser uses the function the
same way; 56 datum/output/T2T tests green).

## W2.4 — sensor-activity coverage from own-ship + identity-keyed cooperative overdue

**W2.4a.** `DeclaredSensorActivity::inCoverage` measured range/azimuth from the
ENU datum origin, assuming the sensor sits at the datum. Own-ship (where the
declared surveillance sensors are mounted) drifts up to the recenter threshold
from the datum, so coverage was wrong on any moving platform. Fixed:
`ISensorActivity::evaluate` takes `own_ship_enu`; `inCoverage` uses
`(track − own_ship)`. `PmbmTracker` captures own-ship ENU each scan from the last
surveillance measurement's `sensor_position_enu` and persists it across scans (so
the empty-scan misdetection branch keeps own-ship's last-known position).

**W2.4b.** `evaluate` was called with hardcoded `std::nullopt` identity and set
`cooperative_overdue` purely on elapsed time, so a radar-only track (no MMSI) was
marked overdue and hard-deleted once past `cooperative_stale_timeout_sec`. Fixed:
the Bernoulli's `b.mmsi`/`b.platform_id` are passed through, and
`cooperative_overdue` fires only for a track that carries a cooperative identity.

**Tests.** DeclaredSensorActivity: `RangeMeasuredFromOwnShipNotOrigin`,
`SectorAzimuthMeasuredFromOwnShip`, `CooperativeOverdueRequiresIdentity`. PMBM:
`CoverageMeasuredFromOwnShipPosition` (capture+thread) and
`RadarOnlyTrackNotRetiredByCooperativeTimeout` (the regression guard). Existing
cooperative tests seed a cooperative identity, matching the corrected semantics.
Teeth: neuter each fix → its tests RED, the rest green → restored. 172
PMBM/sensor/coverage tests green.

### W2.4 A/B on the deployable config `imm_cv_ct_pmbm_coverage_land_ivgate` — A FINDING FOR THE ARBITER

Bench: fix-ON (this branch) vs fix-OFF (W2.4 neutered) on the deployable config,
all bench workloads incl. `--with-haxr` (47 scenarios). Only W2.4 can move bench
metrics: the bench uses a single fixed datum per run (no recenter fires, so
W2.1/W2.2/W2.3 are inert) and scores in ENU (never via `toGeodeticWithCov`). So
the whole-wave bench delta on this config **equals** the W2.4 delta.

**Headline — the real deployment replays are byte-identical; two workload classes
move:**

- **`philos`, `philos_radartruth`, `haxr` (the real deployment targets):
  BYTE-IDENTICAL.** W2.4 does not disturb them. philos own-ship drift within the
  clip stays inside the 1000 m radar range so no coverage decision flips, and
  philos tracks carry AIS identity so the identity gate is a no-op.
- **`autoferry` (real, moving platform) — W2.4a (coverage).** autoferry declares
  no cooperative channel, so its entire shift is the own-ship coverage fix. The
  `_anchored` variants improve **dramatically and correctly** (e.g.
  `scenario3_anchored` OSPA 100.5→1.34, GOSPA 5.9→1.5; `scenario4_anchored` OSPA
  168.8→1.48, GOSPA 9.0→1.6; `scenario5_anchored` OSPA 118.8→1.33) — the
  anchored-target coverage was being evaluated from the wrong centre. The
  non-anchored variants are MIXED: OSPA mostly down but GOSPA up on some and
  track_breaks/id_switches up on several (`scenario4` id_switches 0→17.5,
  track_breaks 0→5.5; `scenario6` track_breaks 0→9; `scenario3` id_switches
  1→15.5). Mechanism: correct own-ship-centred coverage changes which sweeps
  count as "covered", shifting miss-decay and thus track lifecycle on the moving
  platform.
- **`dense_clutter`, `dense_clutter_datum` (sim) — W2.4b (identity).** These sims
  declare an AIS-only (Cooperative) channel and place own-ship at the origin
  (so W2.4a is inert here). Before the fix the identity-blind cooperative timeout
  retired the identity-less clutter-born tracks at 120 s — incidentally culling
  phantoms. After the correct fix those radar-only tracks are no longer retired
  by AIS silence, so phantoms persist: GOSPA +3…+6, card_err shifts toward
  overcount (−1.5→−0.18), lifetime_ratio up. This is a *correct fix removing an
  incidental (wrong-reason) suppression* — the same shape as wave-1 F2.
- **All other sims** (crossing/head_on/overtaking/ais_dropout/harbor_*/
  shore_clutter_*/…): byte-identical (30/47 scenarios unchanged).

**Arbiter action (ticket §W2.4).** The deployable config's numbers MOVE on the
autoferry scenarios (the Cl-4 gauntlet's env-1/env-2 family) and on dense_clutter.
The Cl-4 adoption (ADR-0003, `70cc273`) was just frozen on this exact config.
**This delta is surfaced as a FINDING to reconcile with the frozen Cl-4 gauntlet
— nothing is re-frozen here.** The autoferry `_anchored` improvements are a clean
win; the non-anchored track_break/id_switch increases and the dense_clutter
phantom persistence are the trade to weigh (both are *correct* behaviour that the
prior bugs were masking). CSVs: `fw2_on.csv` / `fw2_off.csv` (compare tool:
`tools/…` / the ab_compare join used for this table).

## W2.5 + W2.6 — sanitizer fixes

**W2.5 (ASan).** `test_own_ship_nmea.cpp:275` bound `const auto&` to
`*provider.latest()`; `latest()` returns `std::optional<OwnShipPose>` by value,
so the reference dangled and the assertions ran on freed stack. Bind a copy. Grep
confirmed no other optional-by-value-deref-to-ref site.

**W2.6 (UBSan).** `McapWriter::ensureChannel` built an `mcap::Schema` from an
EMPTY `schema_text` (our name-only channels); its data `ByteArray` is empty so
`schema.data.data() == nullptr`, and on the first message mcap wrote the schema
record via `write(nullptr, 0)` → the vendored writer range-copies
`(nullptr, nullptr+0)` → UBSan "null pointer passed as nonnull" (14 tests). mcap
treats `schemaId 0` as "no schema" and skips the schema-record write, so
empty-schema channels now register with id 0. Functionally green in Release (14
foxglove/mcap tests incl. `McapWriter` round-trip).

## W2.7 — the fixture-skip guard

Closes the standing process hole (F-BUILD-3, the 2026-07-08 red-master cause):
~36 real-data tests resolved fixtures relative to CWD, so under `ctest` (cwd =
build/) they silently `GTEST_SKIP`ped while the suite reported green, and nothing
caught a fixture-gated test that skipped when it should have run.

1. **Anchoring.** `tests/support/FixtureGuard.hpp` provides `srcAbs(rel)`
   (absolute path under `NAVTRACKER_SOURCE_DIR`); the direct-path tests
   (HAXR/philos OSPA, radar-truth loader, philos far-cross, autoferry JSON) now
   open fixtures via `srcAbs`. The scenario-run adapters resolve their base dirs
   from env vars: a gtest global environment (`FixtureGuard.cpp`) points
   `SIMMS_DIR` / `RBAD_DIR` / the new `NAVTRACKER_FIXTURE_ROOT` at the source tree
   (only if unset), and `ReplayScenarioRun` now prefixes its philos/HAXR/autoferry
   paths with `NAVTRACKER_FIXTURE_ROOT` (unset ⇒ relative, bit-identical for the
   bench run from the repo root).
2. **Strict mode.** `NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(missing, msg)` wraps every
   fixture-absence gate (both idioms: a missing path, and an empty/`!valid`
   scenario-run load). With env `NAVTRACKER_REQUIRE_FIXTURES=1` a would-be skip
   becomes a FAILURE, so a verification ceremony proves every fixture-gated test
   actually RAN. Default (unset) = ordinary skip, bit-identical.
3. **Watchdog.** `gtest_discover_tests(... PROPERTIES TIMEOUT 300)` — a hung
   fixture test no longer stalls ctest unbounded.

## Verification

- **Full suite from the build dir (the ctest scenario that used to hide skips):**
  `ctest --test-dir build -j`: **1181/1181 pass, 0 failed.** The ~36 real-data
  replays (PhilosOspa, PhilosSunsetLabels, PhilosCoverageDecay6c, HaxrOspa,
  Sim/Rbad/Imazu scenario-runs, AutoFerry, ...) now RUN — previously they
  silently skipped from the build dir.
- **Strict-mode (the 0-skip PROOF):** `NAVTRACKER_REQUIRE_FIXTURES=1 ctest ...`:
  **1181/1181 pass, 0 failed.** Under strict mode any fixture-absence skip is a
  FAILURE, so 0 failures ⇒ every fixture-gated test actually RAN (0 silent
  skips). 272 s.
- **Fixture-guard teeth:** with a bogus fixture dir (`SIMMS_DIR=/nonexistent`),
  `NAVTRACKER_REQUIRE_FIXTURES=1` → the 5 SimMultisensor tests **FAIL** (guard
  fires); without strict mode → the same 5 **SKIP** (default behaviour
  preserved, bit-identical).
- **ASan + UBSan** (`-fsanitize=address,undefined -fno-sanitize-recover`): the
  W2.5 own-ship tests + the 14 W2.6 foxglove/mcap tests (33 total) run **clean,
  exit 0, no sanitizer errors** — W2.5 ASan-clean, W2.6 UBSan-clean. The new
  W2.1 reprojection (Eigen ops) + W2.4 PMBM code (51 more tests) are also
  sanitizer-clean.
- **W2.2/W2.3 teeth:** re-flip the sign → the 3 datum/high-lat tests RED →
  revert → GREEN. **W2.1 teeth:** neuter AIS `onDatumRecentered` → the 2 AIS
  recenter tests RED → restore. **W2.4 teeth:** neuter each sub-fix → its 2–3
  tests RED, the rest green → restore.
- **Adversarial review** (4-lens workflow: geo-sign / adapter-reproject /
  pmbm-activity / fixture-guard; 19 agents, each finding adversarially verified):
  the geo-sign and adapter-reproject lenses came back CLEAN (the −γ sign was
  independently re-derived and confirmed; `toGeodeticWithCov` uses
  `datumAxisRotation(datum, target)` — same convention, correct). 6 real findings
  surfaced. Two MEDIUM were genuine fixture-guard evasions and are FIXED
  (`b7d54f0`): `HeldoutSailboats.Probe` (assertion-free, silent helper return →
  passed with fixtures absent, invisible to skip-diff AND strict mode — now
  TEST-level gated, teeth-proven) and `test_veto_isolation_haxr_ab` (only site[0]
  strict-gated → per-site strict FAIL added). Two minor gaps FIXED (PMBM
  capture-loop gate widened to match the read sites; `setIfUnset` overwrite=1 for
  set-but-empty env). Two accepted-as-documented: own-ship inferred from
  surveillance returns (bounded, inert on continuous-radar deployable replays;
  improve-next = thread OwnShipPose into the tracker, noted in code) and the
  ctest TIMEOUT not being sanitizer-aware (targeted sanitizer runs don't use
  ctest, so no effect on this wave's proof).

## Handoff

- **W2.1, W2.2, W2.3, W2.5, W2.6, W2.7 — merge-ready.** Self-contained, fully
  tested, teeth-proven, docs in sync (CLAUDE.md + integration-guide datum-sink
  list, AxisRotation.hpp convention, learning ch.24).
- **W2.4 — merge-ready as a fix, but its A/B delta on the deployable config is a
  FINDING the arbiter must reconcile with the just-frozen Cl-4 gauntlet
  (ADR-0003).** philos/HAXR are byte-identical, so the Cl-4 *deployment-target*
  numbers are safe; the autoferry (env-1/env-2 family) and dense_clutter numbers
  MOVE. The moves are *correct behaviour the prior bugs were masking* (own-ship
  coverage; radar-only tracks no longer culled by AIS silence) — the anchored
  autoferry improvements are a clean win, the non-anchored track_break/id_switch
  increases and the dense_clutter phantom persistence are the trade. **Nothing
  re-frozen here** (per the ticket).
- **Findings-file marks deferred to the arbiter** (same as wave 1): the review's
  `10-bughunt-findings.md` is untracked working state on no commit, so the FIXED
  marks live here + in the eval-log rather than in another session's files.
- Commit on this branch; not merged/pushed to master.
