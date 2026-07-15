# Pre-release fix wave — wave 1 (2026-07-12)

Branch `fixwave-wave1` off master `43d955f`. Origin: the 2026-07-09 pre-release
deep review (`docs/reviews/2026-07-09-prerelease-review/`), implementer ticket
`docs/superpowers/plans/2026-07-12-fixwave-wave1-ticket.md`. Three
synthesis-independent CONFIRMED findings were scheduled; **F3 was HELD** (see
below). TDD throughout: failing test first, then fix, then green.

## Scope decision — how the three findings landed

The implementer prompt named base `3be2bce` and F3 = "align docs to ENU." Master
had advanced past it twice: `43d955f` (arbiter amendment: F3 direction is a user
decision, synthesis leaned code→NED, "hold F3, do F1+F2 now") and then `fc39302`
(**user resolution**: F3 goes DUAL-API and is unheld). This branch honours the
newest authority at each step:

- **F1** — implemented, byte-identical, CRITICAL. Its own merge package.
- **F2** — implemented + reviewed, but the A/B disproved the ticket's
  attribute-only premise (below). Per the house rule ("surface config-driven
  real-data shifts as decisions") it is **held on this branch, not merge-ready**;
  it becomes its own measured cycle. F1 does not wait on it.
- **F3** — implemented to the `fc39302` dual-API spec (below).

## F1 — CRITICAL: OwnShipNmeaAdapter accepted a no-fix GGA → (0,0) pose/datum

**Finding.** The GGA branch validated nothing: fix-quality (field 5) was never
read, and empty lat/lon fields parse (`parseDdmm("")→0`) to `(0,0)`. A standard
no-fix GGA therefore published a null-island pose. On cold start that initializes
the working datum at (0,0); mid-run it is ~6000 km from a real datum (> 30 km),
firing an auto-recenter to Null Island and shifting every track through a
6000-km-distant tangent plane — silent, unrecoverable ENU corruption. Invariant
#6 (validate at the edge) was violated; the RMC branch already rejects its `V`
status, confirming the GGA gap was an oversight.

**Fix** (`adapters/own_ship/OwnShipNmeaAdapter.cpp`). Validate before any
datum/estimator/provider side effect: reject when the fix-quality field is
`0`/empty, when lat/lon fields are empty, or when the parsed position fails
`edge::isPlausibleLatLon`. A rejected sentence produces NO pose (`ingest`
returns `false`) and increments a new `skippedNoFixGga()` diagnostic counter so
the drop is observable, not silent. Valid fixes are byte-identical to before.

**Tests** (`tests/adapters/own_ship/test_own_ship_nmea.cpp`, 6 new): no-fix
(empty fields, quality 0); quality-0-with-position; empty-lat/lon-with-quality-1;
implausible lat/lon; **fix→no-fix→fix leaves the datum unmoved** (a
recenter-counting `IDatumChangeSink` fires 0 times, latest pose unchanged — RED
showed 2 recenters and a (0,0) pose); and a happy-path guard. RED→GREEN paper
trail confirmed. Doc: integration guide §4 NMEA rejection paragraph.

## F2 — HIGH: PmbmTracker source-touch walk credited misdetected Bernoullis

**Finding** (`core/pmbm/PmbmTracker.cpp:1666`). The source-touch walk appended a
`Track::SourceTouch` to `contribution_history_` for every Bernoulli in the
dominant hypothesis by matching "nearest scan measurement whose
`z.time == b.last_update`." That is false separation: `predict()` advances
`last_update` to the scan time on EVERY Bernoulli, so a MISDETECTED Bernoulli
matched any same-time measurement and was credited the nearest one (no distance
bound) — a contribution it never made.

**Channel precision (review nit — corrected here).** For a PMBM track the
polluted channel is **`Track.recent_contributions`** (the `SourceTouch`
provenance vector), *not* the `TrackOutput.contributing_sources` string list —
PMBM never writes the latter (it is populated only by the flat-tracker /
T2T paths, so it is empty for PMBM tracks). `recent_contributions` /
`contribution_history_` is what feeds the bias-pair extractors, the source-aware
gate, and idle-decay. The finding's "contributing_sources lies" wording is
imprecise for PMBM; the defect and the fix are real regardless.

**Fix.** Key the walk on `Bernoulli::last_claimed_meas_index` (the R2
true-assignment field: scan index on a detection/birth, `-1` on a miss). A
misdetected Bernoulli now gets no touch. `scan` is the immutable `const&` batch
shared by `enumerateChildren` (which sets the index) and this walk, so
`scan[claimed]` is exactly the claimed measurement.

**Blast-radius note (important nuance).** The base bench builder
`makePmbmConfig()` sets `source_aware_misdetection=true` AND `idle_halflife_sec
=10`, so `contribution_history_` feeds the existence path on most pmbm configs —
i.e. the fix is NOT attribute-only by construction there. The ticket expected
byte-identical kinematics/existence/lifecycle; whether that holds is an empirical
question, settled by the A/B below.

**Regression test** (`tests/pmbm/test_pmbm_contribution_provenance.cpp`, new): a
track updated only by radar, coasting through a scan carrying a foreign vessel's
AIS return, must never list AIS in its `recent_contributions`. RED (Actual: true)
→ GREEN. **Note (review, major):** this test guards the emitted-provenance
channel only. It does NOT assert existence/confirmation/deletion timing, so it
would stay green even if the existence-gate change (below) regressed real-target
continuity — the untested risk is called out in the decision.

**Unit + integration verification.** Full PMBM + identity-gate + stale + sensor-
activity unit suite (241 tests) green — the `PmbmIdentityGate` tests that were
written around the old pollution still pass. The cwd-gated real-data integration
tests (PhilosOspa, PhilosClutterMapAB, ReplayScenarioRun over philos/autoferry/
haxr; 20 tests) green, 0 skips.

### F2 byte-identical-metrics A/B — the ticket premise is FALSE

Bench matrix before vs after the fix, 18 pmbm configs × 52 scenarios (philos +
autoferry ±anchored + sim + simms), seeds 5. Non-timing metric rows compared
(`wall_seconds`/`scan_proc_ms*` excluded, as the determinism guard does).

**Result: NOT byte-identical — 29,776 metric rows changed across all 18 pmbm
configs.** This is the headline: the fix is not attribute-only, because
`contribution_history_` is not a pure output. It feeds tracking through **three
coupled paths**: (1) the `source_aware_misdetection` miss gate (`should_misdetect`,
base config = true), (2) the `idle_halflife` decay (`idle_decay_for`, base = 10),
and (3) the emitted `recent_contributions` → `AisArpaPairExtractor` →
`SensorBiasEstimator` → `applyBiasCorrection`, which rewrites measurements. So
`contributing_sources` is a tracking **input**, and fixing the confirmed lie
necessarily changes tracking wherever any path is live.

Where the change lands, and its direction:

- **Pure-radar sim (crossing, overtaking, headon): byte-identical** — every
  metric unchanged (only timing moved). No AIS identity, no bias wiring, gate
  inert. This rules out an indexing bug: the fix is inert exactly where the three
  paths are inactive.
- **philos + philos_radartruth (the Cl-4 sparse-AIS deployment target): gospa
  IMPROVES** on nearly every config (e.g. `_land` 63.13→62.42, `_bundle`
  111.99→99.40, `_birthtarget` 48.50→46.60, plain 99.13→98.10). This is the exact
  case `source_aware_misdetection` was built for — "vessel A's broadcast tells us
  nothing about vessel B" (Config.cpp:356) — and clean provenance stops one
  vessel's touch from wrongly marking another "covered".
- **autoferry (±anchored) + simms: net gospa REGRESSION** — of the changed cells,
  247 regressed (Σ+1401) vs 55 improved (Σ−116). Example `_adapt`/
  `autoferry_scenario13`: gospa 15.05→18.67, gospa_false 110→245, card_err
  −0.22→+0.52 (undercount→overcount), id_switches 3.5→8, lifetime_ratio
  0.67→0.74. Mechanism: correct provenance makes `should_misdetect` return
  "covered" LESS often → less miss-decay → longer track persistence; on
  dense-radar autoferry that surfaces as more phantom persistence. The old false
  touches were incidentally OVER-decaying (helping autoferry, hurting philos).
- **`coverage_land` (gate off, `use_sensor_activity` instead): unchanged on
  philos** (all three paths inert), but **changes on `autoferry_*_anchored`
  only** — via path 3 (the bias loop), confirming the mechanism.

**Conclusion:** F2 is a CONFIRMED-correct fix (the review below confirms the
mechanism; byte-identical on pure-radar sim proves no indexing bug), but it is a
**tracking behavior change**, not the attribute-only fix the ticket assumed —
because provenance is a tracking input. The change **helps the deployment target
(philos) and regresses the seeded autoferry/sim battery** on GOSPA. Acceptance
criterion #2 ("byte-identical metrics") is therefore **not achievable** as
written. This is surfaced to the user/arbiter as a decision (see handoff); it is
NOT shipped silently as a byte-identical bugfix.

## F3 — TrackOutput covariance axis contract — DUAL API (implemented)

**Finding.** The code emits position covariance in ENU `(east, north)` order; the
header, `output-contract.md`, `example.cpp`, the integration guide, and the
Foxglove adapter all claimed/assumed NED `(north, east)` — so the Foxglove
adapter rendered every anisotropic error ellipse 90° rotated, and any consumer
trusting the old doc silently transposed axes.

**Resolution (user, master `fc39302`) — dual API, ambiguous name removed.**
- `toTrackOutputENU(track, datum)` = the true current behaviour, renamed
  (east-first); `toTrackOutputNED(track, datum)` = the north-first copy (permutes
  the 2×2 position covariance). No deprecated alias.
- `toTrackOutput` **removed** — the compile-time break at every call site is the
  consumer audit (house unreachable-footgun rule); no caller can flip silently.
- New `TrackOutput::covariance_frame` (`CovarianceFrame::Enu`/`Ned`) stamped by
  the producer, so a struct in hand retains its convention and axis-sensitive
  consumers may assert on it.
- **Call-site migration (the audit):** Foxglove track drain → `toTrackOutputNED`
  (**fixes** the rotated ellipses; the measurement drain, which has no Track, was
  corrected to the now-explicit ENU helper); T2T `FusedTrackOutput` →
  `toTrackOutputENU` (the fuser is ENU throughout); `NavtrackerSource` unchanged
  (it copies raw ENU directly, never routed through `toTrackOutput`); `example.cpp`
  + `mht_fusion_example.cpp` → `toTrackOutputNED` with a rationale comment.
- **Tests (both conventions pinned, #24 teeth):** the T2T swap-test renamed to the
  ENU contract pin + asserts `covariance_frame == Enu`; new elongated-covariance
  (east≠north) pins in `test_track_output.cpp` for BOTH ENU and NED (which slot
  holds north). Docs updated: `output-contract.md`, integration guide §4/§5/§6,
  header comments, `example.cpp`, learning ch.10.

**Handoff to the user (stands):** your middleware hits the compile-time break on
upgrade and must pick the name matching its downstream — by design. If you were
reading the old (documented-NED) output and it looked right, your data was
isotropic and masked the transpose; verify against the new `covariance_frame`.

## Verification

- Baseline (master `43d955f`) full ctest: 1157/1157 pass; the 13 skips are the
  documented cwd-gated expected set (build-suite report F-BUILD-3), run
  separately from the worktree root (20/20 pass, 0 skips).
- F1: 6 tests, RED→GREEN; 120 GGA-consumer tests (NMEA/OwnShip/Bus/FullStack/
  Emitter) unchanged.
- F2: regression test RED→GREEN; 241 PMBM/identity/stale unit tests + 20 cwd-gated
  real-data integration tests green; A/B (above) **disproves** byte-identical
  (criterion #2) and quantifies the trade; adversarial review pass (below).
- F3: dual-API build clean (compile-break audit passed — all call sites
  migrated); 69 F3-affected tests green (ENU+NED covariance pins, frame-tagged
  swap-test, Foxglove Recorder incl. the fixed covariance mapping, T2T/Fuser,
  RemoteTrackFusion, FullStack).
- Full suite (F1+F2+F3): **1166/1166 pass, 0 failed**; the 13 skips are the
  documented cwd-gated expected set (run separately from the worktree root).
- **Adversarial review of the F2 change (4 independent lenses).** Mechanism
  CONFIRMED CORRECT: index-correctness (last_claimed_meas_index is always a
  valid this-scan index or −1; merges preserve it; per-track-hyp path
  irrelevant; bounds guard defensive; matches the clutter-feed's existing
  pattern), fixes-the-finding (one reviewer git-stashed the fix, rebuilt,
  reproduced RED→GREEN — teeth proven; sole writer; no residual pollution),
  and no dangling refs / no phantom-birth touch. The review's material findings
  are NOT fix bugs — they are the disposition: (1) **the "byte-identical" claim
  is false and the existence-gate change on deployment configs is untested** —
  the regression test guards only the attribute; a real-target continuity
  regression (a genuine AIS dropout now decaying via idle_halflife where the bug
  held existence flat — #25-adjacent) would pass green; (2) the autoferry/simms
  GOSPA regression is a real downstream change (already measured/surfaced).
  Two cleanups applied from the review: the stale walk block comment and the
  `recent_contributions` channel wording. Latent nit noted (not fixed, out of
  scope, unreachable): the empty-scan `enumerateChildren` branch does not reset
  `last_claimed_meas_index` to −1 (guarded off by `!scan.empty()`).

## Handoff (disposition per the user, 2026-07-12)

**Wave 1 lands as: F1 merge now, F3 to the dual-API spec, F2 held-with-paper-trail
into its own cycle.** Commits on this branch are separated per finding so F1 is an
independently-mergeable package.

- **F1 — merge now.** CRITICAL, byte-identical, self-contained (own-ship adapter +
  its test + integration-guide §4). Its commit touches no F2/F3 file.
- **F3 — merge with wave 1.** Dual-API implemented to the `fc39302` spec. Consumer
  surface changed (renamed/removed API, new `covariance_frame` field) → integration
  guide + output-contract updated same-branch. The middleware compile-break note
  above stands.
- **F2 — HELD on this branch, NOT merge-ready.** The fix + regression test remain
  (the mechanism is correct and review-confirmed), but its commit message flags it
  not-merge-ready. It becomes its own ticket because it shifts tracking on 18
  configs with an untested continuity risk on the KEEP config — a measured
  decision, not a fix-wave passenger.
  - **UPDATE 2026-07-15 (F2 provenance cycle measured — see
    `docs/baselines/2026-07-15_f2_provenance_cycle.md`).** On the corrected
    (wave-3) chain: (a) the autoferry regression is REAL and path-(a)
    (source_aware_misdetection) driven — an overcount/phantom-persistence effect,
    NOT bias-loop garbage×broken-chain cancellation — and CONFINED to source-aware-
    gate configs; the DEPLOYED KEEP config carries none of it (byte-identical, and
    +improving on anchored diagnostic, −15.37). (b) KEEP AIS-dropout continuity is
    excellent (lifetime 0.993, 0 id-switches) and byte-identical fix ON vs OFF; the
    idle_halflife risk is doubly moot (idle=0, and radar keeps the track non-idle);
    permanent guard landed. (c) philos improves on source-aware-gate configs but is
    byte-identical on KEEP. **Arbiter verdict 2026-07-15: SHIP** — F2 fix +
    isolation flags + continuity guard + T2T live-pedigree pin, §10 Rider B
    LIFTED (corrected rationale), re-pin document-only (nothing deployed/enforced
    moves; harbor byte-identical too). Branch `f2-provenance-cycle`, handed to the
    arbiter for the merge ceremony.
  - **Acceptance criterion #2 was DISCOVERED UNACHIEVABLE, not waived.** The A/B
    input-path evidence is the paper trail: `recent_contributions` feeds three
    tracking paths (source-aware gate, idle-decay, and the emitted-provenance →
    `applyBiasCorrection` loop), so no attribute-only/byte-identical version of
    this fix exists where any path is live.
  - **Three measurement-first questions for the F2 cycle** (no baseline re-pinning
    until (a) and (b) are answered):
    (a) *Is the autoferry/sim regression real, or was the garbage helping?* The bug
    fed fabricated bias observations into `applyBiasCorrection`; the sim battery may
    have been calibrated around that corruption. Note the review's Theme-3 finding
    that the bias chain itself is broken (half-bias convergence) — "garbage in,
    broken chain" may be two wrongs cancelling, so **sequence the F2 cycle after or
    with wave 3** (bias-chain fix) for a clean read.
    (b) *Is idle-decay on a genuine AIS dropout correct-but-untested or a
    regression?* Needs a continuity/existence-timing test before judgment; the
    decay the bug was masking may be the designed behaviour.
    (c) *Record the philos improvement as the Cl-4-relevant upside.*
- **Rider-B (unchanged by design):** with F2 held, T2T live-pedigree content stays
  UNtrusted — the caveat in the T2T docs remains accurate; it was not touched.
- **Findings-file marks deferred to the arbiter.** The ticket asked to mark the
  findings FIXED in `docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md`,
  but that file is UNTRACKED — the live review session's uncommitted working files,
  on no commit and not on this branch. Writing into another session's uncommitted
  work would violate the parallel-work convention, so the disposition is captured
  here + in the eval-log instead; the arbiter can transcribe the marks (F1 FIXED,
  F3 FIXED, F2 fix-implemented-but-HELD) when reconciling the review at merge.
