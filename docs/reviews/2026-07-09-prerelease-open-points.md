# navtracker — Pre-release Open Points

**Review date:** 2026-07-09 → 2026-07-11
**Commit reviewed:** master @ 317ecfd
**Reviewer:** deep multi-agent review (bug hunt on Fable 5 at max effort;
test/data/readiness audits on Opus 4.8), every high/critical bug-hunt finding
adversarially verified by an independent skeptic that hand-traced the failing
path. Build/sanitizer run locally in an isolated worktree.

Detailed evidence per finding: `docs/reviews/2026-07-09-prerelease-review/`
(`10-bughunt-findings.md`, `20-test-sufficiency-findings.md`,
`30-test-data-findings.md`, `40-release-readiness-findings.md`,
`50-build-suite-report.md`, plus raw JSON under `*-raw/`).

---

## Verdict

**Not ready to ship as-is.** The suite is green (1090/1090 Release; 1089/1090
ASan, the one failure being test-code UB) and the architecture is sound, but
the review found **1 critical + ~13 confirmed high-severity code defects**, a
**licensing blocker** (non-commercial data committed to git), and systematic
**test coverage holes that exactly coincide with the confirmed bugs** — which
is why they survived prior reviews.

Nothing here is unfixable and most fixes are small. The clustering is the real
message: **the weakest area is datum / own-ship-position handling and
angle/frame conventions at the edges** — several independent findings point at
the same seams.

### Counts (deduplicated, defect-level)

| Phase | Findings | Confirmed critical/high | Confirmed medium |
|---|---|---|---|
| Bug hunt (20 units) | 101 | 1 crit + 13 high | 7 |
| Build / sanitizer | 3 | 1 (test-UB) + process | — |
| Test sufficiency | 42 | 6 high | — |
| Test data | 38 | 4 high (3 licensing) | — |
| Release readiness | 10 | 1 licensing-adjacent + gaps | — |

"Unverified" medium/low findings (31 medium, 41 low in the bug hunt alone) are
in the detail files; they were not adversarially checked and should be triaged
before acting.

---

## A. Release blockers — fix before shipping

### A1. [CRITICAL] Own-ship GGA input is completely unvalidated → (0,0) datum poisoning
`adapters/own_ship/OwnShipNmeaAdapter.cpp:120`. The GGA path checks no
fix-quality field and no lat/lon plausibility. A standard **no-fix GGA**
(quality 0, empty fields parsed as 0.0) publishes own-ship at (0°N, 0°E). Via
auto-datum, the working datum initializes/recenters to Null Island, so **every
ENU projection downstream is corrupted**. Directly violates the "validate at
the edges" architecture invariant. Sibling issue: HDT/HDG arriving before the
first GGA publishes a default (0,0) pose (`:109`, confirmed MEDIUM).
*Fix:* reject GGA with quality==0 or empty/implausible lat/lon; don't publish
a pose until a valid fix exists.

### A2. [LICENSING BLOCKER] Non-commercial dataset derivatives committed to git
`data/philos` is **CC-BY-NC-SA-4.0** (non-commercial + share-alike) — the exact
license class the team already rejected for MOANA and Global Fishing Watch —
yet philos-**derived** artifacts are tracked in git and actively validate the
tracker: the three `tests/fixtures/philos/labels/*_labels.csv`,
`tests/fixtures/philos/charts/radar_structure_points.geojson`,
`charts/philos_*` csv/png, and ~10 `docs/baselines/*philos*.csv`. They were
`git add -f`'d past `.gitignore`, contradicting the philos README's own
"never committed" invariant. For a commercially-licensed library this forbids
commercial use and the share-alike term is viral.
*Fix:* purge philos-derived artifacts from git (and history) and the tests that
depend on them, **or** obtain a commercial license from MIT Sea Grant.
Related (not git-tracked, lower urgency): `data/pohang` is CC-BY-NC-4.0 (683 MB
on disk); `data/smd` (Singapore Maritime Dataset) has **no recorded license or
provenance** and is missing from the data manifest.

---

## B. Confirmed high-severity code defects

Grouped by theme. All were independently verified.

### Theme 1 — Datum / own-ship position at the edges (the dominant cluster)
- **Adapters cache a stale datum across recenter** — `AisAdapter.cpp:24` (+ ARPA,
  EO-IR, RemoteTrack): each holds a fixed private `Datum` copy that never
  updates on auto-recenter, so after a recenter their measurements project in
  the old frame. *(Note: with auto-recenter enabled this silently corrupts
  positions for every non-cooperative sensor.)*
- **`datumAxisRotation` doesn't wrap Δlongitude** — `AxisRotation.cpp:15`: a
  recenter across the antimeridian applies a wildly wrong rotation.
- **`datumAxisRotation` has the wrong sign** — `AxisRotation.cpp:17` (verified
  MEDIUM, downgraded from high as bounded/self-correcting): rotates velocity /
  covariance / IMM means / particles by +γ where the correct transform is −γ
  (worse than not rotating). *The unit test `test_datum_shift.cpp:57` pins the
  wrong sign — the suite encodes the bug.* One-line fix; must update the test.
- **`DeclaredSensorActivity` measures coverage from the ENU origin, not
  own-ship** — `DeclaredSensorActivity.cpp:12`; and its cooperative-overdue
  retirement is identity-blind (`:51`), hard-deleting radar-only tracks.

### Theme 2 — Range/bearing & angle conventions
- **Track initiation plants RangeBearing2D polar values as ENU Cartesian** —
  `EkfEstimator.cpp:82`, `UkfEstimator.cpp:123`, `ImmEstimator.cpp:449`,
  `ParticleFilterEstimator.cpp:127` (all four estimators). A radar/EO-IR
  measurement fed as range-bearing (documented as birth-capable in the
  integration guide) births a track at (range_m, bearing_rad) with mixed
  m²/rad² covariance → nothing gates to it → per-scan phantom-track
  proliferation. *Fix:* convert polar→ENU in `initiate()` (using
  `sensor_position_enu`), or make `canInitiateTrack` exclude RangeBearing2D and
  amend the guide.
- **UKF predicted bearing is a linear mean across the ±π wrap** —
  `UkfEstimator.cpp:80` (confirmed MEDIUM): corrupts the update for targets ~due
  west of the sensor.

### Theme 3 — Heading/sensor-bias chain (treat as one repair project)
- **Closed-loop heading-bias double-subtraction** — `HeadingBiasEstimator.cpp:64`:
  the estimate provably converges to *half* the true bias.
- **Per-sensor bias loop, same feedback defect** — `SensorBiasEstimator.cpp:99`.
- **v1 AIS/ARPA pair extractor computes bearings about the datum origin, not
  own-ship** — `AisArpaPairExtractor.cpp:48`: geometrically wrong innovations
  whenever own-ship is away from the datum.
- **Sign convention inconsistent across the five observation kinds** —
  `HeadingBiasEstimator.cpp:46` (marine-compass vs ENU-math frame).

### Theme 4 — Output contract & CPA math (operator-facing)
- **Output covariance axis order contradicts the documented contract** —
  `TrackOutput.cpp:21`: emitted (east, north) while `output-contract.md:8-9`,
  the header, `example.cpp`, the integration guide, and the **Foxglove adapter**
  all assume NED north-first. Every anisotropic error ellipse is rendered 90°
  rotated today (Foxglove already bitten). *Decision needed:* make the code NED
  (recommended — it's the Option-A intent) or re-document as (east,north) and
  fix five sites.
- **Sign error in the CPA tcpa-Jacobian chain term** — `Cpa.cpp:135` (the
  CPA-uncertainty design spec §4.3 has the same error): published
  `sigma_tcpa` ~3× wrong for converging pairs; head-on fallback `sigma_cpa` /
  probability corrupted. Existing tests pass only because the error cancels in
  the direction they exercise.

### Theme 5 — PMBM / pipeline timestamp & association
- **`Tracker::process()` drops soft (JPDA) association results** —
  `Tracker.cpp:144`: no track is updated on the soft path.
- **Spurious SourceTouches on misdetected Bernoullis** — `PmbmTracker.cpp:1666`:
  provenance/contributing-sources lies for missed tracks (found twice,
  independently).
- **Mixed-timestamp scans double-propagate** — `PmbmTracker.cpp:964`: detected
  Bernoulli `last_update` rewound while others advanced → interval propagated
  through F/Q twice.
- **MHT deferred-commitment leaf protection is inert** — `MhtTracker.cpp:531`
  (confirmed MEDIUM): flags always one `branch()` behind.

### Theme 6 — Lifecycle
- **Tentative→Coasting promotion on miss** — `TrackManager.cpp:64` (confirmed
  MEDIUM): a one-hit clutter blip that misses becomes Coasting, which is
  CPA-eligible (Tentative is not) → false collision-risk events. Also violates
  the documented Coasting definition ("was Confirmed").

---

## C. Build, sanitizer & determinism

Full report: `50-build-suite-report.md`.

- **F-BUILD-1 [test UB]** `test_own_ship_nmea.cpp:275` binds `const auto&` to a
  dereferenced temporary `optional` (`latest()` returns by value) →
  use-after-scope; ASan fails the test, GCC warns `-Wdangling-pointer`. The
  test's assertions currently run on garbage. One-line fix (copy).
- **F-BUILD-2 [UB via mcap]** UBSan: null-pointer `memcpy` in the mcap writer,
  triggered from our foxglove path in 14 tests. Guard the adapter against
  passing nullptr/size-0.
- **F-BUILD-3 / test-suite process gap** `ctest` reports 100% green while **~36
  real-data tests never run**: some skip because the documented worktree
  fixture-symlink recipe lands the link *inside* a partially-tracked
  `tests/fixtures/`; others (Imazu, HAXR, R-BAD, Philos replays, SimMultisensor)
  resolve fixtures **relative to CWD and assume project root**, but ctest runs
  in the build dir. Run from project root they are **36/36 green with 0
  sanitizer hits**, so this is a coverage-blindness problem, not a hidden
  failure — but nothing guards those 36 from silent regression, and there is
  **no mechanism to catch a fixture-gated test that skipped when it should have
  run** (your 2026-07-08 red-master-for-a-day cause, still open).
- **Determinism lens: clean bill.** RNG seeding, sort stability, unordered-map
  iteration, and wall-clock usage were traced across the hot path and cleared
  against the replay contract. Only catch: `Tracker::processBatch` never got the
  backlog-#15 batch-sort fix that MHT/PMBM got, so unsorted batches stale-drop
  on the plain-Tracker path.

---

## D. Test-sufficiency gaps that matter

Full list: `20-test-sufficiency-findings.md` (42 findings). The load-bearing
ones — each is a hole the confirmed bugs live in:

- **Same-scan two-sensor fusion of one target is untested end-to-end** — exactly
  the PMBM timestamp-bug path.
- **AIS unavailable sentinels (SOG=1023, COG=3600) never fed to `AisAdapter`** —
  why the sentinel-as-velocity bug went unseen.
- **ISO-8601/BaseDateTime timestamp parsing in `loadAisCsv` entirely untested** —
  the path that silently drops all DMA rows.
- **Crossing/overtaking continuity asserted via a degenerate metric** —
  `countIdSwitches ≤ 2` for two targets *exactly admits a clean full ID-swap*.
  Tighten to `==0` (overtaking already does).
- **ID-never-reused invariant** (an architecture invariant) is asserted by no
  delete→create cycle test anywhere.
- **PMBM replay-determinism** asserted on only two aggregate scalars (mean OSPA +
  final count), not per-step IDs/positions; and **no end-to-end determinism test
  through the real adapters** (NMEA/CSV in → TrackOutput out).
- **No test verifies sim-emitter noise magnitude is self-consistent with the
  covariance the adapter stamps** — the foundation every NEES/consistency test
  rests on.
- Several print-only "tests" with zero assertions (`test_filter_comparison.cpp`);
  a couple of vacuous PMBM tests that skip all assertions when no Bernoulli is
  born; no per-test ctest TIMEOUT (a hang has no watchdog).

---

## E. Test-data issues

Full list: `30-test-data-findings.md` (38 findings). Beyond the licensing
blocker (A2):

#### E.0 Ground-truth & label assumptions — are they usable as-is?

The method for every dataset was: read the loader, write down its column/unit/
epoch/frame expectations, then sample the data (per-column min/max, timestamp
monotonicity, per-uid ranges, quaternion recompute, md5) and check they match —
plus the truth-specific questions (circularity, coverage, cadence, what the
"truth" physically represents). Verdict per truth source:

| Dataset | Truth source | Usable as-is? |
|---|---|---|
| **autoferry** | dataset-provided GT positions | **Yes** — units/frames/epoch cross-checked; ownship+(n,e) NED lands ~4 m from truth, IR/EO bearings gate to true target ~2°, truth velocities sane, per-target monotonic. Trustworthy arm. |
| **sim_multisensor / sim** | synthetic (generator) | **Truth yes, noise no** — truth is perfect by construction (monotonic, shared clock, detections within σ). BUT emitter-injected noise ≠ the covariance the adapter stamps (E-block) → NEES/consistency conclusions built on it are compromised. |
| **HAXR** | AIS-derived (station+range·az) | **With caveats** — frame alignment correct, circularity clean (see below). But truth **velocity hardcoded to 0** for all vessels, and sparse AIS (10–20 s) scored on a 1 Hz clock **without resampling** (unlike the philos path) → OSPA inflated to the cutoff on most windows. Absolute numbers reflect truth sparsity, not tracker error. |
| **philos** | AIS-derived `radar_truth` + video existence/region labels | **Partly misleading** — OSPA truth is the full AIS set, but **22 of 23 targets lie beyond ~976 m radar coverage**, so OSPA is dominated by targets the radar cannot see; only 1 target actually measures tracking. Labels are human/video-derived (weaker than kinematic truth) and used appropriately (existence/region, not accuracy) — but license-blocked (A2). |
| **R-BAD** | authors' reference-tracker output | **Assumption correctly avoided** — the hazard (treating it as accuracy truth) is checked CLEAN: loader/README/generator all label it reference-tracker output, the bench reports only continuity/cardinality *consistency* (no RMSE), Dock_Label asserted-on nowhere. |
| **pohang** | — | **Not usable / not used** — no C++ test consumes it; the generated ownship CSV header isn't recognized by the loader (loads empty silently), and the extractor reads the AHRS quaternion in the wrong component order (assumes w,x,y,z; data is x,y,z,w) → physically-impossible heading/roll/pitch. Staged-only, must be fixed before any use. |
| **SMD** | on-shore ObjectGT labels | Not consumed by any test; no misuse. Provenance/license gap (E). |

**Circularity (the scariest assumption) is clean:** the concern — same AIS fed
as a sensor *and* used as truth (self-validation) — was checked; the default
HAXR path feeds radar plots only and scores vs AIS-derived truth (no AIS as
sensor), and the LOS-guard AB doesn't set `HAXR_FEED_AIS`. Holds on the checked
paths.

**Bottom line:** ground-truth is usable as-is for **autoferry** and (structurally)
sim; **HAXR and philos real-data OSPA numbers should not be read as tracker
accuracy** until the coverage-filtering (philos) and zero-velocity + cadence/
resample (HAXR) issues are addressed; **pohang truth is not usable** until the
quaternion and loader-header bugs are fixed; **R-BAD's dangerous assumption is
correctly avoided.**

#### E.1 Dataset disposition — keep / fix / drop (recommendations)

| Dataset | License | Wired into tests? | Recommendation |
|---|---|---|---|
| autoferry | CC-BY-4.0 (commercial OK) | yes (5) | **Keep** — the trustworthy accuracy arm. |
| sim / sim_multisensor | own | yes (27) | **Keep**, but fix the emitter-noise-vs-stamped-covariance mismatch (E-block) before trusting NEES/consistency. |
| HAXR (data/dlr) | CC-BY-4.0 | yes (8) | **Keep**, but fix truth zero-velocity + add truth resample-to-clock before reading OSPA as accuracy. |
| philos | **CC-BY-NC-SA-4.0** | yes (33) | **Blocker (A2)** — purge committed derivatives from git+history or get a commercial license; also filter OSPA truth to in-coverage targets. |
| R-BAD | authors' terms | yes (30) | **Keep** — consistency arm, assumption correctly handled. |
| SMD | **none recorded** | no (2, fixture-internal) | Add license/provenance and confirm commercial use before any reliance; else drop. |
| **pohang** | **CC-BY-NC-4.0** | **no (0 C++)** | **Drop (recommended)** — see below. |
| dma / kystverket / marinecadastre / stonesoup-clone | mixed | no | Mark staging-only or remove (~2.3 GB unused). |

**Should we use Pohang? Recommendation: no — don't invest in it for this
release, and probably not at all under the commercial constraint.**

- *What it could offer:* a real radar+lidar+nav dataset in a **confined canal** —
  geometrically more extreme (continuous canal-wall clutter) than autoferry
  (fjord), HAXR/philos (harbor), or Imazu (open sea).
- *Why that value doesn't land here:*
  1. **License CC-BY-NC-4.0 (non-commercial)** — the exact class already rejected
     for MOANA (D7) and GFW (D10); navtracker is a commercial product.
  2. **The phenomenology it stresses is out of scope** — canal walls come out of
     the extractor as single meaningless blobs (σ_az up to 95°, σ_r up to 658 m),
     i.e. it stresses detection→plot *extraction*, which is ruled upstream's job.
  3. **No independent kinematic truth** → consistency arm at best, and R-BAD
     already fills that role with cleaner provenance.
  4. **Buggy/unfinished** — wrong quaternion component order, a generated header
     the loader silently rejects, 24% degenerate lidar rows, a byte-identical
     duplicate file. Real work for a redundant, license-blocked arm.
- *Action:* delete `data/pohang` (683 MB) to match the MOANA/GFW precedent, or —
  if kept for throwaway internal research only — isolate it and state the
  non-commercial restriction in the `pohang_radar`/`pohang_lidar` fixture READMEs
  (which currently omit it). Would only reconsider if all three hold at once: a
  genuine confined-water need, a commercial license, and a decision that
  extraction-stress is in scope — none true today.

- **autoferry is clean and fully validated** (CC-BY-4.0, documented provenance;
  units/frames/epoch/truth all cross-checked). Good.
- **Pohang data-quality** (if kept as an eval arm): ownship extractor reads the
  AHRS quaternion in the wrong component order; generated ownship CSV header not
  recognized by the loader; radar extractor emits canal-wall returns as single
  meaningless-centroid plots.
- **HAXR**: sparse AIS truth (10–20 s) scored against a 1 Hz eval clock inflates
  OSPA; AIS-derived truth velocity hardcoded to zero for all vessels.
- **Baselines lack provenance** — no CSV records git sha / compiler / host, so
  pinned numbers aren't reproducible off-machine; baselines README is stale.
- ~2.3 GB of fetched data (dma, kystverket, marinecadastre, stonesoup clone) is
  staged but unused by any test — should be marked staging-only.

---

## F. Release readiness / hygiene

Full list: `40-release-readiness-findings.md`.

- **No install/export/package story** — `CMakeLists.txt` has zero `install()` /
  `export()` rules and no `find_package(navtracker)` config, though the
  integration guide tells consumers to link the targets. Document the supported
  consumption path (add_subdirectory) and ideally add export targets.
- **ADR-0002 "promote static→moving within bounded latency" has no implementing
  code or test** — the load-bearing half of "presence over classification".
  Needs a scenario test (anchored object begins moving → moving track within N s).
- **Repo ships ~200 MB of baselines + copyrighted PDFs** — incl. an Elsevier
  publisher-copyrighted article (`S0029801822005753-…helgesen-2022`, 6.4 MB);
  redistribution is a copyright concern. `todo.md` (a personal KF/IMM
  learning-notes scratch file) is tracked and should not ship.
- Minor doc drift: CLAUDE.md/README name only Eigen+GoogleTest; conanfile also
  requires mcap + nlohmann_json.

### What's solid (verified clean)
- Layering invariant: `navtracker_core` links only Eigen (no I/O deps).
- All 20 port interfaces have virtual destructors.
- Learning docs: 29 chapters, index resolves, every major algorithm covered.
- TODO hygiene: one TODO in all shipping code.
- Determinism of the MHT/PMBM hot path (see C).
- GOSPA kernel externally validated (prior work); autoferry data validated.

---

## G. Suggested fix ordering

1. **A1 (GGA validation)** and **F-BUILD-1/2** — small, safe, mechanical.
2. **A2 (licensing)** — decide philos disposition; it gates the release
   independent of code.
3. **Theme 4** (output covariance NED decision + CPA sign) — operator-facing,
   one needs a contract decision from you.
4. **Theme 1** (datum/own-ship edges) — the antimeridian wrap, datum-shift sign
   + its test, and stale-adapter-datum together; highest cluster payoff.
5. **Theme 3** (bias chain) as one project; **Theme 5** (PMBM/pipeline
   timestamps) as one project.
6. **Theme 2** (RangeBearing2D init) — decide convert-vs-forbid, then fix all
   four estimators together with the one TODO in `AutoferryJsonReplay.hpp:52`.
7. Backfill the **Section D tests alongside each fix** (each confirmed bug has a
   matching coverage hole — close them together), and add the **fixture-skip
   guard** + per-test timeouts.
8. Hygiene (Section F) last.

## H. Process notes / caveats
- Do fixes in worktrees on branches (per CLAUDE.md); the recorded file:line
  anchors are against 317ecfd and will drift once edits land.
- The bug hunt ran at maximum depth (Fable 5) with adversarial verification;
  the audit phases (D/E/F) ran on Opus 4.8 and the readiness sweep (F) was done
  inline by the orchestrator without a second verifier — treat its findings as
  high-confidence-but-single-pass.
- "Unverified" medium/low findings in the detail files were **not** adversarially
  checked; triage before acting (some are duplicates of confirmed items seen
  from a different unit).
