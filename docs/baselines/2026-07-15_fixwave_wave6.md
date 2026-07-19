# Pre-release fix wave — wave 6 (final): release hygiene + ADR-0002 test + HAXR truth

Branch `fixwave-wave6`, off master `b8b754d` (merge-base with master is the
current tip, so wave-5 — incl. the 600 s veto timeout — is already in the base;
no rebase needed). Ticket:
`docs/superpowers/plans/2026-07-15-fixwave-wave6-hygiene-ticket.md`. Origin: the
pre-release review synthesis §F (release readiness) + §E (test-data fixes) — the
last non-licensing items between the repo and the review's ship bar. Two
substantive pieces (W6.1 ADR-0002 test, W6.2 HAXR truth), the rest hygiene.
Commits per finding; not merged/pushed. Suite under `NAVTRACKER_REQUIRE_FIXTURES=1`.

---

## W6.1 — ADR-0002 rule-3 bounded-latency promotion (substantive) — `83a3022`

ADR-0002's 2026-07-03 amendment rule 3 ("an object represented as static that
starts moving must be promoted to a vessel track within bounded latency") is the
load-bearing half of presence-over-classification, and it had **no implementing
test** (ADR line 188: "unverified today").

**Investigation first (stop-and-report contingency): the promotion PATH EXISTS**,
so this is a test, not a design gap. It is EMERGENT, not an explicit `promote()`
call (a full-tree search found none): a mover's returns leave the occupancy
footprint → the vacated cells decay → its fresh returns birth an ordinary track.
The deployable `imm_cv_ct_pmbm_coverage_land_ivgate` wires **no** occupancy model,
so an open-water anchored vessel there is never suppressed — it is tracked as a
low-SOG vessel and promotion is immediate.

**Deliverable:**
- New config-agnostic bench metric **`promotion_latency`** (`core/benchmark`
  Metrics/Sweep, additive per-truth `:truth_N`): scans from a truth's motion onset
  (first present step with |velocity| > 0.5 m/s) to the first present step where it
  is BOTH moving AND assigned to a track. 0 = tracked through the transition; a
  large sentinel = never promoted while moving. Lower-is-better (comparator
  default); not a headline column, so `canonicalMetrics()` is unchanged.
- `tests/benchmark/test_adr0002_promotion.cpp`, three levels:
  - **Metric unit test (the teeth):** synthetic BenchResults give latency 0
    (tracked through), **3** (rebirth after 3 moving steps), **6** (never-promoted
    sentinel), 0 (never moves) — proves the metric measures a real latency, so the
    scenario gates are not vacuous.
  - **Deployable gate:** `harbor_anchored_gets_underway` (truth_6 anchored at
    (−300,50), underway at t=11, 8 m/s) → worst-seed **promotion_latency 0 scans**,
    mean lifetime 0.97, **id_switches 0** (stable track_id through stop→go — the
    second ADR-0002 gap, unasserted before), track_breaks 0. Banded bound
    latency ≤ 2 (margin), lifetime ≥ 0.9, id_switches == 0.
  - **Detector gate:** the compact anchored boat is (correctly) kept as a vessel by
    the extent discriminator (latency 0), while the occupancy layer is active on the
    scene (occ_peak_structures 5.6, occ_suppress_hits 25 = the pier).
- The suppress→rebirth latency (an object that IS suppressed as a hazard then moves)
  is bounded **≤ 5 scans** at the model level by
  `test_live_occupancy_model.VacatedCellsRecoverWithinBoundedLatency` (cross-ref).
- ADR-0002 status notes updated; `test_sweep` row count 7→8 per-truth metrics.

## W6.2 — HAXR truth fixes (bench-side only; tracker untouched) — `1b8e1cc`

Two truth-side bugs made HAXR OSPA/GOSPA unreadable as tracker accuracy: (1) truth
velocity hardcoded to zero in `HaxrTruthLoader` (the HAXR AIS CSV carries no
SOG/COG); (2) sparse AIS truth (~10–20 s/fix) scored on a ~1 Hz clock with no
resample/hold. **Both fixed by the ONE shared helper the philos path already
uses** — `resampleTruthToClock(truth, 1.0, 30.0)` (`core/scenario/TruthResample`,
interpolates position AND finite-differences velocity) — added to the HAXR bench
path (`ReplayScenarioRun::HaxrScenarioRun`) and the standalone `test_haxr_ospa.cpp`.
No reinvention.

**A/B (deployable, kattwyk_08_t40, 302,509 raw CFAR plots, 1 seed):**

| metric | BEFORE (zero-vel, unresampled) | AFTER (resampled) | reading |
|---|---|---|---|
| gospa_false (m²) | 20956 | 20912 | **~unchanged** — over-count is truth-independent |
| gospa_missed (m²) | 826 | **7922** | sparsity HID the miss; dense truth reveals it |
| gospa_localization (m²) | 4.22 | 44.74 | before, only near-perfect matches scored |
| card_err_mean (tracks) | 100.7 | 65.0 | before inflated by absent truth |
| sog_rmse (m/s) | 0.247 | **1.279** | before scored vs a FAKE zero velocity |
| ospa_mean (m) | 496.6 | 472.0 | — |
| gospa_mean (m) | 143.4 | 168.1 | — |

**The old numbers reflected truth sparsity + fake-zero velocity, not tracker
error** — exactly the ticket's thesis. The fix does not improve the tracker; it
makes the metric MEASURE the tracker (which, on raw CFAR, is over-count +
high-miss). Standalone `HaxrOspa`: 199.51 → 195.89 m (cardinality-dominated either
way; still < 200 assert). Dated baseline snapshots NOT retro-edited; the corrected
numbers supersede the sparsity-artifact figures for reading HAXR as accuracy. Full
detail in the dated eval-log entry (2026-07-18). The veto/LOS HAXR A/B gates use
relative ON-vs-OFF assertions (common-mode truth change cancels) — they survive.

## W6.3 — repo shipping hygiene

- **W6.3.1 (`73b010c`) — copyrighted PDF + scratch.** The paywalled Elsevier PDF
  (Helgesen 2022, PII S0029801822005753) removed from HEAD, replaced by a
  citation-only `.pdf.stub.md` (DOI + open Autoferry dataset link). History rewrite
  stays PARKED with the philos A2 decision (removal is HEAD-only). `todo.md`
  (personal study scratch) removed. Sweep found no other copyrighted PDFs; the two
  remaining are author-posted arXiv preprints (kept; `docs/references/README`
  documents the redistribution policy).
- **W6.3.2 (`7d37d67`) — baseline provenance.** The `CsvProvenance` header + wiring
  already existed but emitted `git_sha`/`compiler`/`host` as `"unknown"`. Now
  populated: git sha (CMake configure-time, `<sha> (clean|dirty)`, dirty over
  TRACKED files only), compiler (`gcc 13.3.0`), host (`Linux x86_64`). Verified on a
  minimal run. Dated snapshots not retro-stamped.
- **W6.3.3 (`17ec555` + `8ca02c0`) — version-track the data manifest (arbiter ruling
  Option 3).** The bigger catch: the license/provenance manifest (`data/README.md` —
  which datasets are non-commercial, what was removed and why) was itself
  **unversioned**; `data/.gitignore`'s `!README.md` showed tracking was always
  intended but a parent `/data/` rule silently defeated it. Fixed the top-level
  `.gitignore` (`/data/*` + `!/data/README.md` + `!/data/.gitignore`; payload still
  ignored), and committed the manifest **record-vs-interpretation**: commit 1 =
  `data/README.md` at the main tree's content **byte-identical** (pristine record,
  carries the pohang-REMOVED row); commit 2 = the W6.3.3 staging-only markers for
  `dma`/`kystverket`/`marinecadastre`/`stonesoup` (fetched but not consumed by the
  C++ suite) + a "now git-tracked" note. Worktree note: `data/` is now a real
  directory with inner-level symlinks to the main tree's payload subdirs.
  **The main tree is untouched** — the arbiter reconciles the tracked manifest vs
  the main tree's untracked original at merge (byte-compare-then-replace, since
  commit 1 is byte-identical).
- **W6.3.4 (`73b010c`) — dep-doc drift.** CLAUDE.md tech-stack + README build now
  list the real Conan deps (Eigen 3.4, mcap 1.4, nlohmann_json 3.11, GoogleTest
  1.14); mcap + nlohmann_json were previously undocumented.

## W6.4 — install/export (decision-scoped) — `c13cd12`

The review: zero `install()`/`export()` while the guide tells consumers to link
targets. **The export did NOT balloon**, so it shipped (not stopped at docs):
- project() gains VERSION 0.1.0; GNUInstallDirs; core/nmea/t2t include dirs use
  BUILD/INSTALL_INTERFACE (build tree unchanged, install tree = `include/`,
  structure-preserving).
- `install(TARGETS … EXPORT)` + `install(EXPORT navtracker::)` + header install +
  `configure_package_config_file` + version file. `navtrackerConfig.cmake.in`
  re-finds ONLY Eigen3 (the sole PUBLIC external dep of the exported set;
  nlohmann_json/mcap are PRIVATE to non-exported targets).
- **Teeth:** `examples/consumer_find_package/` — a standalone `find_package(navtracker)`
  consumer. Verified end to end: install → configure with `-Dnavtracker_DIR` + the
  Conan toolchain → compiled, linked `navtracker::navtracker_core`, ran
  ("north=1112.96 m").
- integration-guide §1 documents all three consumption paths (add_subdirectory /
  FetchContent / find_package) + the Eigen3 dependency contract.

---

## Verification

- **Full suite `NAVTRACKER_REQUIRE_FIXTURES=1 ctest -j8`: 1225/1225 pass, 0 failed,
  0 skips** (strict mode ⇒ 0 fixture skips). VetoIsolationHaxrAB 528 s (under the
  600 s cap; wave-5's bump stays load-bearing under `-j` load). **Both HAXR A/B gates
  (LosGuardHaxrAB, VetoIsolationHaxrAB) survive the W6.2 truth change** — their
  relative ON-vs-OFF assertions are common-mode to the truth. HaxrOspa and all three
  ADR-0002 promotion tests green in-suite.
- Tracker byte-untouched (acceptance #2): W6.1 adds a bench metric + test; W6.2 is
  bench-side truth handling; W6.3/W6.4 are docs/build/CMake. No `core/` tracker
  logic changed.
- **No Cl-4 gauntlet headline row moves** (the deployable's HAXR numbers are
  re-priced by the truth fix, but those were sparsity artifacts, not frozen Cl-4
  gauntlet rows).

## Handoff

- All merge-ready on `fixwave-wave6`; per-finding commits; **not merged/pushed**.
- **Arbiter action — W6.3.3 data manifest:** at merge, `data/README.md` will collide
  with the main tree's untracked original. Commit 1 is byte-identical to it, so the
  procedure is the same byte-compare-then-replace used for the review files: confirm
  no divergence, let the tracked manifest supersede. Do NOT expect the payload
  subdirs to be tracked (they stay ignored).
- W6.4's `examples/consumer_find_package/` is a standalone example, not wired into
  the main build/CI (it consumes an INSTALLED navtracker).
