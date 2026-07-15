# W1 — Build + suite + sanitizer report (master 317ecfd, 2026-07-09)

Environment: isolated worktree `/home/andreas/workspace/navtracker-review-build`
(detached @ 317ecfd), g++ 13.3, Conan 2.14, Release + ASan/UBSan builds.
Logs: `<worktree>/review-logs/`.

## Results

| Run | Result | Notes |
|---|---|---|
| Release build, `-Wall -Wextra` | OK | 140 warnings: 122 `-Wmissing-field-initializers` (noise), rest triaged below |
| Release ctest (first run) | 1090/1090 passed, 26 s | **87 gtest cases SKIPPED** — caused by the fixture-symlink trap, see below |
| ASan+UBSan build (`-O1 -g`, NO `-DNDEBUG` → asserts + Eigen checks live) | OK | |
| ASan ctest (first run) | 1089/1090, **1 FAILED** | stack-use-after-scope, see F-BUILD-1 |
| Re-runs with fixtures properly linked | in progress at time of writing | `ctest_release_fixed.log`, `ctest_asan_fixed.log`, skip lists `skipped_*_fixed.txt` |

## Confirmed findings

### F-BUILD-1 (test-code UB, sanitizer-confirmed): dangling reference in test_own_ship_nmea.cpp:275

`tests/adapters/own_ship/test_own_ship_nmea.cpp:275`:
`const auto& p = *provider.latest();` — `OwnShipProvider::latest()` returns
`std::optional<OwnShipPose>` **by value** (core/own_ship/OwnShipProvider.hpp:105).
Dereferencing the temporary optional and binding a reference does NOT extend the
temporary's lifetime → every later read of `p` (lines 279, 288 …) is
use-after-scope. ASan fails the test deterministically
(`OwnShipNmeaAdapterTest.RmcStaleTriggersEstimatorFallback`); GCC 13 also warns
`-Wdangling-pointer` at :279/:288 in Release. The test currently passes in
Release only by luck of stack reuse — its assertions are running on garbage,
i.e. this test's coverage is void.
Fix: `const auto p = *provider.latest();` (copy). Grep found no other
`auto& x = *…optional-by-value…` site in the repo.

### F-BUILD-2 (UB via third-party contract, sanitizer-confirmed): mcap writer null-pointer memcpy

UBSan: `mcap/writer.inl:58:37: runtime error: null pointer passed as argument 1,
which is declared to never be null` — fired in 14 foxglove-related tests
(test IDs 1076–1089). Trigger is on the navtracker side: something in
`adapters/foxglove/` (McapWriter / recorder path) hands mcap a null data
pointer with size 0 (e.g. empty `std::vector::data()` or default Message
fields). Formally UB; benign today, but a hardened libc / different compiler
may trap. Fix in our adapter: never pass nullptr (use a static dummy byte or
skip the write when empty).

### F-BUILD-3 (fixture-trap variant, process finding): partially-tracked `tests/fixtures/` defeats the documented symlink recipe

CLAUDE.md says worktree users should "symlink the fixture dirs". But
`tests/fixtures/` contains TRACKED files (e.g. `static/`, small fixtures), so a
worktree checkout materializes `tests/fixtures/` as a real directory →
`ln -s <main>/tests/fixtures tests/fixtures` silently creates
`tests/fixtures/fixtures` INSIDE it, and 87 gtest cases skip "fixture not
reachable" while ctest still reports 100% green. Exactly the 2026-07-08
silent-skip failure mode, now with a mechanical cause. Suites affected:
SimMultisensorScenarioRun(10), ReplayScenarioRun(10), PhilosCoverageDecay6c(10),
Imazu22(8), Rbad(6), PhilosSunsetLabels(6), RadarTruthLoader(4), PhilosFarCross(4),
PhilosCloseApproachLabels(4), LosShadowGuard(4), CameraBearingSmoke(4), + 2 each
PhilosOspa/PhilosClutterMapAB/PhilosCloseApproachCpa/LosGuardHaxrAB/HaxrOspa/
GeoJsonCoastline/ClutterBurstBirthConfirmProbe/AutoferryJsonReplay.
Fix candidates: per-subdirectory symlinks in the recipe; or an env var
(`NAVTRACKER_FIXTURE_DIR`) honored by tests; or a skip-count guard test/script
that fails when skips exceed the expected-skip list.

## Compiler-warning triage (non-noise)

Non-test code: all benign —
- `core/scenario/Builders.cpp:87` unused function `makeShoreMeasurement`
- `adapters/replay/PlotCsvReplayAdapter.cpp:19` unused function `wrap`
- `core/benchmark/Config.cpp:1233` `-Wcomment` multi-line comment

Test code (worth a cleanup pass, some hint at test-logic problems):
- `test_own_ship_nmea.cpp:279,288` `-Wdangling-pointer` (= F-BUILD-1)
- `test_autoferry_json_replay.cpp:53,135` (`-Warray-bounds`/uninit family)
- `test_track_manager.cpp:65,66`, `test_measurement_builders.cpp:22`,
  `test_joint_events.cpp:37` (uninitialized/maybe-uninitialized family)
- 122× `-Wmissing-field-initializers` — mostly benign brace-init noise, but it
  is the same warning class that would catch a *real* forgotten config field.

## Re-run with fixtures properly linked (2026-07-10 早)

- Release: 1090/1090 passed. ASan: 1089/1090 (only known F-BUILD-1). No new
  sanitizer hits from the ~17 recovered tests (Philos*, CameraBearingSmoke,
  GeoJsonCoastline, LosShadowGuard, LosGuardHaxrAB).
- **~27 tests STILL skip even with all fixtures present** — root cause found
  (upgrades F-BUILD-3): these tests resolve fixture paths **relative to CWD
  and assume project root** (e.g. `tests/replay/test_radar_truth_loader.cpp:8`
  "Fixture paths are relative to project root"), but ctest runs with CWD =
  build dir. So `ctest` **never** executes them, in ANY tree — they run only
  via the bench driver started from project root, or manually. The suite has
  two conventions side by side (some tests walk up / try prefixes and DO run
  under ctest; these don't). "ctest 100% green" is structurally blind to:
  AutoferryJsonReplay, ClutterBurstBirthConfirmProbe.Race, HaxrOspa,
  Imazu22ScenarioRun(4), PhilosClutterMapAB, PhilosFarCross(2), PhilosOspa,
  RadarTruthLoader(2), RbadScenarioRun(3), ReplayScenarioRun(5),
  SimMultisensorScenarioRun(5).
  Candidate fix: set per-test `WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}` in
  gtest_discover_tests / add_test, or a fixture-root helper that walks up.
- Follow-up executed: those suites run manually from worktree root under
  Release AND ASan → `review-logs/rootcwd_release.log`, `rootcwd_asan.log`.
  **Result: 36/36 PASSED in both.** Release 69 s, ASan ~25 min,
  **0 sanitizer hits** on the real-data replay paths (PMBM/MHT over Imazu,
  HAXR, Philos, Autoferry, R-BAD, SimMultisensor). So the cwd-gating is a
  coverage/process problem, not hiding failures today — but nothing in CI
  guards these 36 from regressing silently.

---

# Reconciliation disposition — 2026-07-15

- **F-BUILD-1** (test UB — `const auto&` bound to a dereferenced temporary `optional`, `test_own_ship_nmea.cpp`) — FIXED, W2 `34367f6`.
- **F-BUILD-2** (UBSan null-pointer `memcpy` in the mcap writer on the foxglove path) — FIXED, W2 `34367f6`.
- **F-BUILD-3** (coverage blindness — ~36 real-data tests skipping silently; fixture-symlink recipe; CWD-relative fixture resolution) — fixture-skip guard added W2 `34367f6`; the **0-skip strict ceremony** (`NAVTRACKER_REQUIRE_FIXTURES=1`, skip-list diffed by name) is now the merge standard and has caught this class since.
- **Determinism** — clean bill confirmed; the one gap (plain `Tracker::processBatch` never got the backlog-#15 batch sort) — FIXED, W5.3 `e8d99af` (fixwave-wave5, pending merge).
