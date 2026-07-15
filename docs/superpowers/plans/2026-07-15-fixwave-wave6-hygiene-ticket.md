# Implementer prompt — fix wave, wave 6 (final): release hygiene + the ADR-0002 bounded-latency test + HAXR truth fixes

Status: ready to hand off AFTER the wave-5 rebase merges. Paste everything
below the line. Origin: the pre-release review synthesis §F (release
readiness) + §E (test-data fixes) — the last non-licensing items between
the repo and the review's ship bar. Mostly mechanical; two substantive
pieces (the ADR-0002 test, the HAXR truth fixes). Budget ~2 days.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-fw6 -b fixwave-wave6` off post-merge
master, own build dir, fixtures inner-level symlinked; suite under
`NAVTRACKER_REQUIRE_FIXTURES=1`. Commit on your branch; never merge/push
master.)

## W6.1 — ADR-0002 bounded-latency promotion: the missing test (substantive)

The review's sharpest readiness finding: "promote static→moving within
bounded latency" — the load-bearing half of presence-over-classification —
has NO implementing test anywhere. Build the scenario test: an object
represented as static (anchored/hazard) begins moving → a moving track
exists within a bounded time. Measure the actual latency on the deployable
config first, then pin a banded bound with real margin (#24 shapes). If
measurement reveals the promotion PATH ITSELF is missing (not just the
test), STOP-AND-REPORT — that's a design gap for the arbiter/user, not a
test fix.

## W6.2 — HAXR truth fixes (the §E caveats that block reading its OSPA as accuracy)

1. Truth velocity hardcoded to zero for all vessels → derive from
   consecutive AIS fixes (finite-difference, flagged where the gap is
   too long).
2. Sparse AIS truth (10–20 s) scored on a 1 Hz clock without resampling →
   add truth hold/resample-to-clock (the philos path already does this —
   reuse its mechanism, don't reinvent).
   A/B the HAXR rows before/after and record the corrected numbers as a
   dated eval-log entry — the old numbers reflected truth sparsity, not
   tracker error; say so.

## W6.3 — repo shipping hygiene

1. Remove from HEAD (history rewrite stays PARKED with the philos A2
   decision — do not rewrite history): the Elsevier publisher-copyrighted
   PDF (`docs/references/S0029801822005753-…helgesen-2022….pdf` — replace
   with a stub containing the DOI + citation), `todo.md` (personal
   scratch), any other copyrighted PDFs found by a sweep of
   `docs/references/`.
2. Baseline provenance: add a small header convention (git sha, date,
   compiler/host) for NEW baseline CSVs going forward + a one-line README
   note; do NOT retro-edit existing dated snapshots (they're immutable
   records).
3. `data/README.md` staging-only markers for the ~2.3 GB unused fetched
   sets (dma, kystverket, marinecadastre, stonesoup clone) — mark or
   delete per the manifest's own conventions (pohang precedent).
4. Doc drift: CLAUDE.md/README dependency list (add mcap + nlohmann_json).

## W6.4 — install/export story (decision-scoped, not gold-plating)

The review: zero `install()`/`export()` rules while the guide tells
consumers to link targets. Minimum honest fix: DOCUMENT the supported
consumption path (add_subdirectory / FetchContent) in the integration
guide §1, and add basic `install()` + export targets for
`navtracker_core`/`navtracker_nmea`/`navtracker_t2t` IF it stays under
half a day — a `find_package(navtracker)` smoke test in a scratch dir as
the teeth. If it balloons (Conan interplay, version files), STOP at the
documentation fix and file the export work as a backlog item with what
you learned.

## Acceptance

1. W6.1 test landed (or the stop-and-report); W6.2 corrected HAXR numbers
   in a dated entry; W6.3 removals + conventions; W6.4 at whichever depth
   it honestly reached.
2. Full suite green under strict mode; no behavior changes outside W6.2's
   truth handling (bench-side only — tracker untouched).
3. Write-up `docs/baselines/2026-07-15_fixwave_wave6.md` + eval-log entry.
4. Commit on your branch; do not merge or push master.
