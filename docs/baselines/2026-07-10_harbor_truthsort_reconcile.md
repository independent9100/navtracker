# Harbor truth-sort reconciliation — is the yardstick's `card_err` real, or still fragmentation?

**Date:** 2026-07-10 · **Branch:** `harbor-truthsort-reconcile` · **North-star:** Cl-3
measurement integrity / Cl-4 input.

**Verdict (one line):** The truth-sort fix is **fully landed and stable**; harbor
truth is **not fragmented**; the harbor `card_err ~8–11` is a **REAL over-count**
(phantom tracks on the uncharted pier + transient sea clutter), **not** a harness
artifact. The promotion dossier's harbor caveat was wrong and is corrected below.

This closes the ticket `docs/superpowers/plans/2026-07-10-harbor-truthsort-reconcile-ticket.md`.
The ticket's premise — that the *corrected* yardstick "should read card_err ≈ 0
(from +11.64)" — was an inversion: `+11.64` **is** the corrected number.

---

## Step 1 — Inventory: what landed?

The `pmbm-harbor-truth-sort-fix` branch ref is gone, but its entire content is on
master, in two commits, both landed **2026-07-02**:

| Commit | What it did |
|---|---|
| `3ee491f` `fix(bench): sort truth in additive builders — harbor baseline was fragmented` | Added `sortTruthByTime` helper; called it in **all three** additive builders (`addFixedClutter`, `addAnchoredBoats`, `addUniformClutter`); added the two regression guards (`Builders.AddAnchoredBoats…` + `HarborCompleteTruth.TruthIsTimeSortedIntoFortyCompleteGroups`); recorded the corrected baseline. |
| `3aa9c58` `fix(bench): close remaining truth-sort gaps found by review` | Sorted the HAXR replay truth path (same bug family); switched `sortMeasurementsByTime`/`sortTruthByTime` to the exact `Timestamp` comparator (nanos, not lossy `.seconds()` doubles). |

**Every path feeding `harbor_complete_truth` is sorted.** The scenario is built
(`adapters/benchmark/SimScenarioRun.cpp:503`) as
`buildCrossingTargetsScenario` → `addAnchoredBoats` (sorts, `Builders.cpp:704`)
→ `addFixedClutter` (sorts, `:669`) → `addUniformClutter` (sorts last, `:757`).
`groupTruth` (`core/benchmark/BenchRunner.cpp:87`) buckets by exact time, so a
sorted stream yields exactly one group per scan.

**Contract tests exist and pin *no-fragmentation*, not just ordering.**
`tests/benchmark/test_harbor_complete_truth.cpp:71`
(`TruthIsTimeSortedIntoFortyCompleteGroups`) asserts (a) truth is time-sorted
**and** (b) it groups — exactly as `BenchRunner::groupTruth` does — into **40
complete `{1..5}` groups, not 80 fragmented ones**. This is the structural check,
and it is compiled (`CMakeLists.txt:361`) and green.

Classification: **sort = LANDED. Every-path coverage = LANDED. No-fragmentation
contract test = LANDED.** Nothing missing, nothing never-existed.

## Step 2 — Measure: is harbor truth still fragmented?

Directly, from the generated scenario (seed 0), grouping truth exactly as the
bench does:

| id | kind | n_samples | t_first | t_last | monotonic runs | in all 40 groups |
|---:|---|---:|---:|---:|---:|:--|
| 1 | mover | 40 | 1.00 | 40.00 | 1 | yes |
| 2 | mover | 40 | 1.00 | 40.00 | 1 | yes |
| 3 | anchored boat | 40 | 1.00 | 40.00 | 1 | yes |
| 4 | anchored boat | 40 | 1.00 | 40.00 | 1 | yes |
| 5 | anchored boat | 40 | 1.00 | 40.00 | 1 | yes |

Total: 200 truth samples → **40 distinct time-groups**, each holding all five ids.
**Truth is clean: 5 objects, each a single contiguous run, present in every scan.**
No object fragments into multiple metric-side segments. (Reproduced via a
throwaway diagnostic in the harbor contract test, then reverted; the standing
contract test asserts the same 40-group property permanently.)

## Step 3 — Resolution: the over-count is REAL

Today's harbor yardstick (`imm_cv_ct_pmbm`, 5-seed mean), measured in this
worktree, is **byte-identical to the 2026-07-02 corrected baseline**:

| metric | today | 3ee491f "corrected" | pre-fix (fragmented, invalid) |
|---|---:|---:|---:|
| card_err_mean | **11.64** | 11.64 | 13.32 |
| gospa_mean | 50.63 | 50.63 | 53.02 |
| gospa_false | 2362 | 2362 | 2705.5 |
| gospa_missed | 34 | 34 | 41.5 |
| lifetime_ratio | 0.974 | 0.974 | 0.92 |

The fragmentation fix moved `card_err 13.32 → 11.64` (and lifetime `0.92 → 0.974`).
It **never** moved it toward zero. `+11.64` is the number on clean truth.

**What the over-count is — which tracker, which objects, phantom or duplicate:**

- **Tracker:** the canonical `imm_cv_ct_pmbm`. All harbor configs sit in the same
  band (dossier §1d: mht 8.83, candidate/`coverage_land` 8.0, canonical pmbm 11.1;
  single-seed vs the 11.64 five-seed here).
- **Phantom, not duplicate, not missed.** `card_err_mean` is
  `|confirmed_tracks| − |truth|` averaged per scan (`core/benchmark/Metrics.cpp:94`).
  With 5 truth, `+11.64` means **~16–17 confirmed tracks per scan, ~11–12 of them
  phantom**. `gospa_missed = 34` (low) and `lifetime_ratio = 0.974` (high) confirm
  the 5 real targets **are** tracked — the excess is additive false tracks, and
  `gospa_false = 2362` dominates the GOSPA sum.
- **Which objects:** the **uncharted pier** — a fixed *extended* structure, a
  120 m line of **13 points** (`SimScenarioRun.cpp:67`, x ∈ [−60,60] at y=−350,
  10 m apart) at P_D 0.9 → ~11.7 persistent returns/scan. That maps almost 1:1 to
  the ~11–12 phantom tracks. Plus transient uniform sea clutter (5/scan). These
  carry **no truth** by design (§5.3 of `synthetic-clutter-bench.md`), so each
  phantom scores as a pure false alarm.

**Three independent confirmations that the over-count is the pier (structural):**
1. Geometry: 13 persistent pier points ≈ the 11.64 excess.
2. Charting the pier removes it: the R5 A/B (`synthetic-clutter-bench.md:458`)
   records `imm_cv_ct_pmbm_static` dropping `card_err 11.64 → 7.43`,
   `gospa_false 2362 → 1518` — the charted-obstacle birth prior suppresses exactly
   these phantoms. `test_harbor_boat_near_pier.cpp:149` gates `card_static < card_base`.
3. The occupancy/structure-suppressing config (`coverage_land`) cuts canonical
   `card_err 11.1 → 8.0` in the dossier.

This is precisely the static-structure over-count the harbor yardstick exists to
measure (ADR-0001 offshore no-birth zone, ADR-0002 charted suppression, and the
Cl-4 `<50 m no-birth zone` blocker). It is a genuine tracker property, not a
metric bug.

## What was actually wrong: the promotion dossier's caveat

The authoritative harbor docs are **correct**:
`docs/algorithms/synthetic-clutter-bench.md` §5.3/§5.5/§5.6 explain the ordering
invariant, the landed fix, the contract test, and treat `card_err_mean` /
`gospa_false` as *real* over-count signals for the M2 promotion gate.

Only the **promotion dossier** (`2026-07-10_pmbm_promotion_dossier.md`) carried a
stale, inverted caveat (§1d and the limitations line). It claimed the
`pmbm-harbor-truth-sort-fix` branch was "unmerged at `d94471e`" and that
`card_err ~8–11` was "inflated by a known truth-fragmentation harness artifact."
Both are false:

- The fix commits `3ee491f` and `3aa9c58` are **ancestors of `d94471e`** (the
  dossier's own base). The dossier measured on **already-corrected, time-sorted
  truth** — the artifact was not present in its numbers.
- Truth is not fragmented (Step 2), so nothing inflated the number.

The dossier's **delta reasoning stays valid** (candidate 8.0 < champion 8.8 <
canonical-pmbm 11.1 — a lower over-count is better). What changes is that the
**absolute is also meaningful**: it is a real phantom-pier count, not a discardable
artifact — which is exactly why `coverage_land`/`_static` reducing it is a genuine
improvement and why the Cl-4 one-config effort must drive it down.

The dossier's §1d caveat and limitations line were corrected in place with a dated
`[correction 2026-07-10]` note pointing here (not a silent edit).

## Acceptance checklist

1. Step-1 inventory (commits `3ee491f`/`3aa9c58`, all-paths-sorted, contract test
   present) ✔ · Step-2 fragmentation table (clean, 5 objects, 40 groups) ✔ ·
   Step-3 resolution (over-count is real; dossier caveat corrected) ✔.
2. **Zero tracker config/tuning changes. No harness/test changes** (no piece was
   missing — the diagnostic was throwaway and reverted). Docs only.
3. Full suite green in the worktree: **100% passed, 0 failed of 1093**, 45
   `***Skipped` — the standard fixture-gated set (Boston GeoJSON, HAXR, philos,
   RBAD, replay/sim-ms/Imazu scenario runs; all require local-only data absent
   from the worktree), 0 did-not-run.
