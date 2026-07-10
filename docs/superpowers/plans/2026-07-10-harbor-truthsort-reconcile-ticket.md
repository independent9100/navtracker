# Implementer prompt — harbor truth-sort reconciliation: is the yardstick's card_err real, or still fragmentation?

Status: ready to hand off. Paste everything below the line. Origin: the
promotion dossier (2026-07-10, merged e80c1e8) reports harbor
`card_err` absolute values ~8–11 with the caveat "confounded by the unmerged
`pmbm-harbor-truth-sort-fix` branch". The arbiter's check shows that caveat is
at least PARTLY stale: `sortTruthByTime` IS in master (`core/scenario/
Builders.cpp:122`, landed via 3ee491f + 3aa9c58 — the same commits the Cl-1
drift analysis credited), and the old branch ref no longer exists. But the
2026-07-02 record says the CORRECTED yardstick should read card_err ≈ 0
(from +11.64), and the dossier still measured ~8–11 — so either a piece of
the old fix never landed, or the harbor over-count is REAL and the dossier's
caveat under-sells a genuine finding. Both outcomes matter: the harbor row
feeds the promotion decision and the new Cl-4 (one-config) line. Budget
~half day to 1 day. North-star tag: Cl-3 measurement integrity / Cl-4 input.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-harborfix -b harbor-truthsort-reconcile`, own
build dir; harbor + philos fixtures from the MAIN tree; skips named).
MEASUREMENT/RECONCILIATION first — code only if a missing piece is proven.

## Step 1 — inventory: what did the 2026-07-02 branch contain, and what landed?

The branch ref is gone. Reconstruct its intended content from the record:
the 2026-07-02 eval-log/baseline entries and the harbor-yardstick M1 notes
(truth-fragmentation fix = `sortTruthByTime` + CONTRACT TESTS pinning
truth-stream integrity). Then check master: the sort exists
(`Builders.cpp:122`, called at 669/704). Do the contract tests exist? Is the
sort applied on EVERY path that feeds `harbor_complete_truth` (loader vs
additive builders — the 2026-06 truth-fragmentation bugs were exactly
"one path sorted, another not")? Report the inventory: landed / missing /
never-existed, with commits.

## Step 2 — measure: is harbor truth still fragmented?

Directly, not by inference: load `harbor_complete_truth` as the bench does,
and count truth segments per true object (same-id truth broken into
multiple metric-side objects = fragmentation). The 2026-06-10 forensics
method applies (fragmented truth pegs card_err/id_switches upward, punishing
the tracker for the harness's sins). Deliverable: a per-object table —
either "truth is clean, N objects, contiguous" or "object X fragments into
k segments at times t…".

## Step 3 — resolve, whichever way it points

- **If a piece is missing:** land it (additive; contract tests included —
  banded/structural per #24, no exact pins), re-run the harbor yardstick
  both contenders, and report the corrected M2-gate numbers (expectation
  from the 2026-07-02 record: card_err +11.64 → ≈0, lifetime ≥ 0.974 — if
  the corrected numbers do NOT land near that, say so; do not force them).
- **If truth is clean and card_err ~8–11 is REAL:** that is a genuine
  harbor over-count finding the dossier's caveat currently hides — write it
  up as such (which tracker, which objects, phantom or duplicate), and
  correct the dossier + eval-log caveat lines (a dated correction section,
  not a silent edit).
- Either way: the dossier's harbor row gets a follow-up note pointing here,
  so the promotion decision reads the right number.

## Acceptance

1. Step-1 inventory with commits; Step-2 fragmentation table; Step-3
   resolution (landed fix + corrected yardstick numbers, OR the real-
   over-count write-up + dossier correction).
2. Zero tracker config/tuning changes; harness/test changes only if a
   missing piece is proven, and then additive with contract tests.
3. Full suite green in your worktree, skips named.
4. Stop-and-report: the fragmentation exists but fixing it requires
   changing truth SEMANTICS (not just ordering/grouping) — that is a
   yardstick-design decision for the arbiter, not a patch.
