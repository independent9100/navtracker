# Implementer prompt — backlog mediums, batch 1: pick and fix the top plausible-opens from your own triage (#26–#38)

Status: ready to hand off. Paste everything below the line. Origin: the
review reconciliation filed 24 PLAUSIBLE-OPEN medium findings as
improvement-backlog entries #26–#38 (hand-traced, re-anchored to master,
none release-blocking). The entries are self-contained in
`docs/algorithms/improvement-backlog.md` — this ticket has you pull the
first batch. Budget ~2 days including the selection checkpoint.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-bm1 -b backlog-mediums-batch1` off current
master, own build dir, fixtures inner-level symlinked; suite under
`NAVTRACKER_REQUIRE_FIXTURES=1`. Commit on your branch; never merge/push
master. If you still have a worktree from a previous merged ticket
(e.g. `../navtracker-csrc`), tear it down first.)

## Step 1 — selection (checkpoint BEFORE any code)

From backlog #26–#38, rank by (severity × deployable-path relevance ×
fix-cost), and propose the top 3–5 for this batch. Per candidate, two
lines: the defect mechanism, and how you'll teeth-prove the fix. Explicitly
name what you are NOT pulling and why (pull-based items stay parked; a
finding whose fix would change tracking behavior on the deployable config
gets flagged as its-own-cycle material — the F2 precedent — not batched).
Hand the selection to the arbiter; code starts on the GO.

## Step 2 — fix the approved batch

TDD per item (failing test → fix → green), teeth per the #24 standard,
one commit per item. A/B duty on anything touching a tracking path:
deployable-config rows = finding rule, same as the fix waves. Docs riding
where a fix touches consumer surface or documented behavior.

## Acceptance

1. Selection checkpoint honored; each landed item marked FIXED (dated,
   with sha) in its backlog entry.
2. Full suite green under strict mode; adversarial review if any fix
   touches a tracker hot path.
3. Write-up `docs/baselines/<date>_backlog_mediums_batch1.md` + eval-log
   entry.
4. Commit on your branch; do not merge or push master.
