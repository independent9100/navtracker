# Implementer prompt — review reconciliation: commit the review record, mark the fixed findings, triage the unverified mediums

Status: ready to hand off (independent of wave 5 — docs + triage only, no
code). Paste everything below the line. Origin: the 2026-07-09→11
pre-release review is finished and its fix wave is now substantially
merged, but the review's own artifacts are still UNTRACKED working files,
no finding carries a FIXED mark, and the synthesis itself says the 31
medium / 41 low unverified bug-hunt findings "should be triaged before
acting." This ticket closes the review arc as a durable record. Budget
~1–1.5 days. NO code changes — triage verdicts become backlog entries or
follow-up tickets, never drive-by fixes.

---

You are working in the navtracker repo (read `CLAUDE.md`; worktree
`git worktree add ../navtracker-rrec -b review-reconciliation`, own build
dir only if you need to run anything; commit on your branch, never
merge/push master).

## Step 1 — commit the review record

`docs/reviews/2026-07-09-prerelease-open-points.md` and
`docs/reviews/2026-07-09-prerelease-review/` are untracked (the review
session's working state; that session is done). Commit them AS-IS first
(one commit, no edits) so the record has a pristine baseline — BUT check
for local-data references first: nothing under `docs/reviews/` may embed
licensed data content (grep for large embedded blobs; the raw JSON finding
files are fine). `err.txt` and `next.md` at repo root: inspect briefly —
if they are review-session scratch, delete; if user files, leave and note.

## Step 2 — dated FIXED/DISPOSITIONED marks

Append a dated disposition section to each findings file (do NOT rewrite
original finding text — append, per the house correction style), covering:

- Wave 1: F1 GGA (FIXED fa4db84), F3 axis contract (RESOLVED dual-API
  fa4db84).
- F2 cycle: SourceTouch (FIXED 6fcd44e, measured disposition; note the
  Rider-B channel-conflation correction — the finding conflated
  `recent_contributions` with `contributing_sources`).
- Wave 2: stale adapter datum, antimeridian, −γ sign (+ the corrected test
  pin), DeclaredSensorActivity ×2, F-BUILD-1/2, fixture-skip guard
  (34367f6).
- Wave 3: bias chain ×4 (b284f8f). Wave 4: RangeBearing2D init, UKF/IMM
  circular mean, CPA sign + spec (738e542). Wave 5: soft-assoc,
  double-propagate, batch sort, Tentative→Coasting, MHT leaf + D-backfill
  (merge sha at reconciliation time).
- A1/A2: pohang REMOVED (user 2026-07-12); philos licensing DEFERRED by
  user decision — still OPEN as the last release blocker; Elsevier PDF
  removal rides wave 6.
- Cross-check your list against the eval-log entries and the
  `docs/baselines/2026-07-1x_fixwave_*.md` write-ups — every mark carries
  its merge sha.

## Step 3 — triage the unverified mediums (the substantive half)

The 31 unverified MEDIUM bug-hunt findings (10-bughunt-findings.md + raw
JSON): for each, classify with a one-line verdict:

- **DUPLICATE** of a confirmed-and-fixed finding (the synthesis warns many
  are the same defect seen from another unit) — mark with the fix sha.
- **OBSOLETE** — the code it described was rewritten by the fix wave
  (check against the wave diffs).
- **REFUTED** — hand-trace shows it was wrong (say why in two lines).
- **PLAUSIBLE-OPEN** — real enough to keep: file it as a numbered
  improvement-backlog entry (severity, file:line re-anchored to current
  master, one-line mechanism) — entries only, NO fixes.

The 41 lows: sweep for duplicates/obsolete only; keep the remainder as a
single grouped backlog note (do not spend per-finding depth on lows).

## Acceptance

1. Review record committed pristine, then marks + triage as separate
   commits (record vs interpretation stay distinguishable).
2. Every confirmed finding carries a dated disposition + sha; the medium
   triage table complete; plausible-opens filed in
   `docs/algorithms/improvement-backlog.md`.
3. No code changes anywhere. Suite untouched (docs-only diff).
4. Write-up: a short closing section in the open-points doc ("fix-wave
   outcome", dated) — the review arc's last word.
5. Commit on your branch; do not merge or push master.
