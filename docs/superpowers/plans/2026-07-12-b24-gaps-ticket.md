# Implementer prompt — close the two #24 stop-and-report gaps: T2T invariant-5 end-to-end gate (b24-2) + PMBM multi-hypothesis test scenario (b24-1)

Status: ready to hand off AFTER the #24 sweep merge lands on master (your
branch must include it — both parts build on the swept tests). Paste
everything below the line. Origin: your own #24 sweep's stop-and-report
findings (`docs/baselines/2026-07-11_b24_assertion_sweep.md`). These are
design gaps, not assertion tweaks — hence a separate ticket with production
surface allowed where stated. Budget ~1–1.5 days total, b24-2 first.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-b24g
-b b24-gaps`, own build dir, fixtures inner-level symlinked, 0-skip runs;
commit on your branch, never merge/push master). TDD both parts.

## Part 1 — b24-2: per-fused-track contributing-arm counts (T2T invariant 5, end-to-end)

The scenario-6 invariant-5 check ("both targets fused despite shared MMSI —
external id is never the fusion key") cannot currently gate a vessel SPLIT
end-to-end because `BenchResult` carries no per-fused-track
contributing-arm information. Allowed production surface: **additive
fields on the T2T bench-result path only** (e.g. per fused track: set of
contributing source ids at last fuse) — no engine behavior change, no
fusion-math change, byte-identical fused output (prove: existing t2t suite
untouched-green before your test additions).

Then upgrade the scenario-6 check to gate the real invariant: with two
targets sharing an MMSI, the fused picture holds TWO tracks, each fed by
the correct arms (not one merged track, not a track fed by both targets'
reports). Teeth proof per the #24 standard: one mutation (e.g. force the
associator to key on MMSI) must trip it — observe, revert, record.

Guard: pedigree content still never asserted from live upstream
`contributing_sources` (the PmbmTracker:1666 bug stands); the new counts
are T2T-engine-side bookkeeping (which arms the FUSER used), not upstream
sensor pedigree — keep that distinction explicit in the field name and doc
comment.

## Part 2 — b24-1: a scan that actually exercises the PMBM k-best machinery

`KBestDominanceCutoffDropsSiblingsBelowGap` and
`AltBirthGateStripsBirthsInWeakAltOnly` (tests/pmbm/test_pmbm_phase8.cpp)
run on a scan that collapses to a single global hypothesis (n_off==1,
gated_alts==0) — the dominance-cutoff and alt-birth-strip mechanisms have
ZERO behavioral coverage today (your O1/O2 baseline observations). Design a
minimal deterministic scenario that provably sustains ≥2 competing global
hypotheses at the assertion scan (ambiguous two-track/two-measurement
geometry is the classic shape: two tracks, two measurements placed so both
assignment hypotheses survive gating with comparable likelihoods), then:

- assert the preconditions (`ASSERT_GE(n_off, 2)`, `ASSERT_GT(gated_alts,
  0)`) so the tests can never silently re-vacuate;
- assert the mechanisms' observable effects with #24-valid shapes (banded
  / structural, margins on adaptive comparisons);
- teeth proof: disable the cutoff (config) → observe the sibling survive →
  revert.

Tests-only for this part — if you find the mechanisms UNREACHABLE with any
legal config/scenario (a dead code path), that is a stop-and-report finding
for the backlog, not something to force.

## Acceptance

1. Part-1: additive BenchResult surface documented (integration guide
   config/appendix only if a Config struct changed — likely not), upgraded
   scenario-6 gate with teeth proof; t2t suite green.
2. Part-2: preconditions asserted, mechanisms behaviorally covered with
   teeth proof, or the unreachability stop-and-report.
3. Full suite green at 0 skips in your worktree; production diff limited
   to the Part-1 additive bench-result path.
4. Write-up `docs/baselines/2026-07-12_b24_gaps.md` (short — the two
   before/after tables + teeth records). Eval-log entry.
