# Implementer prompt — §14.11: populate PMBM's contributing_sources from the truthful claimed-source channel

Status: ready to hand off. Paste everything below the line. Origin: the F2
cycle's Finding A (2026-07-15) — the DEPLOYABLE tracker (PMBM) leaves
`TrackOutput.contributing_sources` EMPTY while the flat/MHT path populates
it genuinely per-update. Nobody noticed because the field was distrusted
until F2; now that the claimed-source channel is truthful (keyed on
`last_claimed_meas_index`, teeth-proven, E2E pedigree pin in place), the
empty field is the odd one out: consumers of the deployable config get no
source attribution, and a PMBM-backed T2T source yields all-Unknown
pedigree where genuine content is available. Filed as design spec §14.11
("empty = honest vs populated = useful"). Budget ~1–1.5 days.

**Arbiter design ruling (user-relayed): POPULATE.** Rationale: (a) the
channel is now truthful and guarded by tests, so "empty = honest" no longer
buys anything; (b) output consistency across trackers is consumer surface —
the guide documents the field without a per-tracker asterisk; (c) T2T live
pedigree from a PMBM-backed source becomes genuine instead of all-Unknown,
completing the story the Rider-B lift started. The E2E pedigree-content pin
(`test_t2t_live_pedigree_content.cpp`) becomes the cross-layer guard.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-csrc -b pmbm-contributing-sources`, own
build dir, fixtures inner-level symlinked; suite under
`NAVTRACKER_REQUIRE_FIXTURES=1`. Commit on your branch; never merge/push
master.)

## Design note first (short, four-part per house standard)

Where the attribution lives (the truthful `recent_contributions`/claimed-
source records on the Bernoulli), how it maps to the output field
(set of source ids that GENUINELY updated the carried Bernoulli — define
the retention window and say why; the flat/MHT path's semantics are the
reference, match them unless PMBM's hypothesis structure forces a
documented difference), what happens on hypothesis switch (the aggregated
track adopts the carrying Bernoulli's history — same last-write-wins
convention as the R11 platform_id adoption), and ways-to-improve.

## Build constraints

1. **Attributes only, byte-identical metrics:** contributing_sources is an
   output attribute; kinematics/existence/lifecycle must be untouched —
   prove with the R3 two-class A/B (metrics identical, attributes may
   differ) across the bench.
2. The field's content must satisfy the SAME invariant F2's regression
   test pins: a sensor that never genuinely updated the track never
   appears. Extend that test (or add a sibling) to the PMBM output path.
3. **T2T composition:** with a PMBM-backed `NavtrackerSource`, live
   pedigree now carries genuine Used entries — extend the E2E pedigree
   pin: PMBM two-sensor track → pedigree shows both Used; radar-only →
   AIS absent/Unknown. Teeth per the #24 standard.
4. Consumer surface: `docs/output-contract.md` (the field's per-tracker
   note updated), integration guide (§ output fields + the T2T §3.10
   pedigree note that currently says PMBM sources read all-Unknown),
   design spec §14.11 marked resolved. Drift-guard stays green.
5. Determinism: iteration over source sets must be ordered (no
   unordered-map order leaking into output).

## Acceptance

1. Design note; the attributes-only A/B proof; the extended invariant +
   E2E pedigree tests teeth-proven; docs set complete.
2. Full suite green under strict mode; adversarial review before handoff
   (PMBM output path).
3. Write-up `docs/baselines/2026-07-15_pmbm_contributing_sources.md` +
   eval-log entry.
4. Commit on your branch; do not merge or push master.
