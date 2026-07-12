# Implementer prompt — pre-release fix wave, wave 1: the three synthesis-independent CONFIRMED findings (GGA critical + spurious SourceTouch + TrackOutput axis contract)

Status: ready to hand off. Paste everything below the line. Origin: the
2026-07-09 pre-release deep review (`docs/reviews/2026-07-09-prerelease-review/`).
The full fix wave is triaged once the review's open-points synthesis lands;
these three do NOT need to wait — each is CONFIRMED by an adversarial
verifier, is synthesis-independent, and two of them block other work (T2T
pedigree trust; consumer axis-convention safety). The review is pinned at
master 317ecfd — your fixes land after that pin, so there is no interference
with the still-running review session. Budget ~1.5–2 days. TDD throughout
(failing test first — each finding IS a failing-test spec).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-fw1
-b fixwave-wave1`, own build dir, fixtures inner-level symlinked, 0-skip
runs; commit on your branch, never merge/push master). Read each finding's
full text + verifier evidence in
`docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md` before
fixing — the verifier sections contain the reproduction recipes.

## F1 — CRITICAL: OwnShipNmeaAdapter accepts no-fix GGA → (0,0) pose/datum

A GGA sentence with fix-quality 0 (or empty lat/lon fields) currently
produces a pose at (0,0) — which can INITIALIZE THE DATUM at null island
and silently poison every subsequent ENU conversion. This violates
architecture invariant 6 (adapters validate at the edges). Fix: validate
fix quality and field presence/plausibility in the adapter; an invalid
sentence produces NO pose update (and a parse-reject the adapter's existing
error/diagnostic path can surface). Tests: no-fix GGA, empty-field GGA,
fix-quality transitions (fix → no-fix → fix must not move the datum), and
the first-fix-initializes-datum happy path unchanged. Consumer surface:
document the rejection behavior in the integration guide's NMEA section
(one paragraph).

## F2 — HIGH: PmbmTracker:1666 spurious SourceTouch — contributing_sources lies

`contributing_sources` can list sensors that did not actually contribute to
a track (the review's finding at PmbmTracker.cpp:1666; cross-confirmed by
the B6 lens pass). Fix the touch to fire only on genuine contribution, per
the finding's mechanism. Expect OUTPUT-attribute changes only: kinematics/
existence/lifecycle must be byte-identical (prove with the R3 two-class A/B
method — metrics identical, attributes may differ), and add a regression
test that a sensor which never updates a track never appears in its
contributing_sources. Note downstream: this unblocks trusting T2T live
pedigree content (the §10 Rider-B caveat) — say so in the write-up, but do
NOT change T2T tests here (their handcrafted-fixture discipline stays).

## F3 — HIGH: TrackOutput covariance axis contract — header says NED, code emits (east, north)

Empirically re-confirmed twice (review B4 + the T2T Rider-A probe): the
emitted position-covariance ordering is ENU (east, north); the header and
`docs/output-contract.md` claim NED. **Resolution direction: align the
DOCUMENTATION AND NAMES to the measured ENU behavior — zero behavior
change.** Rationale (record it in the write-up): every live consumer
(Foxglove adapter after its fix, the T2T self-adapter with its pinned
swap-test) has verified against the ACTUAL ordering; flipping the code
would silently break all of them, the exact failure mode this project
does not ship. Work: correct the header comment + `docs/output-contract.md`
(worked example included) + the integration guide §6 note; keep/extend the
T2T swap-detecting test as the permanent contract pin (it should now be
asserting the DOCUMENTED convention, not a workaround); grep for other
places that repeat the NED claim (learning docs, example.cpp comments) and
fix them in the same pass. Flag in the handoff, addressed to the user: any
consumer code written against the old DOC (NED interpretation) is silently
transposing covariance axes today — the user should check their middleware's
reading of `TrackOutput` covariance.

## Acceptance

1. TDD paper trail per finding (failing test → fix → green).
2. F2's byte-identical-metrics A/B proof; F1's datum-safety tests; F3 with
   zero behavior change and the contract pinned by test.
3. Docs riding the same branch: integration guide (F1 rejection behavior,
   F3 §6 correction), output-contract.md (F3), header comments. The
   config-coverage drift-guard stays green.
4. Full suite green at 0 skips in your worktree. Adversarial review pass
   on the F2 change (PMBM hot path) before handoff — same standard as
   always.
5. Write-up `docs/baselines/2026-07-12_fixwave_wave1.md` + eval-log entry;
   mark the three findings FIXED in a dated note in the review's findings
   file (do not rewrite the original finding text).
6. Commit on your branch; do not merge or push master.

## AMENDMENT 2026-07-12 (arbiter) — F3 direction is now a USER DECISION; hold F3, do F1+F2 first

The open-points synthesis (§B Theme 4) surfaced evidence that flips the
F3 analysis: the NED assumption is not just in the header — it is in
`output-contract.md`, `example.cpp`, the integration guide, AND the
Foxglove adapter, which is rendering every anisotropic error ellipse 90°
rotated TODAY. So "align docs to code (ENU)" means fixing five
consumer-facing sites; "align code to docs (NED)" means one code change
that makes all five right at once (and flips the T2T swap-test, which was
designed for exactly this). The synthesis recommends code→NED. The
arbiter's earlier docs→ENU rationale ("every live consumer verified against
actual ordering") is WRONG for Foxglove — it assumed NED and is bitten.

**Do NOT implement F3 until the user decides the contract direction** (they
must also check which convention their own middleware reads). Implement F1
and F2 now; F3 becomes a one-liner + five doc/test touchups (NED) or a
five-site doc fix (ENU) once decided. Whichever direction: the swap-test
remains the permanent pin of the DECIDED convention.
