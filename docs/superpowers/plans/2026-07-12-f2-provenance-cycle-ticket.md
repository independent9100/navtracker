# Implementer prompt — F2 provenance cycle: re-price the SourceTouch fix as a deliberate tracking change (sequence WITH/AFTER wave 3)

Status: ready to hand off ONLY together with or after wave 3 (bias chain) —
see the sequencing constraint below. Paste everything below the line.
Origin: wave-1 F2 (PmbmTracker:1666 spurious SourceTouch) was implemented,
4-lens adversarially reviewed, and teeth-proven — then correctly HELD from
merge (commit `faaea83` on branch `fixwave-wave1`, preserved; do not delete
that branch until this cycle has cherry-picked it). Reason: provenance
(`recent_contributions`) is a tracking INPUT — it feeds the source-aware
miss-gate, idle-decay, and the AIS-ARPA→`applyBiasCorrection` loop — so
fixing the confirmed lie changes tracking on all 18 PMBM configs. Measured
at hold time: philos IMPROVES (the Cl-4 deployment regime), the seeded
autoferry/sim GOSPA battery net-REGRESSES, and the KEEP config carries an
untested real-target-continuity risk (a genuine AIS dropout now decays
existence via idle_halflife where the bug held it flat — #25-adjacent).
Acceptance criterion "byte-identical" was DISCOVERED UNACHIEVABLE, not
waived (`docs/baselines/2026-07-12_fixwave_wave1.md` has the input-path
evidence). Budget ~2 days. North-star tag: Cl-3/Cl-4 integrity.

**Sequencing constraint (binding):** run WITH or AFTER wave 3. The
autoferry regression may be two wrongs cancelling: the bug fed GARBAGE
bias observations into a bias chain that is ITSELF broken (Theme 3:
closed-loop double-subtraction converges to half-bias). Measuring the F2
fix against the broken chain answers the wrong question.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-f2c
-b f2-provenance-cycle`, own build dir, fixtures inner-level symlinked,
0-skip runs; commit on your branch, never merge/push master). Start by
cherry-picking `faaea83` (the reviewed fix + its regression test).

## Q(a) — is the autoferry/sim regression real, or was the garbage helping?

Isolate the three input paths the fix changes (miss-gate, idle-decay, bias
loop) and attribute the regression: (1) A/B with the AIS-ARPA bias-feed
disabled on both arms — if the regression vanishes, it lived in the bias
loop; (2) repeat on top of the wave-3 bias-chain fix — if the regression
inverts or vanishes there, it was garbage×broken-chain cancellation and
the fix is clean on correct machinery. Deliverable: a per-path attribution
table, per workload.

## Q(b) — KEEP-config continuity through a genuine AIS dropout

Build the continuity test FIRST (the `sim_ms_ais_dropout` scenario exists):
with the fix ON vs OFF, measure track survival, existence trajectory, and
re-acquire behavior through the dropout window on the KEEP config. Then
judge: decay-during-dropout may be the DESIGNED behavior the bug was
masking — if survival stays acceptable (no ADR-0002 presence violation:
the track must not vanish into nothing while the target is detectable by
the other sensor), baseline it; if decay is too aggressive, that is an
`idle_halflife` re-pricing question (config), NOT a reason to revert the
correctness fix. Frame the trade; do not decide it.

## Q(c) — record the philos win

Quantify the philos improvement under the fix (the Cl-4-relevant upside)
alongside (a)/(b) so the decision reads all three numbers together.

## Then — and only then — the disposition

Hand the three answers back with a frame-the-trade summary (name each
failure mode and when it hurts). NO baseline re-pinning before the
checkpoint. If the verdict is ship: baselines re-pin in the same PR, the
continuity test lands as the permanent guard, and the T2T live-pedigree
caveat (§10 Rider B / the T2T docs) gets LIFTED in the same change —
that's the downstream unlock this cycle buys.

## Acceptance

1. Q(a) attribution table; Q(b) continuity test + measurement; Q(c)
   philos number. Checkpoint before any re-pinning.
2. Full suite green at 0 skips; adversarial re-review of the final merged
   form if it deviates from `faaea83`.
3. Write-up `docs/baselines/<date>_f2_provenance_cycle.md` + eval-log
   entry; update the wave-1 write-up's HELD note with the outcome.
4. Commit on your branch; do not merge or push master.
