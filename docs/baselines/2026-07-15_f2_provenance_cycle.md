# F2 provenance cycle — measured disposition (2026-07-15)

Branch `f2-provenance-cycle` off master `068a30f` (wave-3 bias-chain + wave-4
merged). Ticket: `docs/superpowers/plans/2026-07-12-f2-provenance-cycle-ticket.md`
(GATE SATISFIED). Origin: F2 held from fix-wave wave 1 (`faaea83`) because the
source-touch fix is a tracking INPUT, not attribute-only. This cycle answers the
three measurement-first questions on the CORRECTED (wave-3) bias chain and frames
the trade for the arbiter. **No baseline re-pinning done — checkpoint pending.**

## What shipped on the branch
1. `fb41217` — cherry-pick of `faaea83` (the reviewed F2 fix + its regression
   test). Auto-merged clean onto master; verified SEMANTICALLY correct against
   wave-2's W2.4b identity-keyed retirement: the fix's `scan[claimed]` reads the
   same `scan` vector that `enumerateChildren` indexes when it sets
   `last_claimed_meas_index` (set-sites 965/1027/1176; read at 1693). Disjoint
   from W2.4b.
2. `aff08a3` — bench path-isolation flags `--no-bias-feed` / `--force-no-idle` /
   `--force-no-source-aware` (research tooling; byte-identical unless passed).
3. `2e2b635` — the Q(b) permanent continuity guard
   `tests/benchmark/test_cl4_ais_dropout_continuity.cpp`.

## A/B method
ON = `f2-provenance-cycle` (F2 fix), OFF = master `068a30f`. The fix is an
UNCONDITIONAL code change (no runtime toggle), so ON = this build, OFF = master
build. Both arms share the wave-3-corrected bias chain, so the wave-3 EO/IR seed
mis-scaling (parked, ADR-0003 addendum) is COMMON-MODE and cancels in ON−OFF —
the clean read the sequencing constraint was designed to give. Δ = ON−OFF, seeds=3.

## Q(a) — is the autoferry/sim regression real, or was the garbage helping?

**Answer: REAL and path-(a)-driven; NOT garbage×broken-chain cancellation.**

Attribution ladder (Σ ON−OFF gospa_mean per bucket; L0 all paths, L1 −biasfeed(c),
L2 −biasfeed−idle(b), L3 all off):

| config | bucket | L0 | L1 | L2 | L3 |
|---|---|---|---|---|---|
| `_adapt` (a✓b✓c✓) | autoferry | +40.07 | +40.07 | +53.06 | 0.000 |
| | autoferry_anchored | +55.02 | +59.72 | +116.80 | 0.000 |
| | simms | −2.52 | −2.52 | −2.80 | 0.000 |
| `_pmbm` (a✓b✓c✓) | autoferry | +43.52 | +43.52 | +62.90 | 0.000 |
| | autoferry_anchored | +106.05 | +104.70 | +155.56 | 0.000 |
| KEEP `_coverage_land_ivgate` (only c) | autoferry | 0.000 | 0.000 | 0.000 | 0.000 |
| | autoferry_anchored | **−15.37** | 0.000 | 0.000 | 0.000 |
| | simms/philos/sim | 0.000 | — | — | 0.000 |

- **L3 = 0.000 everywhere** → the fix is byte-identical with all three consumers
  disabled: it has NO reach beyond (a) source-aware gate, (b) idle-decay, (c) bias
  loop. Attribution method validated. `sim_pure` = 0 at every level.
- On the source-aware-gate family, removing the bias loop (L1) does NOT remove the
  autoferry regression → **it does not live in the bias loop**. Removing idle (L2)
  GROWS the delta → idle-decay was MASKING part of it. Path (a)
  source_aware_misdetection is the driver.
- The regression PERSISTS on the corrected chain → the "garbage bias × broken
  chain, two wrongs cancelling" hypothesis is REFUTED for the autoferry regression.
- **Character = overcount / phantom persistence** (L0, source-aware family):
  gospa_false EXPLODES (+1644/+1327 adapt; +2148/+2904 plain), gospa_missed DROPS
  (−173/−331), card_err +9…+14, lifetime_ratio +0.78, id_switches +57. Cleaner
  provenance → `should_misdetect` says "covered" LESS → less miss-decay → longer-
  lived phantom tracks on dense radar. Exactly the wave-1 mechanism.
- **The regression is CONFINED to source-aware-gate configs.** The DEPLOYED KEEP
  config (`source_aware=off`, `idle=0`) carries NONE of it: byte-identical on
  autoferry/simms/philos/sim, and IMPROVES anchored diagnostic by −15.37 (via the
  bias loop, path c; L1=0.000 confirms c is KEEP's only live channel). The big win
  is `scenario16_anchored` 8.42→1.52 — the row wave-3's mis-scaled EO/IR seed had
  DEGRADED; cleaner provenance → better bias pairs partly recovers it.

## Q(b) — KEEP-config continuity through a genuine AIS dropout

**Answer: survival is excellent and the fix is byte-identical; no trade on the
deployed config. The idle_halflife concern is doubly moot.**

`sim_ms_ais_dropout` (vessel 257000401 loses AIS t=200..380 s, radar-detectable
throughout) on the KEEP config:
- **Fix ON vs OFF byte-identical** — PMBM per-scan diag stream md5-identical; all
  40 non-timing metric rows identical.
- **Survival:** lifetime_ratio 0.993 (dropout) / 0.992 (steady); id_switches 0
  (identity retained across dropout + re-acquire, R11); ≥2 tracks in output every
  scan through the window, held at output floor r=0.1. ADR-0002 presence SATISFIED.
- **The ticket's idle_halflife-decay risk is doubly moot:** idle=0 on the deployed
  config, AND (teeth-proven) even idle_halflife_sec=3 leaves lifetime at 0.993 —
  idle fires on TOTAL misdetection, not partial sensor loss; radar keeps the track
  non-idle. The lever that WOULD break presence is source_aware_misdetection
  (teeth: forcing it true → RED), which is precisely why KEEP keeps it off.
- Permanent guard `test_cl4_ais_dropout_continuity.cpp` lands to defend this.

## Q(c) — philos improvement under the fix

L0 ON−OFF gospa_mean (− = improves): `_adapt` philos −0.57 / radartruth −1.68;
`_pmbm` −1.03 / −1.94. Reproduced on the corrected chain — the source-aware gate
working as designed (one vessel's touch no longer marks another "covered").

**Caveat (honest):** the philos win accrues to the source-aware-gate family, NOT
to the DEPLOYED KEEP config, where philos is BYTE-IDENTICAL under the fix. So the
deployment-relevant value of the fix is not the philos number; it is (1)
correctness (removes a confirmed provenance lie), (2) byte-identical on the
deployed config's real workloads, (3) improves the deployed anchored diagnostic
(−15.37), (4) unblocks the T2T live-pedigree caveat.

## Frame-the-trade (for the arbiter — decision, not a verdict)

| failure mode | when it hurts | who carries it |
|---|---|---|
| autoferry/anchored overcount (phantom persistence) | dense-radar, source-aware gate ON | `_adapt`/`_pmbm`/`_land`/`_bundle`… — NON-deployed diagnostic configs |
| philos-neutral on deployed config | sparse-AIS deployment (Cl-4) | KEEP config gets correctness, not the philos number |
| — (no continuity regression) | AIS dropout on deployed config | none: byte-identical, survival 0.993 |

**The deployed config (`coverage_land_ivgate`) sees only upside**: byte-identical
on real workloads, +improvement on anchored diagnostic, and the confirmed
provenance lie removed. The regression is real but lives entirely on configs that
are not deployed (source-aware gate on). The philos upside likewise lives there,
not on the deployed config.

## Recommendation (pending arbiter verdict)
SHIP the correctness fix. Rationale: on the deployed surface it is byte-identical
+ improving + honest; the regression is confined to non-deployed source-aware
configs and is already understood (path a, overcount). If SHIP:
- re-pin the source-aware-family baselines that legitimately move (document as a
  correctness-driven shift, per the freeze-commit rule);
- land the continuity guard (done, `2e2b635`);
- lift the T2T live-pedigree caveat (§10 Rider B / T2T docs) — the provenance the
  T2T pedigree reads is now truthful.
Alternative if the source-aware-family regression is unacceptable for those
diagnostic configs: keep the fix but leave source_aware_misdetection off on any
config where the overcount is not tolerable (it already is off on the deployed one).

## Verification
- Cherry-pick semantic check: PASS (see above).
- F2 regression test `PmbmContributionProvenance.MisdetectedTrackDoesNotInheritForeignSource`: GREEN.
- Continuity guard: GREEN, teeth-proven (source_aware=true → RED).
- Full suite (strict `NAVTRACKER_REQUIRE_FIXTURES=1`): **1206/1207 passed, 0
  skips** (1207 = prior 1206 + this cycle's continuity guard, which passed). The
  sole failure is the pre-existing wave-2 `VetoIsolationHaxrAB.VetoIsolatedOnAisArm
  ThreeSites` 300 s TIMEOUT knife-edge — it starves under both `-j8` and `-j6`
  but PASSES CLEAN STANDALONE at 265 s (verified this cycle via the gtest binary,
  no ctest timeout). Inherited infra flake, flagged for wave-2; the F2 fix does
  not touch the haxr/veto path.
- NO baseline re-pinning; NO T2T caveat lift — awaiting checkpoint verdict.
