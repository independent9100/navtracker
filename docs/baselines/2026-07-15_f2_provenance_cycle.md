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

## Disposition — SHIP (arbiter verdict, 2026-07-15)

Verdict: **ship, re-pin (shrunken), lift the T2T caveat with its own pin.** What
was done on this branch:

### The correctness-driven shift (the re-pin record)
**The pre-fix numbers were measured on lying provenance** — the source-touch walk
credited misdetected Bernoullis measurements they never claimed, and that
polluted the miss-gate/idle/bias paths. The fix removes the lie, so the metric
changes are corrections, not regressions in the usual sense.

- **Named path-(a) line (accepted, confined):** on the source-aware-gate
  diagnostic configs (`makePmbmConfig` family — `imm_cv_ct_pmbm`, `_adapt`,
  `_adapt_k3`, `_land`, `_bundle`, `_static`, `_occupancy*`, `_birthtarget`,
  `_cmap`, …) the fix raises autoferry/anchored GOSPA via an overcount/phantom-
  persistence effect (path a, `source_aware_misdetection`), fully attributed and
  accepted. **Warning for the future:** if any of these diagnostic configs is
  ever considered for deployment, this overcount is the line to re-read first —
  the deployed config avoids it precisely by keeping the source-aware gate off.
- **Harbor is byte-identical on EVERY source-aware config** (measured, 0.000
  delta on gospa/false/card/lifetime/id over 5 seeds) — harbor_complete_truth is
  pure-radar with no AIS cross-source provenance, so the three paths are inert,
  exactly like the pure-sim scenarios.
- **Re-pin surface (shrunken, per arbiter):** nothing deployed, frozen, or
  test-enforced moves. Cl-4/KEEP headline (env-1 15.49 / env-2 8/8 13.75 /
  harbor 9.53) is byte-identical (KEEP config). Every numeric-metric test
  (incl. the hard-banded `test_adapt_k3_harbor_knife_guard`) stays green — no
  re-banding. **No baseline CSV was edited** (dated snapshots are immutable
  history by convention); the record of what shifted is this doc + the eval-log
  entry. What moved is confined to non-deployed source-aware configs on
  autoferry (±anchored) / simms / philos.

### T2T live-pedigree caveat — LIFTED with a corrected rationale + its own pin
- New E2E pin `tests/integration/test_t2t_live_pedigree_content.cpp`: a live
  two-sensor pipeline (radar+AIS → one Tracker → `NavtrackerSource` → fuser)
  proves live pedigree CONTENT is truthful (radar-only track → radar Used, AIS
  absent; both-fed → both Used). Teeth-proven (feed the radar-only track AIS →
  pedigree lists AIS → RED).
- **Correction (propagate to the review reconciliation):** §10 Rider B and the
  review finding it quoted said `contributing_sources` carried spurious entries
  from PmbmTracker:1666. That **conflated two channels.** The F2 bug polluted
  `recent_contributions` (SourceTouch), NOT `contributing_sources` (the field the
  T2T self-adapter reads). PMBM never writes `contributing_sources`; the flat/MHT
  path fills it genuinely per-update. So the T2T pedigree was never corrupted by
  the F2 bug — the lift rests on the E2E pin, not on "F2 fixed it." The
  findings-file marks should record that F2 fixed the provenance side-channel
  (`recent_contributions`), which the emitted `contributing_sources` string list
  is a separate matter.
- **Diagnostics-only reaffirmed:** pedigree selects the independence verdict for
  operator value and never enters the CI weights (v1 design). "Caveat lifted" =
  live pedigree content is now assertable, NOT that pedigree may steer fusion.

### faaea83 deviation statement (acceptance #2 / Rider 3)
The merged fix (`fb41217`) is **byte-identical to `faaea83`**: the 44 added/
removed lines of the `PmbmTracker.cpp` source-touch change and the CMakeLists
test registration are identical; the cherry-pick auto-merged with no manual
resolution and is disjoint from wave-2's W2.4b (identity-keyed retirement). **No
deviation → no adversarial re-review triggered; the prior 4-lens review carries.**

### Follow-ups (NOT on this branch — see design spec §14.11)
1. PMBM leaves `TrackOutput.contributing_sources` empty while flat/MHT populate
   it — a consumer-surface inconsistency, harmless-but-uninformative for T2T.
   Post-F2 the truthful `recent_contributions` could feed it. Design decision
   (empty=honest / populated=useful) captured in §14.11; not built.
2. The Rider-B channel-conflation correction (above) propagates to the review
   reconciliation / findings-file marks.

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
- Continuity guard `test_cl4_ais_dropout_continuity.cpp`: GREEN, teeth-proven.
- T2T live-pedigree pin `test_t2t_live_pedigree_content.cpp`: GREEN, teeth-proven.
- Post-verdict (2026-07-15): T2T caveat LIFTED (corrected rationale + E2E pin);
  re-pin is document-only (no baseline CSV edited, no test re-banded — nothing
  deployed/frozen/enforced moves); two follow-ups filed to §14.11 / review recon.
