# Pre-water window — selected work while the real test waits (2026-07-05)

NOT a new backlog. This SELECTS from the existing queues (improvement-backlog,
data-expansion D1–D12, review-fix R-items, parked north-star rows) for the
window between "research arc closed" and "deployment facts arrive / water
test". Each item cites its home ticket; status lives there. Drop this doc
when the water date firms.

## Tier 1 — directly improves the water test (do first)

1. **R11 identity surfacing** — RUNNING (implementer). Ticket §R11.
2. **Target-reported kinematics** — backlog **#20** (new): AIS SOG/COG/
   heading/**nav-status** (veto data path!) + TTM one-shot birth seed +
   swap diagnostic. ~1–2 days. The nav-status half upgrades the
   corroboration stack before the water test.
3. **Bearing-wedge hazard** — backlog #17 option 1 (camera-only contact
   safety net; "never invisible" without a position). Buildable NOW
   (no deployment facts needed). ~1–2 days incl. output-contract.
4. **Nav-input guard, fact-free half** — backlog #18: staleness signal +
   jump detection (SOG-gated heading source waits for facts). ~1 day.
5. **Per-pose heading σ** — backlog #16. ~half day; guide §5 updates.

## Tier 2 — measurement integrity + unused data (cheap, parallel)

6. **D2 Stone Soup GOSPA cross-validation** — DONE 2026-07-06 (b5b3ea5):
   kernel == Stone Soup to 1.42e-14 on 1 sim + 1 real arm, conventions
   matched (c=20/p=2/α=2); both prior metric incidents confirmed as
   truth-grouping faults UPSTREAM of the now-validated kernel. Exporter +
   re-runnable tool shipped (`tools/stonesoup_gospa_crosscheck.py`).
   See `docs/algorithms/gospa-crosscheck.md`.
7. **D7 MOANA feasibility** — NO-GO 2026-07-06 (b711258): CC-BY-NC-SA
   (commercial gate FAILS); no per-detection Doppler in the published data
   (corrects this item's own premise — the Doppler prototype path is dead
   until deployment hardware facts); raw PNG imagery would need the parked
   front-end extraction anyway. Non-AIS truth exists on only 1 of 7
   sequences and is legally unusable for us.
8. **D8 R-BAD feasibility** — GO 2026-07-06 (30323ac, merged e807cb6):
   CC-BY-4.0 (commercial OK), radar-detection CSVs + synced video, 31.6 GB,
   non-AIS labels + video for independent label passes. REGIME CAVEAT:
   automotive mmWave FMCW (60-81 GHz), NOT marine X-band — corroborates the
   berthing scene on a new sensor class, not a 3rd marine geography. Next
   step when picked up: extract 1-2 station-hours + label-scored replay.
9. **R8.8 car_carrier_near re-extraction + occlusion video pass** — CODE
   HALF DONE 2026-07-06 (3855efd): extractor fallbacks (densest NavSatFix +
   quaternion-yaw heading, convention verified to 0.2-2.2 deg) + fail-loud
   guard (fails on old broken output, passes all 7 clips, check_ownship.py
   --all); re-extracted 8739 rows, rotation undone (shore-median 121->50 m);
   R4 re-check: coverage IMPROVES (UNION@50m 53.6->59.5%), the
   (42.3583,-71.0464) UNKNOWN SURVIVES with robust support (16 returns/25 m,
   87% of clip) — now the top question for the video pass. LABELLING PASS DONE
   2026-07-06: R8.8 FULLY CLOSED — shadow interval measured (yachts radar-
   silent 50-85 s behind GENTLE LEADER, present whole clip), R4 UNKNOWN
   resolved = moored motor yachts, labels committed
   (car_carrier_near_labels.csv), LOS-guard test now buildable against truth.
10. **ais_ferry_far + almost_cross measurement pass** — DONE 2026-07-06
    (09edb64): PMBM ~46-54 m radar-only vs AIS truth (19 s spot check,
    stated weight); AIS fusion transforms mechanics (MHT lifetime 0->0.42);
    almost_cross ADR-0002 persistence canary PASS. Side finding: the
    "independent" philos_radartruth was AIS-in-radar-frame all along —
    relabeled everywhere, independence audit CLEAN (nothing shipped was
    load-bearing on it). See docs/baselines/2026-07-06_philos_farcross.md.

## Tier 3 — bigger quality fronts (pick AT MOST ONE, design-first)

11. **Clutter/birth-model investigation** — CLOSED 2026-07-07 (c7d2b6a):
    measured negative at the birth channel, zero implementation waste
    (binding §5.0 probe: burst phantoms confirm in ONE scan; perfect birth
    fix floors card_err ~2.69 > MHT +2.51; neither candidate built).
    Durable: the lambda_C-cancellation INVARIANT (pmbm-design §3.2.2),
    contract-boundary finding (the burst is upstream-extraction input;
    tracker-level residual is small), deep pivots parked as backlog #23
    with post-water triggers. Redirect closed BOTH ways (suppression =
    increment 8; birth = this campaign).
12. **Cl-1 cold-start env-1 gap** (bearing-heavy open water, no anchor) —
    the one open headline-claim gap. ~1 week.
13. **D10 GFW anchorages (+ what's left of D3)** — anchored-vessel stats
    + KEEP-side corroboration prior. ~1–2 days, registration first.

## Stays parked (unchanged reasons)

Stage 2 evidential grid + stationary IMM; TBD channel; D4/D5/D11/D12;
backlog #19 (needs camera tracking pass + id-survival measurement);
RTS promotion (anchored-mode structural fix); MHT static-mechanism port.

## Suggested consumption

While R11 runs: #2 + #6 in parallel → D7/D8 feasibility (#7/#8) →
#3/#4/#5 → video-pass items (#9/#10, needs user) → THEN choose the one
Tier-3 front, informed by whatever D7 said about Doppler and whatever
the deployment facts said about the radar.
