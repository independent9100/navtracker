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
7. **D7 MOANA feasibility** — the only in-hand path to (a) real radar
   with NON-AIS truth and (b) a **Doppler-capable radar** (W-band) to
   prototype the increment-8 Doppler direction before hardware answers
   arrive. ~half day go/no-go.
8. **D8 R-BAD feasibility** — 69 h berthing radar + synced video
   (hour-scale steady state on a 3rd geography + label passes). ~half day.
9. **R8.8 car_carrier_near re-extraction + occlusion video pass** — CODE
   HALF DONE 2026-07-06 (3855efd): extractor fallbacks (densest NavSatFix +
   quaternion-yaw heading, convention verified to 0.2-2.2 deg) + fail-loud
   guard (fails on old broken output, passes all 7 clips, check_ownship.py
   --all); re-extracted 8739 rows, rotation undone (shore-median 121->50 m);
   R4 re-check: coverage IMPROVES (UNION@50m 53.6->59.5%), the
   (42.3583,-71.0464) UNKNOWN SURVIVES with robust support (16 returns/25 m,
   87% of clip) — now the top question for the video pass. REMAINING: the
   occlusion labelling session (user + analyst; clip ready, closest approach
   t_rel 110-120 s, no AIS on 2020 bags).
10. **ais_ferry_far + almost_cross measurement pass** — DONE 2026-07-06
    (09edb64): PMBM ~46-54 m radar-only vs AIS truth (19 s spot check,
    stated weight); AIS fusion transforms mechanics (MHT lifetime 0->0.42);
    almost_cross ADR-0002 persistence canary PASS. Side finding: the
    "independent" philos_radartruth was AIS-in-radar-frame all along —
    relabeled everywhere, independence audit CLEAN (nothing shipped was
    load-bearing on it). See docs/baselines/2026-07-06_philos_farcross.md.

## Tier 3 — bigger quality fronts (pick AT MOST ONE, design-first)

11. **Clutter/birth-model investigation** (increment-8 redirect): the
    diffuse over-count. Full gate suite mandatory (λ_C history: KEEP
    over-deletion + dense_clutter spiral). ~1 week.
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
