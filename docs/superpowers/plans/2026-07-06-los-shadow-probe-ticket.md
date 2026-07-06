# Implementer prompt — LOS/shadow probe: does coverage-aware decay false-fire behind an occluder? (R8.8 payoff)

Status: ready to hand off. Paste everything below the line. Origin: the
2026-07-06 R8.8 occlusion labelling pass (eval-log entry + committed labels
`tests/fixtures/philos/labels/car_carrier_near_labels.csv`) produced measured
ground truth for the shadowing question the decay sector model has carried
since R8.8 was opened: moored yachts (`unknown_w860` row), present the ENTIRE
clip, radar-silent t 50–85 s while GENTLE LEADER crosses their bearing at
150–250 m. This ticket MEASURES what the occupancy/coverage-decay layer does
with that interval. It does NOT implement an LOS guard — that's an arbiter
decision made on this ticket's numbers.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`). Mission:
measurement-only probe of coverage-aware decay behavior under a real occlusion.
Main tree is fine if it's free; worktree if another lane is active.

## Ground truth you are testing against

`car_carrier_near_labels.csv`, row `unknown_w860` (42.3583, −71.0464, r=50 m):
present whole clip; radar returns cease t 50–85 s (occluder: the carrier,
crossing at 150–250 m on the same bearing); returns resume at the same cell.
Header caveats in that file apply (t relative to first ownship timestamp
1603390014.90; heading has a ~2° convention residual).

## The question, precisely

Coverage-aware decay decays occupancy cells that the sensor SWEPT but got no
return from ("observed empty"). A shadowed cell is swept in azimuth but
physically unreachable — the return stops at the occluder. If the sector
model treats shadowed cells as observed-empty, a real moored vessel's
occupancy evidence erodes during every close passage of a large ship —
degrading exactly the ADR-0002 presence channel the layer exists to provide.
The increment-8 verdict left this as "LOS guard or negligible?" — undecidable
then for lack of truth. Now there's truth.

## The probe

1. Wire `car_carrier_near` through the occupancy-enabled replay path
   (`PHILOS_CLIP` env from the farcross pass selects the clip; use
   `imm_cv_ct_pmbm_coverage_land` so the LiveOccupancyModel + coverage decay
   are ON; the scenario must carry a datum or the layer silently no-ops —
   the HAXR datum bug class, check it wires via occ diagnostics).
2. Instrument (additive, diagnostic-only — the model already exposes cell
   state; add a probe/query only if nothing existing serves): per scan,
   record for the `unknown_w860` cell(s): occupancy/persistence mass,
   whether the cell fell inside an estimated coverage sector (i.e. was
   "swept"), and any decay applied.
3. Report the three intervals separately: pre-shadow (5–50 s),
   shadow (50–85 s), post-shadow (85–120 s):
   - Was the cell inside the self-estimated coverage sector during the
     shadow? (If the sector estimator already excludes it — because the
     carrier's returns truncate the estimated swept range on that bearing —
     the answer may be "no false-fire by construction"; that would be the
     happy finding and the fromReturns largest-contiguous-cluster behavior
     might already deliver it. Measure, don't assume either way.)
   - How much occupancy mass did the cell lose during the shadow vs an
     un-shadowed control interval of equal length?
   - Did anything downstream change (hazard emitted/retired, veto state)?
4. Same probe on ONE sim scenario as a control: `sim_ms_anchored_camera`
   (anchored vessel, no occluder) — establishes the no-shadow baseline decay
   for comparison.
5. Deliverables: dated eval-log entry with the interval table + a one-
   paragraph verdict in three possible shapes: (a) "no false-fire — sector
   estimation already excludes shadowed cells, LOS guard unnecessary,
   evidence attached"; (b) "false-fire measured, magnitude X — LOS guard
   design sketch attached for the arbiter" (sketch = don't decay cells
   beyond a strong closer return on the same bearing; do NOT implement);
   (c) "layer doesn't fire at all on this clip — wiring gap, report it."
   Findings for the arbiter either way.

## Acceptance

1. Interval table (pre/shadow/post + sim control) committed with the
   eval-log entry; any added diagnostics are additive and default-inert.
2. No behavior change to the decay model itself — measurement only.
3. Full suite green if any code was touched; skip-guarded on fixture absence.
4. Budget ~1 day. Stop-and-report: if the occupancy layer won't wire on this
   clip within ~2 h (datum/config fights), report the shape of the problem —
   that's finding (c), not a failure.
