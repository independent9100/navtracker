# Cl-4 Phase 2 Stage 0 — vessel-past-structure gate: STOP-AND-REPORT (G1 FAILS)

**Date:** 2026-07-11 · **Branch:** `cl4-phase2-floor-veto` · **North-star:** Cl-4 Phase 2 (Stage 0 gate).
**Measurement only — zero tracker code** (census extension `tools/cl4_a3_census.cpp --mode stage0`).
Ticket: `docs/superpowers/plans/2026-07-11-cl4-phase2-floor-veto-build-ticket.md`.

## Verdict (one line)

**G1 FAILS, G2 passes → STOP-AND-REPORT. The build does not proceed to Stage 1.** The
occupancy veto is a **spatial-region** veto with no motion discrimination: a vessel
transiting parallel to and near the pier has its floor-satisfying endpoint inside the
flagged region for the whole pass, so it is vetoed for the duration — added latency ≫ 15 s
(4/5 seeds *never* revive within the run; 1/5 at 20 s), at every distance inside the 25 m
ramp and at every speed ≤ 3 m/s. This is exactly the gap the Phase-1d BUILD-ELIGIBLE
verdict was contingent on, now measured and failed. Per the ticket, the endgame decision
(per-geography residual vs re-price) is the arbiter's and the user's — **not** a reason to
tune the veto until it passes.

## Setup

A synthetic vessel transiting **parallel to the 120 m pier** (`x ∈ [−60, 60]`, `y = −350`),
at offset `yoff` (distance from the pier line), speed 2–4 m/s, one radar return per scan
(P_D 1, ~the pier's own density) with 4 m Gaussian noise (deterministic per seed). Fed
scan-by-scan into the **real** harbor occupancy stream (pier + boats + sea clutter) plus the
vessel return, through the shipped `LiveOccupancyModel` (default params). Floor = M=8 in
30 s AND net displacement ≥ 50 m (the Phase-1d corner). Two veto neighborhoods:
**bare-cell** (`suppression_radius_m ≈ 1`, veto only on a structure cell) and **ramp**
(`= 25`, the Phase-1d default). Baseline = the **same transit with no pier** (occupancy
never flags → veto inert → revives at floor time, confirmed: rel latency 0).

Grid note: with 25 m cells and a 25 m ramp, "inside the ramp" ≈ the pier's own cell row or
the one adjacent — the bare-cell/ramp distinction only appears in the adjacent band.

## G1 — added revival latency (parallel transit alongside the pier), seed 0

| speed | dist from pier | bare-cell (r=1) | ramp (r=25) |
|---|---|---|---|
| 2 m/s | 22 / 32 / 50 m | never / never / never | never / never / never |
| 3 m/s | 22 / 32 / 50 m | never / **20 s** / **20 s** | never / never / **20 s** |
| 4 m/s | 22 / 32 / 50 m | **21 s** / **21 s** / **10 s** | never / never / never |

- **Every entry inside the 25 m ramp** (≤ ~30 m) is `never` or ≫ 15 s.
- The only entry meeting ≤ 15 s is **4 m/s at 50 m** (i.e. **outside** the ramp — the
  vessel is clear of the structure) under the bare-cell veto. It does not represent
  vessel-*inside*-ramp.
- 2 m/s never revives at any distance (vetoed the whole 40-scan pass while alongside).

Across 5 seeds at the representative 3 m/s / 30 m point (bare-cell): revival latency =
{20 s, never, never, never, never} — **G1 fails on 5/5** (the margin box only tightens it).
Sub-floor note: a 1 m/s transit moves 30 m in 30 s < D=50 m, so it never satisfies the floor
at all — correctly left to the static-hazard channel (the standing scope note), not a G1 case.

## G2 — pier stays vetoed with the vessel present: PASS

Pier-line structure cells flagged with the vessel injected = **12–13 on all 5 seeds** (vs
the vessel-absent baseline) — the vessel's returns do not un-flag or fragment the pier's
connected component (occupancy is monotone in returns: adding the vessel can only add
persistence). The pier-walk chains remain fully vetoed. **G2 holds.**

## Why G1 fails — structural, not a tuning miss

The floor admits a birth by **displacement**; the veto blocks it by **position** (endpoint
in a flagged cell). The pier is excluded because its cells flag. But a vessel transiting
alongside the pier occupies **the same cells** while it passes — so the same veto that
catches the pier's laundered walk (Phase 1c) also catches the vessel. No spatial-only rule
separates "a vessel's moving endpoint in cell C" from "a pier-walk's endpoint in cell C"
when C is alongside the pier; the veto has no motion channel. The bare-cell neighborhood
helps marginally (revives at 30 m vs the ramp's 50 m) but still costs 20 s and still fails
inside the ramp. This is the **same laundering that killed the kinematic floor in Phase 1c**
(a linear structure is kinematically a transit) reappearing in occupancy space: a vessel
lingering near linear structure is spatially indistinguishable from the structure.

Phase 1d did not see this because its fixtures never put a moving vessel in the flagged
region: env-2's shore is a polygon (no dense returns → no structure formed → veto inert),
and harbor's near-pier boat is stationary. The veto's Phase-1d pass was **spatial
separation of vessels and structure**, which is exactly what the Cl-4 near-shore deployment
case does not have.

## What this closes and what it leaves

The floor+veto — the last candidate from the Phase-1 arc — joins A3 (camera is a clutter
source), the kinematic floor (association launders linear-structure walks), and the plain
persistence veto: **every measured mechanism that tries to revive near-shore vessels while
excluding near-shore linear structure fails, because near a linear structure a transiting
vessel and the structure are not separable by the signal the mechanism keys on** (sensor
identity, kinematics, or occupancy position).

Per the ticket this returns the endgame to the arbiter + user:
- **Per-geography residual:** ship `pmbm_land` (or the coverage stack) with a documented
  near-shore-linear-structure limitation; the Cl-4 "one config" claim carries an explicit
  geographic caveat rather than a false universality.
- **Re-price:** revisit the coverage/land cost model itself (path c) — orthogonal to the
  revive-near-structure framing this arc exhausted.
- The one signal not yet exhausted is a **charted/map extent prior** (ADR-0002 static
  obstacles) that knows a pier is a *fixed charted line* and can therefore exempt a moving
  vessel near it by identity rather than by position/motion — but that requires charts
  (not chart-free) and is a different ticket, not a tune of this veto.

## Reproduce

```
cmake --build build --target navtracker_cl4_a3_census
# G1: vessel parallel to pier, added latency vs no-pier
./build/bench/navtracker_cl4_a3_census --mode stage0 --scenario harbor_complete_truth \
    --seed 0 --transit-speed 3 --transit-yoff -320 --ramp 1     # with pier (veto)
./build/bench/navtracker_cl4_a3_census --mode stage0 --scenario harbor_complete_truth \
    --seed 0 --transit-speed 3 --transit-yoff -320 --with-pier 0 # baseline (no pier)
# G2 signal on stderr: "[G2] pier-line structure cells flagged (vessel present)=…"
```
Fixtures symlinked from the main tree (gitignored, not committed).
