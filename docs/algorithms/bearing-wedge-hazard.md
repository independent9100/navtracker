# Bearing-wedge hazard (camera-only "never invisible" safety net)

Reference for the `BearingWedgeModel` (`core/static/BearingWedgeModel.hpp`),
backlog #17 option 1. Plain-English introduction:
[`docs/learning/28-bearing-wedge-hazard.md`](../learning/28-bearing-wedge-hazard.md).

## Problem

A target only the camera sees (a kayak, a small wooden boat — radar-silent) yields
a **`Bearing2D`** stream. Bearing-only measurements cannot initiate a track (a
single bearing has no range, so there is no position to birth). Under ADR 0002
"presence over classification" that object must still appear as *something* — the
sharpest real-world case of the "never invisible" rule. The clean fix is a
distance sensor (camera + range → the existing range/bearing path). This model is
the **defense-in-depth fallback** for when there is no range: surface the
*direction*.

## Math

**State (one wedge).** A wedge is a planar sector, not a point:

- apex `A` — own-ship ENU position at detection, stored in the model's fixed
  **anchor frame** (see Assumptions);
- centre bearing `β` — the measurement value verbatim, in the internal
  `Bearing2D` convention `β = atan2(dN, dE)` (math angle, CCW from east);
- half-width `w = max(k·σ_β, w_min)`, `k = half_width_sigma_mult` (default 2),
  `w_min = min_half_width_rad` (default ≈ 1.5°);
- range extent `[0, R]`, `R` optional — `nullopt` means unbounded (range unknown,
  the defining case).

The wedge region is `{ A + r·[cos β', sin β'] : r ∈ [0, R], |β' − β| ≤ w }`
(east = `cos`, north = `sin` in the math convention).

**Composed σ (important).** `σ_β` passed to `observeBearing` must be the
**composed** bearing uncertainty `σ_β = σ_camera ⊕ σ_heading` (added in
quadrature), because the fed bearing is *relative bearing + own-ship heading*, so
heading error is part of the true angular uncertainty. Calibration of the philos
camera showed a median ≈ 0.45° but a p90 tail ≈ 1.32°; `w_min` guards against an
implausibly thin wedge on an optimistic σ. (Ties to backlog #16, per-pose
heading σ.)

**Claim (handover), recomputed every drain.** A confirmed track at ENU position
`T` *claims* a wedge iff, from the wedge apex, it lies inside the sector:

```
d = T − A,   ρ = ‖d‖
claimed  ⇔   ρ ≥ ρ_min                       (not degenerate at the apex)
        ∧   (R absent ∨ ρ ≤ R)               (inside the range extent)
        ∧   |atan2(d_N, d_E) − β| ≤ w + m     (inside the angular span; m = margin)
```

A claimed wedge is **suppressed from output**, not deleted. Because the claim is
recomputed on every drain from the current confirmed-track set, the wedge
reappears the instant no track occupies its direction. Nothing is latched.

**Output.** `toBearingWedgeOutput` converts the anchor-frame apex to geodetic via
the anchor datum and the math bearing to a **true bearing** `β_true = (90° − β_deg)
mod 360` (CW from north), the operator-facing convention.

## Assumptions

- **The fed bearing is the `Bearing2D` measurement value** (math angle
  `atan2(dN, dE)`) and `σ_β` is the composed camera⊕heading σ. Feeding a true
  bearing, or a camera-only σ, is a caller error.
- **Anchor-frame storage.** Apexes live in a fixed anchor datum (the model's
  construction datum), exactly like `LiveOccupancyModel`. An own-ship auto-recenter
  updates only the current→anchor transform (`onDatumRecentered`); apexes never
  move, so there is no per-wedge shift and no round-trip drift. The consumer MUST
  register the model as an `IDatumChangeSink` (see the integration guide).
- **Local tangent planes.** Anchor and working ENU frames share N/E orientation
  to within meridian convergence (< 0.5° inside 30 km), so a bearing carried
  across a recenter is unchanged to that tolerance — the same tolerance the
  output-contract already accepts for NED covariance rotation.
- **`contact_id` identifies a physical contact only while continuously reported.**
  The emitting sensor may reuse the number after a drop/swap; the model treats a
  reappeared-after-prune key (or a `suspect`-flagged report) as a new contact and
  mints a fresh `wedge_id` (never-reused holds for real).
- **Standalone.** The model is not on the PMBM hot path. A consumer (or thin
  wiring) feeds camera bearings and the confirmed-track set each cycle.

## Rationale

- **Wedge, not a range-parameterised birth.** Options 2 (waterline monocular
  range) and 3 (range-parameterised bearing-only initiation) give a *position*
  but need calibration / real TMA work. The wedge needs neither and is the
  cheapest thing that satisfies "never invisible": a direction is operator-
  actionable ("keep clear of that line") even without range. CPA is *not*
  computable (no position) — documented, not hidden.
- **Suppression, not retirement (the load-bearing decision).** A one-way
  `retire()` on claim has a hole: with an unbounded wedge, a *near* vessel
  crossing the bearing of a *far*, still-seen camera contact would permanently
  erase the far contact — the ADR-0002 forbidden failure (an object represented
  as nothing). Recomputing suppression each drain, and removing a wedge only when
  the camera goes quiet (`pruneStale`), makes presence beat absence. Same shape as
  the occupancy corroboration veto.
- **Anchor frame over apex-shifting.** Both keep apexes correct across a recenter;
  the anchor frame (already used by `LiveOccupancyModel`) avoids repeated
  round-trip shifts and their accumulated drift.

## Ways to improve / what to test next

- **PMBM auto-wiring.** This increment is standalone. Next: route *unclaimed*
  `Bearing2D` returns from the tracker to `observeBearing`, and feed the confirmed
  tracks automatically, so a consumer gets wedges without hand-wiring. Experiment:
  a scenario test with a camera-only target + a radar target crossing its bearing;
  assert the wedge shows, suppresses on crossing, and reappears — end to end.
- **Waterline range (option 2).** Upgrade a wedge to a coarse range+bearing
  measurement when a horizon/pitch calibration is available (`r ≈ h/tan(depression)`,
  σ_r ~ r²). Compare wedge-only vs coarse-range on a controlled fixture.
- **Angular merge.** Two contacts at nearly the same bearing produce overlapping
  wedges. Consider merging for display; measure operator-clutter vs missed-contact
  trade-off.
- **Keep-clear alarm for wedges.** A `StaticHazardEvaluator` analogue that warns
  when own-ship's course crosses a wedge; needs an angular, not radial, geometry.
