# Stage 1b-ii — live structure detector (presence over classification)

- **Date:** 2026-07-03
- **Status:** IN PROGRESS (implementer). Design of record below; increments TDD.
- **Governing decisions:** ADR 0002 + its 2026-07-03 amendment
  ("presence over classification"); the 2026-07-01 honest-static-occupancy
  design (superseded on the detector by the 2026-07-03 regime measurement + R4).
- **North-star:** Cl-3 (`docs/algorithms/comparison-baselines.md`).

## Why this exists (the two measurements that set the design)

1. **Regime measurement (eval-log 2026-07-03 follow-up):** birth-only occupancy
   suppression works + is safe on tuned synthetic churn but is inert on real
   philos at every tuning — fine cells never classify (sparsity + own-ship
   projection smear); coarse 100 m cells classify but the phantoms are already
   confirmed tracks the birth channel can't reach. The wall is the *detector*,
   not the channel.
2. **R4 (eval-log 2026-07-03):** on real philos, dwell/persistence does NOT
   separate the ~35 % KEEP craft from the ~50 % SUPPRESS structure — anchored
   boats are as persistent + compact as piers. **No occupancy-grid tuning can
   make that split; only chart/AIS/camera corroboration can.** ~50 % of the
   over-count mass is chart-confirmed structure (suppressible); ~35 % is real
   craft; ~14 % unknown.

## The goal (ADR 0002 amendment, do not re-litigate)

Presence over classification. For every persistent object in the water:
1. **Never invisible** — it is a track OR a static hazard, never neither.
2. **Correct classification preferred, not required** — anchored-vessel-shown-
   as-static-hazard-with-keep-clear is an acceptable degraded mode when no
   sensor can tell.
3. **Static → moving must recover** — anything represented as static is promoted
   to a moving track within bounded latency once it moves.

This reframes R4's 35 % KEEP: those craft need not stay *tracks*; they may be
*hazards*, PROVIDED conservation + recovery hold. That is far more achievable
than "never suppress a real vessel", and strictly safer than today's silent
over-count.

## Design

`LiveOccupancyModel` evolves (still one object; params-configured). The detector
is a config (`imm_cv_ct_pmbm_occupancy_detector`), not a new class.

1. **Coarse grid.** Default detector cell 100 m (R4: fires on philos; 25–50 m
   don't). Grid stays datum-stable (anchor frame; IDatumChangeSink).
2. **Persistence, low/no extent floor.** EWMA persistence per cell as today. The
   extent≥4 gate is RETIRED as the KEEP protector (the amendment lets a compact
   anchored boat be suppressed-as-hazard). A small floor (≥1–2 cells) only
   rejects single-return noise. Discrimination moves to corroboration.
3. **Conservation by construction (the load-bearing safety property).**
   `birthSuppression(q)` is DERIVED from the emitted hazard set: it is a ramp
   over `obstacles()` (1.0 inside `footprint_radius_m`, linear to 0 at
   `keep_clear_radius_m`), so `suppression(q) > 0 ⇒ q ∈ some emitted hazard's
   keep-clear ring`. Suppression into nothing is impossible. Each persistent
   structure component emits ONE `StaticObstacle` (`is_charted=false`,
   `source_id="live_occupancy"`) whose footprint covers ALL its cells → clean
   operator output, still conserved.
4. **Corroboration KEEP-guard (same milestone).** Optional inputs:
   - *Chart:* a live hazard coincident with a charted obstacle → classification
     confirmed (label; suppression stays — it IS structure).
   - *AIS:* suppression is VETOED within a radius of a recent AIS vessel fix (an
     AIS-known vessel must track, amendment rule where-we-can-discriminate).
     Belt-and-suspenders: tracked AIS vessels are already excluded from the
     occupancy feed by the unclaimed-only rule; this guards the not-yet-tracked
     case.
5. **Recovery (static → moving).** Inherent in EWMA decay: when a suppressed
   region's returns cease (vessel underway), its cells forget below the bar →
   drop from the hazard set → suppression lifts → the mover births normally. α
   tuned so recovery latency ≤ N scans while genuine structure still holds.

## Gates (all TDD)

- **Conservation invariant (unit):** over a query grid, every `q` with
  `birthSuppression(q) > 0` is inside some emitted hazard's `keep_clear_radius`.
- **Recovery gate (scenario):** `harbor_anchored_gets_underway` — an anchored
  non-AIS boat classified into a structure region gets underway mid-run; must
  confirm as a moving track within N scans; the vacated cells decay out of the
  hazard set (no permanent pin). Complete truth.
- **AIS KEEP-guard (unit/scenario):** a persistent region under a recent AIS fix
  is NOT suppressed.
- **Layer 1 — philos:** the detector CLASSIFIES real structure
  (`occ_peak_structures > 0` at 100 m; SUPPRESS canaries hit, KEEP canaries not).
- **Layer 2 — HAXR hours:** birth-only suppression actually reduces the phantom
  over-count at steady state on real hour-long churn (the 20 s philos clips are
  too short to show confirmed-cohort re-birth).
- **dense_clutter:** byte-identical (uniform clutter never persists).
- **off / determinism:** unwired ⇒ bit-identical; deterministic replay.

## Increments

1. Conservation refactor (suppression ⊆ hazards) + coarse/low-extent params +
   conservation unit test. Preserves 1b-i behaviour at 1b-i params.
2. Recovery gate scenario + α latency check.
3. Corroboration input (chart confirm + AIS veto) + wiring + tests.
4. Detector config + Sweep wiring; Layer-1 philos + Layer-2 HAXR A/B.
5. Docs: `docs/algorithms/live-static-occupancy.md` four-part update +
   `docs/learning/26-static-obstacles.md` chapter + figure; ADR 0002 staging;
   comparison-baselines row; eval-log.
