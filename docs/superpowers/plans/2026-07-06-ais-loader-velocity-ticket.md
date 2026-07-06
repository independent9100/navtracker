# Implementer prompt — replay AIS loader: emit SOG/COG velocity (backlog #20 follow-up)

Status: ready to hand off. Paste everything below the line. Origin: backlog
#20 follow-up (filed 2026-07-06, `docs/algorithms/improvement-backlog.md`) —
reproduced twice: the philos farcross pass and the sim-gates battery both
carry `sog/cog/nav_status` columns that `loadAisCsv` silently drops, so the
#20 increment-2 velocity path has never run against measurable truth.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`). Small,
sharply-scoped change: teach the replay AIS loader to optionally emit what
the fixtures already carry. Main tree is fine; ~half day.

## The change

1. `loadAisCsv` (replay adapters): parse `sog`/`cog`/`nav_status` columns
   when present (tolerate their absence — old fixtures stay valid).
   - `nav_status` → `hints.nav_status` (this one is arguably a bug fix, but
     it feeds the anchored-veto path → treat it as gated too, same flag).
   - SOG/COG → PositionVelocity2D measurement content, reusing the SAME
     rules as `AisAdapter` increment 2 (threshold `sog_velocity_min_mps`,
     polar Jacobian + isotropic floor — do NOT reimplement: extract/reuse the
     existing helper so replay and NMEA paths cannot drift apart).
2. Behind one default-off toggle (env or loader param, matching the
   `PHILOS_RADAR_ONLY` / `SIMMS_*` pattern). **Default off = byte-identical**
   — prove it: full suite + one bench byte-compare on a philos clip and one
   simms scenario.

## Pricing when ON (this is the point of the ticket)

- Sim gates (`--with-simms`, all 6 scenarios): fusion arm with velocity ON
  vs OFF — the first accuracy-measured exercise of the #20 velocity path.
  Honest truth by construction; report the standing metrics delta per
  scenario. Hypothesis to test, not assume: velocity content should improve
  dropout/continuity scenarios and must not hurt the anchored scenario
  (low-SOG fallback should make it inert there — check nav_status 1 vessel).
- philos ais_ferry_far fusion arm (mechanics only, circular truth — label it).
- Findings for the arbiter; no default flipped, no config promoted.

## Acceptance

1. Default-off byte-identical proof stated.
2. ON-pricing table (simms 6 scenarios + farcross mechanics), dated eval-log
   entry.
3. Shared-helper reuse verified (no duplicated Jacobian math).
4. Full suite green. If the shared-helper extraction wants to touch
   `AisAdapter`'s internals beyond a mechanical move, stop and report.
