# Implementer prompt — sim NMEA arm: end-to-end test of navtracker_nmea against honest truth

Status: ready to hand off. Paste everything below the line. Origin: named
follow-up from the multi-sensor sim survey + battery (2026-07-06) — the sim
feeds decoded-CSV AIS, which bypasses `AisAdapter`/`navtracker_nmea`
entirely; the delivered NMEA path (where #20 increments 1–3 live: heading/
nav_status parsing, SOG/COG velocity, sentinel handling) is tested by unit
tests only. This arm closes that: same seeded truth, emitted as real AIVDM
sentences, driven through the real adapter, scored against sim truth.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
worktree mandatory per the parallel-work convention:
`git worktree add ../navtracker-sim-nmea -b sim-nmea-arm`). You built the
sim battery; this extends it. Budget ~2 days.

## Layer 2 extension (Python, generator package)

1. Add an NMEA emitter to `tests/fixtures/sim_multisensor/generator/`:
   truth + the existing M.1371-lite scheduler → **AIVDM sentences** via
   `pyais` (MIT — encode types 1/3 position reports + periodic type 5
   statics; multipart handled by the library). Pin the dependency in
   requirements(.lock).txt. Same seeds ⇒ byte-identical sentence streams
   (prove once, two clean-venv runs).
2. Emit alongside (not replacing) the decoded CSV — one truth, two
   encodings. The scheduler stays shared so cadence/dropout/nav-status
   behavior is identical across both arms by construction.
3. Fidelity floor (don't gold-plate): correct 6-bit payload encoding,
   position quantization per M.1371 (1/10000 arcmin), SOG in knots with the
   1023 not-available sentinel exercised at least once, nav_status set
   (incl. the anchored vessel's 1), heading 511 sentinel exercised once.
   Deliberate imperfections stay in scope ONLY if the scheduler already
   models them — no new fault types this ticket.

## C++ arm

4. Extend `SimMultisensorScenarioRun` (or a sibling reader) with an
   NMEA-AIS arm: sentences → `AisAdapter` (the real `navtracker_nmea`
   parser) → measurements, replacing the decoded-CSV AIS stream when
   `SIMMS_AIS_NMEA=1` (env→param, default off = today's arm, byte-identical
   proven).
5. **The equivalence gate (the deliverable):** on the same scenario + seed,
   the NMEA arm and the decoded-CSV arm must produce equivalent tracker
   output — identical up to the quantization the NMEA encoding itself
   introduces (position 1/10000 arcmin, SOG 0.1 kn steps). Assert bounded
   metric deltas (state the bound from the quantization arithmetic, don't
   tune it) and identical track/identity structure (same confirmed count,
   same mmsi surfacing, id stability through the dropout scenario). Any
   larger divergence = an adapter bug found by this gate — report it, that's
   the ticket succeeding.
6. Run the full 6-scenario battery on the NMEA arm (MHT default +
   `imm_cv_ct_pmbm_coverage_land`); results table next to the battery doc.
   The #20 paths get their first end-to-end NMEA exercise: confirm
   nav_status flows to the anchored-veto path and the velocity emission
   respects the nav_status gate (both now live in AisAdapter defaults).

## Acceptance

1. Emitter committed (generator source only; sentence files stay local,
   checksummed in the eval-log entry); determinism proven.
2. Equivalence gate green with the quantization-derived bound stated; any
   divergence beyond it written up as a finding (not silently widened).
3. Battery results table + dated eval-log entry; suite green; skip-guarded
   on fixture absence; default-off byte-identical proven.
4. Docs: sensor-reference gains a note that the sim NMEA arm exists and
   what fidelity it has (and lacks); no consumer-surface change expected —
   if a Config appears, the drift-guard rules apply.
5. Stop-and-report: pyais encoding fights the M.1371-lite scheduler's
   assumptions for >2 h; or the equivalence gate exposes an adapter bug
   (that's a STOP for triage, not something to absorb into the bound).
