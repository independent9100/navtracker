# Implementer prompt — LOS/shadow guard for coverage-aware decay (verdict-b fix)

Status: ready to hand off. Paste everything below the line. Origin: the
2026-07-06 LOS/shadow probe (eval-log entry, merged d83f734) returned verdict
(b): coverage-aware decay does NOT sweep-inflate behind an occluder, but the
~10 baseline observed-empty decays go UNOPPOSED while the occluder removes
the shadowed object's returns (touches 8→2) — eroding a real moored vessel's
occupancy mass 24× (0.141→0.006) and its hazard presence 72%→51% during a
35 s passage. Bounded and self-healing, but it degrades the ADR-0002
presence channel on every close passage of a large ship — routine in a busy
harbor. The arbiter ruled: implement the guard.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first,
including the parallel-work convention — worktree mandatory:
`git worktree add ../navtracker-los-guard -b los-shadow-guard`).

## The fix (the probe's sketch, now the spec)

In the coverage-aware decay path (`LiveOccupancyModel` + sector logic):
**do not decay a cell that lies beyond a strong closer return on (about) the
same bearing in the same scan** — its line of sight is blocked; "no return"
there is not evidence of emptiness. Design parameters (config, per-instance,
documented): what counts as "strong closer return" (n_cells/amp floor — the
probe's occluder had n_cells 100+), the bearing tolerance (the probe measured
median 13° burst sectors; the guard's shadow wedge should derive from the
occluder's angular extent, not a magic constant), and a range margin beyond
the occluder. Conservative direction: when in doubt, DON'T decay (unobserved
cells simply don't decay — the standing safe direction).

## Proof obligations

1. **RED→GREEN on the probe itself**: `test_philos_los_shadow_probe.cpp` is
   your acceptance instrument. With the guard ON, the shadow interval's mass
   erosion must substantially disappear (target: shadow-interval decays on
   the `unknown_w860` cell ≈ 0; mass retained same order as pre-shadow;
   hazard presence stays high through 50–85 s). Extend the probe test with
   the guarded assertion rather than writing a parallel one.
2. **No false shielding**: the sim control (`sim_ms_anchored_camera`, no
   occluder) and the standing gates must show decay behavior UNCHANGED where
   no occluder exists — the guard must only fire behind strong closer
   returns. dense_clutter_datum + harbor + philos KEEP: unchanged (this
   touches the occupancy layer only; PMBM tracking output should be
   byte-identical when the occupancy detector is OFF — prove default
   configs unaffected).
3. Unit tests for the shadow-wedge geometry itself (occluder extent →
   shadowed cells; edge cases: occluder at cell's own range, multiple
   occluders, bearing wraparound).
4. Config default: the arbiter's intent is guard ON wherever coverage-aware
   decay is ON (it's a correctness fix, not an option) — but ship it as a
   per-instance config with ON default for the decay-enabled configs and
   price the delta honestly vs pre-guard on the HAXR occupancy arm
   (increment-8 A/B metrics must not regress; the guard should be near-inert
   there since HAXR is a fixed shore station — say what you find).
5. Docs: algorithm doc four-part update (the decay doc gains the LOS
   assumption + guard); learning-doc subsection (shadowing is geometric —
   diagram per the rules); integration-guide/config-reference row for the
   new knobs (drift-guard will insist).

## Acceptance

Guarded probe assertions green; no-occluder behavior proven unchanged;
default-config byte-identity proven; HAXR delta reported; suite green; docs
complete. Budget ~1 day. Stop-and-report: if the sector representation
can't express "blocked beyond range R on this bearing" without restructuring
(rather than annotating) the decay path, report the shape instead of
rebuilding the layer.
