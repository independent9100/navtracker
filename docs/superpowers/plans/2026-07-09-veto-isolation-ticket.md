# Implementer prompt — anchored-vessel veto isolation: give the always-on veto an A/B partner and measure it on HAXR holding AIS constant

Status: ready to hand off. Paste everything below the line. Origin: the
increment-8 HAXR AIS-arm finding (eval-log: the veto's wiring is validated on
real data but its ISOLATED benefit is "unmeasurable without a veto-ON/OFF
toggle holding AIS constant — clean follow-up; the always-on veto has no A/B
partner today"). North-star tag: Cl-3 corroboration seam / Stage-1b closeout.
Budget ~half day to 1 day.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-veto -b veto-isolation-ab`, own build dir;
HAXR + philos fixtures from the MAIN tree; skips named by name in the
handoff; remember the ctest-from-build-dir trap — wire build/tests +
build/data symlinks or run the binary from the worktree root).

## Step 1 — the toggle (tiny, disciplined)

The corroboration suppression veto lives in `LiveOccupancyModel`
(`veto_radius_m` / `veto_window_s`, always active when vessel fixes arrive).
Add a per-instance config field (e.g. `corroboration_veto_enabled`, default
`true` = today's behavior) — ctor-threaded through the existing Config, no
global/static state (house rule). Default path byte-identical: prove it the
Phase-1 way (one HAXR run + one philos run, states/metrics identical
before/after your change with the flag at default).

Consumer-surface note: a new Config field ⇒ integration-guide entry + config
appendix row in the SAME commit (the drift-guard test will remind you).

## Step 2 — the isolated measurement (the point)

On the HAXR AIS-arm setup from increment 8 (same sites: kattwyk, parkhafen,
seemannshöft; same decimation; AIS arm ON in BOTH runs — that is the entire
trick: AIS constant, veto toggled):

- veto ON vs veto OFF, per site: occ_suppress_hits, occ_peak_structures,
  lifetime_ratio, card_err, gospa — the increment-8 metric set, so the rows
  are comparable to the eval-log entry.
- The question the entanglement blocked: does the veto LIFT suppression near
  anchored/cooperative vessels (protecting real vessels from being
  suppressed as structure), and what does that cost in phantom suppression
  elsewhere? Report the delta as the veto's isolated contribution.
- Also run the sim side once for a controlled anchor: the anchored-vessel
  sim scenario (`sim_ms_anchored_camera`) veto ON/OFF — perfect truth, so
  the protective effect is directly attributable there.

## Step 3 — verdict for the arbiter

One of: (a) veto's isolated effect is real and protective → keep ON,
document the measured contribution; (b) veto is inert once AIS is held
constant → keep ON (it's cheap + principled) but record honestly that its
benefit is currently unmeasured-to-nil on these sites; (c) veto HURTS
(lifts suppression that was correctly killing phantoms near vessels) →
report, don't change the default — that's an arbiter/user decision.

## Acceptance

1. Toggle shipped per-instance, default-ON byte-identical (proof included);
   integration-guide + appendix updated in the same commit.
2. HAXR 3-site A/B table + sim anchored A/B, committed baseline doc
   `docs/baselines/2026-07-09_veto_isolation.md` + dated eval-log entry
   (checksums, exact commands).
3. No default/behavior change beyond adding the toggle; verdict (a)/(b)/(c)
   stated with the numbers.
4. Full suite green, skips named. Banded assertions only if any test is
   added (#24 — and no cross-config pins on marginal regions).
5. Stop-and-report: the veto can't be toggled without touching the
   conservation invariant (veto only reduces suppression to 0 — if the
   refactor endangers that, stop); or the HAXR AIS-arm harness from
   increment 8 no longer reproduces its baseline row.
