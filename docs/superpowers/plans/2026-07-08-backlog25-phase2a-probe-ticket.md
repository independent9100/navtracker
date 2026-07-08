# Implementer prompt — backlog #25 Phase 2a: runaway census + offline probe of the velocity/innovation bound (no build)

Status: ready to hand off. Paste everything below the line. Origin: #25
Phase 1 (`docs/baselines/2026-07-08_b25_localization.md`, merged 3ae355f)
found H3 — velocity-state runaway under close-pass mis-association — plus a
systemic surprise: diverged-but-Confirmed tracks exist even single-target
(imazu_01: 46% of confirmed rows >50 m/s). Before we BUILD any lever, this
phase answers two questions offline, with binding kill-criteria — the
clutter-campaign discipline (a §5.0-style probe killed a doomed build there;
same rule here). North-star tag: Cl-3 (#25 is the deployment-choice
discriminator). Zero behavior change anywhere in this ticket.

NOTE on suite state: master is red on 3 documented sunset 6c tests
(eval-log 2026-07-08 correction; fix in flight on a separate ticket). Your
verification standard: full suite green EXCEPT exactly those 3, named in
the handoff — nothing else.

---

You are working in the navtracker repo (read `CLAUDE.md`; worktree
`git worktree add ../navtracker-b25p2 -b backlog25-phase2a`, own build dir;
imazu + autoferry + philos fixtures reached from the MAIN tree, ran-vs-
skipped BY NAME in the handoff). You built the Phase-1 tooling
(`IPmbmDiagnosticSink`, `tools/pmbm_closepass_trace.py`) — this ticket
consumes it. Budget ~1 day.

## Question A — the runaway census (who diverges, really?)

Phase 1 showed runaway exists even in single-target imazu_01. Before
calling it "a systemic IMM stability defect" we need the population split.
Over all 22 imazu cases + the 6 sim_ms scenarios (PMBM canonical, diag
export ON): for every confirmed-track row with speed > 50 m/s, attribute
it — (a) target-born track vs clutter-born track (use truth distance at
birth); (b) contested scan (≥2 tracks sharing in-gate measurements) vs
uncontested; (c) which IMM mode is dominant; (d) update cadence (dt since
last detected update). Deliverable: a census table. If the bulk of
single-target runaway rows are short-lived CLUTTER-born Bernoullis, the
"systemic defect" reading shrinks and a velocity bound doubles as a clutter
killer — say so explicitly either way.

## Question B — offline probe: would a velocity/innovation bound work?

Simulate the lever ON PAPER from the exported traces (no estimator change):
a track update is "flagged" when it implies speed > V_max or a position
innovation > D_max meters (sweep V_max ∈ {25, 50, 75} m/s ≈ 50–150 kn,
D_max ∈ {100, 200, 400} m).

- **Detection side (imazu_15/22 + the other 4 dying tracks):** at which
  scan would the bound first flag the diverging track, relative to (i) the
  scan the estimate leaves the 100 m gate and (ii) the CPA window? A bound
  that flags AFTER the loss is already irreversible is worthless — measure
  the margin.
- **False-fire side (the kill criterion):** on data with honest truth and
  legitimate maneuvers — the autoferry scenarios (real workload) + the
  sim_ms scenarios + imazu single-target controls — how often would each
  (V_max, D_max) flag a HEALTHY track (one within the gate of its truth)?
  Report false-fire rate per config per dataset.

**Binding kill-criteria (agree-before-measure, the campaign rule):** the
lever graduates to a Phase-2b build ticket ONLY if some swept setting flags
≥5 of the 6 dying tracks BEFORE gate-exit AND false-fires on <1% of healthy
confirmed track-scans on autoferry+sim_ms. If no setting passes both,
report the probe as negative and rank the alternates (ambiguity-gated soft
update; coalescence guard) with the evidence — do not build anything.

## Deliverables

1. Census table (A) + probe tables (B) committed as
   `docs/baselines/2026-07-09_b25_phase2a_probe.md` + dated eval-log entry;
   analysis script in `tools/` (consumes existing exports; no core change).
2. A one-page verdict: build / don't-build the bound, per the binding
   criteria; if build — the recommended (V_max, D_max) starting band and
   where the guard belongs (estimator update vs association gate), for the
   arbiter's Phase-2b ticket.
3. Zero core/config changes; suite green except the 3 documented sunset
   reds, skips named.
4. Stop-and-report: the diag export lacks a quantity you need (say which);
   or the census shows Phase-1's H3 attribution itself is confounded
   (e.g. the "dying tracks" are clutter-born — that would reframe #25).
