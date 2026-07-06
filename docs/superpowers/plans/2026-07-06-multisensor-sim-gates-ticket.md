# Implementer prompt — multi-sensor sim gates: Layer 1+3 skeleton + first scenario battery

Status: DONE 2026-07-06 (3d9543a, merged; suite 1058/1058 on merge). 6-scenario battery live behind --with-simms; first controlled fusion-vs-radar-only numbers (fusion ~2x on ais_dropout OSPA 33 vs 67); clutter_burst discriminator bites (+2.5/+3.5 card_err — the future lambda_C gate); ADR-0002 camera-only wedge canary PASS; out-of-model maneuvers cost identity not position (MHT 12.5 id-switches vs PMBM 0). #20 loader gap reproduced (follow-up filed in backlog). Paste everything below the line to the implementer
agent. Origin: the 2026-07-06 simulation survey
(`docs/superpowers/plans/2026-07-06-multisensor-sim-survey.md` — READ IT
FIRST, it is the design document this ticket executes). Motivation: every
multi-sensor feature shipped recently (#16/#17/#18/#20, AIS veto, wedge,
cooperative) is gated by unit/mechanics tests only — no controlled accuracy
gate exists for any fusion arm, and the philos farcross pass proved real data
cannot provide one (no independent truth exists there). Sim truth is
independent of every sensor BY CONSTRUCTION — that's the whole point.

---

You are working in the navtracker repo (C++17, CMake+Conan, read `CLAUDE.md`
first, then the survey doc above). Mission: build the first multi-sensor
simulation scenario battery — seeded Python generation → per-sensor CSV
fixtures + truth CSV → a bench scenario-run — and wire ONE fusion accuracy
gate. This is the Layer-1+3 skeleton from the survey; the full AIS/M.1371
observation layer and Fossen dynamics are follow-up tickets, but thin first
versions of the observation models ship here.

## Setup — worktree (a second implementer works the main tree in parallel)

`git worktree add ../navtracker-sim-gates -b multisensor-sim-gates` off
current master. Own build dir; Conan sandbox gotcha per CLAUDE.md build notes.

## Hard boundaries (standing rulings — do not bend)

1. **Nothing enters the delivered targets.** Generation is Python under
   `tests/fixtures/sim_multisensor/` (venv, own README + requirements.txt
   with PINNED versions — determinism depends on it). The only C++ additions
   are bench/test infrastructure (a scenario-run class in
   `adapters/benchmark/`, tests) — never `navtracker_core`/`navtracker_nmea`
   library surface. No new Conan dependency.
2. **Licenses: MIT/BSD/Apache only.** Approved: DNV `ship-traffic-generator`
   (MIT), `pyais` (MIT, optional this ticket), numpy/pandas. The survey's
   rejected list (RadarSimPy, Vo toolbox, AGPL tools) stays rejected.
3. **Determinism is a deliverable, not a hope.** Seeded generation
   (per-component seeds + pinned global numpy seed, single-threaded);
   generated fixtures stay LOCAL (tests/fixtures/ is gitignored) with
   sha256 checksums recorded in the eval-log entry as the drift guard —
   anyone can regenerate from the committed script + pinned deps and must
   get the checksummed bytes. Verify once: two clean-venv generation runs →
   identical checksums.
4. **Anti-model-matched-optimism (the methodology core).** At least two
   scenarios must have truth dynamics OUTSIDE the tracker's IMM model set
   (rudder-rate-limited turns, speed loss in turns — simple kinematic
   implementations are fine; full Fossen models are the follow-up), and at
   least one scenario must have NON-Poisson clutter (a simple gamma-modulated
   Poisson / compound-K count field per the survey, ~50 lines of numpy).
   A sim gate that flatters the filter's own assumptions is worse than no
   gate — it would manufacture false confidence.

## Layer 1 — truth generation

- Adopt DNV `ship-traffic-generator` for encounter geometry (head-on,
  crossing give-way/stand-on, overtaking around a moving own-ship). FIRST
  TASK: verify its seed control — the survey found it undocumented. If it
  can't be seeded cleanly, pin its randomness at the wrapper level (fixed
  inputs → cache its JSON output as the seeded artifact) rather than
  patching upstream.
- Propagate its waypoint plans to a truth CSV (time, vessel id, lat/lon or
  ENU, SOG, COG, heading, nav_status) at ~1 Hz. Anchored vessels: at least
  one scenario includes a vessel with nav_status=1 at anchor (watch-circle
  jitter, not perfectly still).

## Layer 2 — thin observation models (Python, over the truth CSV)

- **Radar plots**: per-scan (~2.5 s rotation) detection with configurable
  Pd (make it range-dependent in at least one scenario), Gaussian
  range/azimuth noise with per-plot sigmas in the CSV (the HAXR column
  pattern), Poisson clutter default + the compound-K field for the
  designated scenario. Own-ship-relative: plots are range/azimuth from the
  moving own-ship, like real life.
- **AIS (minimal honest version)**: decoded-CSV AIS (the loadAisCsv-style
  columns INCLUDING sog/cog/nav_status) at a simplified M.1371 cadence —
  SOG-dependent report interval (2–10 s underway by speed band, 3 min at
  anchor per nav-status), position quantization, and a scripted dropout
  window in the AIS-dropout scenario. Full 27-type NMEA emission via pyais
  is the follow-up ticket — do not gold-plate here.
- **Camera bearing-only**: bearing measurements from own-ship with a
  composed σ (camera ⊕ heading, the #16 convention), limited FOV, for at
  least one scenario (the bearing-wedge/#17 exercise: one vessel visible to
  camera only — radar-silent by scripted Pd=0 for that target).
- **Own-ship**: pose stream at ~10 Hz with GPS noise; one scenario includes
  a staleness gap + a heading fault window (feeds #18's guard — even if the
  gate only asserts the tracker survives it).

## Scenario battery (5–6, each multi-seed via generation seed)

1. `sim_ms_crossing` — 3-vessel crossing geometry, radar+AIS.
2. `sim_ms_headon` — head-on pair, radar+AIS.
3. `sim_ms_overtaking` — overtaking + one maneuvering (out-of-model
   dynamics) vessel, radar+AIS.
4. `sim_ms_ais_dropout` — AIS dies mid-scenario for one vessel; track must
   survive on radar with identity retained (R11) and re-attach on AIS return.
5. `sim_ms_clutter_burst` — compound-K clutter field + a spatial burst;
   radar+AIS; the over-count instrument (this scenario is the future
   clutter/birth-investigation gate — design it to discriminate: a
   spatially-varying λ_C model should beat uniform-λ_C here MEASURABLY).
6. `sim_ms_anchored_camera` — anchored nav_status=1 vessel + the
   camera-only radar-silent contact (never-invisible exercise: #17 wedge +
   ADR-0002 canary).

## Layer 3 — C++ wiring + the fusion gate

- A `SimMultisensorScenarioRun` in `adapters/benchmark/` (pattern:
  `HaxrScenarioRun` — env-pointed CSV dirs, skip-guarded on absence,
  datum set so occupancy/static wiring activates). Bench flag `--with-simms`.
- **The fusion gate (the deliverable that matters):** radar+AIS arm scored
  against complete truth — HONEST accuracy (sim truth is independent of all
  sensors by construction; state this in the doc). Assert per scenario:
  standing metrics recorded (gospa/card_err/lifetime/id_switches), plus
  targeted assertions — single-track-per-vessel in fusion (no dual-track),
  mmsi surfaced (R11), identity stable through the dropout scenario,
  anchored vessel never vanishes (ADR-0002), camera-only contact surfaces
  as wedge/hazard (never invisible). Skip-guarded tests + bench rows.
- Determinism test per scenario (same fixtures replayed twice → identical).

## Deliverables

1. Generation package (committed) + regeneration README + pinned deps;
   fixture checksums in a dated eval-log entry.
2. Scenario battery generated locally; bench runs of MHT-default +
   `imm_cv_ct_pmbm_coverage_land` on all scenarios; results table in
   `docs/baselines/2026-07-06_sim_multisensor_battery.md` (or dated).
3. The fusion gate tests green; full suite green.
4. Findings for the arbiter, not acted on — especially: where the fusion
   arm's accuracy lands vs the radar-only arm (the first controlled
   fusion-vs-single-sensor number this project has ever had), and anything
   the #20 SOG/COG columns reveal (the replay loader currently drops them —
   known follow-up; note whether your scenario-run hits the same gap).
5. Docs discipline: no integration-guide change expected (bench-only; if a
   Config struct is added, the drift-guard tells you what to do). A short
   learning-doc note ONLY if you introduce a genuinely new concept
   (compound-K clutter qualifies — one subsection + figure per the
   learning-docs rule).
6. Budget ~2 days. Stop-and-report triggers: trafficgen unseedable even at
   wrapper level; generated scenarios look degenerate (vessels teleporting,
   encounters not materializing); the fusion gate exposes a real tracker bug
   (that's a finding to report, not to fix in this ticket).
