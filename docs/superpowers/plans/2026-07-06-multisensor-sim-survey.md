# Multi-sensor simulation for navtracker — survey + recommendation (2026-07-06)

Three parallel research sweeps (tracking-research frameworks / marine robotics
simulators / AIS-traffic scenario generators) evaluated what we could adopt to
build **controlled multi-sensor scenario gates with complete truth** — the
gap exposed by the philos `radartruth` finding (no real-data clip can give
honest accuracy truth for a fusion arm). Integration pattern is fixed: offline
seeded generation → per-sensor CSV fixtures + truth CSV → existing C++ replay.
All generation tooling stays OUTSIDE the delivered targets (Python, fixtures/
tools) per the 2026-07-06 extraction-boundary ruling.

## The convergent finding

**No single simulator provides what we need; three small MIT-licensed pieces
plus thin in-house observation layers do.** Specifically:

- Robotics sims (VRX/Gazebo, MARUS/Unity, Stonefish, HoloOcean) model robot
  sensors (lidar/sonar/camera), NOT navigation radar or AIS — verified at
  code level (VRX: no radar; MARUS: advertised radar absent from released
  code). Adopting them buys wave physics we don't need at plot level. SKIP.
- **Nobody simulates AIS.** Not Stone Soup, not MATLAB's commercial toolbox,
  no open tool: the ITU-R M.1371 observation model (SOG/turn/nav-status-
  dependent cadence, Class A/B schedules, quantization, slot-collision
  dropouts, stale static data) must be built in-house (~300 lines over a
  protocol library). This is also the highest-value emitter for us — the
  anchored-vessel veto, #20 paths, and cooperative logic all turn on AIS
  cadence quirks.
- Detection-level sea clutter is a gap everywhere open-source; compound-K
  counts + correlated texture is ~50 lines of numpy (literature-backed),
  pluggable into Stone Soup's ClutterModel.

## Recommended architecture (three layers, all seeded/deterministic)

### Layer 1 — truth trajectories (two complementary sources)

| tool | role | license / status |
|---|---|---|
| **DNV ship-traffic-generator** ("trafficgen") | scripted COLREG encounter geometry: head-on / crossing / overtaking around own-ship, JSON out (waypoints+SOG+MMSI+static data) | MIT, active (v0.9.0 2026-04). Seed control undocumented — verify/patch |
| **Fossen PythonVehicleSimulator** (+MSS reference) | physically credible maneuvering (validated models: frigate→304 m tanker, Clarke83 parametric): realistic turns, speed loss, rudder limits — truth dynamics that deliberately DIFFER from the tracker's IMM set | MIT, active (Fossen himself) |

Supporting: **Imazu 22** canonical hard encounters transcribed as a named
regression suite (trivial); **DMA / NOAA MarineCadastre AIS archives** via
MovingPandas (BSD-3) as the realism corpus — real trajectories replayed
through our observation layers (sim-primary / real-reality-check methodology).

### Layer 2 — sensor observation models (in-house; the part nobody ships)

- **Radar plots**: our existing emitter machinery, upgraded with the effect
  checklist from MATLAB `fusionRadarSensor` (the industry-standard statistical
  model): range-dependent Pd, resolution-limited merging, bias, scan timing;
  compound-K clutter field (NOT flat Poisson — anti-model-matched-optimism).
  Alternative/complement: **Stone Soup**'s `RadarRotatingBearingRange`
  (RPM+FOV dwell) + `SwitchMultiTargetGroundTruthSimulator` +
  `SwitchDetectionSimulator` (mode-dependent Pd) — MIT, already in our venv.
- **AIS**: **pyais** (MIT, very active — encode/decode all 27 types,
  multipart) + in-house ITU-R M.1371 scheduler + seeded fault injectors →
  emits BOTH NMEA sentences and decoded CSV (exercises both ingest paths).
- **Camera bearing-only**: trivial from truth + composed σ (#16); Stone Soup
  `PassiveElevationBearing` as reference.
- **Own-ship/GNSS faults**: staleness, jumps, low-SOG COG — feeds #18's
  guard tests.

### Layer 3 — export + freeze

Seeded generation script → CSV fixtures in the existing schema, md5-pinned,
frozen per the fixture discipline. Determinism engineering: per-component
seeds + pinned global numpy seed, single-threaded generation.

## Rejected / flagged (do not adopt)

- **RadarSimPy** — commercial license required for company use; IQ-level
  (wrong abstraction) anyway.
- **Vo & Vo RFS toolbox** — academic-use-only terms.
- **VRX / MARUS / Stonefish / HoloOcean / UNav-Sim** — no radar/AIS at the
  level we need; GPL (Stonefish) / packaging (HoloOcean) issues besides.
- **aisdb** — AGPL-3.0: offline-tooling-only if ever, corporate flag.
- **Brest tracklet dataset & similar CC BY-NC** — non-commercial.
- **Bridge Command** (GPL-2, active) — NOT a pipeline component (GUI-bound,
  undocumented protocol), but the one open system modeling radar shadowing /
  sea clutter / RACON artifacts at picture level: keep as a free REFERENCE
  for which degradation effects our radar emitter should imitate.
- **NTNU colav-simulator** (MIT, active) — the only tool with native
  radar+AIS observing shared truth, but a heavy stack (Gymnasium/SB3/ENC)
  duplicating our own concern; second-wave option for behavioral/reactive
  traffic, not the first integration.
- **García-Fernández MTT repo** (BSD-2) — not a generator, but port its
  canonical PMBM benchmark scenario parameters into Layer 1 for
  literature-comparable GOSPA numbers (Cl-3 credibility).

## Effort estimate

- First frozen scenario battery (Stone Soup radar+bearing or trafficgen
  geometry + existing emitters, truth CSV, 4–6 scenarios: crossing, head-on,
  overtaking, AIS dropout, clutter burst, anchored-vessel): **~2 days**.
- AIS M.1371 observation layer (pyais + scheduler + fault injectors): **~2 days**.
- Fossen-dynamics truth upgrade + Imazu suite + archive corpus lane:
  **incremental, ~1 week total** spread over the campaign that consumes it.

## Why this matters now (the argument for the discussion)

Every multi-sensor feature shipped in the last two weeks (#16/#17/#18/#20,
veto, wedge, cooperative, RemoteTrack) is gated by unit/mechanics tests only —
no controlled accuracy gate exists for any fusion arm, and the philos pass
proved real data cannot provide one. This pipeline gives fusion what
harbor_complete_truth gave the core tracker. The model-matched-optimism
mitigation is designed in, not bolted on: truth dynamics ≠ filter models
(Fossen/switching), clutter ≠ Poisson (compound-K), Pd ≠ constant
(mode/range-dependent).

Decision pending with the user; if adopted, first ticket = Layer-1+3 skeleton
with the 4–6 scenario battery and one fusion gate wired into the bench.
