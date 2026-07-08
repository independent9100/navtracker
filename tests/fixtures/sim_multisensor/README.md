# Multi-sensor simulation fixtures (`sim_multisensor`)

Seeded, deterministic generation of a **multi-sensor scenario battery with
complete independent truth** — the controlled accuracy gate for navtracker's
fusion arms. Real-data clips (philos, HAXR) cannot provide honest fusion truth
(no truth source is independent of the sensors being scored); simulation truth
is independent of every sensor *by construction*. This is what
`harbor_complete_truth` gave the core tracker, extended to the fusion problem.

Design ticket: `docs/superpowers/plans/2026-07-06-multisensor-sim-gates-ticket.md`
(and the survey it executes, `…-multisensor-sim-survey.md`). Consumed by the C++
`SimMultisensorScenarioRun` bench scenario (`adapters/benchmark/`), gated behind
`--with-simms`.

Nothing here enters the delivered navtracker targets — generation is tooling
only (extraction-boundary ruling, 2026-07-06). The generated CSVs and the venv
are git-ignored (`/tests/fixtures/` is ignored wholesale); only the generator
source, this README, and the pinned requirements are committed.

## Layout

```
sim_multisensor/
  generator/            committed Python package (the generator)
    geo.py              WGS-84 ENU datum, bit-compatible with core/geo/Datum.cpp
    truth.py            Layer 1: trafficgen wrapper + out-of-model kinematics
    clutter.py          Poisson + compound-K radar clutter fields
    sensors.py          Layer 2: radar / AIS / camera / own-ship observation models
    scenarios.py        the 6-scenario battery
    writer.py           loader-compatible CSV serialization + sha256
    generate.py         CLI
  requirements.txt      pinned direct deps
  requirements.lock.txt full frozen closure (exact reproduction)
  README.md             this file
  .venv/                git-ignored
  <scenario>_s<seed>/   git-ignored generated fixtures + meta.txt
  CHECKSUMS.txt         git-ignored sha256 drift guard
```

## Regenerating the battery

```bash
cd tests/fixtures/sim_multisensor
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt          # or requirements.lock.txt for exact closure
python -m generator.generate             # writes EVERY family at seed 0 (--family all)
python -m generator.generate --family imazu   # just the Imazu 22 (no trafficgen needed)
python -m generator.generate --family sim_ms  # just the original 6-scenario battery
python -m generator.generate --seed 1    # a second seed
python -m generator.generate --verify    # in-memory determinism self-check
```

`--family` selects a scenario family: `sim_ms` (the 6 multi-sensor scenarios,
trafficgen geometry), `imazu` (the 22 fixed-geometry Imazu encounters, explicit
placement — no trafficgen), or `all` (default). Fixtures land in
`<scenario>_s<seed>/` with `ownship.csv`, `ais.csv`, `radar_plots.csv`,
`camera_bearings.csv`, `truth.csv`, `meta.txt`.

## Determinism contract (a deliverable, not a hope)

Generation is a pure function of `(scenario, seed)`:

* trafficgen uses stdlib `random`; the wrapper calls `random.seed(spec.seed)`
  before every generation (verified deterministic — see the eval-log entry).
* every numpy draw comes from a per-`(scenario, sensor)`
  `np.random.default_rng([seed, salt])` stream; no global numpy state.
* single-threaded; CSV floats written with fixed-precision format strings.

Result: **byte-identical** files across processes (verified with a changed
`PYTHONHASHSEED`). The sha256 of every generated CSV is recorded in the dated
eval-log entry — anyone regenerating from the committed script + pinned deps
must reproduce those bytes. `CHECKSUMS.txt` is the local manifest.

## CSV schemas

Column orders are exact matches for the navtracker replay loaders, so fixtures
drop straight in:

| file | loader | columns |
|---|---|---|
| `ownship.csv` | `loadOwnshipCsv` | `unix_time,lat,lon,heading_deg` |
| `ais.csv` | `loadAisCsv` | `unix_time,mmsi,lat,lon,sog_mps,cog_deg,nav_status,name` |
| `radar_plots.csv` | `loadPlotCsvBodyFrame` | `tod,range_m,azimuth_deg,sigma_r_m,sigma_az_deg,n_cells,amp_max,station` |
| `camera_bearings.csv` | `loadCameraBearingsCsv` | `unix_time,camera,bearing_rel_deg,sigma_deg,confidence,u_px,v_px,w_px,h_px,frame` |
| `truth.csv` | `loadSimTruthCsv` (new) | `unix_time,truth_id,mmsi,lat,lon,sog_mps,cog_deg,heading_deg,nav_status` |

Notes:
* `radar_plots` / `camera_bearings` azimuths are **own-ship hull-relative**
  (moving platform); `loadPlotCsvBodyFrame` / `loadCameraBearingsCsv` compose
  the reported own-ship heading to recover the world bearing.
* `loadAisCsv` currently reads only `time/mmsi/lat/lon` and **drops
  `sog_mps/cog_deg/nav_status`** (the #20 velocity-path gap). We emit those
  columns anyway (honest record + ready for the follow-up); the fusion gate
  reports the gap rather than working around it.
* `truth` positions/velocities are projected through the *same* `geo::Datum`
  as the measurements, so truth and sensors share one ENU frame exactly — the
  reason `geo.py` replicates the C++ datum bit-for-bit.

## Anti-model-matched-optimism (the methodology core)

A sim gate that flatters the filter's own assumptions manufactures false
confidence. So, by design:

* **Out-of-IMM-model dynamics** (≥2 scenarios): the give-way vessel in
  `sim_ms_crossing` and the overtaking vessel in `sim_ms_overtaking` execute
  **rudder-rate-limited turns with turn-coupled speed loss** — time-varying
  turn rate + varying speed, which neither the CV nor the CT IMM mode can
  represent.
* **Non-Poisson clutter** (≥1 scenario): `sim_ms_clutter_burst` uses a
  **compound-K** (gamma-modulated Poisson) field — spatially clumpy, heavy
  tailed — plus a localized burst. A tracker whose clutter term assumes flat
  Poisson should measurably under-perform a spatially-varying-λ model here.
* **Range-dependent Pd**, position quantization, M.1371 cadence, GPS/heading
  faults — all present, none assumed away.

## The battery

| scenario | geometry | exercises |
|---|---|---|
| `sim_ms_crossing` | 3-vessel crossing | give-way **rudder-limited turn** (out-of-model), radar+AIS |
| `sim_ms_headon` | head-on pair | closing geometry, radar+AIS |
| `sim_ms_overtaking` | overtaking + maneuver | **out-of-model** maneuvering vessel |
| `sim_ms_ais_dropout` | AIS dies mid-run for one vessel | identity survival on radar (R11) + re-attach |
| `sim_ms_clutter_burst` | **compound-K** field + burst | over-count / clutter-model discrimination |
| `sim_ms_anchored_camera` | anchored (nav_status=1) + camera-only radar-silent contact | ADR-0002 never-invisible + #17 wedge |

## The Imazu 22 family (`--family imazu`)

`imazu_01`..`imazu_22`: the canonical Imazu-problem encounter set (Imazu 1987)
as fixed-geometry scenarios — own-ship + 1-3 targets in every head-on / crossing
/ overtaking combination (cases 1-4 single-target, 5-11 two-target, 12-22
three-target). Unlike the `sim_ms` battery these use **explicit placement**
(`ExplicitInitial` in `truth.py`), not trafficgen, because each case is a
*specific* published geometry, not a random encounter of a given COLREG type.
Own-ship never manoeuvres; targets run constant-velocity through CPA — a TRACKER
regression suite for **identity stability through close crossings** (not a COLAV
suite). radar+AIS arm, no camera. Geometry: CORALL `imazu_cases.py` (verbatim,
MIT); speeds preserve CORALL's tuned collision ratio at a physical own-ship
speed; ranges ×0.5. Full provenance + source-divergence note in
`generator/imazu.py`; results in `docs/baselines/2026-07-08_imazu22.md`. Consumed
by the same `SimMultisensorScenarioRun` class via `--with-imazu`.
