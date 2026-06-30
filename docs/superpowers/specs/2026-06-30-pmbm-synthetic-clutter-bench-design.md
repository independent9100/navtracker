# Synthetic Multi-Target + Shore-Clutter Bench — Design

**Status:** Approved (brainstorming) — 2026-06-30
**Author:** subagent-driven session
**Relates to:** `docs/superpowers/specs/2026-06-30-pmbm-land-clutter-prior-design.md` (the land model this bench A/Bs), `docs/algorithms/comparison-baselines.md` (north-star: Cl-2 scenario breadth, Cl-3 clutter realism)

---

## 1. Problem

Our scenario coverage has two gaps that block confident algorithm comparison:

1. **Geometry breadth (Cl-2).** We test crossing, overtaking, head-on, and a couple of
   parallel/clutter cases, but not *systematically*: parallel lanes at varying spacing,
   crossings at controlled angles, in-line/overtaking convoys, and target density are not
   swept. Association failures (track swaps, merges, spawns) are geometry-driven, and we
   cannot currently dial geometry.

2. **Realistic clutter (Cl-3), with perfect truth.** The land-clutter prior (task A) was
   validated only on **philos real data**, where ground truth is AIS-derived and imperfect,
   and where we *cannot* fully separate "stationary shore clutter" from "anchored ship near
   shore." We need a controlled scene where shore clutter is injected by us — so truth is
   exact — and where the land model can be switched on/off to measure its effect cleanly.

This bench closes both: parametric geometry generators + a stationary shore-clutter injector,
both with perfect ground truth, runnable through the existing `runSweep` + GOSPA/OSPA harness.

## 2. Goals / Non-goals

**Goals**
- Parametric synthetic scenarios for parallel, crossing-angle, and in-line/overtaking
  geometries, with a target-density knob, perfect truth, seeded determinism.
- A stationary shore-clutter injector: fixed false returns on/near a synthetic coastline,
  recurring every scan, with no associated truth track.
- A near-shore **real-target** validator scenario, to prove the inland-only hard gate does
  not delete a genuine vessel sitting at the waterline (the "anchored ship" protection).
- An in-memory coastline hook so a synthetic scenario's **own** `CoastlineGeometry` both
  seeds the shore clutter AND drives the land model — one shoreline, used twice — enabling a
  clean `use_land_model` A/B on perfect truth.

**Non-goals**
- No third-party simulator (Stonefish/VRX). Out of scope, decided previously.
- No new metrics. Reuse GOSPA / OSPA / cardinality-error already in the harness.
- No new tracker behaviour. This is measurement infrastructure; the only tracker-side change
  is the bench *wiring* hook, not core estimation/association.
- No physically-modelled radar clutter (RCS, multipath). Shore clutter is a fixed-position
  point process — the controlled analogue of the philos returns, not a sensor simulation.

## 3. Architecture (hexagonal placement)

```
core/scenario/Builders.{hpp,cpp}     NEW pure builders (parametric geometry + shore clutter)
core/benchmark/ScenarioRun.hpp       NEW optional hook: syntheticCoastline()
core/benchmark/Sweep.cpp             prefer in-memory coastline over file path
adapters/benchmark/SimScenarioRun.cpp NEW ScenarioRun subclasses; register in defaultSimScenarios
core/benchmark/Config.cpp            (no change expected; reuse imm_cv_ct_pmbm_coverage[_land])
tests/...                            unit tests per builder + a wiring/integration test
```

Builders stay **pure** (`core/scenario`): deterministic given a seed, no I/O, no wall-clock.
Scenario *registration* and any descriptor wiring live in `adapters/benchmark`. The land model
core (`CoastlineModel`, `CoastlineGeometry`) is unchanged — it already accepts an in-memory
`CoastlineGeometry`; only the bench plumbing learns to pass one.

## 4. Coordinate frame (the one subtlety)

Synthetic measurements are generated directly in **ENU** (existing pattern; synthetic
`Scenario.datum` is empty today). The land model, however, queries
`CoastlineModel::clutterPrior(enu)` → `datum.toGeodetic(enu)` → `geom.priorAtGeodetic(lat,lon)`,
so it needs both a datum and a geodetic polygon.

Resolution: each shore-clutter scenario picks a **fictitious coastal datum** (a plausible
lat/lon origin), defines its shoreline as ENU points, converts those points to geodetic via
`datum.toGeodetic(...)` to build the `LandPolygon`/`CoastlineGeometry`, and sets
`Scenario.datum` to that datum. The clutter injector places blips in ENU; the land model maps
ENU→geodetic through the *same* datum to query the *same* polygon. Round-trip consistent by
construction. `CoastlineGeometry` computes shoreline distance in local-equirectangular metres,
so the ramp widths (`inland_halfwidth_m`, `offshore_halfwidth_m`) stay metric regardless.

## 5. Components

### 5.1 Parametric geometry builders (`core/scenario/Builders.{hpp,cpp}`)

Add builders that generate N constant-velocity targets in configurable geometry. Reuse the
existing per-step truth + Position2D-measurement + isotropic-noise idiom from
`buildParallelTargetsScenario` / `buildCrossingTargetsScenario`. Each returns a `Scenario`
(measurements + truth, no datum unless shore clutter is added — see 5.3). Truth ids are
assigned `1..N` in a documented, deterministic order.

- `buildParallelLaneScenario(n_targets, lane_spacing_m, velocity, lane_length_start, sample_times, pos_noise_std_m, seed)`
  — N targets on parallel lanes, same heading, evenly spaced by `lane_spacing_m`. Stresses
  resolution / track-merge as spacing shrinks.
- `buildCrossingAngleScenario(crossing_angle_deg, speed_mps, crossing_point, sample_times, pos_noise_std_m, seed)`
  — two CV targets meeting at `crossing_point` with relative course `crossing_angle_deg`
  (swept externally at 30/60/90). Truth emitted in a documented order.
- `buildConvoyScenario(n_targets, gap_m, speed_mps, overtaker_speed_mps, sample_times, pos_noise_std_m, seed)`
  — `n_targets` in a single lane spaced by `gap_m` at `speed_mps`, plus one faster overtaker
  at `overtaker_speed_mps` running up the same lane. Stresses in-line association + overtaking.

Density is the `n_targets` argument; angle is `crossing_angle_deg`; spacing is
`lane_spacing_m` / `gap_m`. Sweeping is done by registering multiple descriptor instances
(see 5.4), not by branching inside a builder.

### 5.2 Stationary shore-clutter injector (`core/scenario/Builders.{hpp,cpp}`)

```
Scenario addShoreClutter(Scenario base,
                         const CoastlineGeometry& shore,
                         const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& clutter_enu_points,
                         double detection_prob,        // P_D per point per scan
                         double pos_noise_std_m,
                         std::uint32_t seed);
```

- Takes an existing `Scenario` (the geometry from 5.1) and adds fixed clutter returns.
- `clutter_enu_points` are fixed ENU positions on/inland of `shore` (caller supplies; a helper
  `sampleShoreClutterPoints(shore, datum, ...)` may generate them along the polygon — see 5.3).
- For each scan timestamp already present in `base.measurements`, each clutter point emits a
  Position2D measurement with probability `detection_prob` (seeded Bernoulli), at its fixed
  position plus isotropic Gaussian noise (`pos_noise_std_m`). **Same nominal positions every
  scan** — this is the defining property distinguishing shore clutter from the existing
  uniform-Poisson `buildClutterCrossingScenario` (which redraws random positions each scan).
- Clutter measurements carry `source_id = "sim_shore"` and create **no** `TruthSample` — so
  GOSPA/cardinality count them as the over-count the land model should remove.
- Sets `base.datum = datum` so the land model can be wired.

### 5.3 Synthetic coastline (`core/scenario/Builders.{hpp,cpp}`)

```
struct SyntheticShore { CoastlineGeometry geometry; geo::Datum datum;
                        std::vector<Eigen::Vector2d> clutter_enu_points; };
SyntheticShore buildSyntheticShore(/* shore_y_m, pier params, clutter spacing, datum origin */);
```

- Defines a simple shoreline in ENU: land occupies y ≥ `shore_y_m` with one or two
  rectangular piers protruding into the water, water below. Converts the ENU outline to
  geodetic via `datum.toGeodetic(...)` → `LandPolygon` (outer ring, lon/lat) →
  `CoastlineGeometry` with explicit `CoastlinePriorParams`.
- Generates `clutter_enu_points` spaced along the land side of the shoreline (and piers),
  i.e. points that fall in the hard-gate plateau region (well inland) — the realistic case
  where the mask should suppress.
- Returns geometry + datum + clutter points together so a scenario builds the *same* shore
  once and uses it for both clutter generation and the land-model hook.

### 5.4 ScenarioRun subclasses + registration (`adapters/benchmark/SimScenarioRun.cpp`)

For each scenario, a `ScenarioRun` subclass that:
- `descriptor()` returns a stable `label`, multi-seed config, and `detection_table` declaring
  realistic per-sensor (P_D, λ_C) so MHT and PMBM both see the clutter environment.
- `generate(seed)` calls the builder(s) with that seed.
- For shore-clutter scenarios, **overrides `syntheticCoastline()`** (new hook, 5.5) to return
  the scenario's `CoastlineGeometry`, and leaves `coastline_geojson_path` empty.

Registered in `defaultSimScenarios()`. Proposed scenario set:

| label | builder(s) | what it stresses |
|---|---|---|
| `parallel_lanes_dense` | parallel, small spacing, N=4 | track merge / resolution |
| `crossing_30` / `crossing_60` / `crossing_90` | crossing-angle | angle-dependent association |
| `convoy_overtake` | convoy | in-line + overtaking |
| `shore_clutter_open` | crossing + shore clutter | land model A/B, no near-shore truth |
| `shore_clutter_nearshore` | shore clutter + a slow real target at the waterline | inland-only hard gate must NOT kill the real target |

(Exact spacings/angles/counts pinned in the plan; the table is the scenario set, not tuning.)

### 5.5 In-memory coastline hook (`core/benchmark/ScenarioRun.hpp`, `core/benchmark/Sweep.cpp`)

Add to the `ScenarioRun` port:

```cpp
// Optional in-memory coastline for synthetic scenarios. Default = none, so
// every existing scenario is untouched. When present AND config.use_land_model,
// Sweep builds a CoastlineModel from this geometry + Scenario.datum, in
// preference to coastline_geojson_path. Real-data scenarios keep using the path.
virtual std::optional<CoastlineGeometry> syntheticCoastline() const { return std::nullopt; }
```

`Sweep.cpp` land-wiring (currently lines ~342–350) becomes: if `config.use_land_model`,
prefer `run.syntheticCoastline()` (when present and `scen.datum` set) → construct
`CoastlineModel(geometry, *scen.datum)`; else fall back to today's file-path branch. Nothing
else in Sweep changes.

## 6. Algorithm documentation (per CLAUDE.md)

The shore-clutter injector and the parametric generators are new algorithmic components and
get the four-section treatment in `docs/algorithms/` (math of the geometry/clutter model;
assumptions — fixed-position point process, seeded Bernoulli detection; rationale — why
fixed-position vs uniform-Poisson for shore; what-to-test-next). The learning chapter
`docs/learning/25-land-clutter-prior.md` gets a short addition pointing to the synthetic A/B
as the controlled validation of the real-data result. No new learning chapter is needed
(geometry generators are bench infrastructure, not a new tracking concept).

## 7. Testing

- **Unit (per builder):** truth count/order, measurement counts, determinism (same seed →
  identical measurements; different seed → different noise), geometry invariants (lane spacing,
  crossing angle, convoy ordering).
- **Shore clutter:** clutter points fixed across scans (same nominal positions); `source_id`
  is `sim_shore`; no `TruthSample` created for clutter; detection probability honoured
  statistically over many scans with a fixed seed.
- **Synthetic shore consistency:** every generated `clutter_enu_point` returns a high prior
  (≥ hard-gate threshold) from the scenario's own `CoastlineModel` (ENU→geodetic→prior
  round-trip); a point well offshore returns ~0.
- **Wiring/integration:** with `use_land_model=true`, Sweep builds the land model from
  `syntheticCoastline()` (not the path); the near-shore real target survives (lifetime_ratio
  preserved) while shore-clutter cardinality drops.
- **Determinism harness:** existing replay-twice test stays green (new scenarios deterministic).

## 8. Measurement plan (the deliverable)

Run the new scenarios under `imm_cv_ct_pmbm_coverage` (no land) vs `imm_cv_ct_pmbm_coverage_land`
(land), plus MHT for reference. Record in `evaluation-log.md`:

- Geometry scenarios: GOSPA / OSPA / id-switches / cardinality across the geometry sweep,
  PMBM vs MHT — does the breadth surface any regression the existing set misses?
- Shore-clutter scenarios: cardinality-error and false-track count with land OFF vs ON.
  **Success = land ON drives shore-clutter cardinality-error to ~0 without dropping any real
  target**, including the near-shore one. This is the perfect-truth confirmation of the
  philos real-data result.

Update `comparison-baselines.md` with the decision and where this lands on the Cl-2/Cl-3 axes.

## 9. Decisions

| # | Decision | Rationale |
|---|---|---|
| a | In-memory coastline (Option 2), not a committed GeoJSON fixture | Core `CoastlineModel` already takes a `CoastlineGeometry`; file-only was an artifact of the philos real-data path. One in-code shoreline used for both clutter and land model keeps "one shoreline, twice" trivially true, with no gitignore games. |
| b | Fixed-position shore clutter, not uniform-Poisson | Shore returns recur at the same physical spots scan-to-scan; that persistence is exactly what fools a tracker into births. Uniform-Poisson (existing `buildClutterCrossingScenario`) models a different phenomenon. |
| c | Fictitious coastal datum per shore scenario | The land model is datum-based; synthetic scenarios are datum-less today. A per-scenario datum makes ENU generation and geodetic land-query consistent by construction. |
| d | Reuse existing configs/metrics; no new tracker behaviour | This is measurement infrastructure. The only product change is the bench wiring hook. |
| e | Near-shore real-target validator included | Perfect truth uniquely lets us prove the inland-only hard gate protects a genuine waterline vessel — the "anchored ship" concern philos data could not settle. |

## 10. Risks

- **Round-trip frame error.** If the ENU→geodetic polygon construction and the land model's
  ENU→geodetic query disagree, the A/B is meaningless. Mitigated by the §7
  "Synthetic shore consistency" unit test (clutter points must read high prior
  from the scenario's own model; an offshore point must read ~0).
- **Over-fitting scenarios to the land model.** If shore clutter is placed only deep inland,
  the test is too easy. Mitigation: include some near-waterline clutter and the near-shore real
  target so the soft-ramp / hard-gate boundary is genuinely exercised.
- **Scope creep into a physics simulator.** Explicitly bounded: fixed-position point process,
  not RCS/multipath modelling.
```
