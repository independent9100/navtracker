# Baseline Benchmark Harness — Design Spec

**Date:** 2026-06-07
**Status:** Draft — pending user review

## Problem

Today the repo contains multiple ad-hoc comparison tests
(`tests/scenario/test_filter_comparison.cpp`,
`test_jpda_comparison.cpp`, `test_mht_comparison.cpp`,
`test_multi_seed_sweep.cpp`) and two replay-OSPA tests
(`tests/replay/test_philos_ospa.cpp`, `test_haxr_ospa.cpp`). Each
prints metrics to stdout in its own ad-hoc format. There is **no
single artifact** that records "approach X scored Y on scenario Z under
conditions W." Without such an artifact, comparing a future
improvement against the current behaviour reduces to eyeballing
unlabelled numbers — exactly the situation that motivates this work.

## Goal

Establish a **baseline benchmark harness** that:

1. Sweeps a fixed, named set of tracking configurations across a
   fixed set of synthetic scenarios and real-world replays.
2. Records every metric in a single labelled CSV artifact whose
   schema makes the *(configuration × scenario × seed × metric)*
   tuple explicit on every row.
3. Renders the CSV into a human-readable Markdown table.
4. Lets future runs (improvement A, improvement B) produce CSVs
   with the same schema, so a comparison is a join — not a guess.

Improvements are not in scope here. The harness is.

## Non-goals

- Benchmarking ParticleFilter (excluded from the initial config
  set; seed-by-seed variance handled separately later).
- Wiring new replay datasets (`pohang`, `dlr`, `marinecadastre`,
  `kystverket`). Out of scope — only `philos` and `haxr` are
  benchmarked.
- Runtime / wall-clock metrics. The first baseline measures only
  accuracy, continuity, and per-track state error.
- Deleting or modifying existing comparison tests. They stay; the
  harness lives alongside.

## Architecture

The harness follows the repo's hexagonal layout:

```
core/benchmark/                 pure domain, no I/O
  Config.hpp                    named configuration:
                                  estimator factory + associator factory + label
  ScenarioRun.hpp               abstract: yields truth tracks + measurements;
                                  source-agnostic (sim vs replay)
  Sweep.hpp/.cpp                the matrix loop:
                                  for each (config, scenario, seed) -> run -> Metrics
  Metrics.hpp/.cpp              OSPA, continuity (lifetime ratio, breaks, id switches),
                                  RMSE (pos/vel/heading) — single assignment function
                                  shared across all metrics
  CsvWriter.hpp/.cpp            one row per (run_id, config, scenario, seed, metric)

adapters/benchmark/             I/O adapters that feed Sweep
  SimScenarioRun.hpp/.cpp       wraps sim/ emitters + TruthTrajectory
  ReplayScenarioRun.hpp/.cpp    wraps existing replay loaders (philos, haxr)

bench/
  baseline_matrix.cpp           ~50 LOC: build the matrix, call Sweep::run,
                                  write CSV
  render_markdown.cpp           reads CSV -> writes Markdown companion
  CMakeLists.txt                executables: navtracker_bench_baseline,
                                  navtracker_bench_render

docs/baselines/
  README.md                     what the matrix is, how to run, how to read
  2026-06-07_baseline.csv       committed baseline data
  2026-06-07_baseline.md        rendered companion
```

### Invariants preserved

- `core/benchmark/` has **zero I/O**, same rule as the rest of the
  domain core. Scenario sources are ports (`ScenarioRun`); `sim`
  and `replay` are adapters.
- Source dependencies point inward: `bench/` depends on `adapters/`
  depends on `core/`. Never the reverse.
- Estimator and associator strategies remain swappable through the
  same factory pattern the runtime uses.

### Note on existing build wiring

The `core/scenario/` directory (`Ospa.cpp`, `Harness.cpp`,
`Metrics.cpp`, etc.) is currently compiled into `navtracker_core` —
meaning runtime consumers get scenario-test utilities they didn't
ask for. This is pre-existing tech debt, not introduced by this
spec. The benchmark harness reuses those utilities cleanly. A
future cleanup could move them into a separate library; out of
scope here.

## Configurations

Five named configurations form the baseline matrix. Each row
differs from a neighbour along exactly one axis, so when a metric
moves the *axis responsible* is identifiable.

| Label | Estimator | Motion model | Associator | Diagnostic role |
|---|---|---|---|---|
| `ekf_cv_gnn` | EKF | ConstantVelocity2D | GNN | Textbook baseline; the floor every other config must beat |
| `ekf_cv_jpda` | EKF | ConstantVelocity2D | JPDA | Isolates the association upgrade (GNN → JPDA) |
| `ukf_cv_gnn` | UKF | ConstantVelocity2D | GNN | Isolates the estimator upgrade (EKF → UKF) |
| `ukf_ct_gnn` | UKF | CoordinatedTurn | GNN | Adds non-linear motion model — for crossing/turning |
| `imm_cv_ct_jpda` | IMM(CV+CT) | — | JPDA | Most expressive config in tree today |

Adding a sixth row is a one-line registration in
`bench/baseline_matrix.cpp`. The factory pattern keeps the
matrix declarative.

ParticleFilter is deliberately excluded from the initial baseline:
sampling variance makes seed-by-seed comparison noisier than the
other estimators, and the right way to baseline it is a separate
multi-seed study at higher N. To be revisited.

## Scenarios

Two sources, both implementing the `ScenarioRun` port.

### Synthetic (multi-seed)

Each run uses seeds `[0..9]` (10 seeds). Cell value reported as
`mean ± stddev` per metric.

| Scenario | What it stresses |
|---|---|
| `crossing` | Data association under track-crossing geometry |
| `overtaking` | Sustained close-aboard, similar headings |
| `head_on` | Closing-rate extremes, CPA coverage |
| `parallel_targets` | Sustained ambiguity between two valid hypotheses |
| `ais_dropout` | Sensor outage handling; track continuity under missing data |
| `clock_skew` | Robustness to timestamp skew at fusion |
| `non_cooperative` | Radar/EO-IR only target (no AIS); bearing-only initiation |

### Replay (single seed; deterministic file input)

| Replay | Source |
|---|---|
| `philos` | Existing `test_philos_ospa.cpp` loader |
| `haxr` | Existing `test_haxr_ospa.cpp` loader |

### Matrix size

`7 synthetic × 10 seeds + 2 replays × 1 seed = 72 runs per config × 5 configs = 360 runs per benchmark execution.`

## Metric definitions

Each metric is defined once in `core/benchmark/Metrics.{hpp,cpp}`
and consumed by both the bench binary and (optionally, in future)
by the existing comparison tests if they choose to call it.
Following the four-part documentation standard from `CLAUDE.md`.

### OSPA — `ospa_mean`, `ospa_p95`

- **Math.** For each evaluation timestep `t`, compute the OSPA
  distance between the set of confirmed track positions
  `{x̂_i(t)}` and the truth set `{x_j(t)}` with cutoff
  `c = 500 m` and order `p = 2`. Report mean and p95 over
  evaluation steps.
- **Assumptions.** Evaluation timesteps are aligned to truth times
  (1 Hz for synthetic; native truth rate for replay).
- **Rationale.** Standard MTT metric; jointly penalises
  cardinality and localisation error. A config cannot reduce OSPA
  by silently dropping hard tracks.
- **Improve next.** Try OSPA(2) (window-based, penalises ID
  switches directly). Defer until baseline lands.

### Track continuity — `lifetime_ratio`, `track_breaks`, `id_switches`

- **Math.** Per truth track *j*, define the assignment
  `a(j,t)` = track id holding *j* at *t*, computed via per-step
  Hungarian matching under a gate `d_assoc = 100 m`. Then:
  - `lifetime_ratio_j = |{t : a(j,t) ≠ ∅}| / |T_j|`
  - `track_breaks_j` = number of maximal intervals where `a(j,t) = ∅`
  - `id_switches_j` = number of times `a(j,t) ≠ a(j,t−1)` while
    both are non-empty
  Aggregate as means across truth tracks, then across seeds.
- **Assumptions.** Truth tracks have stable identity; gate is wide
  enough to bridge brief noise excursions but tight enough not to
  merge distinct truths.
- **Rationale.** `CLAUDE.md` names ID stability as an
  architectural guarantee; OSPA alone misses silent ID churn that
  doesn't change cardinality.
- **Improve next.** Replace per-step Hungarian with longest-common-
  subsequence over the run — better at handling brief
  swap-then-swap-back artefacts.

### Per-track state RMSE — `pos_rmse_m`, `sog_rmse_mps`, `cog_rmse_deg`

- **Math.** For each truth track *j*, over timesteps where it's
  assigned to some track *i* by the same Hungarian function as
  above, compute RMSE of `(x̂_i − x_j)` in metres, `(SOG_i − SOG_j)`
  in m/s, and wrapped angular difference
  `wrap(COG_i − COG_j) ∈ (−180°, 180°]` in degrees. Aggregate per truth track (mean, not pooled), then
  across seeds.
- **Assumptions.** The same assignment used by continuity metrics
  defines the estimate↔truth pairs — one assignment function,
  consistent semantics across all metrics.
- **Rationale.** Decomposes OSPA so an improvement's source is
  visible: position fit, velocity, or course.
- **Improve next.** Add NEES / NIS consistency to check whether
  reported covariances are calibrated.

## CSV schema

One CSV per benchmark run. **Long format**, one row per
*(config × scenario × seed × metric)*. New metrics are new values
of the `metric` column, not new columns.

```
run_id,config,scenario,seed,metric,value,unit
2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_mean,73.42,m
2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_p95,184.10,m
2026-06-07_baseline,ekf_cv_gnn,crossing,0,lifetime_ratio,0.94,ratio
2026-06-07_baseline,ekf_cv_gnn,crossing,0,track_breaks,0.20,count
2026-06-07_baseline,ekf_cv_gnn,crossing,0,id_switches,0.10,count
2026-06-07_baseline,ekf_cv_gnn,crossing,0,pos_rmse_m,18.7,m
2026-06-07_baseline,ekf_cv_gnn,crossing,0,sog_rmse_mps,0.31,m/s
2026-06-07_baseline,ekf_cv_gnn,crossing,0,cog_rmse_deg,2.4,deg
```

Schema rules:

- `run_id` — single string identifying the whole run
  (e.g. `2026-06-07_baseline`, `2026-06-08_improvement_a`).
- `seed` — `0..N-1` for synthetics, `0` for replays.
- `unit` — recorded per row; never implicit. No silent unit
  reinterpretation across runs.
- **One file per run, never edited in place.** Compared runs sit
  as separate files; comparison is a join on
  `(config, scenario, metric)`.

### CSV header comment block

Every CSV begins with `#`-prefixed lines (parsers skip them)
recording run provenance:

```
# run_id: 2026-06-07_baseline
# started_at: 2026-06-07T10:14:22Z
# git_sha: 7a3c1f2 (clean)
# build_type: Release
# compiler: gcc 13.2.0
# host: linux x86_64
# seeds: [0,1,2,3,4,5,6,7,8,9]
# configs: 5
# scenarios: 9
# total_runs: 360
# elapsed_seconds: 184.2
```

A baseline checked into the repo is thus fully self-describing:
what code, what configuration, what scenario, what seed, what
number, what unit.

## Markdown rendering

`render_markdown.cpp` reads a CSV and writes a sibling `.md`. One
section per scenario, one table per scenario: rows are configs,
columns are metrics, cells are `mean ± stddev` aggregated across
seeds.

```
## crossing  (10 seeds)

| config           | ospa_mean (m) | ospa_p95 (m) | lifetime_ratio | id_switches | pos_rmse (m) | sog_rmse (m/s) | cog_rmse (deg) |
|------------------|---------------|--------------|----------------|-------------|--------------|----------------|----------------|
| ekf_cv_gnn       | 73.4 ± 4.1    | 184.1 ± 22.0 | 0.94 ± 0.02    | 0.10 ± 0.30 | 18.7 ± 1.1   | 0.31 ± 0.04    | 2.4 ± 0.3      |
| ekf_cv_jpda      | 68.2 ± 3.6    | 161.5 ± 18.4 | 0.96 ± 0.01    | 0.05 ± 0.22 | 17.0 ± 0.9   | 0.30 ± 0.04    | 2.2 ± 0.3      |
| ukf_cv_gnn       | 72.9 ± 4.0    | 181.7 ± 21.0 | 0.94 ± 0.02    | 0.10 ± 0.30 | 18.5 ± 1.0   | 0.30 ± 0.03    | 2.4 ± 0.3      |
| ukf_ct_gnn       | 64.1 ± 3.2    | 152.3 ± 17.5 | 0.97 ± 0.01    | 0.00 ± 0.00 | 16.2 ± 0.8   | 0.28 ± 0.03    | 1.9 ± 0.2      |
| imm_cv_ct_jpda   | 60.7 ± 2.9    | 141.0 ± 15.8 | 0.98 ± 0.01    | 0.00 ± 0.00 | 15.1 ± 0.7   | 0.27 ± 0.03    | 1.7 ± 0.2      |
```

The rendered Markdown also mirrors the CSV header block as a
top-of-document summary.

## Comparison tool

A small `compare` utility (C++ in `bench/compare.cpp`, no Python
to avoid adding a runtime dep) takes N CSV paths and emits a
single Markdown diff:

```
bench/compare \
  docs/baselines/2026-06-07_baseline.csv \
  docs/baselines/2026-06-08_improv_a.csv \
  > docs/baselines/improv_a_vs_baseline.md
```

Output has the same table shape as the rendered Markdown, but
cells are `baseline → new (±Δ)` with a directional indicator
(`▲` improvement, `▼` regression — sign convention per metric is
defined in `bench/compare.cpp` so it's not metric-by-metric
guesswork). Improvements and regressions are visually obvious;
every number remains tied to its
`(config × scenario × metric × unit)` address.

## Determinism

The benchmark inherits the repo-wide determinism guarantee.

- **Seeds.** Every synthetic scenario takes `seed ∈ [0..9]`. Same
  seed ⇒ identical truth, identical noise, identical injection
  order. The existing `SimulatedSensorBus` already sorts by
  timestamp; we keep that.
- **Stochastic configs.** Sampling inside JPDA event enumeration
  (and PF later) gets an RNG seeded from
  `hash(scenario_seed, config_label)`. Two configs at the same
  scenario seed don't share an RNG.
- **Replay.** File-driven by construction. A `--check-determinism`
  flag re-runs the matrix and hashes CSV rows (excluding wall-clock
  fields); mismatch fails the run.
- **No wall-clock anywhere** except the `started_at` field in the
  CSV header. The engine advances on message timestamps only.
- **Build/host capture.** The CSV header records git SHA (with a
  dirty marker if working tree ≠ HEAD), `CMAKE_BUILD_TYPE`,
  compiler version, host OS — enough provenance to reproduce.

## Workflow

The benchmark is intended to be run by hand at decision points,
not on every commit. Typical lifecycle:

### 1. Establish the baseline (today)

```
bench/baseline_matrix --run-id 2026-06-07_baseline \
                      --out docs/baselines/
bench/render_markdown docs/baselines/2026-06-07_baseline.csv
git add docs/baselines/2026-06-07_baseline.{csv,md}
git commit -m "Baseline benchmark: 2026-06-07"
```

The CSV + MD are now the frozen baseline, versioned in the repo.

### 2. Implement improvement A on a branch. Then:

```
bench/baseline_matrix --run-id 2026-06-08_improv_a \
                      --out docs/baselines/
bench/render_markdown docs/baselines/2026-06-08_improv_a.csv
git add docs/baselines/2026-06-08_improv_a.{csv,md}
git commit -m "Improvement A benchmark"
bench/compare docs/baselines/2026-06-07_baseline.csv \
              docs/baselines/2026-06-08_improv_a.csv \
              > docs/baselines/improv_a_vs_baseline.md
```

`improv_a_vs_baseline.md` is the artifact that decides whether the
improvement is worth keeping.

### 3. Same for improvement B; optional three-way comparison

```
bench/compare docs/baselines/2026-06-07_baseline.csv \
              docs/baselines/2026-06-08_improv_a.csv \
              docs/baselines/2026-06-09_improv_b.csv \
              > docs/baselines/three_way.md
```

Configs and scenarios remain fixed across runs. Adding a new
config means re-running *every* benchmark in the comparison set so
the schema stays joinable.

## Testing the harness itself

Tests live under `tests/benchmark/`. The harness must be at least
as trustworthy as the code it measures.

- **Unit — `Metrics`:**
  - OSPA on hand-crafted truth/track sets with closed-form answers.
  - Continuity: synthetic ID-switch sequence, verify
    `id_switches` and `track_breaks` count correctly.
  - RMSE on linear trajectories with known Gaussian noise.
- **Integration — bench binary:** run on a single tiny scenario,
  single config, 2 seeds; assert CSV has expected row count
  (`1 config × 1 scenario × 2 seeds × 8 metrics = 16 rows`),
  expected schema, expected header fields.
- **Determinism:** run the bench twice with identical args; hash
  CSV row contents (excluding header `started_at`); must match
  exactly.
- **Boundary:** existing scenario and replay tests remain green —
  confirming `core/benchmark/` doesn't perturb the libraries it
  reuses.

No modifications to production code are required to add the
harness. That is itself an invariant of the design.

## Risks and open questions

- **`core/scenario/` shipped in `navtracker_core`.** Pre-existing;
  the harness reuses these utilities. Not introduced or worsened
  by this work. Out of scope to refactor here.
- **JPDA stochasticity.** If JPDA event enumeration draws random
  samples in the current implementation, the
  `hash(scenario_seed, config_label)` RNG seeding scheme keeps
  things reproducible but means JPDA and GNN rows for the same
  scenario seed see different noise *inside the estimator*. Truth
  and sensor measurements stay identical; only internal-to-config
  randomness diverges. Documented behaviour, not a bug.
- **Scenario clock alignment for OSPA.** Evaluation grid for OSPA
  assumed at 1 Hz for synthetics. Real replays have native truth
  rates — `philos` and `haxr` use whatever rate their loaders
  emit. Documented per replay in `docs/baselines/README.md`.
- **PF deferred.** Not in initial baseline; revisited as a
  separate study once baseline is in place.

## Out of scope (explicit)

- New replay datasets (`pohang`, `dlr`, `marinecadastre`,
  `kystverket`). Wiring these is a separate, larger effort.
- Runtime / wall-clock metrics.
- Removing or modifying existing comparison tests in
  `tests/scenario/`.
- Refactoring `core/scenario/` out of `navtracker_core`.
- A CI integration that auto-runs the benchmark. Intentionally a
  manual command — full sweep takes minutes and is run at
  decision points, not on every commit.

## Files touched (anticipated)

- New: `core/benchmark/{Config,ScenarioRun,Sweep,Metrics,CsvWriter}.{hpp,cpp}`
- New: `adapters/benchmark/{SimScenarioRun,ReplayScenarioRun}.{hpp,cpp}`
- New: `bench/{baseline_matrix,render_markdown,compare}.cpp`,
  `bench/CMakeLists.txt`
- New: `tests/benchmark/test_metrics.cpp`,
  `test_bench_integration.cpp`, `test_bench_determinism.cpp`
- Modified: top-level `CMakeLists.txt` to add `navtracker_benchmark`
  library and three bench executables; wire tests into
  `navtracker_tests`.
- New: `docs/baselines/README.md`, `docs/baselines/2026-06-07_baseline.csv`,
  `docs/baselines/2026-06-07_baseline.md` (produced by first run).
