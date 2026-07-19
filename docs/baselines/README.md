# Baseline benchmarks

This directory holds the labelled, machine-diffable artifacts produced
by the benchmark harness. Each `*.csv` is a single execution; the
matching `*.md` is its rendered companion; any `*_vs_*.md` is a
comparison between two or more runs.

The harness exists so improvements can be measured against a fixed
baseline rather than guessed at from scattered stdout. See the design
spec at `docs/superpowers/specs/2026-06-07-baseline-benchmark-harness-design.md`
for full schema, metric definitions, and architectural context.

## How to run a baseline

```bash
cmake --build build --target navtracker_bench_baseline -j

./build/bench/navtracker_bench_baseline \
    --run-id $(date +%Y-%m-%d)_baseline \
    --out docs/baselines/

./build/bench/navtracker_bench_render \
    docs/baselines/$(date +%Y-%m-%d)_baseline.csv

git add docs/baselines/$(date +%Y-%m-%d)_baseline.{csv,md}
git commit -m "Baseline benchmark: $(date +%Y-%m-%d)"
```

The full sweep is `5 configs × (7 synthetic × 10 seeds + 2 replays)` = 360 runs.
Replays read fixture files under `tests/fixtures/` and `data/`.

### Flags

- `--run-id ID` — identifier for this run; becomes the filename and the `run_id` column. Default: `run_<timestamp>`.
- `--out DIR` — output directory. Default: `docs/baselines/`.
- `--seeds N` — number of seeds per multi-seed scenario. Default: 10.
- `--skip-replays` — skip the philos and haxr replays. Useful for smoke testing when you don't want to depend on fixtures being present.

## How to compare runs

```bash
./build/bench/navtracker_bench_compare \
    docs/baselines/2026-06-07_baseline.csv \
    docs/baselines/2026-06-08_improv_a.csv \
    > docs/baselines/improv_a_vs_baseline.md
```

The output table joins on `(scenario, config, metric)`. Cells show
`baseline → new (±Δ <indicator>)` where:
- `▲` improvement,
- `▼` regression,
- `·` no change (`|Δ| < 1e-9`).

Sign convention is baked into the comparator: `lifetime_ratio` is
higher-is-better; everything else (OSPA, RMSE, breaks, switches) is
lower-is-better.

Three-or-more-way comparison works too — just pass more CSV paths.

## What's in a CSV

The first lines are `#`-prefixed provenance recording who/what/where:

```
# run_id: 2026-06-07_baseline
# started_at: 2026-06-07T10:14:22Z
# git_sha: abc1234 (clean)
# build_type: Release
# compiler: gcc 13.2.0
# host: linux x86_64
# seeds: [0,1,2,3,4,5,6,7,8,9]
# configs: 5
# scenarios: 9
# total_runs: 360
# elapsed_seconds: 184.2
```

Then a column header and data rows:

```
run_id,config,scenario,seed,metric,value,unit
2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_mean,18.2,m
2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_p95,15.0,m
...
```

One row per `(run_id, config, scenario, seed, metric)`. Long format —
adding a metric is a new value of the `metric` column, not a new column.
Files are never edited in place; compared runs sit as separate files.

## Configurations

Five named configurations. Each row differs from a neighbour along
exactly one axis, so when a metric moves the *axis responsible* is
identifiable.

| Label | Estimator | Motion model | Associator | Role |
|---|---|---|---|---|
| `ekf_cv_gnn` | EKF | ConstantVelocity2D | GNN(50.0) | Textbook baseline |
| `ekf_cv_jpda` | EKF | ConstantVelocity2D | JPDA(20.0, 0.9, 1e-4) | Isolates association upgrade |
| `ukf_cv_gnn` | UKF | ConstantVelocity2D | GNN(50.0) | Isolates estimator upgrade |
| `ukf_ct_gnn` | UKF | CoordinatedTurn(0.5, 0.1) | GNN(50.0) | Adds non-linear motion |
| `imm_cv_ct_jpda` | IMM(CV5State + CT) | — | JPDA(20.0, 0.9, 1e-4) | Most expressive |

Constants are lifted from the existing scenario tests (`test_crossing.cpp`,
`test_filter_comparison.cpp`, `test_jpda_comparison.cpp`). Source of truth
for the factories: `core/benchmark/Config.cpp`.

Adding a sixth row is a one-line registration in `defaultConfigs()`.

## Scenarios

**Synthetic (10 seeds each, multi-seed):**

| Label | Builder | What it stresses |
|---|---|---|
| `crossing` | `buildCrossingTargetsScenario` | Data association under track-crossing geometry |
| `overtaking` | `buildOvertakingScenario` | Sustained close-aboard, similar headings |
| `head_on` | `buildCrossingTargetsScenario` (anti-parallel) | Closing-rate extremes |
| `parallel_targets` | `buildParallelTargetsScenario` | Sustained ambiguity between two valid hypotheses |
| `ais_dropout` | `buildCrossingDropoutScenario` | Track continuity under sensor outage |
| `clock_skew` | `buildStraightLineScenario` + `applySkew(50 ms)` | Robustness to timestamp jitter |
| `non_cooperative` | `buildBearingOnlyScenario` | Bearing-only initiation; range unobservable |

**Replay (single seed, file-driven):**

| Label | Source |
|---|---|
| `philos` | `tests/fixtures/philos/out/ais_ferry_near/` (own-ship + AIS + radar plots) |
| `haxr` | `tests/fixtures/haxr_cfar/out/kattwyk_08_t40.csv` + `data/dlr/` truth |

Source of truth for the adapters: `adapters/benchmark/SimScenarioRun.cpp`,
`adapters/benchmark/ReplayScenarioRun.cpp`.

## Metrics

Eight columns, defined once in `core/benchmark/Metrics.cpp` and reused
across every comparison.

- `ospa_mean` / `ospa_p95` (m) — mean and 95th percentile of per-step OSPA at `cutoff = 500 m`. Standard MTT accuracy metric.
- `lifetime_ratio` (ratio, [0,1]) — mean across truths of `|{t : a(t) ≠ ∅}| / |T|` under a 100 m assignment gate. Higher is better.
- `track_breaks` (count) — mean number of maximal unassigned intervals per truth track.
- `id_switches` (count) — mean number of assigned-track-id changes per truth track.
- `pos_rmse_m` / `sog_rmse_mps` / `cog_rmse_deg` — per-track decomposed error using the same assignment as continuity. Lets you see whether an improvement came from position fit, velocity, or course.

Full math (and the "improve next" list per metric) lives in the
`// Math / Assumptions / Rationale / Improve next` blocks above each
function in `core/benchmark/Metrics.cpp`.

## Determinism

Same code + same seeds + same scenarios ⇒ bit-identical CSV rows.
Tested by `BenchDeterminism.RepeatedSweepProducesIdenticalRows`. If a
future change breaks this, that test fails and the cause should be
fixed at the source, not papered over.

## Real-world AutoFerry baseline (`2026-06-08_autoferry`)

The first baseline to include true-ground-truth real-world data: the nine
AutoFerry `milliAmpere` scenarios (see `data/README.md`), plus `philos`,
across all 9 configs. `haxr` is excluded by default (302k plots /
~169-per-scan — the full-enumeration JPDA and MHT configs are intractable
on it without cluster decomposition; pass `--with-haxr` to force it).

Two AutoFerry baselines exist:
- `2026-06-08_autoferry` — **active sensors only** (radar+lidar → Position2D).
- `2026-06-09_autoferry_4sensor` — **all four sensors**: active + EO/IR
  (passive → Bearing2D). Apples-to-apples vs the paper. Side-by-side:
  `autoferry_active_vs_4sensor.md`.

Passive bearings *refine* tracks but never *initiate* one (range is
unobservable from a single look); the track-birth paths drop non-gating
bearing measurements (`canInitiateTrack`), matching the paper's §4.4.1
("only active sensors are used in track initialization").

**Headline finding — the synthetic bench was hiding the real problem.**
On clean synthetic scenarios OSPA is 12–20 m and id_switches ≈ 0. On raw
AutoFerry detections the *same* configs report (mean over the 9 scenarios):

| config | ospa_mean (m) | lifetime | id_switches | pos_rmse (m) |
|---|---|---|---|---|
| ekf_cv_gnn | 469 | 0.91 | 2782 | 11.8 |
| imm_cv_ct_jpda | 460 | 0.91 | 2775 | 11.3 |
| imm_cv_ct_noisy_jpda | 458 | 0.91 | 2723 | 11.8 |
| ekf_cv_mht | 409 | 0.81 | 2109 | 17.4 |
| imm_cv_ct_mht | 407 | 0.81 | 2029 | 17.6 |

Read this carefully:

- **`pos_rmse_m` is 11–18 m** — when a track is assigned to a truth, the
  kinematic estimate is *good*. The estimators (EKF/UKF/IMM) work on real
  noise.
- **OSPA ≈ 460 m (near the 500 m cutoff) and id_switches in the thousands**
  — the cardinality/labelling side is swamped. The bench scores only
  *Confirmed* tracks, so this means **hundreds of clutter false-alarms
  survive M-of-N confirmation and confirm into tracks.** Real port/sea
  clutter, which the synthetic harness never modelled at this density,
  overwhelms the consecutive-count `TrackManager`.
- **Estimator choice barely moves the needle** (IMM vs EKF: ~460 vs ~469);
  **MHT helps** (score-based tree deletion suppresses some clutter:
  OSPA 409 vs 469, id_switches 2109 vs 2782) but fragments tracks
  (track_breaks up, lifetime 0.81 vs 0.91).

**Conclusion.** The binding real-world constraint is **not** the estimator
or the motion model — it is **clutter-robust track lifecycle**. This is
direct empirical support for the top two items in
`docs/algorithms/sota-roadmap.md`: SPRT/LLR score-based confirmation (#1)
and JIPDA existence probability (#2). It also re-confirms the JPDA EHM gap
(#3): `JpdaAssociator` now caps joint-event enumeration at 1e6 and falls
back to greedy hard assignment per overflowing cluster
(`AssociationResult::overflow_fallback`), which is why the JPDA configs ran
at all on this data.

**Adding EO/IR (four-sensor) makes it *worse*, and that's the second
finding.** Enabling the passive bearings (scenario2: 849 active +
**3390 bearings**, the majority of the data) moves the numbers the wrong
way (mean over 9 scenarios, active → four-sensor):

| config | OSPA | pos_rmse (m) |
|---|---|---|
| ekf_cv_gnn | 469 → 477 | 11.8 → 14.3 |
| imm_cv_ct_jpda | 460 → 476 | 11.3 → 15.3 |
| ekf_cv_mht | 409 → 467 | 17.4 → 26.3 |
| imm_cv_ct_mht | 407 → 462 | 17.6 → 26.9 |

This is **not** a loader bug — the bearing convention is verified correct
(residual to the true target is zero-mean, 0.49° ± 4.7°). The cause is the
data: **27% of EO/IR detections are clutter** (>15° from any real target),
and the remaining real bearings carry ~4.7° noise (8–33 m cross-range at
typical 100–400 m ranges). Fed through naive Gaussian hard-gated updates,
the clutter bearings corrupt tracks and the noisy ones pull position. The
tracker has neither outlier-robust updates nor existence-aware association
to benefit from a heavy-clutter passive sensor.

**Conclusion (both findings).** More sensors ≠ better here. The binding
constraints, in priority order, are exactly the top `sota-roadmap.md`
items: (1) **clutter-robust track lifecycle** — SPRT/LLR confirmation to
stop false-alarms confirming; (2) **JIPDA existence probability** —
existence-aware association so passive clutter doesn't corrupt tracks;
(5) **VB-adaptive Student-t robust updates** — so heavy-tailed EO/IR
bearings down-weight outliers instead of pulling the estimate. The
estimator/motion-model layer (EKF/UKF/IMM/MHT) is *not* the bottleneck:
on assigned real targets it tracks to single-digit metres (4.4 m on the
unobscured target in scenario2). The dataset's value is precisely that it
exposes this — the synthetic harness never did.

The `sog_rmse_mps` / `cog_rmse_deg` columns are not meaningful here
(position-only ground truth — same caveat as the other replays).

## Known caveats (read before drawing conclusions)

These are honest limitations of the *current* baseline, not bugs in
the harness. They will show up in baseline numbers and should be
factored into how you read a comparison.

1. **Velocity slicing assumes CV2D state layout `[px, py, vx, vy]`.**
   `BenchRunner` reads `state(0..1)` for position and `state(2..3)` for
   velocity. This holds for `ekf_cv_gnn`, `ekf_cv_jpda`, `ukf_cv_gnn`.
   For configs that use other layouts (`ukf_ct_gnn` uses `CoordinatedTurn`,
   `imm_cv_ct_jpda` uses `ConstantVelocity5State` + CT), velocity-derived
   metrics (`sog_rmse_mps`, `cog_rmse_deg`) will be reported in the CV2D
   convention and may be off. Position-only metrics (OSPA, `pos_rmse_m`,
   continuity, ID switches) are correct for all configs.

2. **Standalone `CoordinatedTurn` does not update its `omega`.**
   The CT motion model expects `omega_` to be written externally by an
   IMM during mixing. Used standalone (the `ukf_ct_gnn` config), `omega`
   stays at whatever it was last set to (0 by default). In practice this
   means `ukf_ct_gnn` and `ukf_cv_gnn` may produce very similar numbers
   — that's an honest reflection of current behaviour, not a harness
   bug. The IMM config (`imm_cv_ct_jpda`) drives CT correctly.

3. **JPDA configs now work end-to-end.** Two bugs surfaced and were
   fixed during baseline construction:
   - `BenchRunner` was using `Tracker::process()` (hard-match only)
     instead of `processBatch()`. Fixed in commit `7dbe9ae`.
   - `ImmEstimator` inherited the no-op `IEstimator::softUpdate`
     default, so JPDA + IMM tracks never folded in any measurement.
     Implemented standard IMM-PDA in commit `77ab6f4`.

   After both fixes: `ekf_cv_jpda` lands close to the GNN baselines
   (~20 m OSPA on crossing) and `imm_cv_ct_jpda` is the top-of-table
   tracker on `ais_dropout` — exactly where its multi-model + soft-
   association combination should pay off most.

4. **Philos replay uses AIS as both measurement *and* truth source.**
   The existing `tests/replay/test_philos_ospa.cpp` set this convention
   (no separate radar-truth loader exists). It scores consistency, not
   absolute accuracy. The harness inherits this convention. The
   `radar_truth.csv` fixture exists but has no loader.

5. **No CSV quoting.** The reader and writer assume that no field
   contains a comma. Today this holds (labels are kebab-case ASCII,
   units are plain). If a future config name introduces a comma, that
   will break the reader silently — needs proper CSV quoting then.

6. **`git_sha`, `compiler`, and `host` are populated (W6.3.2).** `git_sha`
   is captured by CMake at **configure** time (`bench/CMakeLists.txt`) as
   `<short-sha> (clean|dirty)`, where dirty tracks uncommitted changes to
   **tracked** files only; `compiler` and `host` are read from the build
   at compile/run time. Because the sha is a configure-time snapshot,
   **re-run `cmake` before generating a baseline** you want stamped with a
   fresh sha (a rebuild alone will not refresh it). If git is unavailable
   or this is not a checkout, the field falls back to `unknown`. Existing
   dated snapshots are immutable and are NOT retro-stamped.

7. **The first committed baseline (`2026-06-07_baseline.csv`) runs only
   the 7 synthetic scenarios** (5 × 7 × 10 × 8 = 2800 rows). The
   replays (`philos`, `haxr`) were skipped because the full sweep
   doesn't complete within a reasonable wall-clock budget when each of
   the 5 configs re-loads both replays from disk. The unit tests load
   them in ~360 ms, but the per-config repeat in the sweep multiplies
   that. A follow-up should cache the replay `Scenario` per scenario
   instance (load once, replay-by-reference into each config) and then
   re-baseline with replays included. Until that lands, the
   `--skip-replays` flag is the recommended default.

## File layout

```
docs/baselines/
  README.md                              this file
  <run-id>.csv                           one CSV per run
  <run-id>.md                            rendered Markdown companion
  <run-a>_vs_<run-b>.md                  comparison output (optional)
```
