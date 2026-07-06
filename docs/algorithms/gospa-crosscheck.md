# GOSPA metric cross-validation (Stone Soup) — D2

Cross-check of navtracker's GOSPA implementation (`core/scenario/Gospa.hpp`)
against Dstl Stone Soup's independently-authored GOSPA. This is not a new
algorithm; it is an *integrity check* on an existing one. See the reference
for GOSPA itself: `docs/learning/20-tracker-metrics.md` and the equation in
`core/scenario/Gospa.hpp`.

## Why

Every promotion decision in this project hangs on GOSPA deltas. The harness
has twice had a **truth-fragmentation** bug silently corrupt those numbers
(autoferry 2026-06-10; harbor 2026-07-02 — both fixed). Both were faults in
how truth is *grouped/ordered* upstream of the metric kernel, not in the
kernel's arithmetic. An externally-authored metric that agrees with ours on
the same tracks is the cheapest possible hedge against a bug in the kernel
itself — the one part the two prior incidents never exercised.

## Math — the convention both sides share

GOSPA (Rahmathullah, García-Fernández, Svensson 2017), α = 2:

```
GOSPA(X,Y; c,p,α) = ( min_π Σ_{(i,j)∈π} d(x_i,y_j)^p
                      + (c^p/α)·(|X| + |Y| − 2|π|) )^(1/p)
```

with `d(x,y) = min(‖x−y‖, c)`. The two implementations match term for term:

| | navtracker (`Gospa.cpp`) | Stone Soup (`ospametric.py`) |
|---|---|---|
| α | default 2 | **hardcoded 2** in `compute_gospa_metric` |
| cardinality penalty | `c^p/α` per missed/false (`miss`) | `dummy_cost = c^p/α` |
| over-cutoff matched pair | Hungarian routes it to miss+false slots (cost `c^p`) | `const_cmp` check reclassifies it to missed+false |
| headline | `pow(total, 1/p)` | `distance = (loc+missed+false)^(1/p)` |
| decomposition space | pre-root power-p (`GospaComponents`) | pre-root power-p (`value` dict) |
| assignment | Hungarian (`core/association/Hungarian.hpp`) | its own optimal assignment |

Both compute the optimal assignment, so the *cost* is identical even where the
*assignment* is non-unique. Per-scan `compute_gospa_metric` adds **no**
switching penalty on either side (switching is a time-series-level term in
Stone Soup), so the point-set comparison is clean.

Parameters used: **c = 20 m, p = 2, α = 2, switching = 0** — pulled from the
harness (`MetricsParams::gospa_cutoff_m = 20`, `gospaComponents` defaults
p=α=2), not assumed.

## Assumptions

- Both scorers see the *same* positions. The export writes doubles at
  `max_digits10` precision; a truncated CSV would manufacture a spurious
  disagreement (this bit us once — the first export was 6 sig-figs and the
  round-trip diff was ~3e-5).
- Positions are 2-D ENU metres (the frame GOSPA is computed in). Stone Soup's
  default `Euclidean` measure over a `[[east],[north]]` state vector reproduces
  `(truth−est).norm()`.
- The exported `BenchResult` **is** the object the metrics consume — the
  cross-check is faithful by construction, not a reconstruction that could
  drift from the harness config.

## Rationale — why export from inside Sweep, not rebuild the tracker

The `imm_cv_ct_pmbm` tracker is assembled from ~160 lines of config-driven
wiring in `Sweep.cpp` (estimator, PMBM config, detection model, sensor
activity, optional land/obstacle/occupancy/bias). Reconstructing that in a
standalone tool would risk the exported tracks diverging from the scored
tracks — defeating the entire point of a cross-check. Instead a nullable
`SweepParams::export_states_dir` dumps the `BenchResult` in place, right after
`computeMetrics`. Unset → bit-identical behaviour; no consumer surface touched.

## Result (2026-07-06)

PASS on one sim + one real run (stronger than two sims):

| Run | Scans | mean GOSPA (ours / SS) | max per-scan \|Δ\| |
|-----|------:|-----------------------|-------------------|
| `harbor_complete_truth` (sim) | 40 | 49.528608 / 49.528608 | 1.42e-14 |
| `philos` (real ARPA replay) | 20 | 99.129014 / 99.129014 | 1.42e-14 |

Per-scan localisation/missed/false and recovered cardinality counts agree on
every scan. Max deviation 1.42e-14 m is floating-point ordering. The harness
GOSPA kernel is validated by an independent implementation.

## How to reproduce

```bash
# 1. one-time venv (Stone Soup is venv-local, NOT a Conan dependency)
python3 -m venv /tmp/claude/d2-stonesoup-venv
/tmp/claude/d2-stonesoup-venv/bin/pip install ./data/stonesoup/Stone-Soup

# 2. export a run's states + our per-scan GOSPA
cmake --build build --target navtracker_bench_baseline
./build/bench/navtracker_bench_baseline \
    --scenario-eq harbor_complete_truth --config-eq imm_cv_ct_pmbm \
    --seeds 1 --skip-replays --run-id d2 --out /tmp/d2 --export-states-dir /tmp/d2

# 3. re-score with Stone Soup and diff
/tmp/claude/d2-stonesoup-venv/bin/python tools/stonesoup_gospa_crosscheck.py \
    --states /tmp/d2/imm_cv_ct_pmbm__harbor_complete_truth__seed0.states.csv \
    --ours   /tmp/d2/imm_cv_ct_pmbm__harbor_complete_truth__seed0.ours_gospa.csv
# exits 0 on PASS (max |Δ| ≤ tol and no cardinality mismatch)
```

For the real arm, drop `--skip-replays` and use `--scenario-eq philos`.

## Ways to improve / what to test next

- **Time-series GOSPA + switching.** This check covers per-scan point-set
  GOSPA. Stone Soup's `MetricManager` also computes a switching term over
  tracks-with-identity; cross-checking that would validate `TGospa.cpp` too.
  Note the convention caveat: some implementations fold switching into the
  α=2 decomposition differently — document the convention before comparing
  numbers (a documented convention mismatch is a pass; a number mismatch under
  matched conventions is a bug hunt).
- **OSPA arm.** `Ospa.hpp` (the max(|X|,|Y|)-normalised sibling) is not yet
  cross-checked; Stone Soup's `OSPAMetric` subclass makes this a one-liner
  extension of the script.
- **Explicitly out of scope (parked):** running Stone Soup's own trackers
  (GLMB/JPDA/PHD) as a competing baseline — that is a tracker comparison, not
  a metric cross-check, and is real scope creep.
