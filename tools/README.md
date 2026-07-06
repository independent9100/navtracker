# tools/

Offline diagnostic scripts. Not part of the build. Python 3 only,
stdlib-only **except** `stonesoup_gospa_crosscheck.py`, which needs a
venv with Dstl Stone Soup (see its section below).

## `autoferry_r_calibration.py`

Computes empirical per-sensor noise statistics from the AutoFerry
fixture by pairing each detection to its nearest ground-truth target
and reporting the residual variance. Used to set the `R` defaults in
`adapters/replay/AutoferryJsonReplay.cpp::AutoferryLoadOptions`
(item 12 (a)).

Run:

```bash
python3 tools/autoferry_r_calibration.py
```

Output: per-scenario kept/gated counts, then pooled and per-env tables
of `(mean_n, mean_e, sigma_iso) vs configured R, with ratio`. A ratio
much larger than 1.0 means R is currently too tight; the filter is
overconfident.

The mean column is itself diagnostic — a non-zero mean residual is a
**registration bias**, which is what `SensorBiasEstimator` (item 9)
estimates online. The variance and the bias are separate parameters;
this script informs the *variance* part.

## `autoferry_q_calibration.py`

Computes empirical process-noise PSDs (acceleration and turn rate)
from each scenario's ground-truth trajectory. Used to validate the
IMM CV / CT `Q` values in `core/benchmark/Config.cpp` (item 12 (c)).

Run:

```bash
python3 tools/autoferry_q_calibration.py
```

Output: per-scenario, per-env, and pooled `(N, σ_a, PSD_a, σ_ω, PSD_ω)`
versus the configured PSDs in the canonical IMM (CV + CT) config.

Caveat: the truth file repeats positions verbatim between RTK fixes,
so naive central differencing on consecutive samples produces wild
spikes. The script uses a ≥0.5 s differentiation baseline and gates
out turn-rate samples below a minimum speed (heading undefined when
stationary).

## `stonesoup_gospa_crosscheck.py` (D2 — metric integrity)

Re-scores a bench run's per-scan `(truth, track)` sets with Dstl
Stone Soup's independently-authored GOSPA and diffs, scan by scan,
against navtracker's own per-scan GOSPA. An external agreement is the
cheapest hedge against a metric-kernel bug — see
`docs/algorithms/gospa-crosscheck.md` for the convention table and the
2026-07-06 result (navtracker == Stone Soup to floating-point epsilon
on one sim + one real run).

**Needs a venv** (Stone Soup is not a Conan dependency; it lives at
`data/stonesoup/Stone-Soup/`):

```bash
python3 -m venv /tmp/claude/d2-stonesoup-venv
/tmp/claude/d2-stonesoup-venv/bin/pip install ./data/stonesoup/Stone-Soup
```

Inputs come from the C++ exporter (`core/benchmark/GospaExport`), driven
by `navtracker_bench_baseline --export-states-dir DIR`:

```bash
cmake --build build --target navtracker_bench_baseline
./build/bench/navtracker_bench_baseline \
    --scenario-eq harbor_complete_truth --config-eq imm_cv_ct_pmbm \
    --seeds 1 --skip-replays --run-id d2 --out /tmp/d2 --export-states-dir /tmp/d2
/tmp/claude/d2-stonesoup-venv/bin/python tools/stonesoup_gospa_crosscheck.py \
    --states /tmp/d2/imm_cv_ct_pmbm__harbor_complete_truth__seed0.states.csv \
    --ours   /tmp/d2/imm_cv_ct_pmbm__harbor_complete_truth__seed0.ours_gospa.csv
```

Exits 0 on PASS (max per-scan `|Δ|` ≤ `--tol`, default 1e-6 m, and no
cardinality mismatch), 1 otherwise, printing the offending scans.
