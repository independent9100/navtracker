# tools/

Offline diagnostic scripts. Not part of the build. Python 3 only,
no extra dependencies (stdlib).

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
