# Helgesen et al. 2022 — paper reference + comparison

Helgesen, Vasstein, Brekke, Stahl, "Heterogeneous multi-sensor tracking
for an autonomous surface vehicle in a littoral environment",
*Ocean Engineering* 252 (2022) 111168.
[ScienceDirect (paywalled)](https://www.sciencedirect.com/science/article/pii/S0029801822005753).
Local PDF: `docs/references/S0029801822005753-helgesen-2022-heterogeneous-multisensor-littoral.pdf`.

Dataset: <https://github.com/Autoferry/sensor_fusion_dataset>.

## Paper methodology (extracted from §5.8 and §7)

- **GOSPA** with `c = 20 m`, `p = α = 2`, reported **as RMS** across
  time steps (paper §5.8 lines 935-938, Tables 6/7 captions). The 20 m
  cutoff is chosen to match the track-truth assignment threshold used
  elsewhere in the paper.
- Tracker: **asynchronous multi-sensor VIMM-JIPDA**. Their full-fusion
  headline configuration is `L,R,IR,EO` = lidar + radar + EO cameras +
  IR cameras (the same physical sensors navtracker's autoferry replay
  exposes).
- Aggregation: results reported **per environment**, not per scenario.
  Env 1 = scenarios 2-6 (open water). Env 2 = scenarios 13/16/17/22
  (urban channel).
- posRMSE reported per target (Havfruen ; Gunnerus in env 1;
  Havfruen ; Jetboat in env 2).
- "Break.L" is break *length* in seconds (cumulative duration of
  track-loss intervals across the env). We do not have a directly
  comparable metric — `track_breaks` is *count* of break intervals
  per truth.

navtracker side: bench run
`docs/baselines/gospa20m_20260613T174620Z.csv`, single seed, canonical
config `imm_cv_ct_mht` (IMM CV+CT inside TOMHT with IPDA+VIMM
lifecycle and per-sensor detection tables). `MetricsParams` now
defaults to `gospa_cutoff_m = 20.0` to match the paper. `gospa_rms` is
the new aggregate matching the paper's "GOSPA reported as RMS".

## Per-scenario navtracker numbers (`imm_cv_ct_mht`, c = 20 m, p = α = 2)

| Sc | env | GOSPA mean | GOSPA RMS | GOSPA p95 | pos_rmse | breaks (count) | lifetime |
|---|---|---:|---:|---:|---:|---:|---:|
| 2  | 1 | 37.5 | 40.9 | 63.4 | 8.6  | 1.5 | 0.958 |
| 3  | 1 | 45.5 | 46.4 | 58.6 | 25.7 | 1.5 | 0.872 |
| 4  | 1 | 40.6 | 42.8 | 60.4 | 11.4 | 0.5 | 0.937 |
| 5  | 1 | 41.2 | 42.4 | 61.7 | 19.4 | 1.5 | 0.913 |
| 6  | 1 | 41.7 | 44.4 | 63.3 | 34.2 | 3.0 | 0.908 |
| 13 | 2 | 24.2 | 24.6 | 31.8 | 9.9  | 1.0 | 0.773 |
| 16 | 2 | 27.8 | 28.4 | 35.5 | 10.8 | 1.5 | 0.851 |
| 17 | 2 | 31.0 | 31.3 | 37.5 | 36.3 | 2.5 | 0.902 |
| 22 | 2 | 46.4 | 47.0 | 58.3 | 32.2 | 3.5 | 0.837 |

For reference (not in paper): **philos** GOSPA RMS = 76.3, pos_rmse =
38.4 m, lifetime = 0.295 — the low lifetime is honest (most philos
vessels report AIS only twice ~10 s apart in this 20-s fixture).

## Per-environment aggregate (RMS-of-per-scenario-RMS proxy)

| Env | Scenarios | navtracker GOSPA RMS | navtracker pos_rmse (mean per scenario) | navtracker breaks (count per scenario) |
|---|---|---:|---:|---:|
| 1 (open water)   | sc2-6        | **43.4** | 19.8 | 1.6 |
| 2 (urban channel)| sc13/16/17/22| **33.9** | 22.3 | 2.1 |

**Caveat.** A perfectly apples-to-apples aggregate would pool every
per-step GOSPA across an environment's scenarios and then RMS those.
The bench currently emits only per-scenario summaries. RMS-of-per-
scenario-RMS is equivalent only when scenarios share equal step
counts, which they do not. Treat the aggregate column as a proxy.
T-GOSPA (PMBM plan phase 4) would give a principled cross-scenario
trajectory metric.

## Comparison vs Helgesen 2022 (L,R,IR,EO full fusion)

| Env | Paper GOSPA RMS | navtracker GOSPA RMS | Δ |
|---|---:|---:|---:|
| 1 (open water)   | 20.37 | 43.4 | **+23 m (≈ 2.1×)** |
| 2 (urban channel)| 30.97 | 33.9 | +3 m (≈ 1.1×)      |

| Env | Paper posRMSE Havf./other | navtracker posRMSE (per-sc mean) |
|---|---|---:|
| 1 | 38.91 / 9.43  (Havfruen / Gunnerus) | 19.8 |
| 2 | 83.53 / 50.49 (Havfruen / Jetboat)  | 22.3 |

| Env | Paper Break.L (s) | navtracker breaks (count/sc) |
|---|---:|---:|
| 1 | 86.3  | 1.6 |
| 2 | 200.2 | 2.1 |

| Env | Paper ANEES | navtracker nees_mean (per scenario, range) |
|---|---:|---|
| 1 | 15.84 | 4-80 (sc2-6 range, sc5 ≈ 80; see eval log 2026-06-13) |
| 2 | 51.90 | 20-1000+ (sc22 NEES ≈ 1000, sc17 ≈ 470) |

### Read

- **Env 2 (urban channel): navtracker is essentially on par** with
  the paper on GOSPA (33.9 vs 31.0). pos_rmse is much better — but
  partly because the paper averages Havfruen + Jetboat and Jetboat
  was particularly hard for them (50.49 m alone) due to track
  divergence onto docked-boat EO returns (paper Fig. 25). navtracker
  doesn't yet replay individual targets separately to confirm a
  fully apples-to-apples per-target comparison.
- **Env 1 (open water): navtracker is ~2× worse on GOSPA** (43.4 vs
  20.4). pos_rmse is ostensibly better (19.8 vs 38.91/9.43 mean
  24.2) — but the paper's high Havfruen RMSE (38.91 m) comes
  specifically from "track coalescence on Gunnerus" in scenarios 2
  and 5 (paper Fig. 24) where the IR cameras lock Havfruen's track
  onto Gunnerus. navtracker doesn't exhibit that failure mode but
  pays a much larger GOSPA penalty for *cardinality* errors — track
  breaks (1.6 count/scenario for us, but at 87 s of cumulative break
  length for them across the environment we don't have a directly
  comparable scalar to know which is worse in total).
- **Filter consistency is worse on both envs.** Paper env 1 ANEES =
  15.84, env 2 = 51.90 — both well above the χ²-floor of 2 for a
  consistent filter (paper also runs hot, but less so than us). Our
  sc5 NEES = 80, sc22 ≈ 1000. The eval-log 2026-06-13 entries
  document this as backlog item 12 (filter overconfidence on real
  bearing-dominated data). Both trackers underestimate uncertainty;
  ours more aggressively.

### Where the env 1 gap likely sits

GOSPA at `c = 20 m` charges `c²/α = 200` per missed truth per step.
With our 1.5-2 breaks per scenario and breaks lasting some seconds,
the cardinality penalty dominates. The paper's headline VIMM-JIPDA
configuration sits at GOSPA 20 — which means *over the same evaluation
clock* their cardinality penalty is much smaller, either because (a)
their tracks recover faster from misses, (b) their breaks happen
during low-cardinality stretches that contribute less, or (c) their
break-length integration captures something we count differently.
Confirming this requires per-step GOSPA breakdown — queued.

## What can move navtracker's number toward the paper

In priority order:

1. **JIPDA upgrade** (sota-roadmap §2). The paper's tracker *is*
   VIMM-JIPDA. We have IMM-MHT instead — a different but related
   class. Closing this gap is the obvious next algorithmic step.
2. **Inter-sensor registration biases** (backlog item 9). The paper
   carefully calibrates each sensor's mounting offset against
   GNSS-RTK truth; we do not, and unmodelled biases show up as
   posRMSE and inflated NEES.
3. **Item 12 (NEES calibration)** — we are over-confident on real
   bearing-dominated data; honest covariances widen gates and reduce
   spurious breaks.
4. **T-GOSPA + per-target separation** in the bench so the
   comparison can be per-target like the paper.

## Per-target separation (TODO)

The paper's posRMSE column is `(Havfruen ; second_target)`. Our bench
aggregates pos_rmse across all assigned truths and reports a single
number per scenario. Adding per-truth posRMSE to MetricsResult would
let us compare per-target. AutoFerry's `truth.id` distinguishes the
vessels so the bookkeeping is small.
