# Helgesen et al. 2022 — paper reference table

Helgesen, Vasstein, Brekke, Stahl, "Heterogeneous multi-sensor tracking
for an autonomous surface vehicle in a littoral environment",
*Ocean Engineering* 252 (2022) 111168.
[ScienceDirect (paywalled)](https://www.sciencedirect.com/science/article/pii/S0029801822005753)

Dataset: <https://github.com/Autoferry/sensor_fusion_dataset>.

This file holds the paper's published per-scenario numbers — the
reference we benchmark navtracker against on the AutoFerry replays.
**Paper PDF is paywalled and outside the sandbox network whitelist;
numbers below are placeholders to be filled from the published tables.**

## What we expect to find

The paper evaluates an asynchronous multi-sensor JIPDA-IMM (the same
algorithm class that sits at `sota-roadmap.md §2`, our planned JIPDA
upgrade). Performance is reported per scenario, per sensor
configuration, on the metrics catalogued in the master's thesis (Ch. 7):

- **GOSPA mean** — assumed c = 30 m, p = 2, α = 2 (literature default;
  confirm against the paper's §6 / §7).
- **Position RMSE** — per ground-truth target, m.
- **Track-loss / break counts** — per ground-truth target.
- **OSPA mean** is also given in the thesis. The paper may or may not
  include OSPA alongside GOSPA — confirm.

We care about the **all-sensor fused** numbers (radar + lidar + EO + IR),
which the paper presents as the headline configuration. Single-sensor
ablations may also be tabulated; useful as a sanity check but not the
target.

## Per-scenario table

Source: bench run `docs/baselines/gospa_20260613T162409Z.csv` (single
seed, all 9 autoferry scenarios + philos, with GOSPA wired and
defaults c = 30 m, p = α = 2). Canonical navtracker config is
`imm_cv_ct_mht` (IMM CV+CT inside TOMHT with IPDA+VIMM lifecycle).

| Scenario | navtracker GOSPA mean | navtracker GOSPA p95 | navtracker pos_rmse | navtracker breaks | navtracker lifetime | paper GOSPA mean | paper pos_rmse | paper breaks | Δ GOSPA |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| sc2  | 55.4   | 95.0  | 8.6  | 1.5 | 0.958 | — | — | — | — |
| sc3  | 67.0   | 87.6  | 25.7 | 1.5 | 0.872 | — | — | — | — |
| sc4  | 60.0   | 90.3  | 11.4 | 0.5 | 0.937 | — | — | — | — |
| sc5  | 60.3   | 92.5  | 19.4 | 1.5 | 0.913 | — | — | — | — |
| sc6  | 61.7   | 93.6  | 34.2 | 3   | 0.908 | — | — | — | — |
| sc13 | 35.3   | 47.5  | 9.9  | 1   | 0.773 | — | — | — | — |
| sc16 | 41.2   | 52.5  | 10.8 | 1.5 | 0.851 | — | — | — | — |
| sc17 | 45.7   | 56.2  | 36.3 | 2.5 | 0.902 | — | — | — | — |
| sc22 | 69.0   | 85.9  | 32.2 | 3.5 | 0.837 | — | — | — | — |

For context (not a paper scenario, single-seed): **philos** under the
same canonical config — GOSPA mean 106.5 m, p95 152.0 m, pos_rmse
38.4 m, lifetime 0.295 (the low lifetime is honest: most philos
vessels report AIS only twice ~10 s apart in this ~20 s fixture, so
confirming at the second fix already costs half the presence window).

**Reading the navtracker column:**

- env 1 scenarios (sc2-6): GOSPA mean 55-67 m, lifetime 0.87-0.96. The
  bulk of GOSPA cost is the cardinality penalty — with ~2-3 truth
  vessels per scenario and 1-3 track breaks each, every dropped truth
  contributes c²/α = 450 to cost² → √450 = 21 m floor per missed-step.
  Even a perfect-position tracker that drops one of two truths once
  would sit at ~15 m GOSPA.
- env 2 (sc13, 16, 17): GOSPA 35-46 m, somewhat lower despite the urban
  channel — explained partly by shorter duration (less time to
  accumulate breaks). sc22 is the env 2 outlier (69 m).
- pos_rmse is the *positional* term divorced from cardinality: 9-36 m.
  sc3, sc6, sc17, sc22 are the outliers (>25 m) — the maneuvering /
  obscuration scenarios where the filter momentarily diverges before
  catching up.
- track_breaks 0.5-3.5 per scenario — this is the key driver of the
  GOSPA cardinality penalty.

## Methodology cross-checks (TODO)

When numbers are filled in, verify the comparison is apples-to-apples:

- **Truth assignment.** Paper uses RTK-GNSS truth at scenario rate.
  navtracker uses the JSON GT-track frame-aligned at evaluation
  timestamps (`adapters/replay/AutoferryJsonReplay.cpp` reads the same
  scenarioN_groundTruth.json the dataset ships). Should match.
- **GOSPA parameters.** Confirm paper's (c, p, α) match our defaults
  (30 m / 2 / 2 in `core/benchmark/Metrics.hpp`); if not, adjust the
  bench param to match the paper, not the other way around.
- **Sensor selection.** Paper headline = all-sensor fusion. Our bench
  enables EO/IR bearings via `AutoferryLoadOptions::include_bearings`
  (currently true in `defaultAutoferryScenarios()`). Match.
- **Track-loss definition.** Paper likely uses "track absent for ≥ N
  seconds" or similar. Our `track_breaks` metric uses the
  `BenchSink` lifecycle events. Document any definition gap.
- **Scenario subset.** The paper may evaluate more or fewer scenarios
  than the 9 we mirror. Note which scenarios overlap and skip the
  others from the comparison.

## How to fill this in

Open the paper's results section. For each row above, paste the
paper's GOSPA-mean, pos-rmse, and track-loss numbers. Re-run the bench
(commit `0519a78` and onwards) to populate the navtracker column from
the latest `docs/baselines/jpda_persensor_*.csv`. Compute the delta.
Add a paragraph below the table summarizing what the comparison says
and which gaps (algorithmic / configuration / metric-definition)
explain the delta.
