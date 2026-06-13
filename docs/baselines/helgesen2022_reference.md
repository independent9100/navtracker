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

## Per-scenario table (PLACEHOLDERS — pending paper extraction)

| Scenario | GOSPA mean (paper) | pos_rmse (paper) | track-loss (paper) | navtracker `imm_cv_ct_mht` | Δ |
|---|---:|---:|---:|---|---|
| sc2  | —    | —    | —    | (run pending)    | — |
| sc3  | —    | —    | —    | (run pending)    | — |
| sc4  | —    | —    | —    | (run pending)    | — |
| sc5  | —    | —    | —    | (run pending)    | — |
| sc6  | —    | —    | —    | (run pending)    | — |
| sc13 | —    | —    | —    | (run pending)    | — |
| sc16 | —    | —    | —    | (run pending)    | — |
| sc17 | —    | —    | —    | (run pending)    | — |
| sc22 | —    | —    | —    | (run pending)    | — |

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
