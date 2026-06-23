#!/usr/bin/env python3
"""Compute baseline-CSV deltas at full precision.

The bench CSVs in docs/baselines/ are long-format rows of
(run_id, config, scenario, seed, metric, value, unit).  Comparing two
runs by eye is error-prone — twice the eval-log overstated wins
because someone wrote a delta from %.1f-rounded numbers.  This script
reads both CSVs at full float precision, joins on the
(config, scenario, seed, metric) key, and emits one row per metric
with the delta and signed pct change.

By default it filters to top-line metrics (ospa_mean, gospa_mean,
tgospa_mean, id_switches, fragments) and aggregates across seeds via
the median.  --metric repeats narrow the set.  --by-seed disables the
seed aggregation.

Output is markdown that can drop directly into evaluation-log.md.

Usage:
    tools/bench_diff.py baseline.csv candidate.csv
    tools/bench_diff.py baseline.csv candidate.csv --config imm_cv_ct_pmbm_adapt
    tools/bench_diff.py baseline.csv candidate.csv --metric gospa_mean --metric id_switches
    tools/bench_diff.py baseline.csv candidate.csv --by-seed
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

DEFAULT_METRICS = (
    "ospa_mean",
    "gospa_mean",
    "tgospa_mean",
    "id_switches",
    "fragments",
)


def load(path: Path) -> dict:
    """Return {(config, scenario, seed, metric): float}, full precision.

    Bench CSVs start with `#`-prefixed header comments followed by a
    regular CSV body.  csv.DictReader doesn't know about the comments,
    so we strip them first.
    """
    out: dict = {}
    with path.open() as fh:
        lines = [ln for ln in fh if not ln.startswith("#")]
    for row in csv.DictReader(lines):
        key = (row["config"], row["scenario"], row["seed"], row["metric"])
        try:
            out[key] = float(row["value"])
        except (TypeError, ValueError):
            continue
    return out


def aggregate_by_seed(data: dict) -> dict:
    """Median across seeds → {(config, scenario, metric): float}."""
    buckets: dict = defaultdict(list)
    for (cfg, sc, _seed, metric), v in data.items():
        if math.isfinite(v):
            buckets[(cfg, sc, metric)].append(v)
    return {k: statistics.median(vs) for k, vs in buckets.items() if vs}


def fmt_delta(b: float, c: float) -> tuple[str, str, str, str]:
    delta = c - b
    if abs(b) < 1e-12:
        pct = "n/a" if abs(delta) < 1e-12 else "inf"
    else:
        pct = f"{100.0 * delta / b:+.2f}%"
    return f"{b:.4f}", f"{c:.4f}", f"{delta:+.4f}", pct


def render(rows: list[tuple], by_seed: bool) -> str:
    if by_seed:
        head = "| config | scenario | seed | metric | baseline | candidate | delta | pct |"
        sep  = "|---|---|---|---|---:|---:|---:|---:|"
    else:
        head = "| config | scenario | metric | baseline | candidate | delta | pct |"
        sep  = "|---|---|---|---:|---:|---:|---:|"
    lines = [head, sep]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("baseline", type=Path)
    ap.add_argument("candidate", type=Path)
    ap.add_argument("--metric", action="append", default=None,
                    help="repeat to narrow; default = top-line metrics")
    ap.add_argument("--config", action="append", default=None,
                    help="repeat to narrow; default = all configs")
    ap.add_argument("--scenario", action="append", default=None,
                    help="repeat to narrow; default = all scenarios")
    ap.add_argument("--by-seed", action="store_true",
                    help="emit per-seed rows instead of median-across-seeds")
    ap.add_argument("--all-metrics", action="store_true",
                    help="ignore the default top-line metric filter")
    args = ap.parse_args()

    base = load(args.baseline)
    cand = load(args.candidate)

    metric_set = (set(args.metric) if args.metric
                  else (None if args.all_metrics else set(DEFAULT_METRICS)))
    config_set = set(args.config) if args.config else None
    scenario_set = set(args.scenario) if args.scenario else None

    def keep(metric, config, scenario):
        if metric_set is not None and metric not in metric_set:
            return False
        if config_set is not None and config not in config_set:
            return False
        if scenario_set is not None and scenario not in scenario_set:
            return False
        return True

    if args.by_seed:
        keys = sorted(set(base) | set(cand))
        rows = []
        for cfg, sc, seed, metric in keys:
            if not keep(metric, cfg, sc):
                continue
            b = base.get((cfg, sc, seed, metric))
            c = cand.get((cfg, sc, seed, metric))
            if b is None or c is None:
                continue
            cells = fmt_delta(b, c)
            rows.append((cfg, sc, seed, metric, *cells))
        print(render(rows, by_seed=True))
        return 0

    base_agg = aggregate_by_seed(base)
    cand_agg = aggregate_by_seed(cand)
    keys = sorted(set(base_agg) | set(cand_agg))
    rows = []
    for cfg, sc, metric in keys:
        if not keep(metric, cfg, sc):
            continue
        b = base_agg.get((cfg, sc, metric))
        c = cand_agg.get((cfg, sc, metric))
        if b is None or c is None:
            continue
        cells = fmt_delta(b, c)
        rows.append((cfg, sc, metric, *cells))
    print(render(rows, by_seed=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
