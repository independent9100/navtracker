#!/usr/bin/env python3
"""Dossier aggregation helper (2026-07-10 PMBM promotion dossier).

Reads one or more long-format bench CSVs
(`config,scenario,seed,metric,value,unit`, with leading `#` provenance
comments) and pivots the metrics the dossier needs into per-(config,scenario)
rows. Also computes the Cl-1 env aggregate: RMS-of-per-scenario `gospa_rms`
(the same proxy helgesen2022_reference.md / cl1 rescore uses).

stdlib-only. Research tooling, not part of the build.
"""
import argparse
import csv
import math
from collections import defaultdict

# Metrics the dossier reports (long-format `metric` column names).
DEFAULT_METRICS = [
    "gospa_rms", "gospa_mean", "card_err_mean", "gospa_false", "gospa_missed",
    "lifetime_ratio", "track_breaks", "id_switches",
    "scan_proc_ms_mean", "scan_proc_ms_p95", "scan_proc_ms_max",
    "scan_interval_s", "n_scans", "wall_seconds",
]

ENV1 = ["autoferry_scenario2", "autoferry_scenario3", "autoferry_scenario4",
        "autoferry_scenario5", "autoferry_scenario6"]
ENV2 = ["autoferry_scenario13", "autoferry_scenario16", "autoferry_scenario17",
        "autoferry_scenario22"]


def load(paths):
    acc = defaultdict(list)
    for p in paths:
        with open(p, newline="") as f:
            reader = csv.reader(row for row in f if not row.startswith("#"))
            header = next(reader)
            idx = {name: i for i, name in enumerate(header)}
            for r in reader:
                if not r:
                    continue
                cfg = r[idx["config"]]
                scn = r[idx["scenario"]]
                met = r[idx["metric"]]
                try:
                    val = float(r[idx["value"]])
                except ValueError:
                    continue
                acc[(cfg, scn, met)].append(val)
    return {k: sum(v) / len(v) for k, v in acc.items()}


def get(data, cfg, scn, met):
    return data.get((cfg, scn, met))


def rms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def env_agg(data, cfg, scenarios, anchored=False):
    labels = [s + ("_anchored" if anchored else "") for s in scenarios]
    vals = [get(data, cfg, s, "gospa_rms") for s in labels]
    return rms(vals), vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--configs", nargs="*", default=None,
                    help="restrict to these config labels")
    ap.add_argument("--metrics", nargs="*", default=DEFAULT_METRICS)
    ap.add_argument("--scenario-substr", default="",
                    help="only print scenarios containing this substring")
    ap.add_argument("--cl1", action="store_true",
                    help="print env-1/env-2 gospa_rms RMS aggregate")
    args = ap.parse_args()

    data = load(args.csv)
    configs = args.configs or sorted({k[0] for k in data})
    scenarios = sorted({k[1] for k in data if args.scenario_substr in k[1]})

    for cfg in configs:
        print(f"\n### config: {cfg}")
        print("\t".join(["scenario"] + args.metrics))
        for scn in scenarios:
            if not any((cfg, scn, m) in data for m in args.metrics):
                continue
            row = [scn]
            for m in args.metrics:
                v = get(data, cfg, scn, m)
                row.append("" if v is None else f"{v:.4g}")
            print("\t".join(row))

    if args.cl1:
        print("\n### Cl-1 env aggregate (RMS-of-per-scenario gospa_rms)")
        print("config\tcondition\tenv-1\tenv-2")
        for cfg in configs:
            for cond, anc in [("no-AIS", False), ("truth-AIS", True)]:
                e1, _ = env_agg(data, cfg, ENV1, anc)
                e2, _ = env_agg(data, cfg, ENV2, anc)
                s1 = "n/a" if e1 is None else f"{e1:.2f}"
                s2 = "n/a" if e2 is None else f"{e2:.2f}"
                print(f"{cfg}\t{cond}\t{s1}\t{s2}")


if __name__ == "__main__":
    main()
