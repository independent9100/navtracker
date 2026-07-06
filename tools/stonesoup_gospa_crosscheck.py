#!/usr/bin/env python3
"""D2 — GOSPA metric cross-validation against Stone Soup.

Re-scores a navtracker bench run's per-scan (truth, track) sets with Dstl
Stone Soup's independently-authored GOSPA (stonesoup.metricgenerator.
ospametric.GOSPAMetric) and compares, scan by scan, against navtracker's own
per-scan GOSPA (core/scenario/Gospa.hpp, dumped by GospaExport). Agreement
within numerical tolerance validates the harness metric; a mismatch under
matched conventions is a bug hunt (localise the offending scan and assignment
before concluding whose bug it is).

The harness has had two truth-fragmentation bugs silently corrupt metrics
(autoferry 2026-06-10, harbor 2026-07-02); an external agreement is the
cheapest hedge against a metric-code bug. See docs/algorithms/gospa-crosscheck.md.

Convention note (steer #2). navtracker and Stone Soup use the SAME GOSPA
convention: alpha = 2 (hardcoded in Stone Soup's compute_gospa_metric),
cardinality penalty c^p / alpha per missed / false target, and the rooted
headline distance = (localisation + missed + false)^(1/p). The decomposition
fields (localisation, missed, false) are reported by BOTH sides in pre-root
power-p space, so they are directly comparable term-by-term. Per-scan
compute_gospa_metric adds NO switching penalty on either side (switching is a
time-series-level term in Stone Soup), so the point-set comparison is clean.

Inputs (from `navtracker_bench_baseline ... --export-states-dir DIR`):
  <run>.states.csv       scan,time_s,kind,id,east_m,north_m   (kind: truth|track)
  <run>.ours_gospa.csv   scan,time_s,gospa,localisation,missed,false,n_missed,n_false

Usage:
  stonesoup_gospa_crosscheck.py --states X.states.csv --ours X.ours_gospa.csv \
      [--c 20.0] [--p 2.0] [--tol 1e-6]

Run under a venv with Stone Soup installed (see the module docstring in
core/benchmark/GospaExport.hpp and tools/README for the venv recipe).
"""
import argparse
import csv
import sys
from collections import defaultdict
from datetime import datetime, timedelta

from stonesoup.metricgenerator.ospametric import GOSPAMetric
from stonesoup.types.state import State

# A fixed epoch; only the per-scan offset matters (each scan's states must
# share one timestamp for compute_gospa_metric, and scans must differ).
EPOCH = datetime(2020, 1, 1)


def load_states(path):
    """scan -> {'truth': [State...], 'track': [State...]} keyed by scan index."""
    scans = defaultdict(lambda: {"truth": [], "track": []})
    times = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            scan = int(row["scan"])
            times[scan] = float(row["time_s"])
            ts = EPOCH + timedelta(seconds=scan)  # unique per scan
            st = State(
                state_vector=[[float(row["east_m"])], [float(row["north_m"])]],
                timestamp=ts,
            )
            scans[scan][row["kind"]].append(st)
    return scans, times


def load_ours(path):
    """scan -> dict of our exported GOSPA fields."""
    ours = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            scan = int(row["scan"])
            ours[scan] = {
                "gospa": float(row["gospa"]),
                "localisation": float(row["localisation"]),
                "missed": float(row["missed"]),
                "false": float(row["false"]),
                "n_missed": int(row["n_missed"]),
                "n_false": int(row["n_false"]),
            }
    return ours


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--states", required=True, help="<run>.states.csv")
    ap.add_argument("--ours", required=True, help="<run>.ours_gospa.csv")
    ap.add_argument("--c", type=float, default=20.0, help="GOSPA cutoff c (m)")
    ap.add_argument("--p", type=float, default=2.0, help="GOSPA order p")
    ap.add_argument("--tol", type=float, default=1e-6,
                    help="max allowed |Δ| on the rooted GOSPA distance (m)")
    args = ap.parse_args()

    scans, times = load_states(args.states)
    ours = load_ours(args.ours)

    # alpha is hardcoded to 2 inside Stone Soup's compute_gospa_metric; we pass
    # switching_penalty=0 so the per-scan value carries no switching term.
    metric = GOSPAMetric(p=args.p, c=args.c, switching_penalty=0.0,
                         generator_name="d2", tracks_key="tracks",
                         truths_key="truths")
    dummy = (args.c ** args.p) / 2.0  # c^p / alpha, the per-target penalty

    print(f"# D2 GOSPA cross-check  c={args.c} p={args.p} alpha=2 switching=0")
    print(f"# states={args.states}")
    print(f"# {'scan':>4} {'ss_gospa':>12} {'our_gospa':>12} {'|Δ|':>11} "
          f"{'Δloc':>11} {'Δmiss':>9} {'Δfalse':>9} {'nm(ss/our)':>11} "
          f"{'nf(ss/our)':>11}")

    max_dist_diff = 0.0
    worst_scan = -1
    ss_sum = our_sum = 0.0
    n = 0
    count_mismatches = []
    for scan in sorted(scans):
        truth = scans[scan]["truth"]
        track = scans[scan]["track"]
        m, _ = metric.compute_gospa_metric(measured_states=track,
                                           truth_states=truth)
        v = m.value
        ss_gospa = float(v["distance"])
        ss_loc = float(v["localisation"])
        ss_missed = float(v["missed"])
        ss_false = float(v["false"])
        # Recover Stone Soup's cardinality counts from its power-p costs.
        ss_nm = round(ss_missed / dummy)
        ss_nf = round(ss_false / dummy)

        o = ours[scan]
        d_dist = abs(ss_gospa - o["gospa"])
        d_loc = ss_loc - o["localisation"]
        d_miss = ss_missed - o["missed"]
        d_false = ss_false - o["false"]

        if d_dist > max_dist_diff:
            max_dist_diff = d_dist
            worst_scan = scan
        if ss_nm != o["n_missed"] or ss_nf != o["n_false"]:
            count_mismatches.append(scan)
        ss_sum += ss_gospa
        our_sum += o["gospa"]
        n += 1

        print(f"  {scan:>4} {ss_gospa:>12.6f} {o['gospa']:>12.6f} "
              f"{d_dist:>11.2e} {d_loc:>11.2e} {d_miss:>9.1e} {d_false:>9.1e} "
              f"{str(ss_nm)+'/'+str(o['n_missed']):>11} "
              f"{str(ss_nf)+'/'+str(o['n_false']):>11}")

    ss_mean = ss_sum / n if n else 0.0
    our_mean = our_sum / n if n else 0.0
    print(f"\n# scans compared      : {n}")
    print(f"# mean GOSPA  stonesoup: {ss_mean:.6f}")
    print(f"# mean GOSPA  navtracker: {our_mean:.6f}")
    print(f"# mean |Δ|            : {abs(ss_mean - our_mean):.3e}")
    print(f"# max per-scan |Δ|    : {max_dist_diff:.3e}  (scan {worst_scan})")
    print(f"# cardinality mismatches: {count_mismatches if count_mismatches else 'none'}")

    ok = max_dist_diff <= args.tol and not count_mismatches
    print(f"\n# VERDICT: {'PASS — harness GOSPA validated by Stone Soup' if ok else 'FAIL — investigate'}"
          f"  (tol={args.tol:g} m)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
