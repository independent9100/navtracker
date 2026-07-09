#!/usr/bin/env python3
"""Backlog #25 Phase 2b Stage 2 — A/B of the velocity-runaway guard variants.

Given a set of bench runs (OFF baseline + gated variants) that each exported
--export-states-dir + a metric CSV, compares on the agreed gates:

  - the 6 dying cases (imazu_15 / imazu_22 truths): total loss-seconds,
    loss-seconds OVERLAPPING the own-ship CPA, and re-acquire-id count;
  - id-switches on the dense Imazu cases (watch-item: accept-position can snap
    the id onto the neighbour vessel — measure it, don't hide it);
  - GOSPA / cardinality-error / lifetime aggregate (no-regression context).

Layout: <ab-dir>/<variant>/states/*.states.csv and <ab-dir>/<variant>/m/<variant>.csv.
stdlib only; reuses the Phase-1 loss/CPA loaders. Read-only.
"""
import argparse
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all          # noqa: E402
from imazu_trackloss import load_ownship_enu, loss_windows, cpa_scans  # noqa: E402

CFG = "imm_cv_ct_pmbm_coverage_land"
DATUM = (63.45, 10.35)
DYING = {  # scenario -> dying truth ids (Phase-1 six)
    "imazu_15": [257010151, 257010152, 257010153],
    "imazu_22": [257010221, 257010222, 257010223],
}
DENSE = [f"imazu_{n:02d}" for n in range(12, 23)]  # 3-target crossings


def dying_metrics(states_dir, simms_dir):
    """(total_loss_s, cpa_overlap_s, reacquire_ids) summed over the 6 dying truths."""
    tot = cpa = reacq = 0
    per = []
    for scen, truths in DYING.items():
        sp = os.path.join(states_dir, f"{CFG}__{scen}__seed0.states.csv")
        if not os.path.exists(sp):
            continue
        steps = load_states(sp)
        per_scan = assign_all(steps, 100.0)
        own = load_ownship_enu(os.path.join(simms_dir, scen + "_s0", "ownship.csv"), *DATUM)
        _, own_cpa, _ = cpa_scans(steps, own)
        for tid in truths:
            cpa_scan = own_cpa.get(tid, (None, None))[0]
            wins = loss_windows(per_scan, tid)
            t = sum(w["dur"] for w in wins)
            c = sum(w["dur"] for w in wins
                    if cpa_scan is not None and w["start"] <= cpa_scan <= w["end"])
            r = sum(1 for w in wins if w["new_id"])
            tot += t; cpa += c; reacq += r
            per.append((scen, tid, t, c, r))
    return tot, cpa, reacq, per


def load_metric(matrix_csv, metric, scenarios=None):
    """sum of a metric over scenarios (or all imazu) from a bench matrix CSV."""
    total = 0.0; rows = {}
    if not os.path.exists(matrix_csv):
        return 0.0, {}
    with open(matrix_csv) as f:
        for r in csv.DictReader([ln for ln in f if not ln.lstrip().startswith('#')]):
            if r["metric"] != metric or not r["scenario"].startswith("imazu_"):
                continue
            if scenarios is not None and r["scenario"] not in scenarios:
                continue
            v = float(r["value"]); total += v; rows[r["scenario"]] = v
    return total, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ab-dir", required=True)
    ap.add_argument("--simms-dir", required=True)
    ap.add_argument("--variants", default="off,r200,r400,d200,d400")
    args = ap.parse_args()
    variants = args.variants.split(",")

    print("=" * 78)
    print("BACKLOG #25 PHASE 2b STAGE 2 — velocity-runaway guard A/B")
    print("=" * 78)

    print(f"\n{'variant':>8} | {'dying loss_s':>12} | {'CPA-overlap_s':>13} | "
          f"{'reacq-ids':>9} | {'idsw(all imazu)':>15} | {'idsw(dense)':>11} | "
          f"{'gospa_mean_sum':>14}")
    print("-" * 100)
    base = {}
    for v in variants:
        sd = os.path.join(args.ab_dir, v, "states")
        mc = os.path.join(args.ab_dir, v, "m", v + ".csv")
        tot, cpa, reacq, per = dying_metrics(sd, args.simms_dir)
        idsw_all, _ = load_metric(mc, "id_switches")
        idsw_dense, _ = load_metric(mc, "id_switches", set(DENSE))
        gospa, _ = load_metric(mc, "gospa_mean")
        base[v] = dict(tot=tot, cpa=cpa, reacq=reacq, idsw_all=idsw_all,
                       idsw_dense=idsw_dense, gospa=gospa, per=per)
        print(f"{v:>8} | {tot:>12.0f} | {cpa:>13.0f} | {reacq:>9} | "
              f"{idsw_all:>15.0f} | {idsw_dense:>11.0f} | {gospa:>14.1f}")

    if "off" in base:
        o = base["off"]
        print(f"\nΔ vs OFF (negative loss/idsw = improvement):")
        for v in variants:
            if v == "off":
                continue
            b = base[v]
            print(f"  {v:>6}: dying_loss {b['tot']-o['tot']:+.0f}s  "
                  f"CPA-overlap {b['cpa']-o['cpa']:+.0f}s  reacq {b['reacq']-o['reacq']:+d}  "
                  f"idsw_all {b['idsw_all']-o['idsw_all']:+.0f}  "
                  f"idsw_dense {b['idsw_dense']-o['idsw_dense']:+.0f}  "
                  f"gospa {b['gospa']-o['gospa']:+.1f}")

    # per-dying-case detail for the winner-candidates
    print("\nper-dying-case (scen, truth, total_loss_s, CPA-overlap_s, reacq):")
    for v in variants:
        print(f"  [{v}]")
        for (scen, tid, t, c, r) in base[v]["per"]:
            print(f"     {scen} {tid}: loss={t}s cpa_overlap={c}s reacq={r}")


if __name__ == "__main__":
    main()
