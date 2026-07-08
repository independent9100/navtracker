#!/usr/bin/env python3
"""Cardinality-timing analysis for the Imazu battery (backlog #11, Q2).

Answers: is a tracker's cardinality error (over-count / under-count) driven by
the close-crossing geometry, or is it a crossing-independent baseline (e.g.
clutter birth-model noise)? Consumes the bench ``--export-states-dir`` CSV
(``scan,time_s,kind,id,east_m,north_m``) and, per scan, re-derives the same
per-scan truth->track assignment the bench uses (rectangular Hungarian, strict
gate ``d < gate`` m; see ``imazu_switch_forensics.py`` for the faithful port),
then decomposes cardinality into:

  n_extra  = confirmed tracks NOT matched to any truth   (the over-count / false)
  n_missed = present truths NOT matched to any track      (the under-count / lost)
  card_err = n_tracks - n_truth = n_extra - n_missed      (matches bench card_err_mean)

It prints the run-mean of each and a per-time-bucket series of n_extra and
n_missed so the reader can see WHERE and WHEN the over/under-count occurs
relative to the truth-truth CPA (close-pass) scans.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all, dist  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--states", required=True)
    ap.add_argument("--gate", type=float, default=100.0)
    ap.add_argument("--bucket", type=int, default=60, help="time bucket seconds")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    steps = load_states(args.states)
    per_scan = assign_all(steps, args.gate)

    # per-scan cardinality decomposition
    series = []
    tot_extra = tot_missed = tot_truth = tot_track = 0.0
    n = 0
    for st in per_scan:
        n_truth = len(st["truth_pos"])
        n_track = len(st["track_pos"])
        n_assigned = sum(1 for a in st["assigned"].values() if a is not None)
        n_extra = n_track - n_assigned
        n_missed = n_truth - n_assigned
        series.append((st["scan"], n_truth, n_track, n_extra, n_missed))
        tot_extra += n_extra
        tot_missed += n_missed
        tot_truth += n_truth
        tot_track += n_track
        n += 1
    n = max(1, n)

    # truth-truth CPA (min pairwise distance) scans for context
    ids = sorted({t[0] for _, s in steps for t in s["truth"]})
    pos_by_scan = {k: {t[0]: t[1] for t in s["truth"]} for k, s in steps}
    cpas = []
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            A, B = ids[i], ids[j]
            best_d, best_k = math.inf, None
            for k in pos_by_scan:
                pa, pb = pos_by_scan[k].get(A), pos_by_scan[k].get(B)
                if pa and pb:
                    d = dist(pa, pb)
                    if d < best_d:
                        best_d, best_k = d, k
            cpas.append((f"{A}-{B}", round(best_d, 1), best_k))

    print(f"=== {args.label or args.states} ===")
    print(f"n_truth ids={len(ids)}  mean n_track={tot_track/n:.3f}  "
          f"mean n_truth={tot_truth/n:.3f}")
    print(f"card_err (n_track-n_truth) mean = {(tot_track-tot_truth)/n:+.3f}   "
          f"[= extra {tot_extra/n:.3f} - missed {tot_missed/n:.3f}]")
    print("truth-truth CPA (m @ scan):", cpas)
    print(f"per {args.bucket}s bucket:  mean_extra  mean_missed  (over / under count)")
    buckets = {}
    for (scan, nt, ntr, ne, nm) in series:
        b = (scan // args.bucket) * args.bucket
        buckets.setdefault(b, []).append((ne, nm))
    for b in sorted(buckets):
        vals = buckets[b]
        me = sum(v[0] for v in vals) / len(vals)
        mm = sum(v[1] for v in vals) / len(vals)
        bar_e = "#" * int(round(me * 6))
        bar_m = "x" * int(round(mm * 6))
        print(f"  t[{b:3d}-{b+args.bucket-1:3d}]: extra={me:4.2f} {bar_e:<14s} "
              f"missed={mm:4.2f} {bar_m}")


if __name__ == "__main__":
    main()
