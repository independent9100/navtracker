#!/usr/bin/env python3
"""Close-pass track-loss analysis for the Imazu battery (backlog #11, Q2 follow-up).

Quantifies the PMBM close-pass failure mode: at a close crossing PMBM tends to
*drop* the ambiguous target (a stretch of scans where a present truth is matched
to no confirmed track) rather than churn its identity. For each truth this finds
the **loss windows** (maximal runs of assigned->unassigned->reassigned while the
truth is present) and reports, per the arbiter's questions:

  - which cases lose a track and how many windows;
  - how LONG each loss lasts (scans == seconds at the 1 Hz truth cadence);
  - whether the truth RE-CONFIRMS under a NEW track id afterwards (vs the same
    id, vs never recovering = permanent loss to run end);
  - whether the loss window OVERLAPS the CPA window -- both the target<->own-ship
    CPA (the operationally-worst moment: losing a target near its closest point
    of approach to own-ship) and the target<->target CPA (which triggers the
    radar cross-range ambiguity in the first place).

Consumes the bench ``--export-states-dir`` CSV (truth/track ENU per scan) and
the fixture ``ownship.csv`` (lat/lon), reusing the faithful per-scan Hungarian
assignment from ``imazu_switch_forensics.py``. Own-ship lat/lon is mapped to the
same ENU tangent plane as the states via an equirectangular approximation about
the datum (validated to ~4 m at 2.8 km against the states truth positions --
negligible for CPA timing / window overlap).
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all, dist  # noqa: E402

R_EARTH = 6378137.0


def load_ownship_enu(path, lat0, lon0):
    """time_s -> (east,north) about the datum (equirectangular tangent plane)."""
    out = {}
    coslat = math.cos(math.radians(lat0))
    with open(path) as f:
        for row in csv.DictReader(f):
            t = float(row["unix_time"])
            e = math.radians(float(row["lon"]) - lon0) * coslat * R_EARTH
            n = math.radians(float(row["lat"]) - lat0) * R_EARTH
            out[t] = (e, n)
    return out


def own_at(own, t):
    """nearest own-ship sample to time t (own is dense at 0.1 s)."""
    key = min(own.keys(), key=lambda k: abs(k - t))
    return own[key]


def loss_windows(per_scan, truth_id):
    """Maximal runs where the truth is present but unassigned, bracketed by
    the assigned track id immediately before and after."""
    stream = []  # (scan, assigned_or_None) for scans where truth present
    for st in per_scan:
        if truth_id in st["assigned"]:
            stream.append((st["scan"], st["assigned"][truth_id]))
    windows = []
    i = 0
    last_assigned = None
    n = len(stream)
    while i < n:
        scan, a = stream[i]
        if a is None and last_assigned is not None:
            # start of a loss after having been assigned
            start = scan
            j = i
            while j < n and stream[j][1] is None:
                j += 1
            end = stream[j - 1][0]
            after = stream[j][1] if j < n else None
            windows.append({
                "start": start, "end": end, "dur": end - start + 1,
                "before": last_assigned, "after": after,
                "new_id": (after is not None and after != last_assigned),
                "permanent": after is None,
            })
            i = j
        else:
            if a is not None:
                last_assigned = a
            i += 1
    return windows


def cpa_scans(steps, own):
    """per truth: (scan, dist) of min target<->own-ship distance, plus per
    truth-pair min distance scan."""
    ids = sorted({t[0] for _, s in steps for t in s["truth"]})
    pos = {k: {t[0]: t[1] for t in s["truth"]} for k, s in steps}
    times = {k: t for k, s in steps for _ in [0] for t in [s]}  # placeholder
    # scan -> time
    scan_time = {}
    for k, s in steps:
        # time isn't in the grouped dict; recover from any row? use scan==time (1Hz)
        scan_time[k] = float(k)
    own_cpa = {}
    for tid in ids:
        best = (None, math.inf)
        for k in pos:
            p = pos[k].get(tid)
            if p:
                o = own_at(own, scan_time[k])
                d = math.hypot(p[0] - o[0], p[1] - o[1])
                if d < best[1]:
                    best = (k, d)
        own_cpa[tid] = best
    pair_cpa = {}
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            A, B = ids[i], ids[j]
            best = (None, math.inf)
            for k in pos:
                pa, pb = pos[k].get(A), pos[k].get(B)
                if pa and pb:
                    d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                    if d < best[1]:
                        best = (k, d)
            pair_cpa[(A, B)] = best
    return ids, own_cpa, pair_cpa


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--states", required=True)
    ap.add_argument("--ownship", required=True)
    ap.add_argument("--datum", default="63.45,10.35")
    ap.add_argument("--label", default="")
    ap.add_argument("--gate", type=float, default=100.0)
    ap.add_argument("--window", type=int, default=30,
                    help="CPA overlap half-window (s)")
    args = ap.parse_args()

    lat0, lon0 = (float(x) for x in args.datum.split(","))
    steps = load_states(args.states)
    per_scan = assign_all(steps, args.gate)
    own = load_ownship_enu(args.ownship, lat0, lon0)
    ids, own_cpa, pair_cpa = cpa_scans(steps, own)

    print(f"=== {args.label or args.states} ===")
    # nearest truth-truth CPA (the ambiguity trigger); absent for single-target
    if pair_cpa:
        tt = min(pair_cpa.items(), key=lambda kv: kv[1][1])
        print(f"nearest truth-truth CPA: {tt[0][0]}-{tt[0][1]} = {tt[1][1]:.1f} m @ scan {tt[1][0]}")
    else:
        print("nearest truth-truth CPA: (single target — none)")
    print("truth: own-ship CPA (m@scan) | loss windows [start-end dur before->after tag] | overlaps CPA?")
    total_loss = 0
    total_windows = 0
    total_newid = 0
    total_perm = 0
    any_overlap = False
    longest = {"dur": 0}
    subst = 0            # losses >= 10 s
    subst_ov_own = 0     # substantial losses overlapping own-ship CPA
    for tid in ids:
        cpa_k, cpa_d = own_cpa[tid]
        wins = loss_windows(per_scan, tid)
        # partner CPA scans for this truth (nearest other truth)
        partner_scans = [v[0] for k, v in pair_cpa.items() if tid in k]
        descs = []
        for w in wins:
            total_loss += w["dur"]
            total_windows += 1
            if w["new_id"]:
                total_newid += 1
            if w["permanent"]:
                total_perm += 1
            # overlap: does [start,end] intersect [cpa-window, cpa+window] for
            # own-ship CPA or any partner CPA?
            ov_own = cpa_k is not None and not (w["end"] < cpa_k - args.window or w["start"] > cpa_k + args.window)
            ov_pair = any(ps is not None and not (w["end"] < ps - args.window or w["start"] > ps + args.window) for ps in partner_scans)
            if ov_own or ov_pair:
                any_overlap = True
            if w["dur"] >= 10:
                subst += 1
                if ov_own:
                    subst_ov_own += 1
            if w["dur"] > longest["dur"]:
                longest = {"dur": w["dur"], "truth": tid, "start": w["start"],
                           "tag": ("PERM" if w["permanent"] else ("new-id" if w["new_id"] else "same-id")),
                           "ov_own": ov_own, "ov_pair": ov_pair}
            tag = ("PERM" if w["permanent"] else ("new-id" if w["new_id"] else "same-id"))
            ovtag = ("own" if ov_own else "") + ("+tt" if ov_pair else "")
            descs.append(f"[{w['start']}-{w['end']} {w['dur']}s {w['before']}->{w['after']} {tag} ov:{ovtag or 'no'}]")
        print(f"  {tid}: {cpa_d:.0f}m@{cpa_k} | " + (" ".join(descs) if descs else "(no losses)"))
    lg = (f"{longest['dur']}s ({longest.get('tag')}, truth {longest.get('truth')} "
          f"@scan {longest.get('start')}, ov_own={longest.get('ov_own')})"
          if longest["dur"] else "none")
    print(f"SUMMARY: {total_windows} loss windows, total {total_loss}s lost, "
          f"{total_newid} re-confirm new-id, {total_perm} permanent, "
          f"any overlap CPA={any_overlap}")
    print(f"         longest loss={lg}; substantial(>=10s)={subst} "
          f"({subst_ov_own} overlap own-ship CPA)")


if __name__ == "__main__":
    main()
