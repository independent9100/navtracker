#!/usr/bin/env python3
"""Per-event id-switch / break forensics for the Imazu battery (backlog #11).

Re-implements the 2026-06-12 "switch forensics" method (which was a throwaway
analysis, never committed) on the bench ``--export-states-dir`` output. That
CSV (``scan,time_s,kind,id,east_m,north_m``; one row per truth object and one
per confirmed track per scan) is fully scenario-agnostic, so NO core change is
needed to point the method at ``imazu_*`` / ``sim_multisensor``.

Faithfulness: this script re-derives the SAME per-scan truth->track assignment
the bench uses to count id_switches/track_breaks
(``core/benchmark/Metrics.cpp`` ``assignPerStep`` -> per-scan rectangular
Hungarian on ENU Euclidean distance with a strict gate ``d < assoc_gate_m``
(=100 m), then ``computeContinuity``). The Hungarian is a line-for-line port of
``core/association/Hungarian.cpp`` (Jonker-Volgenant), so tie-breaking matches.
The per-truth switch/break totals produced here are validated against the
bench's own ``id_switches:truth_<id>`` / ``track_breaks:truth_<id>`` rows; when
they match, the event classification below is an exact decomposition of the
reported metric.

Each id-switch (a contiguous assigned->assigned change of the assigned track id
for one truth) is classified into exactly one bucket (priority swap > conveyor >
flicker > other):
  swap     : two truths mutually exchange two tracks at a close pass
             (g: X->Y at scan k while h: Y->X at scan ~k).
  conveyor : duplicate-birth handoff -- the truth hands off from an old track to
             a track that was BORN within --window scans of the switch, and/or
             the old track DIES within --window scans after it (track turnover).
  flicker  : near-tie flip between two co-existing established tracks
             (|d1 - d2| <= --flicker-margin m, no birth/death turnover).
  other    : residual.

Breaks (assigned->gap while the truth is present) are censused separately as
break+re-confirm (same-id recovery vs new-id re-confirm).

Usage:
  imazu_switch_forensics.py --states <states.csv> [--meta <meta.txt>]
      [--bench-csv <metrics.csv> --config <label> --scenario <label>]
      [--gate 100] [--window 6] [--flicker-margin 5] [--swap-tol 1] [--json]
"""
import argparse
import csv
import json
import math
from collections import defaultdict


# ----------------------------------------------------------------------------
# Faithful port of core/association/Hungarian.cpp (rectangular JV / LSAP).
# cost is an N x M list of lists; forbidden cells are math.inf.
# Returns row_to_col (len N), -1 where the row got no feasible (finite) column.
# ----------------------------------------------------------------------------
def hungarian_assignment(cost):
    N = len(cost)
    M = len(cost[0]) if N else 0
    if N == 0 or M == 0:
        return [-1] * N
    K = max(N, M)
    max_finite = 0.0
    for i in range(N):
        for j in range(M):
            c = cost[i][j]
            if math.isfinite(c) and abs(c) > max_finite:
                max_finite = abs(c)
    BIG_M = 1.0 + max_finite * 1e3 + 1e9
    C = [[BIG_M] * K for _ in range(K)]
    for i in range(N):
        for j in range(M):
            c = cost[i][j]
            C[i][j] = c if math.isfinite(c) else BIG_M
    for i in range(N, K):
        for j in range(K):
            C[i][j] = 0.0
    for j in range(M, K):
        for i in range(K):
            C[i][j] = 0.0

    INF = math.inf
    u = [0.0] * (K + 1)
    v = [0.0] * (K + 1)
    p = [0] * (K + 1)
    way = [0] * (K + 1)
    for i in range(1, K + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (K + 1)
        used = [False] * (K + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            for j in range(1, K + 1):
                if used[j]:
                    continue
                cur = C[i0 - 1][j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(0, K + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    row_to_col = [-1] * N
    for j in range(1, K + 1):
        i = p[j] - 1
        if i < 0 or i >= N:
            continue
        if j - 1 < M:
            row_to_col[i] = j - 1
    return row_to_col


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ----------------------------------------------------------------------------
# Load states CSV preserving per-scan row order (order affects JV tie-breaking,
# matching the C++ which walks step.truth / step.tracks in emission order).
# ----------------------------------------------------------------------------
def load_states(path):
    scans = defaultdict(lambda: {"truth": [], "track": []})
    order = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            k = int(row["scan"])
            if k not in scans:
                order.append(k)
            pos = (float(row["east_m"]), float(row["north_m"]))
            scans[k][row["kind"]].append((int(row["id"]), pos))
    order.sort()
    return [(k, scans[k]) for k in order]


def track_life(steps):
    """first/last scan each track id is seen, and full presence set."""
    first, last = {}, {}
    for k, s in steps:
        for tid, _ in s["track"]:
            if tid not in first:
                first[tid] = k
            last[tid] = k
    return first, last


def assign_all(steps, gate):
    """Per scan: {truth_id -> assigned track_id or None} + positions."""
    per_scan = []
    for k, s in steps:
        truth = s["truth"]
        track = s["track"]
        n, m = len(truth), len(track)
        assigned = {t[0]: None for t in truth}
        if n and m:
            cost = [[math.inf] * m for _ in range(n)]
            for i in range(n):
                for j in range(m):
                    d = dist(truth[i][1], track[j][1])
                    if d < gate:
                        cost[i][j] = d
            r2c = hungarian_assignment(cost)
            for i in range(n):
                j = r2c[i]
                if j >= 0 and math.isfinite(cost[i][j]):
                    assigned[truth[i][0]] = track[j][0]
        per_scan.append({
            "scan": k,
            "assigned": assigned,
            "truth_pos": {t[0]: t[1] for t in truth},
            "track_pos": {t[0]: t[1] for t in track},
        })
    return per_scan


def continuity_events(per_scan):
    """Replicate computeContinuity's walk; emit switch and break events."""
    prev = {}       # truth_id -> last assigned track id (None when in gap)
    in_gap = defaultdict(lambda: True)
    switches = []   # (truth_id, scan, from_id, to_id)
    breaks = []     # (truth_id, scan, from_id)
    per_truth_sw = defaultdict(int)
    per_truth_br = defaultdict(int)
    present_ids = set()
    for st in per_scan:
        seen = set()
        for tid, a in st["assigned"].items():
            seen.add(tid)
            present_ids.add(tid)
            if a is not None:
                if in_gap[tid]:
                    in_gap[tid] = False
                if tid in prev and prev[tid] is not None and prev[tid] != a:
                    switches.append((tid, st["scan"], prev[tid], a))
                    per_truth_sw[tid] += 1
                prev[tid] = a
            else:
                if not in_gap[tid]:
                    breaks.append((tid, st["scan"], prev.get(tid)))
                    per_truth_br[tid] += 1
                    in_gap[tid] = True
                prev[tid] = None
        # truth ids absent this scan: reset walk (no scoring)
        for tid in list(prev.keys()):
            if tid not in seen:
                in_gap[tid] = True
                prev[tid] = None
    return switches, breaks, dict(per_truth_sw), dict(per_truth_br), present_ids


def classify(switches, per_scan, births, deaths, window, flicker_margin,
             swap_tol):
    by_scan_idx = {st["scan"]: st for st in per_scan}
    # detect mutual swap pairs
    swap_set = set()
    for a in range(len(switches)):
        g, k, x, y = switches[a]
        for b in range(len(switches)):
            if b == a:
                continue
            h, k2, x2, y2 = switches[b]
            if h != g and abs(k - k2) <= swap_tol and x2 == y and y2 == x:
                swap_set.add(a)
                swap_set.add(b)
    out = []
    for idx, (g, k, x, y) in enumerate(switches):
        st = by_scan_idx[k]
        tpos = st["truth_pos"].get(g)
        # d1 = dist to the newly-assigned track y; d2 = 2nd-nearest track < gate
        d1 = d2 = math.inf
        if tpos is not None:
            d1 = dist(tpos, st["track_pos"][y]) if y in st["track_pos"] else math.inf
            others = sorted(dist(tpos, p) for tid, p in st["track_pos"].items() if tid != y)
            d2 = others[0] if others else math.inf
        y_born = births.get(y, k)
        x_death = deaths.get(x, k)
        turnover = (y_born >= k - window) or (x_death <= k + window)
        near_tie = math.isfinite(d1) and math.isfinite(d2) and abs(d1 - d2) <= flicker_margin
        if idx in swap_set:
            cat = "swap"
        elif turnover:
            cat = "conveyor"
        elif near_tie:
            cat = "flicker"
        else:
            cat = "other"
        out.append({
            "truth": g, "scan": k, "from": x, "to": y, "cat": cat,
            "d1": round(d1, 2) if math.isfinite(d1) else None,
            "d2": round(d2, 2) if math.isfinite(d2) else None,
            "y_age_at_switch": k - y_born, "x_dies_after": x_death - k,
        })
    return out


def break_census(breaks, per_scan):
    """For each break, does the truth re-confirm to the SAME or a NEW id?"""
    # build per-truth ordered assignment stream (only present scans)
    stream = defaultdict(list)
    for st in per_scan:
        for tid, a in st["assigned"].items():
            stream[tid].append((st["scan"], a))
    same, new, no_recover = 0, 0, 0
    for (g, k, from_id) in breaks:
        seq = stream[g]
        # find next assigned track after scan k
        nxt = None
        for (s, a) in seq:
            if s > k and a is not None:
                nxt = a
                break
        if nxt is None:
            no_recover += 1
        elif nxt == from_id:
            same += 1
        else:
            new += 1
    return {"total": len(breaks), "recover_same_id": same,
            "reconfirm_new_id": new, "no_recover": no_recover}


def truth_geometry(steps, meta):
    """Pairwise truth-truth CPA (min distance + scan) for context."""
    ids = sorted({t[0] for _, s in steps for t in s["truth"]})
    pos_by_scan = {k: {t[0]: t[1] for t in s["truth"]} for k, s in steps}
    pairs = {}
    for a_i in range(len(ids)):
        for b_i in range(a_i + 1, len(ids)):
            A, B = ids[a_i], ids[b_i]
            best_d, best_k = math.inf, None
            for k in pos_by_scan:
                pa = pos_by_scan[k].get(A)
                pb = pos_by_scan[k].get(B)
                if pa and pb:
                    d = dist(pa, pb)
                    if d < best_d:
                        best_d, best_k = d, k
            pairs[f"{A}-{B}"] = {"cpa_m": round(best_d, 1), "cpa_scan": best_k}
    return {"truth_ids": ids, "pairs": pairs, "encounters": meta}


def parse_meta(path):
    enc = {}
    if not path:
        return enc
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith("mmsi="):
                    parts = dict(p.split("=", 1) for p in line.split() if "=" in p)
                    enc[int(parts["mmsi"])] = parts.get("encounter", "?")
    except OSError:
        pass
    return enc


def bench_per_truth(bench_csv, config, scenario):
    """Read id_switches:truth_<id> / track_breaks:truth_<id> from metrics CSV."""
    sw, br = {}, {}
    if not bench_csv:
        return sw, br
    with open(bench_csv) as f:
        for line in f:
            if line.startswith("#") or line.startswith("run_id,"):
                continue
            parts = line.rstrip("\n").split(",")
            if len(parts) < 6:
                continue
            _, cfg, scn, _seed, metric, val = parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]
            if cfg != config or scn != scenario:
                continue
            if metric.startswith("id_switches:truth_"):
                sw[int(metric.split("_")[-1])] = float(val)
            elif metric.startswith("track_breaks:truth_"):
                br[int(metric.split("_")[-1])] = float(val)
    return sw, br


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--states", required=True)
    ap.add_argument("--meta")
    ap.add_argument("--bench-csv")
    ap.add_argument("--config", default="")
    ap.add_argument("--scenario", default="")
    ap.add_argument("--gate", type=float, default=100.0)
    ap.add_argument("--window", type=int, default=6)
    ap.add_argument("--flicker-margin", type=float, default=5.0)
    ap.add_argument("--swap-tol", type=int, default=1)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    steps = load_states(args.states)
    births, deaths = track_life(steps)
    per_scan = assign_all(steps, args.gate)
    switches, breaks, pt_sw, pt_br, present_ids = continuity_events(per_scan)
    events = classify(switches, per_scan, births, deaths, args.window,
                      args.flicker_margin, args.swap_tol)
    brk = break_census(breaks, per_scan)
    meta = parse_meta(args.meta)
    geom = truth_geometry(steps, {str(k): v for k, v in meta.items()})

    n_tracks = len(births)
    n_truth = len(geom["truth_ids"])
    cats = defaultdict(int)
    for e in events:
        cats[e["cat"]] += 1
    total_sw = len(events)

    # validation against bench per-truth rows
    bench_sw, bench_br = bench_per_truth(args.bench_csv, args.config, args.scenario)
    val = {}
    if bench_sw:
        mine = {tid: pt_sw.get(tid, 0) for tid in bench_sw}
        val["switch_match"] = all(abs(mine[t] - bench_sw[t]) < 1e-6 for t in bench_sw)
        val["mine_switch"] = mine
        val["bench_switch"] = {t: bench_sw[t] for t in bench_sw}
        val["mine_switch_mean"] = round(sum(mine.values()) / max(1, len(mine)), 3)
        val["bench_switch_mean"] = round(sum(bench_sw.values()) / max(1, len(bench_sw)), 3)
    if bench_br:
        mineb = {tid: pt_br.get(tid, 0) for tid in bench_br}
        val["break_match"] = all(abs(mineb[t] - bench_br[t]) < 1e-6 for t in bench_br)
        val["mine_break_mean"] = round(sum(mineb.values()) / max(1, len(mineb)), 3)
        val["bench_break_mean"] = round(sum(bench_br.values()) / max(1, len(bench_br)), 3)

    result = {
        "scenario": args.scenario or args.states,
        "config": args.config,
        "n_truth": n_truth,
        "n_distinct_track_ids": n_tracks,
        "tracks_per_truth": round(n_tracks / max(1, n_truth), 1),
        "total_switch_events": total_sw,
        "switch_census": dict(cats),
        "switch_census_pct": {k: round(100 * v / max(1, total_sw), 1) for k, v in cats.items()},
        "break_census": brk,
        "validation": val,
        "geometry": geom,
        "events": events,
    }

    if args.json:
        print(json.dumps(result, indent=2))
        return

    print(f"=== {result['scenario']}  ({args.config}) ===")
    print(f"truths={n_truth}  distinct track ids={n_tracks} "
          f"({result['tracks_per_truth']} per truth)")
    if val:
        ok = val.get("switch_match")
        print(f"per-truth switches mine={val.get('mine_switch_mean')} "
              f"bench={val.get('bench_switch_mean')}  MATCH={ok}")
        if "break_match" in val:
            print(f"per-truth breaks   mine={val.get('mine_break_mean')} "
                  f"bench={val.get('bench_break_mean')}  MATCH={val.get('break_match')}")
    print(f"total id-switch events={total_sw}")
    for c in ("swap", "conveyor", "flicker", "other"):
        n = cats.get(c, 0)
        print(f"   {c:9s}: {n:4d}  ({result['switch_census_pct'].get(c, 0.0)}%)")
    print(f"breaks: {brk['total']} total -> same-id {brk['recover_same_id']}, "
          f"new-id {brk['reconfirm_new_id']}, no-recover {brk['no_recover']}")
    print("truth-truth CPA (m @ scan):")
    for pr, d in geom["pairs"].items():
        print(f"   {pr}: {d['cpa_m']} m @ scan {d['cpa_scan']}")
    if meta:
        print("encounters:", {k: v for k, v in meta.items()})


if __name__ == "__main__":
    main()
