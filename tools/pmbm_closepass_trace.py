#!/usr/bin/env python3
"""Scan-by-scan PMBM close-pass track-death trace (backlog #25, Phase 1).

Localizes WHY PMBM (`imm_cv_ct_pmbm_coverage_land`) drops a target at a
sustained close pass (Imazu Q2b): does the dying Bernoulli **miss-starve**
(H1: existence r decays smoothly below the output floor while in-gate
measurements exist but the neighbour claims them), die **abruptly** (H2:
r healthy then a prune/cap event drops it in one step), or **diverge in
state first** (H3: the estimate walks off during the ambiguity, the gate
follows it away from the true return, then it coasts / flies off and the
truth is left unassigned)?

Inputs (all from one bench run of the config on one scenario):
  --states     <..>.states.csv     (positions; the existing D2 export)
  --diag-bern  <..>.pmbmbern.csv    (per-scan per-identity MBM diagnostics)
  --diag-scan  <..>.pmbmscan.csv    (per-scan structural events)
  --ownship    fixture ownship.csv  (for CPA timing)
Emits from `navtracker_bench_baseline --export-pmbm-diag-dir` (default-off,
byte-identical diagnostic hook — see PmbmDiagnostics.hpp).

The states.csv `scan` index is the 1 Hz truth-group tick (scan == second);
the diag rows are per processBatch (radar every 2.5 s + AIS at fractional
times), so they are joined to the truth timeline by an AS-OF join (the last
diag row with time <= the truth-scan time) — exactly the density the states
snapshot reads. A faithfulness anchor verifies the diag `confirmed` flag
reproduces the states.csv confirmed-track set.

stdlib only; float() is locale-independent (unlike awk under LC_NUMERIC).
"""
import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all  # noqa: E402
from imazu_trackloss import load_ownship_enu, own_at, loss_windows, cpa_scans  # noqa: E402


def load_diag_bern(path):
    """Return (by_id, times, confirmed_by_time, output_by_time).

    by_id[id] = list of per-scan dicts sorted by time; times = sorted distinct
    diag scan-times; confirmed_by_time[t] / output_by_time[t] = set of ids that
    are Confirmed (mass>=confirm_threshold) / in-output (mass>=floor) at time t.
    """
    by_id = {}
    confirmed_by_time = {}
    output_by_time = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["time_s"])
            iid = int(r["id"])
            row = {
                "time": t,
                "id": iid,
                "agg_mass": float(r["agg_mass"]),
                "r_best": float(r["r_best"]),
                "hyp_count": int(r["hyp_count"]),
                "claimed": int(r["claimed_meas"]),
                "east": float(r["east_m"]),
                "north": float(r["north_m"]),
                "speed": float(r["speed_mps"]),
                "in_dom": r["in_dominant"] == "1",
                "in_output": r["in_output"] == "1",
                "confirmed": r["confirmed"] == "1",
            }
            by_id.setdefault(iid, []).append(row)
            if row["confirmed"]:
                confirmed_by_time.setdefault(t, set()).add(iid)
            if row["in_output"]:
                output_by_time.setdefault(t, set()).add(iid)
    for iid in by_id:
        by_id[iid].sort(key=lambda x: x["time"])
    times = sorted(set(confirmed_by_time) | set(output_by_time) |
                   {row["time"] for rows in by_id.values() for row in rows})
    return by_id, times, confirmed_by_time, output_by_time


def load_diag_scan(path):
    """time -> per-scan structural-event dict."""
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            out[float(r["time_s"])] = {
                "n_meas": int(r["n_meas"]),
                "n_hyp": int(r["n_hyp"]),
                "n_bernoulli": int(r["n_bernoulli"]),
                "n_ids": int(r["n_ids"]),
                "floor": int(r["hyp_dropped_floor"]),
                "cap": int(r["hyp_dropped_cap"]),
                "rmin": int(r["bernoulli_pruned_rmin"]),
            }
    return out


def asof(times, t):
    """Largest diag time <= t (the density the truth-scan snapshot reads)."""
    i = bisect.bisect_right(times, t + 1e-9) - 1
    return times[i] if i >= 0 else None


def diag_row_asof(by_id, iid, t):
    """The id's diag row as-of truth-scan time t (last row with time<=t)."""
    rows = by_id.get(iid)
    if not rows:
        return None
    ts = [r["time"] for r in rows]
    i = bisect.bisect_right(ts, t + 1e-9) - 1
    return rows[i] if i >= 0 else None


def truth_pos_at(steps, tid, scan):
    for k, s in steps:
        if k == scan:
            for i, p in s["truth"]:
                if i == tid:
                    return p
    return None


def faithfulness(steps, by_id, times, confirmed_by_time):
    """Anchor the diag against the states.csv export.

    Containment: every CONFIRMED track in states.csv must appear in the diag
    (as-of that scan) with agg_mass>=0.5 and a matching position. The reverse is
    NOT required: the diag deliberately surfaces MORE than states — sub-floor
    Bernoullis (the point of the hook) and Coasting-status tracks, which
    snapshotAtPmbm drops via its `status==Confirmed` filter while
    refreshAggregatedTracks marks cooperative-overdue ids Coasting
    (PmbmTracker.cpp:2226) even though their mass stays >=0.5. So the anchor is
    states (subset-of) diag (containment) + position agreement, plus a census of the
    expected extras."""
    contained = 0
    states_tracks = 0
    max_pos_err = 0.0
    extras_total = 0
    scans_with_extras = 0
    miss = []  # states tracks NOT found in diag (should be none)
    for k, s in steps:
        t = asof(times, float(k))
        diag_conf = set(confirmed_by_time.get(t, set())) if t is not None else set()
        states_ids = {tid: pos for tid, pos in s["track"]}
        for tid, pos in states_ids.items():
            states_tracks += 1
            row = diag_row_asof(by_id, tid, float(k)) if t is not None else None
            if row is not None and row["agg_mass"] >= 0.5 - 1e-9:
                contained += 1
                err = math.hypot(row["east"] - pos[0], row["north"] - pos[1])
                max_pos_err = max(max_pos_err, err)
            elif len(miss) < 8:
                miss.append((k, tid))
        extras = diag_conf - set(states_ids)
        if extras:
            extras_total += len(extras)
            scans_with_extras += 1
    return {
        "contained": contained, "states_tracks": states_tracks,
        "max_pos_err": max_pos_err, "extras_total": extras_total,
        "scans_with_extras": scans_with_extras, "miss": miss,
    }


def classify(trace, win, meas_near):
    """Heuristic mechanism label for one loss window from the dying id's trace.

    trace: list of per-second dicts {scan,mass,r,claimed,speed,dist} over the
    loss window (as-of join). meas_near: True if raw measurements existed near
    the truth during the loss (in-gate returns available)."""
    if not trace:
        return "no-diag", []
    ev = []
    # H3: state divergence — r stays high (>=0.5) while the track's distance
    # from its truth blows up (>200 m) and/or speed >> target speed (~4 m/s).
    diverged = [x for x in trace if x["r"] >= 0.5 and (x["dist"] is not None and
                x["dist"] > 200) and x["speed"] > 50]
    # H1: miss-starvation — mass decays smoothly (monotone-ish) across many
    # scans through the floor while the track is misdetected (claimed==-1).
    misses = [x for x in trace if x["claimed"] == -1]
    smooth_decay = (trace[0]["mass"] > 0.5 and trace[-1]["mass"] < 0.1 and
                    len([1 for a, b in zip(trace, trace[1:])
                         if b["mass"] <= a["mass"] + 1e-9]) >= 0.7 * (len(trace) - 1))
    # H2: abrupt — a single-scan mass cliff (>0.4 drop) coincident with a
    # structural prune/cap event.
    cliffs = [(a["scan"], b["scan"]) for a, b in zip(trace, trace[1:])
              if a["mass"] - b["mass"] > 0.4]
    if diverged:
        onset = trace[:5]  # loss-onset speeds/dists are interpretable; the
        # CV/CT velocity state then runs off unbounded (>1e4 m/s) if the track
        # keeps coasting, so report onset + a bounded "diverges unbounded" note.
        max_spd = max(x["speed"] for x in trace)
        ev.append(
            f"H3 divergence: {len(diverged)}/{len(trace)} loss scans with r>=0.5 "
            f"& dist>200m & speed>50 m/s (true target ~4 m/s). At loss onset "
            f"speed={[round(x['speed']) for x in onset]} m/s, "
            f"dist={[round(x['dist']) if x['dist'] is not None else None for x in onset]} m; "
            + ("velocity state then diverges unbounded (>1e4 m/s)."
               if max_spd > 1e4 else f"max speed {max_spd:.0f} m/s."))
    if smooth_decay:
        ev.append(f"H1 smooth mass decay {trace[0]['mass']:.2f}->{trace[-1]['mass']:.2f} "
                  f"over {len(trace)} scans, {len(misses)} misses"
                  + ("; measurements present near truth" if meas_near else ""))
    if cliffs:
        ev.append(f"H2 mass cliff(s) at scans {cliffs[:3]}")
    label = "+".join(sorted({e.split()[0] for e in ev})) or "unclassified"
    return label, ev


def load_radar_plots(path):
    """scan(second) -> list of (east,north) radar returns, if a plots CSV is
    given (fixture radar_plots.csv). Used to test whether in-gate measurements
    existed near the truth during a loss (the H1 'measurements exist' clause)."""
    if not path or not os.path.exists(path):
        return None
    out = {}
    with open(path) as f:
        rdr = csv.DictReader(f)
        cols = rdr.fieldnames or []
        # radar_plots.csv columns vary; support (t, east_m, north_m) or range/bearing.
        for r in rdr:
            t = None
            for tc in ("unix_time", "time_s", "t", "tod"):
                if tc in r:
                    t = float(r[tc]); break
            if t is None:
                continue
            e = n = None
            if "east_m" in r and "north_m" in r:
                e, n = float(r["east_m"]), float(r["north_m"])
            if e is None:
                continue
            out.setdefault(round(t), []).append((e, n))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--states", required=True)
    ap.add_argument("--diag-bern", required=True)
    ap.add_argument("--diag-scan", required=True)
    ap.add_argument("--ownship", required=True)
    ap.add_argument("--radar-plots", default="", help="optional fixture radar_plots.csv")
    ap.add_argument("--datum", default="63.45,10.35")
    ap.add_argument("--gate", type=float, default=100.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--truth", type=int, default=0,
                    help="truth id to trace (0 = auto: the longest-loss truth)")
    ap.add_argument("--pad", type=int, default=12, help="scans of context around window")
    args = ap.parse_args()

    lat0, lon0 = (float(x) for x in args.datum.split(","))
    steps = load_states(args.states)
    per_scan = assign_all(steps, args.gate)
    by_id, dtimes, confirmed_by_time, output_by_time = load_diag_bern(args.diag_bern)
    dscan = load_diag_scan(args.diag_scan)
    own = load_ownship_enu(args.ownship, lat0, lon0)
    ids, own_cpa, pair_cpa = cpa_scans(steps, own)
    radar = load_radar_plots(args.radar_plots)

    print(f"================ {args.label or args.states} ================")

    # --- Faithfulness anchor: states confirmed tracks (subset-of) diag ---
    fa = faithfulness(steps, by_id, dtimes, confirmed_by_time)
    print(f"[faithfulness] states confirmed tracks contained in diag (mass>=0.5, "
          f"pos match): {fa['contained']}/{fa['states_tracks']} "
          f"({100.0*fa['contained']/max(1,fa['states_tracks']):.2f}%), "
          f"max pos err {fa['max_pos_err']:.3f} m")
    print(f"    diag extras (sub-floor/Coasting Bernoulli ids not in states, by "
          f"design): {fa['extras_total']} over {fa['scans_with_extras']} scans")
    for k, tid in fa["miss"][:5]:
        print(f"    NOT-CONTAINED (states track missing from diag) scan {k} id {tid}")

    # --- Pick the truth to trace ---
    per_scan_by_scan = {p["scan"]: p for p in per_scan}
    truth_windows = {tid: loss_windows(per_scan, tid) for tid in ids}
    if args.truth:
        target = args.truth
    else:
        target, _ = max(
            ((tid, max((w["dur"] for w in ws), default=0))
             for tid, ws in truth_windows.items()),
            key=lambda kv: kv[1])
    wins = truth_windows[target]
    cpa_k, cpa_d = own_cpa[target]
    print(f"\n[trace] truth {target} (own-ship CPA {cpa_d:.0f} m @ scan {cpa_k}); "
          f"{len(wins)} loss windows")
    # the window we deep-trace: the longest
    win = max(wins, key=lambda w: w["dur"]) if wins else None
    if not win:
        print("  no loss windows for this truth"); return
    print(f"  longest loss window: scan {win['start']}-{win['end']} "
          f"({win['dur']}s), {win['before']}->{win['after']} "
          f"({'PERM' if win['permanent'] else 'new-id' if win['new_id'] else 'same-id'})")
    dying = win["before"]   # the track id serving the truth just before the loss
    print(f"  dying track (id assigned just before the loss) = BernoulliId {dying}")

    # --- Scan-by-scan trace over [start-pad, end+pad] ---
    lo, hi = win["start"] - args.pad, win["end"] + args.pad
    print(f"\n  scan | t | truth_assigned | nConf | dying id {dying}: mass  r     clm  spd(m/s) dist_to_truth | struct(floor/cap/rmin) nBern")
    trace = []
    meas_near = False
    for k in range(max(0, lo), hi + 1):
        ps = per_scan_by_scan.get(k)
        if ps is None:
            continue
        assigned = ps["assigned"].get(target)
        n_conf = len(ps["track_pos"])
        tp = truth_pos_at(steps, target, k)
        row = diag_row_asof(by_id, dying, float(k))
        t_as = asof(dtimes, float(k))
        sc = dscan.get(t_as, {}) if t_as is not None else {}
        if row is not None and tp is not None:
            dist = math.hypot(row["east"] - tp[0], row["north"] - tp[1])
        else:
            dist = None
        # measurements near the truth this scan?
        if radar is not None and tp is not None:
            near = [p for p in radar.get(k, []) if math.hypot(p[0]-tp[0], p[1]-tp[1]) < 100]
            if near and win["start"] <= k <= win["end"]:
                meas_near = True
        if win["start"] <= k <= win["end"] and row is not None:
            trace.append({"scan": k, "mass": row["agg_mass"], "r": row["r_best"],
                          "claimed": row["claimed"], "speed": row["speed"], "dist": dist})
        mark = "  <-LOSS" if (win["start"] <= k <= win["end"]) else ""
        if row is None:
            rowstr = "  (id absent from density)"
        else:
            rowstr = (f"m={row['agg_mass']:.3f} r={row['r_best']:.3f} "
                      f"clm={row['claimed']:>3} spd={row['speed']:>7.0f} "
                      f"dist={dist:>8.0f}" if dist is not None else
                      f"m={row['agg_mass']:.3f} r={row['r_best']:.3f} clm={row['claimed']}")
        print(f"  {k:>4} {k:>4} {str(assigned):>6} {n_conf:>3}   {rowstr:<52} "
              f"f/c/r={sc.get('floor',0)}/{sc.get('cap',0)}/{sc.get('rmin',0)} nB={sc.get('n_bernoulli','?')}{mark}")

    # --- Structural events summed over the loss window ---
    tot = {"floor": 0, "cap": 0, "rmin": 0}
    for t, sc in dscan.items():
        if win["start"] <= t <= win["end"] + 1:
            for kk in tot:
                tot[kk] += sc[kk]
    print(f"\n  structural events during loss window "
          f"[{win['start']}-{win['end']}]: hyp_dropped_floor={tot['floor']} "
          f"hyp_dropped_cap={tot['cap']} bernoulli_pruned_rmin={tot['rmin']}")

    # --- Confirmed-track count vs truth count across the window ---
    print("\n  confirmed-track-count vs present-truth-count across window:")
    counts = []
    for k in range(max(0, lo), hi + 1):
        ps = per_scan_by_scan.get(k)
        if ps is None:
            continue
        counts.append((k, len(ps["track_pos"]), len(ps["truth_pos"])))
    lo_c = min(c for _, c, _ in counts) if counts else 0
    hi_t = max(t for _, _, t in counts) if counts else 0
    print(f"    min confirmed tracks = {lo_c} for up to {hi_t} present truths "
          f"(collapse ratio {lo_c}/{hi_t})")

    # --- Classification ---
    label, ev = classify(trace, win, meas_near)
    print(f"\n  >>> mechanism for truth {target} longest window: {label}")
    for e in ev:
        print(f"        - {e}")
    if radar is not None:
        print(f"        - in-gate radar returns near truth during loss: "
              f"{'YES' if meas_near else 'no/none-loaded'}")


if __name__ == "__main__":
    main()
