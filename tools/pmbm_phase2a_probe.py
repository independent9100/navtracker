#!/usr/bin/env python3
"""Backlog #25 Phase 2a — runaway census + offline velocity/innovation-bound probe.

Consumes the EXISTING Phase-1 diagnostic exports (no core change):
  <dir>/states/<config>__<scenario>__seed0.states.csv      (truth+track positions)
  <dir>/diag/<config>__<scenario>__seed0.pmbmbern.csv       (per-scan per-identity MBM)
  <dir>/diag/<config>__<scenario>__seed0.pmbmscan.csv       (per-scan structural events)

Answers, entirely OFFLINE (the lever is simulated on paper — no estimator change):

  QUESTION A — the runaway census. Over every confirmed-track row with implied
  speed > SPEED_CENSUS m/s, attribute it:
    (a) target-born vs clutter-born track  (truth distance at the track's birth)
    (b) contested scan (>=2 confirmed tracks within CONTEST_R m) vs uncontested
    (c) dominant IMM mode                   -- NOT EXPORTED (see stop-and-report)
    (d) update cadence: dt since the track's last DETECTED update (claimed>=0).

  QUESTION B — would a velocity/innovation bound work? Sweep
    V_max in {25,50,75} m/s   (implied-speed bound, from speed_mps directly)
    D_max in {100,200,400} m  (position-innovation PROXY = per-diag-row position
                               jump ||pos(k)-pos(k-1)||; see the coupling note).
  Detection side: for each of the 6 dying tracks, the first scan the bound flags
  the diverging track, vs (i) the scan its estimate leaves the 100 m gate and
  (ii) the own-ship CPA. False-fire side: fraction of HEALTHY confirmed
  track-scans (track within GATE m of a truth) the bound would flag, per config
  per dataset.

  BINDING KILL-CRITERIA (agreed before measuring): the lever graduates to a
  Phase-2b build ONLY if some (V_max,D_max) flags >=5 of 6 dying tracks BEFORE
  gate-exit AND false-fires on <1% of healthy confirmed track-scans on
  autoferry+sim_ms. Else: negative -> rank the alternates.

stdlib only; float() is locale-independent (unlike awk under LC_NUMERIC).
"""
import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all           # noqa: E402
from imazu_trackloss import load_ownship_enu, loss_windows, cpa_scans  # noqa: E402
from pmbm_closepass_trace import load_diag_bern, load_diag_scan, asof  # noqa: E402

# --- knobs (documented in the baseline doc) ---
SPEED_CENSUS = 50.0     # m/s: "runaway" row threshold for the census
BORN_GATE = 100.0       # m: born within this of a truth => target-born, else clutter-born
CONTEST_R = 200.0       # m: >=2 confirmed tracks within this of each other => contested
GATE = 100.0            # m: match gate (track "within gate of a truth" => healthy/assigned)
V_MAX = [25.0, 50.0, 75.0]
D_MAX = [100.0, 200.0, 400.0]

# The six dying (scenario, truth_id, dying_track_id) from Phase 1
# (docs/baselines/2026-07-08_b25_localization.md "All six dying tracks").
DYING = [
    ("imazu_15", 257010151, 6),
    ("imazu_15", 257010152, 7),
    ("imazu_15", 257010153, 517),
    ("imazu_22", 257010221, 7),
    ("imazu_22", 257010222, 7),
    ("imazu_22", 257010223, 1),
]

IMAZU_DATUM = (63.45, 10.35)


# ------------------------------------------------------------------ helpers
def truth_index(steps):
    """scan -> [(tid,(e,n))]; and sorted scan list (scan == second, 1 Hz)."""
    idx = {k: s["truth"] for k, s in steps}
    return idx, sorted(idx)


def nearest_truth(idx, scan_keys, t):
    """(min dist from (e,n) is computed by caller); here return the truth list
    at the state-scan nearest to time t."""
    if not scan_keys:
        return []
    # scan == second; snap to nearest integer scan that exists
    k = int(round(t))
    if k in idx:
        return idx[k]
    # fallback: nearest available scan
    import bisect
    i = bisect.bisect_left(scan_keys, k)
    cand = []
    if i < len(scan_keys):
        cand.append(scan_keys[i])
    if i > 0:
        cand.append(scan_keys[i - 1])
    if not cand:
        return []
    kk = min(cand, key=lambda x: abs(x - k))
    return idx[kk]


def min_truth_dist(idx, scan_keys, e, n, t):
    """min distance from (e,n) to any truth present near time t; (dist, tid)."""
    best = (math.inf, None)
    for tid, p in nearest_truth(idx, scan_keys, t):
        d = math.hypot(e - p[0], n - p[1])
        if d < best[0]:
            best = (d, tid)
    return best


def birth_class(by_id, idx, scan_keys):
    """id -> ('target'|'clutter', born_dist, born_time, n_rows, lifetime_s)."""
    out = {}
    for iid, rows in by_id.items():
        b = rows[0]
        d, _ = min_truth_dist(idx, scan_keys, b["east"], b["north"], b["time"])
        cls = "target" if d <= BORN_GATE else "clutter"
        life = rows[-1]["time"] - rows[0]["time"]
        out[iid] = (cls, d, b["time"], len(rows), life)
    return out


def confirmed_positions_at(by_id, t):
    """[(id,(e,n))] for ids whose as-of diag row at time t is confirmed."""
    import bisect
    out = []
    for iid, rows in by_id.items():
        ts = [r["time"] for r in rows]
        i = bisect.bisect_right(ts, t + 1e-9) - 1
        if i >= 0 and rows[i]["confirmed"] and abs(rows[i]["time"] - t) < 1e-6:
            out.append((iid, (rows[i]["east"], rows[i]["north"])))
    return out


def with_jump(rows):
    """annotate each diag row with 'jump' = ||pos - prev pos|| (m), dt to prev,
    and 'gap' = dt since last DETECTED update (claimed>=0)."""
    prev = None
    last_det_t = None
    for r in rows:
        r["jump"] = (math.hypot(r["east"] - prev["east"], r["north"] - prev["north"])
                     if prev else 0.0)
        r["dt"] = (r["time"] - prev["time"]) if prev else 0.0
        r["gap"] = (r["time"] - last_det_t) if last_det_t is not None else 0.0
        if r["claimed"] >= 0:
            last_det_t = r["time"]
        prev = r
    return rows


def flagged(r, vmax, dmax, mode="or"):
    """mode: 'v' (speed only), 'd' (position-jump only), 'or' (either)."""
    v = r["speed"] > vmax
    d = r["jump"] > dmax
    if mode == "v":
        return v
    if mode == "d":
        return d
    return v or d


# ------------------------------------------------------------------ loading
def scenario_triplets(states_dir, diag_dir, config):
    """[(scenario, states_path, bern_path, scan_path)] for a config."""
    out = []
    for sp in sorted(glob.glob(os.path.join(states_dir, f"{config}__*.states.csv"))):
        base = os.path.basename(sp)[:-len(".states.csv")]
        scen = base.split("__")[1]
        bp = os.path.join(diag_dir, base + ".pmbmbern.csv")
        cp = os.path.join(diag_dir, base + ".pmbmscan.csv")
        if os.path.exists(bp) and os.path.exists(cp):
            out.append((scen, sp, bp, cp))
    return out


class Scen:
    def __init__(self, scen, sp, bp, cp):
        self.name = scen
        self.steps = load_states(sp)
        self.by_id, self.dtimes, self.conf_by_t, self.out_by_t = load_diag_bern(bp)
        for iid in self.by_id:
            with_jump(self.by_id[iid])
        self.dscan = load_diag_scan(cp)
        self.idx, self.scan_keys = truth_index(self.steps)
        self.truth_ids = sorted({t[0] for _, s in self.steps for t in s["truth"]})
        self.birth = birth_class(self.by_id, self.idx, self.scan_keys)
        self.single_target = len(self.truth_ids) == 1
        # per-scan set of track ids that are Hungarian-ASSIGNED to a truth
        # (within GATE). "Healthy" for false-fire == a track that IS the
        # assigned tracker of a truth this scan -- NOT merely a fast phantom
        # passing within GATE m of some truth (which we WANT to flag).
        self.assigned_ids = {}
        for ps in assign_all(self.steps, GATE):
            self.assigned_ids[ps["scan"]] = {tid for tid in ps["assigned"].values()
                                             if tid is not None}


# ------------------------------------------------------------------ Question A
def census(scens):
    """Aggregate + per-scenario runaway census. Returns (agg, per_scenario)."""
    per = []
    agg = {"rows": 0, "target": 0, "clutter": 0,
           "contested": 0, "tracks_t": set(), "tracks_c": set(),
           "clutter_lifetimes": [], "target_lifetimes": [],
           "gaps": [], "st_rows": 0, "st_target": 0, "st_clutter": 0}
    for s in scens:
        row_n = tgt = clt = cont = 0
        tset, cset = set(), set()
        for iid, rows in s.by_id.items():
            cls = s.birth[iid][0]
            for r in rows:
                if not r["confirmed"] or r["speed"] <= SPEED_CENSUS:
                    continue
                row_n += 1
                agg["rows"] += 1
                agg["gaps"].append(r["gap"])
                # contested proxy
                cp = confirmed_positions_at(s.by_id, r["time"])
                others = [q for (jid, q) in cp if jid != iid
                          and math.hypot(q[0] - r["east"], q[1] - r["north"]) <= CONTEST_R]
                if others:
                    cont += 1
                    agg["contested"] += 1
                if cls == "target":
                    tgt += 1
                    agg["target"] += 1
                    tset.add(iid)
                    agg["tracks_t"].add((s.name, iid))
                else:
                    clt += 1
                    agg["clutter"] += 1
                    cset.add(iid)
                    agg["tracks_c"].add((s.name, iid))
                if s.single_target:
                    agg["st_rows"] += 1
                    agg["st_target" if cls == "target" else "st_clutter"] += 1
        for iid in tset:
            agg["target_lifetimes"].append(s.birth[iid][4])
        for iid in cset:
            agg["clutter_lifetimes"].append(s.birth[iid][4])
        per.append((s.name, row_n, tgt, clt, cont, len(tset), len(cset),
                    s.single_target, len(s.truth_ids)))
    return agg, per


# ------------------------------------------------------------------ Question B: detection
def gate_exit_time(s, truth_id, dying_id):
    """Returns (permanent_exit_t, first_excursion_t).

    The dying track THRASHES: it leaves the 100 m gate, re-acquires the target,
    leaves again, ... before a final permanent departure. We report both:
      first_excursion_t = first time it leaves the gate after having been inside
                          (onset of trouble);
      permanent_exit_t  = last time it is inside the gate, then leaves and never
                          returns (the point of no return / irreversible loss).
    A useful lever must flag before permanent_exit_t (still recoverable)."""
    rows = s.by_id.get(dying_id, [])
    last_in = None
    exit_t = None
    first_exc = None
    for r in rows:
        d, _ = truth_dist_of(s, truth_id, r)
        if d is not None and d <= GATE:
            last_in = r["time"]
            exit_t = None  # reset; came back in gate
        elif d is not None and d > GATE and last_in is not None:
            if exit_t is None:
                exit_t = r["time"]
            if first_exc is None:
                first_exc = r["time"]
    return exit_t, first_exc


def truth_dist_of(s, truth_id, r):
    """distance from diag row r to a SPECIFIC truth id at r's as-of scan."""
    tp = None
    for tid, p in nearest_truth(s.idx, s.scan_keys, r["time"]):
        if tid == truth_id:
            tp = p
    if tp is None:
        return None, None
    return math.hypot(r["east"] - tp[0], r["north"] - tp[1]), tp


def first_flag_time(s, dying_id, vmax, dmax, after=None, mode="or"):
    for r in s.by_id.get(dying_id, []):
        if after is not None and r["time"] < after:
            continue
        if flagged(r, vmax, dmax, mode):
            return r["time"]
    return None


def detection_probe(scens_by_name):
    """For each dying track and each (V_max,D_max): flag time vs gate-exit + CPA."""
    results = []
    # CPA per scenario (needs ownship)
    for (scen, truth_id, dying_id) in DYING:
        s = scens_by_name.get(scen)
        if s is None:
            results.append((scen, truth_id, dying_id, None, None, None, {}))
            continue
        exit_t, first_exc = gate_exit_time(s, truth_id, dying_id)
        cpa = s.cpa.get(truth_id) if hasattr(s, "cpa") else None
        cfg = {}
        for vmax in V_MAX:
            for dmax in D_MAX:
                ft = first_flag_time(s, dying_id, vmax, dmax)
                # margin: gate-exit time minus flag time (positive => before)
                margin = (exit_t - ft) if (exit_t is not None and ft is not None) else None
                before = (ft is not None and exit_t is not None and ft <= exit_t)
                cfg[(vmax, dmax)] = (ft, margin, before)
        results.append((scen, truth_id, dying_id, exit_t, first_exc, cpa, cfg))
    return results


# ------------------------------------------------------------------ Question B: false-fire
def is_healthy(s, iid, t):
    """The track iid IS the assigned tracker of some truth at the state-scan
    nearest time t (i.e. genuinely well-associated, within GATE)."""
    k = int(round(t))
    ass = s.assigned_ids.get(k)
    if ass is None and s.assigned_ids:
        k = min(s.assigned_ids, key=lambda x: abs(x - k))
        ass = s.assigned_ids.get(k)
    return bool(ass) and iid in ass


def false_fire(scens, mode="or"):
    """per (V_max,D_max): flagged healthy / total healthy confirmed track-rows,
    over the supplied scenarios (aggregate). Healthy == the confirmed track is
    the Hungarian-assigned tracker of a truth this scan (excludes fast phantoms
    that merely wander within GATE m of a truth -- those we WANT to flag)."""
    counts = {(v, d): [0, 0] for v in V_MAX for d in D_MAX}  # [flagged, total]
    per_scen = {}
    for s in scens:
        sc_counts = {(v, d): [0, 0] for v in V_MAX for d in D_MAX}
        for iid, rows in s.by_id.items():
            for r in rows:
                if not r["confirmed"]:
                    continue
                if not is_healthy(s, iid, r["time"]):
                    continue
                for vmax in V_MAX:
                    for dmax in D_MAX:
                        counts[(vmax, dmax)][1] += 1
                        sc_counts[(vmax, dmax)][1] += 1
                        if flagged(r, vmax, dmax, mode):
                            counts[(vmax, dmax)][0] += 1
                            sc_counts[(vmax, dmax)][0] += 1
        per_scen[s.name] = sc_counts
    return counts, per_scen


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--states-dir", required=True)
    ap.add_argument("--diag-dir", required=True)
    ap.add_argument("--simms-dir", default="", help="fixture root (for imazu ownship.csv CPA)")
    ap.add_argument("--config", default="imm_cv_ct_pmbm_coverage_land")
    args = ap.parse_args()

    trips = scenario_triplets(args.states_dir, args.diag_dir, args.config)
    scens = [Scen(*t) for t in trips]
    by_name = {s.name: s for s in scens}
    imazu = [s for s in scens if s.name.startswith("imazu_")]
    simms = [s for s in scens if s.name.startswith("sim_ms_")]
    autoferry = [s for s in scens if s.name.startswith("autoferry_")]
    autoferry_unanch = [s for s in autoferry if not s.name.endswith("_anchored")]

    # attach CPA to the two dying scenarios
    for scen in ("imazu_15", "imazu_22"):
        s = by_name.get(scen)
        if s is None or not args.simms_dir:
            continue
        own_path = os.path.join(args.simms_dir, scen + "_s0", "ownship.csv")
        if os.path.exists(own_path):
            own = load_ownship_enu(own_path, *IMAZU_DATUM)
            ids, own_cpa, _ = cpa_scans(s.steps, own)
            s.cpa = own_cpa

    print("=" * 78)
    print(f"BACKLOG #25 PHASE 2A PROBE  (config={args.config})")
    print(f"scenarios: {len(imazu)} imazu, {len(simms)} sim_ms, "
          f"{len(autoferry)} autoferry ({len(autoferry_unanch)} unanchored)")
    print(f"knobs: SPEED_CENSUS={SPEED_CENSUS} BORN_GATE={BORN_GATE} "
          f"CONTEST_R={CONTEST_R} GATE={GATE}")
    print("=" * 78)

    # ---- Question A: census over imazu + sim_ms ----
    print("\n########## QUESTION A — runaway census (imazu + sim_ms) ##########")
    agg, per = census(imazu + simms)
    print(f"\nconfirmed rows with speed>{SPEED_CENSUS:.0f} m/s: {agg['rows']}")
    print(f"  target-born-track rows : {agg['target']:6d} "
          f"({100.0*agg['target']/max(1,agg['rows']):.1f}%)  "
          f"[{len(agg['tracks_t'])} distinct target tracks]")
    print(f"  clutter-born-track rows: {agg['clutter']:6d} "
          f"({100.0*agg['clutter']/max(1,agg['rows']):.1f}%)  "
          f"[{len(agg['tracks_c'])} distinct clutter tracks]")
    print(f"  contested (>=2 conf tracks within {CONTEST_R:.0f} m): {agg['contested']} "
          f"({100.0*agg['contested']/max(1,agg['rows']):.1f}%)")

    def med(xs):
        xs = sorted(xs)
        return xs[len(xs) // 2] if xs else 0.0
    print(f"  median lifetime: target-born tracks {med(agg['target_lifetimes']):.1f} s, "
          f"clutter-born tracks {med(agg['clutter_lifetimes']):.1f} s")
    print(f"  median update-gap at runaway rows: {med(agg['gaps']):.2f} s "
          f"(dt since last DETECTED update)")
    print(f"  (c) dominant IMM mode: NOT EXPORTED -- see stop-and-report")

    st = [p for p in per if p[7]]
    print(f"\nSINGLE-TARGET scenarios ({len(st)}): "
          f"{', '.join(p[0] for p in st)}")
    print(f"  runaway rows: {agg['st_rows']}  "
          f"target-born {agg['st_target']} ({100.0*agg['st_target']/max(1,agg['st_rows']):.1f}%)  "
          f"clutter-born {agg['st_clutter']} ({100.0*agg['st_clutter']/max(1,agg['st_rows']):.1f}%)")

    print("\nper-scenario: scen | nTruth single? | runaway rows | tgt/clt rows | tgt/clt tracks | contested")
    for (name, rn, tgt, clt, cont, nt, nc, single, ntruth) in per:
        if rn == 0:
            continue
        print(f"  {name:24s} {ntruth:2d} {'Y' if single else ' '}  "
              f"rows={rn:5d}  t/c={tgt:5d}/{clt:5d}  tracks t/c={nt:3d}/{nc:3d}  cont={cont}")

    # ---- Question B: detection side ----
    print("\n########## QUESTION B — detection side (6 dying tracks) ##########")
    det = detection_probe(by_name)
    print("\ndying track | first-excursion / PERMANENT gate-exit t | CPA scan(dist) "
          "| flags-before-PERMANENT-exit per (Vmax,Dmax)")
    graduate_counts = {(v, d): 0 for v in V_MAX for d in D_MAX}
    for (scen, tid, did, exit_t, first_exc, cpa, cfg) in det:
        cpastr = f"{cpa[0]}({cpa[1]:.0f}m)" if cpa else "n/a"
        fe = round(first_exc, 1) if first_exc is not None else None
        print(f"\n  {scen} truth {tid} id {did}: first-excursion t={fe}  "
              f"PERMANENT gate-exit t={exit_t}  CPA={cpastr}")
        for vmax in V_MAX:
            line = f"    V<={vmax:>4.0f}: "
            for dmax in D_MAX:
                ft, margin, before = cfg[(vmax, dmax)]
                if before:
                    graduate_counts[(vmax, dmax)] += 1
                fs = f"D<={dmax:>3.0f} flag@{ft if ft is None else round(ft,1)}"
                fs += f"(margin {margin:+.1f}s)" if margin is not None else "(no-exit/flag)"
                fs += "[BEFORE]" if before else "[after/none]"
                line += fs + "  "
            print(line)
    print("\n  >>> #dying-tracks flagged BEFORE gate-exit, per (Vmax,Dmax) [need >=5/6]:")
    for vmax in V_MAX:
        print("    " + "  ".join(
            f"V{vmax:.0f}/D{dmax:.0f}={graduate_counts[(vmax,dmax)]}/6"
            for dmax in D_MAX))

    # ---- Question B: false-fire side ----
    print("\n########## QUESTION B — false-fire side (healthy confirmed track-rows) ##########")
    imazu_st = [s for s in imazu if s.single_target]
    for label, group in [("autoferry_unanchored (REAL workload)", autoferry_unanch),
                         ("autoferry_anchored", [s for s in autoferry if s.name.endswith("_anchored")]),
                         ("sim_ms", simms),
                         ("imazu single-target controls", imazu_st)]:
        if not group:
            continue
        counts, _ = false_fire(group)
        tot = counts[(V_MAX[0], D_MAX[0])][1]
        print(f"\n  {label}: {len(group)} scenarios, {tot} healthy confirmed track-rows")
        for vmax in V_MAX:
            print("    " + "  ".join(
                f"V{vmax:.0f}/D{dmax:.0f}={counts[(vmax,dmax)][0]}"
                f"({100.0*counts[(vmax,dmax)][0]/max(1,counts[(vmax,dmax)][1]):.2f}%)"
                for dmax in D_MAX))

    # ---- kill-criteria verdict ----
    print("\n########## BINDING KILL-CRITERIA ##########")
    ff_counts, _ = false_fire(autoferry_unanch + simms)
    print("criteria: some (Vmax,Dmax) flags >=5/6 dying BEFORE gate-exit AND "
          "<1% false-fire on autoferry_unanchored+sim_ms")
    any_pass = False
    for vmax in V_MAX:
        for dmax in D_MAX:
            g = graduate_counts[(vmax, dmax)]
            f, t = ff_counts[(vmax, dmax)]
            ffr = 100.0 * f / max(1, t)
            ok = (g >= 5) and (ffr < 1.0)
            any_pass = any_pass or ok
            print(f"  V<={vmax:>4.0f} D<={dmax:>4.0f}: dying-before-exit={g}/6  "
                  f"false-fire={ffr:.3f}%  -> {'PASS' if ok else 'fail'}")
    print(f"\n  >>> VERDICT (OR of speed|jump): "
          f"{'BUILD (>=1 config passes)' if any_pass else 'NEGATIVE — no config passes both'}")

    # ---- axis separation: is the OR failure driven by V (speed) or D (jump)? ----
    # V flags EARLY (great margin) but false-fires on healthy velocity-state
    # spikes; D (position jump) flags near gate-exit (safe but late). Report each
    # axis alone so the arbiter sees the detection-margin/false-fire tradeoff.
    print("\n########## AXIS SEPARATION — speed-only (V) vs position-jump-only (D) ##########")
    for mode, sweep, lbl in [("v", V_MAX, "SPEED-ONLY  (speed>Vmax)"),
                             ("d", D_MAX, "JUMP-ONLY   (||dpos||>Dmax)")]:
        print(f"\n  --- {lbl} ---")
        # detection: dying tracks flagged before gate-exit
        gc = {x: 0 for x in sweep}
        margins = {x: [] for x in sweep}
        for (scen, tid, did) in DYING:
            s = by_name.get(scen)
            if s is None:
                continue
            exit_t, _ = gate_exit_time(s, tid, did)
            for x in sweep:
                ft = first_flag_time(s, did, x, x, mode=mode)
                if ft is not None and exit_t is not None and ft <= exit_t:
                    gc[x] += 1
                    margins[x].append(exit_t - ft)
        ffc, _ = false_fire(autoferry_unanch + simms, mode=mode)
        for x in sweep:
            # for single-axis, index the counts by the matching diagonal key
            key = (x, D_MAX[0]) if mode == "v" else (V_MAX[0], x)
            # recompute false-fire for this axis threshold directly:
            f = t = 0
            for s in (autoferry_unanch + simms):
                for iid, rows in s.by_id.items():
                    for r in rows:
                        if not r["confirmed"] or not is_healthy(s, iid, r["time"]):
                            continue
                        t += 1
                        if (r["speed"] > x if mode == "v" else r["jump"] > x):
                            f += 1
            ffr = 100.0 * f / max(1, t)
            mmed = sorted(margins[x])[len(margins[x]) // 2] if margins[x] else None
            thr = f"{x:.0f} m/s" if mode == "v" else f"{x:.0f} m"
            print(f"    thr={thr:>8}: dying-before-exit={gc[x]}/6  "
                  f"median-margin={('%.0fs' % mmed) if mmed is not None else 'n/a':>6}  "
                  f"false-fire(af+simms)={ffr:.3f}%  "
                  f"-> {'PASS' if gc[x] >= 5 and ffr < 1.0 else 'fail'}")
    print("\n  (interpretation: SPEED flags early but false-fires on healthy velocity"
          "\n   spikes; JUMP is the posterior-position proxy for innovation — cleaner"
          "\n   on healthy tracks but flags near gate-exit. A TRUE innovation gate"
          "\n   (measurement-prediction) is not reconstructable offline; see verdict.)")


if __name__ == "__main__":
    main()
