#!/usr/bin/env python3
"""Backlog #25 Phase 2b Stage 1 — re-probe the position-innovation gate on the
TRUE measurement innovation (not the Phase-2a posterior-jump proxy).

Consumes the extended Phase-2b diag export (PmbmDiagRecorder with the new
innov_east_m,innov_north_m,innov_norm_m,imm_weights columns). Answers:

  (B) BINDING re-probe: on D_max in {100,200,400} m applied to the TRUE
      innovation norm, is the Phase-2a verdict reproduced?
        - detection: >=5/6 dying tracks flagged BEFORE permanent gate-exit;
        - false-fire: <1% of healthy confirmed track-scans on autoferry+sim_ms.
      The innovation only exists on DETECTION scans (a measurement was applied);
      an acceptance gate can only act there — so the gate is evaluated on
      detection rows, and false-fire is reported on the detection action surface
      (and, for comparability with Phase-2a, over all healthy confirmed rows).

  (Placement) does the runaway START with ONE oversized accepted innovation
      (-> gate belongs at update-acceptance: reject the bad measurement) or a
      SEQUENCE of moderate ones (-> clamp/deweight inside the estimator)? Read
      the imazu_15/22 dying-track innovation + IMM-mode timelines.

stdlib only; reuses the Phase-2a/1 loaders. Read-only; no core dependency.
"""
import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imazu_switch_forensics import load_states, assign_all           # noqa: E402
from imazu_trackloss import load_ownship_enu, cpa_scans              # noqa: E402
from pmbm_phase2a_probe import (DYING, GATE, D_MAX, IMAZU_DATUM,     # noqa: E402
                                truth_index, nearest_truth)


def load_diag_innov(path):
    """id -> list of per-scan dicts (sorted by time), carrying the true
    innovation norm + IMM weights. Tolerant of the pre-2b schema (innov absent
    -> norm None)."""
    by_id = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            iid = int(r["id"])
            inn = r.get("innov_norm_m")
            imm = r.get("imm_weights", "") or ""
            row = {
                "time": float(r["time_s"]),
                "claimed": int(r["claimed_meas"]),
                "east": float(r["east_m"]),
                "north": float(r["north_m"]),
                "speed": float(r["speed_mps"]),
                "confirmed": r["confirmed"] == "1",
                "innov": (float(inn) if inn not in (None, "", "-1",) and float(inn) >= 0
                          else None),
                "imm": [float(x) for x in imm.split("|") if x != ""],
            }
            by_id.setdefault(iid, []).append(row)
    for iid in by_id:
        by_id[iid].sort(key=lambda x: x["time"])
    return by_id


class Scen:
    def __init__(self, scen, sp, bp):
        self.name = scen
        self.steps = load_states(sp)
        self.by_id = load_diag_innov(bp)
        self.idx, self.scan_keys = truth_index(self.steps)
        self.truth_ids = sorted({t[0] for _, s in self.steps for t in s["truth"]})
        self.single_target = len(self.truth_ids) == 1
        self.assigned_ids = {}
        for ps in assign_all(self.steps, GATE):
            self.assigned_ids[ps["scan"]] = {t for t in ps["assigned"].values()
                                             if t is not None}


def triplets(states_dir, diag_dir, config):
    out = []
    for sp in sorted(glob.glob(os.path.join(states_dir, f"{config}__*.states.csv"))):
        base = os.path.basename(sp)[:-len(".states.csv")]
        scen = base.split("__")[1]
        bp = os.path.join(diag_dir, base + ".pmbmbern.csv")
        if os.path.exists(bp):
            out.append((scen, sp, bp))
    return out


def truth_dist(s, truth_id, row):
    for tid, p in nearest_truth(s.idx, s.scan_keys, row["time"]):
        if tid == truth_id:
            return math.hypot(row["east"] - p[0], row["north"] - p[1])
    return None


def gate_exit(s, truth_id, dying_id):
    """(permanent_exit_t, first_excursion_t) — the point of no return and the
    onset (same definition as Phase-2a)."""
    rows = s.by_id.get(dying_id, [])
    last_in = exit_t = first_exc = None
    for r in rows:
        d = truth_dist(s, truth_id, r)
        if d is not None and d <= GATE:
            last_in = r["time"]; exit_t = None
        elif d is not None and d > GATE and last_in is not None:
            if exit_t is None: exit_t = r["time"]
            if first_exc is None: first_exc = r["time"]
    return exit_t, first_exc


def first_innov_flag(s, dying_id, dmax):
    """first DETECTION scan whose true innovation exceeds dmax."""
    for r in s.by_id.get(dying_id, []):
        if r["innov"] is not None and r["innov"] > dmax:
            return r["time"]
    return None


def is_healthy(s, iid, t):
    k = int(round(t))
    ass = s.assigned_ids.get(k)
    if ass is None and s.assigned_ids:
        k = min(s.assigned_ids, key=lambda x: abs(x - k))
        ass = s.assigned_ids.get(k)
    return bool(ass) and iid in ass


def false_fire(scens, dmax):
    """flagged / total on (a) healthy confirmed DETECTION rows (the acceptance
    gate's action surface) and (b) all healthy confirmed rows (Phase-2a-comparable)."""
    det_f = det_t = all_f = all_t = 0
    for s in scens:
        for iid, rows in s.by_id.items():
            for r in rows:
                if not r["confirmed"] or not is_healthy(s, iid, r["time"]):
                    continue
                all_t += 1
                flagged = r["innov"] is not None and r["innov"] > dmax
                if flagged: all_f += 1
                if r["innov"] is not None:      # detection row (gate can act)
                    det_t += 1
                    if flagged: det_f += 1
    return det_f, det_t, all_f, all_t


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--states-dir", required=True)
    ap.add_argument("--diag-dir", required=True)
    ap.add_argument("--simms-dir", default="")
    ap.add_argument("--config", default="imm_cv_ct_pmbm_coverage_land")
    args = ap.parse_args()

    scens = [Scen(*t) for t in triplets(args.states_dir, args.diag_dir, args.config)]
    by_name = {s.name: s for s in scens}
    simms = [s for s in scens if s.name.startswith("sim_ms_")]
    af_unanch = [s for s in scens if s.name.startswith("autoferry_")
                 and not s.name.endswith("_anchored")]
    imazu_st = [s for s in scens if s.name.startswith("imazu_") and s.single_target]

    for scen in ("imazu_15", "imazu_22"):
        s = by_name.get(scen)
        if s and args.simms_dir:
            op = os.path.join(args.simms_dir, scen + "_s0", "ownship.csv")
            if os.path.exists(op):
                _, own_cpa, _ = cpa_scans(s.steps, load_ownship_enu(op, *IMAZU_DATUM))
                s.cpa = own_cpa

    print("=" * 78)
    print(f"BACKLOG #25 PHASE 2b STAGE 1 — TRUE-INNOVATION re-probe (config={args.config})")
    print(f"scenarios: {len([s for s in scens if s.name.startswith('imazu_')])} imazu, "
          f"{len(simms)} sim_ms, {len(af_unanch)} autoferry_unanch")
    print("=" * 78)

    # ---------- Detection side ----------
    print("\n########## DETECTION — TRUE innovation flag vs permanent gate-exit ##########")
    grad = {d: 0 for d in D_MAX}
    for (scen, tid, did) in DYING:
        s = by_name.get(scen)
        if s is None:
            continue
        exit_t, first_exc = gate_exit(s, tid, did)
        cpa = getattr(s, "cpa", {}).get(tid)
        cpastr = f"scan {cpa[0]} ({cpa[1]:.0f} m)" if cpa else "n/a"
        print(f"\n  {scen} truth {tid} id {did}: first-exc t={first_exc} "
              f"PERMANENT gate-exit t={exit_t}  CPA={cpastr}")
        for d in D_MAX:
            ft = first_innov_flag(s, did, d)
            before = ft is not None and exit_t is not None and ft <= exit_t
            if before:
                grad[d] += 1
            margin = (exit_t - ft) if (ft is not None and exit_t is not None) else None
            print(f"    D<={d:>4.0f} m: first innov>D @ t={ft}  "
                  f"margin={('%+.1fs' % margin) if margin is not None else 'n/a':>8}  "
                  f"{'[BEFORE]' if before else '[after/none]'}")
    print("\n  >>> dying flagged BEFORE permanent gate-exit (need >=5/6): " +
          "  ".join(f"D{d:.0f}={grad[d]}/6" for d in D_MAX))

    # ---------- False-fire side ----------
    print("\n########## FALSE-FIRE — TRUE innovation on healthy confirmed tracks ##########")
    for label, group in [("autoferry_unanch (REAL)", af_unanch), ("sim_ms", simms),
                         ("imazu single-target", imazu_st)]:
        if not group:
            continue
        print(f"\n  {label}: {len(group)} scenarios")
        for d in D_MAX:
            df, dt, af, at = false_fire(group, d)
            print(f"    D<={d:>4.0f} m: detection-surface {df}/{dt}="
                  f"{100.0*df/max(1,dt):.3f}%   all-rows {af}/{at}="
                  f"{100.0*af/max(1,at):.3f}%")

    # ---------- Binding verdict ----------
    print("\n########## BINDING VERDICT (true innovation) ##########")
    ff_scens = af_unanch + simms
    print("criteria: >=5/6 dying before permanent gate-exit AND <1% false-fire "
          "(autoferry_unanch+sim_ms)")
    any_pass = False
    for d in D_MAX:
        df, dt, af, at = false_fire(ff_scens, d)
        ff_det = 100.0 * df / max(1, dt)
        ff_all = 100.0 * af / max(1, at)
        ok = grad[d] >= 5 and ff_det < 1.0 and ff_all < 1.0
        any_pass = any_pass or ok
        print(f"  D<={d:>4.0f} m: dying-before-exit={grad[d]}/6  "
              f"false-fire det={ff_det:.3f}% all={ff_all:.3f}%  -> {'PASS' if ok else 'fail'}")
    print(f"\n  >>> STAGE-1 BINDING: {'PASS — true innovation reproduces the proxy verdict' if any_pass else 'FAIL — stop at checkpoint'}")

    # ---------- Placement ----------
    print("\n########## PLACEMENT — one oversized innovation or a sequence? ##########")
    for (scen, tid, did) in [("imazu_15", 257010151, 6), ("imazu_22", 257010223, 1),
                             ("imazu_22", 257010221, 7)]:
        s = by_name.get(scen)
        if s is None:
            continue
        exit_t, _ = gate_exit(s, tid, did)
        rows = [r for r in s.by_id.get(did, []) if r["innov"] is not None]
        # window: 40 s before the first innov>200 up to that point
        first200 = first_innov_flag(s, did, 200.0)
        if first200 is None:
            print(f"\n  {scen} id {did}: no innov>200 (n/a)"); continue
        # detections in [first200-40, first200+5]
        win = [r for r in rows if first200 - 40 <= r["time"] <= first200 + 5]
        pre = [r["innov"] for r in rows if r["time"] < first200]
        pre_max = max(pre) if pre else 0.0
        print(f"\n  {scen} truth {tid} id {did}: first innov>200 @ t={first200} "
              f"(permanent gate-exit t={exit_t})")
        print(f"    max innovation BEFORE that first>200 flag: {pre_max:.1f} m "
              f"({'MODERATE run then a jump -> update-acceptance gate' if pre_max < 200 else 'already-large sequence -> estimator clamp'})")
        print("    innovation + dominant IMM mode around onset (detection scans):")
        for r in win[:16]:
            immstr = "/".join(f"{w:.2f}" for w in r["imm"]) if r["imm"] else "-"
            print(f"      t={r['time']:7.1f}  innov={r['innov']:8.1f} m  "
                  f"spd={r['speed']:9.1f}  imm=[{immstr}]")


if __name__ == "__main__":
    main()
