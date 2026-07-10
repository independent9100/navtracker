#!/usr/bin/env python3
"""Reproduce the phase2b Stage-2 CPA-overlap-loss metric with fresh dossier
states, for gate OFF (coverage_land) vs gate ON (ivgate).

Uses the EXACT tool definitions (tools/pmbm_phase2b_ab.py dying_metrics):
the 6 dying TRUTHS are the 3 truths each of imazu_15 + imazu_22; CPA-overlap =
a loss window that straddles the own-ship CPA scan; summed over the 6 truths."""
import os
import sys

sys.path.insert(0, "tools")
from imazu_switch_forensics import load_states, assign_all
from imazu_trackloss import load_ownship_enu, loss_windows, cpa_scans

DATUM = (63.45, 10.35)
DYING = {
    "imazu_15": [257010151, 257010152, 257010153],
    "imazu_22": [257010221, 257010222, 257010223],
}
CONFIGS = {
    "gate OFF (coverage_land)": ("states/cl", "imm_cv_ct_pmbm_coverage_land"),
    "gate ON  (ivgate)":        ("states/iv", "imm_cv_ct_pmbm_coverage_land_ivgate"),
}
SIMMS = "tests/fixtures/sim_multisensor"


def dying_metrics(states_dir, cfg):
    tot = cpa = reacq = 0
    rows = []
    for scen, truths in DYING.items():
        sp = os.path.join(states_dir, f"{cfg}__{scen}__seed0.states.csv")
        steps = load_states(sp)
        per_scan = assign_all(steps, 100.0)
        own = load_ownship_enu(os.path.join(SIMMS, scen + "_s0", "ownship.csv"), *DATUM)
        _, own_cpa, _ = cpa_scans(steps, own)
        for tid in truths:
            cpa_scan = own_cpa.get(tid, (None, None))[0]
            wins = loss_windows(per_scan, tid)
            t = sum(w["dur"] for w in wins)
            c = sum(w["dur"] for w in wins
                    if cpa_scan is not None and w["start"] <= cpa_scan <= w["end"])
            r = sum(1 for w in wins if w["new_id"])
            tot += t; cpa += c; reacq += r
            rows.append((scen, tid, t, c, r))
    return tot, cpa, reacq, rows


for name, (sdir, cfg) in CONFIGS.items():
    tot, cpa, reacq, rows = dying_metrics(sdir, cfg)
    print(f"\n### {name}")
    print(f"  6-dying-truths: total_loss={tot}s  CPA-overlap_loss={cpa}s  reacquire_new_id={reacq}")
    for scen, tid, t, c, r in rows:
        print(f"    {scen} {tid}: loss={t}s cpa_overlap={c}s reacq={r}")
