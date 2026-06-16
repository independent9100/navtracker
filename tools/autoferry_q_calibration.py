#!/usr/bin/env python3
"""Per-mode IMM Q calibration from AutoFerry ground truth.

Estimates the empirical PSDs used by the IMM CV and CT motion models
by walking ground-truth trajectories per target, computing per-axis
acceleration variance (CV) and turn-rate variance (CT), and reporting
the equivalent process-noise PSD.

Compare to the configured values in core/benchmark/Config.cpp:
  kImmCv5AccelPsd  = 0.5
  kImmCv5OmegaPsd  = 0.01
  kImmCtAccelPsd   = 0.5
  kImmCtOmegaPsd   = 0.1

A ratio empirical / configured >> 1 means the configured Q is too
small — the filter expects the target to be more predictable than it
actually is, and NEES drifts up between active-sensor returns.
"""

import json
import math
import statistics
from pathlib import Path

SCENARIOS = ['scenario2', 'scenario3', 'scenario4', 'scenario5', 'scenario6',
             'scenario13', 'scenario16', 'scenario17', 'scenario22']

CONFIGURED_CV_ACCEL_PSD = 0.5
CONFIGURED_CV_OMEGA_PSD = 0.01
CONFIGURED_CT_ACCEL_PSD = 0.5
CONFIGURED_CT_OMEGA_PSD = 0.1


def load_truth(label, data_dir):
    """Return dict target_id -> sorted list of (t, n, e) tuples."""
    gt_path = data_dir / label / f'{label}_groundTruth.json'
    if not gt_path.exists():
        return None
    with open(gt_path) as f:
        gt_root = json.load(f)
    truths = {}
    for scan in gt_root:
        for tgt in scan:
            tid = tgt['targetID']
            pos = tgt['position']  # [N, E, 0]
            t = tgt['time']
            truths.setdefault(tid, []).append((t, pos[0], pos[1]))
    for tid in truths:
        # Sort and dedupe (same (t, n, e) repeated when no new GT fix).
        seen = set()
        cleaned = []
        for entry in sorted(truths[tid]):
            key = (entry[0], round(entry[1], 4), round(entry[2], 4))
            if key in seen:
                continue
            seen.add(key)
            cleaned.append(entry)
        truths[tid] = cleaned
    return truths


MIN_DT_FOR_DIFF = 0.5      # seconds; longer baseline → less noise
MIN_SPEED_FOR_HEADING = 0.5  # m/s; below this, heading is undefined


def find_neighbour(samples, k, target_dt, direction):
    """Walk `direction` (+1 or -1) from index k until samples[j][0] is at
    least target_dt away in time. Return j or None if off the edge."""
    n = len(samples)
    t0 = samples[k][0]
    j = k + direction
    while 0 <= j < n:
        if abs(samples[j][0] - t0) >= target_dt:
            return j
        j += direction
    return None


def compute_kinematics(samples):
    """From [(t, n, e), ...] return list of (t, vn, ve, an, ae, omega).

    Velocities via differences over a >= MIN_DT_FOR_DIFF baseline (the
    AutoFerry truth file repeats positions verbatim between fixes, so
    central differencing on consecutive samples explodes noise). Turn
    rates skipped when speed is below MIN_SPEED_FOR_HEADING (heading is
    undefined when the boat is stationary).
    """
    n = len(samples)
    if n < 5:
        return []

    vel = [None] * n
    for k in range(n):
        lo = find_neighbour(samples, k, MIN_DT_FOR_DIFF, -1)
        hi = find_neighbour(samples, k, MIN_DT_FOR_DIFF, +1)
        if lo is None or hi is None:
            continue
        dt = samples[hi][0] - samples[lo][0]
        if dt <= 0:
            continue
        vn = (samples[hi][1] - samples[lo][1]) / dt
        ve = (samples[hi][2] - samples[lo][2]) / dt
        vel[k] = (vn, ve)

    out = []
    for k in range(n):
        if vel[k] is None:
            continue
        lo = find_neighbour(samples, k, MIN_DT_FOR_DIFF, -1)
        hi = find_neighbour(samples, k, MIN_DT_FOR_DIFF, +1)
        if lo is None or hi is None:
            continue
        if vel[lo] is None or vel[hi] is None:
            continue
        dt = samples[hi][0] - samples[lo][0]
        if dt <= 0:
            continue
        an = (vel[hi][0] - vel[lo][0]) / dt
        ae = (vel[hi][1] - vel[lo][1]) / dt
        speed_lo = math.hypot(vel[lo][0], vel[lo][1])
        speed_hi = math.hypot(vel[hi][0], vel[hi][1])
        omega = None
        if speed_lo > MIN_SPEED_FOR_HEADING and speed_hi > MIN_SPEED_FOR_HEADING:
            h_lo = math.atan2(vel[lo][1], vel[lo][0])
            h_hi = math.atan2(vel[hi][1], vel[hi][0])
            dh = h_hi - h_lo
            while dh > math.pi:
                dh -= 2 * math.pi
            while dh < -math.pi:
                dh += 2 * math.pi
            omega = dh / dt
        out.append((samples[k][0], vel[k][0], vel[k][1], an, ae, omega))
    return out


def aggregate(per_target):
    """Aggregate acceleration and turn-rate samples across targets.

    omega = None entries (target stationary, heading undefined) are
    dropped from the omega list but their accels still count.
    """
    accels_n = []
    accels_e = []
    omegas = []
    for kinematics in per_target.values():
        for _, _, _, an, ae, w in kinematics:
            accels_n.append(an)
            accels_e.append(ae)
            if w is not None:
                omegas.append(w)
    return accels_n, accels_e, omegas


def report(label, accels_n, accels_e, omegas):
    if not accels_n:
        print(f'{label}: no samples')
        return
    # PSD ≈ σ_a² for a continuous-time white-noise acceleration model,
    # assuming the observed accelerations are i.i.d. samples of the
    # continuous process. This is the same convention the code uses.
    var_an = statistics.pvariance(accels_n)
    var_ae = statistics.pvariance(accels_e)
    var_a = 0.5 * (var_an + var_ae)
    var_w = statistics.pvariance(omegas)
    print(f"{label}")
    print(f"  N samples           = {len(accels_n)}")
    print(f"  var(an)             = {var_an:8.4f} m²/s⁴")
    print(f"  var(ae)             = {var_ae:8.4f} m²/s⁴")
    print(f"  σ_a (iso)           = {math.sqrt(var_a):8.4f} m/s²")
    print(f"  PSD (var, iso)      = {var_a:8.4f}  (vs configured "
          f"CV={CONFIGURED_CV_ACCEL_PSD}, CT={CONFIGURED_CT_ACCEL_PSD})")
    print(f"  ratio vs CV         = {var_a / CONFIGURED_CV_ACCEL_PSD:6.2f}x")
    print(f"  ratio vs CT         = {var_a / CONFIGURED_CT_ACCEL_PSD:6.2f}x")
    print(f"  var(omega)          = {var_w:9.5f} rad²/s²")
    print(f"  σ_omega (deg/s)     = {math.degrees(math.sqrt(var_w)):8.3f}")
    print(f"  PSD (var)           = {var_w:9.5f}  (vs configured "
          f"CV={CONFIGURED_CV_OMEGA_PSD}, CT={CONFIGURED_CT_OMEGA_PSD})")
    print(f"  ratio vs CV omega   = {var_w / CONFIGURED_CV_OMEGA_PSD:6.2f}x")
    print(f"  ratio vs CT omega   = {var_w / CONFIGURED_CT_OMEGA_PSD:6.2f}x")
    print()


def main():
    data_dir = Path('/home/andreas/workspace/navtracker/data/autoferry')
    per_scenario = {}
    for sc in SCENARIOS:
        truths = load_truth(sc, data_dir)
        if truths is None:
            continue
        kine = {tid: compute_kinematics(samples)
                for tid, samples in truths.items()}
        per_scenario[sc] = kine

    # Per scenario
    print('=== Per-scenario truth kinematics ===\n')
    for sc, kine in per_scenario.items():
        accels_n, accels_e, omegas = aggregate(kine)
        report(sc, accels_n, accels_e, omegas)

    # Per env
    for env_name, scs in [('env 1 (open water, sc2-6)',
                            ['scenario2', 'scenario3', 'scenario4',
                             'scenario5', 'scenario6']),
                           ('env 2 (urban channel)',
                            ['scenario13', 'scenario16', 'scenario17',
                             'scenario22'])]:
        all_an, all_ae, all_w = [], [], []
        for sc in scs:
            if sc not in per_scenario:
                continue
            for kinematics in per_scenario[sc].values():
                for _, _, _, an, ae, w in kinematics:
                    all_an.append(an)
                    all_ae.append(ae)
                    all_w.append(w)
        print(f'=== {env_name} ===')
        report(env_name, all_an, all_ae, all_w)

    # Pooled
    all_an, all_ae, all_w = [], [], []
    for kine in per_scenario.values():
        for kinematics in kine.values():
            for _, _, _, an, ae, w in kinematics:
                all_an.append(an)
                all_ae.append(ae)
                all_w.append(w)
    print('=== Pooled all 9 scenarios ===')
    report('pooled', all_an, all_ae, all_w)


if __name__ == '__main__':
    main()
