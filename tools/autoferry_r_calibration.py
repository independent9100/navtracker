#!/usr/bin/env python3
"""Per-sensor R calibration from AutoFerry residuals.

Walks every AutoFerry scenario, pairs each detection to its nearest-
in-time / nearest-in-space ground-truth target, computes the residual
(z - truth), and reports the empirical noise statistics per sensor.

The result tells us how the configured R in
adapters/replay/AutoferryJsonReplay.cpp (currently lidar=2 m, radar=5 m,
bearing=0.0873 rad ~ 5 deg) compares to reality.
"""

import json
import math
import os
import statistics
from pathlib import Path

# Match AutoferryJsonReplay.cpp::loadAutoferryScenario:
#   measurement layout: [n, e] vessel-fixed NED relative to ownship for
#     sensors 1 (lidar) and 2 (radar). For multiple detections per
#     scan, layout is 2xM: [[n0, n1, ...], [e0, e1, ...]].
#   bearing: NED atan2(e, n) measured from north (sensors 3 IR, 4 EO).
#   ownshipPosition: [N, E] in Piren NED.
# Truth: position [N, E, 0] in same Piren NED frame.

SCENARIOS = ['scenario2', 'scenario3', 'scenario4', 'scenario5', 'scenario6',
             'scenario13', 'scenario16', 'scenario17', 'scenario22']

# Configured noise stds we are checking against (from
# AutoferryLoadOptions defaults).
CONFIGURED = {
    'lidar': 2.0,   # m per axis, Position2D
    'radar': 5.0,   # m per axis, Position2D
    'ir':    0.0873,  # rad, Bearing2D (~5 deg)
    'eo':    0.0873,
}

# Gating thresholds to reject clutter when pairing to truth.
POSITION_GATE_M = 30.0    # generous; AutoFerry clutter is mostly far
BEARING_GATE_RAD = 0.3    # ~17 deg


def active_points(meas):
    """Return list of (n, e) tuples from a Position2D measurement field."""
    if isinstance(meas, list):
        if len(meas) == 0:
            return []
        if isinstance(meas[0], list):
            # 2xM layout: [[n0..nM-1], [e0..eM-1]]
            ns, es = meas[0], meas[1]
            return list(zip(ns, es))
        elif len(meas) >= 2 and not isinstance(meas[0], list):
            # flat [n, e]
            return [(meas[0], meas[1])]
    return []


def bearing_values(meas):
    """Return list of bearings (rad, NED-from-north) from a Bearing2D field."""
    if isinstance(meas, list):
        if len(meas) == 0:
            return []
        if isinstance(meas[0], list):
            return list(meas[0])
        return [float(v) for v in meas]
    return [float(meas)]


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def truth_at_time(truths_by_target, t):
    """For each target, find the truth sample nearest in time to t.

    truths_by_target: dict target_id -> sorted list of (t, n, e).
    Returns dict target_id -> (n, e) interpolated/nearest.
    """
    out = {}
    for tid, samples in truths_by_target.items():
        # Binary-search nearest.
        # Samples are sorted by t.
        lo, hi = 0, len(samples) - 1
        best = samples[0]
        best_dt = abs(samples[0][0] - t)
        while lo <= hi:
            mid = (lo + hi) // 2
            dt = abs(samples[mid][0] - t)
            if dt < best_dt:
                best_dt = dt
                best = samples[mid]
            if samples[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid - 1
        if best_dt < 2.0:  # within 2 s
            out[tid] = (best[1], best[2])
    return out


def process_scenario(label, data_dir):
    det_path = data_dir / label / f'{label}_detections.json'
    gt_path = data_dir / label / f'{label}_groundTruth.json'
    if not det_path.exists() or not gt_path.exists():
        return None

    with open(det_path) as f:
        detections = json.load(f)
    with open(gt_path) as f:
        gt_root = json.load(f)

    # Build truths_by_target: tid -> sorted list of (t, n, e).
    truths = {}
    for scan in gt_root:
        for tgt in scan:
            tid = tgt['targetID']
            pos = tgt['position']  # [N, E, 0]
            t = tgt['time']
            truths.setdefault(tid, []).append((t, pos[0], pos[1]))
    for tid in truths:
        truths[tid].sort(key=lambda x: x[0])

    residuals = {
        'lidar': [],   # list of (rn, re) m
        'radar': [],
        'ir':    [],   # list of rad
        'eo':    [],
    }
    gated_out = {'lidar': 0, 'radar': 0, 'ir': 0, 'eo': 0}

    for d in detections:
        sid = int(round(d['sensorID']))
        t = d['time']
        os_n, os_e = d['ownshipPosition'][0], d['ownshipPosition'][1]
        truth_now = truth_at_time(truths, t)

        if sid in (1, 2):
            sensor_key = 'lidar' if sid == 1 else 'radar'
            for n_rel, e_rel in active_points(d['measurement']):
                # Absolute Piren NED point
                z_n = os_n + n_rel
                z_e = os_e + e_rel
                # Pair to nearest truth
                best = None
                best_r = math.inf
                for tid, (tn, te) in truth_now.items():
                    r = math.hypot(z_n - tn, z_e - te)
                    if r < best_r:
                        best_r = r
                        best = (tn, te)
                if best is None or best_r > POSITION_GATE_M:
                    gated_out[sensor_key] += 1
                    continue
                residuals[sensor_key].append((z_n - best[0], z_e - best[1]))

        elif sid in (3, 4):
            sensor_key = 'ir' if sid == 3 else 'eo'
            for b_ned in bearing_values(d['measurement']):
                # Predicted bearing to each truth from ownship
                best = None
                best_r = math.inf
                for tid, (tn, te) in truth_now.items():
                    dn = tn - os_n
                    de = te - os_e
                    b_pred = math.atan2(de, dn)  # NED atan2(e, n)
                    r = abs(wrap_angle(b_ned - b_pred))
                    if r < best_r:
                        best_r = r
                        best = r
                if best is None or best_r > BEARING_GATE_RAD:
                    gated_out[sensor_key] += 1
                    continue
                residuals[sensor_key].append(best * math.copysign(1.0, 1.0))

    return residuals, gated_out


def report(all_residuals):
    print(f"{'sensor':10s} {'N':>7s} {'mean_n':>10s} {'mean_e':>10s} "
          f"{'sigma_n':>10s} {'sigma_e':>10s} {'sigma_iso':>10s} "
          f"{'configured':>12s} {'ratio':>8s}")
    for sensor in ['lidar', 'radar']:
        rs = all_residuals[sensor]
        if not rs:
            print(f"{sensor:10s}  (no residuals)")
            continue
        ns = [r[0] for r in rs]
        es = [r[1] for r in rs]
        mean_n = statistics.mean(ns)
        mean_e = statistics.mean(es)
        sigma_n = statistics.pstdev(ns)
        sigma_e = statistics.pstdev(es)
        sigma_iso = math.sqrt(0.5 * (sigma_n ** 2 + sigma_e ** 2))
        configured = CONFIGURED[sensor]
        ratio = sigma_iso / configured
        print(f"{sensor:10s} {len(rs):7d} {mean_n:10.3f} {mean_e:10.3f} "
              f"{sigma_n:10.3f} {sigma_e:10.3f} {sigma_iso:10.3f} "
              f"{configured:12.3f} {ratio:8.2f}x")

    print()
    print(f"{'sensor':10s} {'N':>7s} {'mean (rad)':>12s} {'mean (deg)':>12s} "
          f"{'sigma (rad)':>12s} {'sigma (deg)':>12s} "
          f"{'configured':>12s} {'ratio':>8s}")
    for sensor in ['ir', 'eo']:
        rs = all_residuals[sensor]
        if not rs:
            print(f"{sensor:10s}  (no residuals)")
            continue
        mean_r = statistics.mean(rs)
        sigma_r = statistics.pstdev(rs)
        configured = CONFIGURED[sensor]
        ratio = sigma_r / configured
        print(f"{sensor:10s} {len(rs):7d} {mean_r:12.4f} "
              f"{math.degrees(mean_r):12.3f} "
              f"{sigma_r:12.4f} {math.degrees(sigma_r):12.3f} "
              f"{configured:12.4f} {ratio:8.2f}x")


def main():
    data_dir = Path('/home/andreas/workspace/navtracker/data/autoferry')
    all_res = {'lidar': [], 'radar': [], 'ir': [], 'eo': []}
    per_scenario = {}
    for sc in SCENARIOS:
        result = process_scenario(sc, data_dir)
        if result is None:
            continue
        residuals, gated = result
        per_scenario[sc] = (residuals, gated)
        for k in all_res:
            all_res[k].extend(residuals[k])

    print('=== Per-scenario residual counts (after pairing + gating) ===')
    for sc, (residuals, gated) in per_scenario.items():
        counts = {k: len(v) for k, v in residuals.items()}
        print(f'{sc:12s} kept={counts}  gated_out={gated}')

    print()
    print('=== Pooled empirical noise — all scenarios ===')
    report(all_res)

    # Also per environment (env 1 = sc2..sc6, env 2 = sc13/16/17/22).
    for env_name, scs in [('env1 (open water, sc2-6)',
                            ['scenario2', 'scenario3', 'scenario4',
                             'scenario5', 'scenario6']),
                           ('env2 (urban channel)',
                            ['scenario13', 'scenario16', 'scenario17',
                             'scenario22'])]:
        env_res = {'lidar': [], 'radar': [], 'ir': [], 'eo': []}
        for sc in scs:
            if sc not in per_scenario:
                continue
            for k in env_res:
                env_res[k].extend(per_scenario[sc][0][k])
        print()
        print(f'=== {env_name} ===')
        report(env_res)


if __name__ == '__main__':
    main()
