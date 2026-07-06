"""Layer 2 — thin sensor observation models over the truth tracks.

Each model turns continuous truth into the exact CSV rows navtracker's replay
loaders already parse (schemas verified against the loaders — see README). The
models are deliberately *thin but honest*: real per-scan/per-report cadence,
real noise consistent with the reported sigmas, own-ship-relative geometry for
radar/camera, and the M.1371 cadence quirks that AIS logic downstream turns on.

Every stochastic draw comes from a per-(scenario,sensor) numpy Generator so the
whole battery is a pure function of the scenario seed.
"""

from __future__ import annotations

import math

import numpy as np

from . import clutter as clut
from .geo import Datum, marine_bearing_from_enu, wrap_deg_180
from .truth import MPS_TO_KNOTS, ScenarioSpec, TruthTrack

# AIS position resolution: 1/10000 of a minute of arc = 1/600000 degree.
_AIS_POS_QUANT_DEG = 1.0 / 600000.0


def _rng(spec: ScenarioSpec, salt: int) -> np.random.Generator:
    return np.random.default_rng([spec.seed, salt])


# --- radar -------------------------------------------------------------------
def radar_plots(spec: ScenarioSpec, own: TruthTrack, targets: list[TruthTrack],
                scan_dt_s: float = 2.5, base_pd: float = 0.9,
                sigma_r_m: float = 18.0, sigma_az_deg: float = 0.7,
                r_max_m: float = 8000.0):
    """Per-scan detections (own-ship body frame) + clutter. Returns list of
    dict rows for radar_plots.csv."""
    rng = _rng(spec, 0x2AD5)
    rows = []
    scans = np.arange(0.0, spec.duration_s + 1e-9, scan_dt_s)
    for t in scans:
        op = own.sample(t)
        oe, on, ohead = op["e"], op["n"], op["heading"]
        # --- real detections ---
        for tk in targets:
            if not tk.radar:
                continue                     # scripted radar-silent (camera-only)
            s = tk.sample(t)
            de, dn = s["e"] - oe, s["n"] - on
            rmeas = math.hypot(de, dn)
            if rmeas > r_max_m or rmeas < 5.0:
                continue
            pd = base_pd
            if spec.radar_range_dependent_pd:
                pd = base_pd * math.exp(-(rmeas / (0.7 * r_max_m)) ** 2)
            if rng.random() > pd:
                continue
            bearing_world = marine_bearing_from_enu(de, dn)
            az_body = (bearing_world - ohead) % 360.0
            r_obs = rmeas + rng.normal(0.0, sigma_r_m)
            az_obs = (az_body + rng.normal(0.0, sigma_az_deg)) % 360.0
            rows.append(_plot_row(t, max(r_obs, 1.0), az_obs, sigma_r_m,
                                  sigma_az_deg, n_cells=6, amp=30.0))
        # --- clutter ---
        if spec.clutter_model == "compound_k":
            cr, caz = clut.compound_k_plots(rng, lam_bar_per_m2=6e-8,
                                            r_min=50.0, r_max=r_max_m)
        else:
            cr, caz = clut.poisson_plots(rng, lam_per_m2=2e-8,
                                         r_min=50.0, r_max=r_max_m, scan_dt_s=scan_dt_s)
        for r, az in zip(cr, caz):
            rows.append(_plot_row(t, float(r), float(az), sigma_r_m,
                                  sigma_az_deg, n_cells=2, amp=12.0))
        # --- localized burst ---
        if spec.clutter_burst is not None:
            b0, b1, be, bn = spec.clutter_burst
            if b0 <= t <= b1:
                br, baz = clut.burst_plots(rng, oe, on, ohead, be, bn,
                                           radius_m=150.0, count=25)
                for r, az in zip(br, baz):
                    rows.append(_plot_row(t, float(r), float(az), sigma_r_m,
                                          sigma_az_deg, n_cells=3, amp=18.0))
    rows.sort(key=lambda x: x["tod"])
    return rows


def _plot_row(tod, r, az, sr, saz, n_cells, amp):
    return {"tod": tod, "range_m": r, "azimuth_deg": az, "sigma_r_m": sr,
            "sigma_az_deg": saz, "n_cells": n_cells, "amp_max": amp, "station": "own"}


# --- AIS ---------------------------------------------------------------------
def _ais_interval_s(sog_mps: float, nav_status: int) -> float:
    """ITU-R M.1371 Class-A reporting interval, simplified (speed bands only;
    course-change acceleration omitted — do not gold-plate)."""
    kn = sog_mps * MPS_TO_KNOTS
    if nav_status == 1:                 # at anchor
        return 180.0 if kn <= 3.0 else 10.0
    if kn > 23.0:
        return 2.0
    if kn > 14.0:
        return 6.0
    return 10.0


def ais_reports(spec: ScenarioSpec, datum: Datum, targets: list[TruthTrack]):
    """Decoded-AIS rows with SOG-dependent M.1371 cadence, position
    quantization, and a scripted dropout window. Returns rows for ais.csv."""
    rng = _rng(spec, 0xA15)
    rows = []
    for tk in targets:
        if not tk.ais:
            continue
        # deterministic per-vessel phase so vessels don't all report in lockstep
        phase = rng.random() * 8.0
        t = phase
        while t <= spec.duration_s:
            s = tk.sample(t)
            interval = _ais_interval_s(s["sog"], tk.nav_status)
            in_dropout = (spec.ais_dropout is not None
                          and spec.ais_dropout[0] == tk.mmsi
                          and spec.ais_dropout[1] <= t <= spec.ais_dropout[2])
            if not in_dropout:
                lat, lon = datum.enu_to_lonlat(s["e"], s["n"])
                lat_q = round(lat / _AIS_POS_QUANT_DEG) * _AIS_POS_QUANT_DEG
                lon_q = round(lon / _AIS_POS_QUANT_DEG) * _AIS_POS_QUANT_DEG
                rows.append({
                    "unix_time": t, "mmsi": tk.mmsi, "lat": lat_q, "lon": lon_q,
                    "sog_mps": round(s["sog"], 2),
                    "cog_deg": round(s["cog"], 1),
                    "nav_status": tk.nav_status, "name": tk.name})
            t += interval
    rows.sort(key=lambda x: x["unix_time"])
    return rows


# --- camera (bearing-only) ---------------------------------------------------
def camera_bearings(spec: ScenarioSpec, own: TruthTrack, targets: list[TruthTrack],
                    frame_dt_s: float = 1.0, fov_half_deg: float = 30.0,
                    sigma_cam_deg: float = 0.4, sigma_heading_deg: float = 0.6,
                    camera_name: str = "bow"):
    """Bearing-only detections for camera-visible targets within the FOV.
    Reported sigma is the composed camera⊕heading σ (#16 convention)."""
    rng = _rng(spec, 0xCA3)
    # composed σ: independent camera pointing error and heading error add in
    # quadrature (both map to the same absolute-bearing axis).
    sigma_deg = math.hypot(sigma_cam_deg, sigma_heading_deg)
    rows = []
    frames = np.arange(0.0, spec.duration_s + 1e-9, frame_dt_s)
    for t in frames:
        op = own.sample(t)
        oe, on, ohead = op["e"], op["n"], op["heading"]
        for tk in targets:
            if not tk.camera:
                continue
            s = tk.sample(t)
            de, dn = s["e"] - oe, s["n"] - on
            bearing_world = marine_bearing_from_enu(de, dn)
            rel = wrap_deg_180(bearing_world - ohead)     # hull-relative, 0 = bow
            if abs(rel) > fov_half_deg:
                continue                                   # outside FOV
            rel_obs = rel + rng.normal(0.0, sigma_cam_deg)
            rows.append({
                "unix_time": t, "camera": camera_name,
                "bearing_rel_deg": round(rel_obs, 4), "sigma_deg": round(sigma_deg, 4),
                "confidence": 0.9, "u_px": 960, "v_px": 540, "w_px": 40, "h_px": 60,
                "frame": int(round(t / frame_dt_s))})
    rows.sort(key=lambda x: x["unix_time"])
    return rows


# --- own-ship pose -----------------------------------------------------------
def ownship_poses(spec: ScenarioSpec, datum: Datum, own: TruthTrack,
                  pose_dt_s: float = 0.1, gps_sigma_m: float = 2.5,
                  heading_sigma_deg: float = 0.4):
    """~10 Hz own-ship pose with GPS noise, an optional staleness gap, and an
    optional heading-fault window (feeds #18's guard). Returns rows for
    ownship.csv. The heading FAULT corrupts only the *reported* heading — the
    true motion is unchanged — so radar/camera body-frame bearings (built from
    truth) reconstruct to a biased world position downstream, exactly as a real
    gyro fault would degrade fusion."""
    rng = _rng(spec, 0x0553)
    rows = []
    times = np.arange(0.0, spec.duration_s + 1e-9, pose_dt_s)
    for t in times:
        if (spec.ownship_stale_gap is not None
                and spec.ownship_stale_gap[0] <= t <= spec.ownship_stale_gap[1]):
            continue                                       # no pose published
        s = own.sample(t)
        # GPS noise in ENU, then to lat/lon so the datum round-trips honestly.
        e = s["e"] + rng.normal(0.0, gps_sigma_m)
        n = s["n"] + rng.normal(0.0, gps_sigma_m)
        lat, lon = datum.enu_to_lonlat(e, n)
        hdg = s["heading"] + rng.normal(0.0, heading_sigma_deg)
        if (spec.ownship_heading_fault is not None
                and spec.ownship_heading_fault[0] <= t <= spec.ownship_heading_fault[1]):
            hdg += spec.ownship_heading_fault[2]           # injected gyro bias
        rows.append({"unix_time": t, "lat": lat, "lon": lon,
                     "heading_deg": hdg % 360.0})
    return rows
