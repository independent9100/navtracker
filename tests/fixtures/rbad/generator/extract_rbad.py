#!/usr/bin/env python3
"""R-BAD (Radar-Based Berthing-Aid Dataset) fixture extractor.

Turns the dataset's *Labelled Buffers* CSVs into navtracker's standard replay
fixture shapes for a label-scored, fixed-frame berthing replay.

WHAT R-BAD IS (Zenodo record 16936465, doi:10.5281/zenodo.16936465, CC-BY-4.0;
paper MDPI Electronics 14(20):4065). 69 h of synchronized FMCW **mmWave** radar
point clouds + video collected onboard an operational Ro-Ro/passenger ferry
across 13 ports (arrivals, departures, port-idle, cruising). Sensors: four TI
FMCW chips — IWR6843 (60-64 GHz) + AWR1443/1642/1843 (77-81 GHz), 20 Hz.

REGIME CAVEAT (governs every claim downstream): this is an **automotive/
industrial mmWave FMCW** sensor class, NOT marine X-band. It corroborates the
berthing *scene* on a new sensor; it is not a third marine-radar geography and
philos/HAXR tuning is not expected to transfer.

WHY THE LABELLED BUFFERS (not the 31.6 GB raw): the labelled CSV rows already
carry the per-plot cloud (the `Points` column: [x,y,z,v_doppler,snr]) plus the
authors' clustered objects and labels, covering Arrivals (the berthing
archetype) across all 13 ports. The 31.6 GB raw archive is video-dominated; its
only added value is the synced MP4 for a future independent manual label pass
(the deferred route, triggered only if berthing results ever need independent
kinematic truth). So this extractor needs no multi-GB download.

TWO STRUCTURAL FACTS resolved from the data (paper full text is bot-blocked):
  * NO EGO POSE anywhere in the dataset (no nav/GPS/IMU file; no ownship
    columns). Detections are sensor **body-frame** Cartesian metres. We treat
    the buffer as a FIXED-FRAME relative-tracking scene: own-ship is the origin,
    body frame == a fixed ENU frame. Consequence carried into the docs: nothing
    is world-stationary in a body frame, so the anchored/moored static-hazard
    logic (ADR 0002) is UNTESTABLE here by construction.
  * The clusters + `Tracking_ID` are the authors' OWN onboard clustering/
    tracking pipeline (labelled `Points` match the raw points exactly). They are
    a REFERENCE TRACKER, **not** ground truth. Any score against them is a
    cross-tracker *consistency* measure (reported vs_reference_tracker), never an
    accuracy claim. `Dock_Label` (binary 0/1) semantic meaning is unconfirmed
    (paper blocked) => report its distribution, assert nothing on it.

OUTPUT (per selected arrival buffer, one scenario dir under --out-root):
  radar_plots.csv       tod,range_m,azimuth_deg,sigma_r_m,sigma_az_deg,
                        n_cells,amp_max,station,v_doppler_mps,snr_db
                        One row per clustered object per frame = ONE PLOT at its
                        centroid (contract-correct plot-level input; NOT the raw
                        constituent points). azimuth marine (N=0, CW) so the
                        loader recovers ENU E=X (starboard), N=Y (forward). The
                        first 8 columns are navtracker's standard plot schema
                        (loadPlotCsv reads exactly those and ignores extras); the
                        last two carry per-plot Doppler + SNR forward as COLUMNS
                        ONLY — the first in-hand dataset with per-detection
                        Doppler, kept replayable for a future prototype probe.
  reference_tracks.csv  tod,ref_id,east_m,north_m,dock_label
                        The authors' reference-tracker trajectories (ref_id =
                        their Tracking_ID) + the binary Dock_Label. Consumed as
                        the consistency reference, NOT truth.
  meta.txt              provenance + per-buffer stats + the caveats above.

FAIL-LOUD INTEGRITY GUARDS (the R8.8 lesson): every buffer is validated BEFORE
any file is written; on any violation the extractor names the buffer + the
failed check and writes NOTHING (all-or-nothing across the run) — no placeholder
CSV can ship. Determinism: rows are explicitly sorted and floats written at
fixed precision, so re-runs are byte-identical (the checksum contract).

Usage (from the main working tree, data already unzipped under _download/):
    python tests/fixtures/rbad/generator/extract_rbad.py

Stdlib only — no third-party dependencies (see requirements.txt).
"""
from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import math
import os
import statistics
import sys

# Native sensor frame rate (Hz). The labelled buffers are sub-sampled to ~1 Hz
# for annotation (Frame_ID advances ~20 per second), so tod = Frame_ID / this
# yields clean ~1 Hz scans. Guarded against the data below.
FRAME_RATE_HZ = 20.0

# Curated representative subset: 2 ports x 3 arrival approaches, spanning a wide
# density range (21..235 reference IDs, 1..8 objects/frame). The full labelled
# set is ~121 min of arrivals across 13 ports, all available locally; this is a
# reality-check subset, not a tuning corpus. To scale up, add rows here.
SCENARIOS = [
    ("rbad_kalimnos_16", "Kalimnos", "log_Kalimnos_16_05_25-12_32_labeled.csv"),
    ("rbad_kalimnos_17", "Kalimnos", "log_Kalimnos_17_02_55-07_15_labeled.csv"),
    ("rbad_kalimnos_3", "Kalimnos", "log_Kalimnos_3_00_00-02_08_labeled.csv"),
    ("rbad_kos_11", "Kos", "log_Kos_11_05_06-11_05_labeled.csv"),
    ("rbad_kos_16", "Kos", "log_Kos_16_04_30-10_30_labeled.csv"),
    ("rbad_kos_5", "Kos", "log_Kos_5_03_30-07_15_labeled.csv"),
]

STATION = "rbad"  # single station at ENU origin (0,0); see RbadScenarioRun.


class RbadIntegrityError(Exception):
    """Raised when a buffer fails a fail-loud integrity guard."""


def _parse_time(t: str) -> float:
    """MM:SS or HH:MM:SS -> seconds (annotation clock, 1 s resolution)."""
    parts = t.split(":")
    parts = [int(p) for p in parts]
    if len(parts) == 2:
        return parts[0] * 60 + parts[1]
    if len(parts) == 3:
        return parts[0] * 3600 + parts[1] * 60 + parts[2]
    raise RbadIntegrityError(f"unparseable Time value {t!r}")


def _load_rows(path: str) -> list[dict]:
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def _points(row: dict) -> list[list[float]]:
    """Parse the labelled `Points` cell: list of [x, y, z, v_doppler, snr]."""
    pts = ast.literal_eval(row["Points"])
    return [[float(v) for v in p] for p in pts]


def _derive(rows: list[dict]):
    """Compute per-plot + per-reference records and buffer statistics.

    Returns (plots, refs, stats). Does NOT write anything.
    """
    # De-duplicate exact-centroid double-labels. A few buffers assign TWO
    # Tracking_IDs to ONE physical detection (identical Frame_ID,X,Y,Z,Num_Points,
    # sometimes with conflicting Dock_Label) — an annotation artifact of the
    # authors' pipeline. Feeding two identical plots would inject a duplicate
    # detection cloud (the clutter_burst duplicate-cloud lesson: contract-
    # violating input), so we collapse to one row per physical detection, keeping
    # the lowest Tracking_ID (deterministic). Applied to plots AND reference so
    # they stay 1:1. Count is reported; nothing else is altered.
    groups: dict = {}
    order: list = []
    for r in rows:
        key = (r["Frame_ID"], r["X"], r["Y"], r["Z"], r["Num_Points"])
        if key not in groups:
            groups[key] = r
            order.append(key)
        elif int(float(r["Tracking_ID"])) < int(float(groups[key]["Tracking_ID"])):
            groups[key] = r
    n_dup = len(rows) - len(order)
    rows = [groups[k] for k in order]

    frames = sorted({int(float(r["Frame_ID"])) for r in rows})
    first_frame = frames[0]
    times = [_parse_time(r["Time"]) for r in rows]

    plots: list[dict] = []
    refs: list[dict] = []
    all_ranges: list[float] = []
    all_az: list[float] = []
    all_dop: list[float] = []
    all_snr: list[float] = []
    dock_counts: dict[str, int] = {}
    id_positions: dict[int, list[tuple[float, float]]] = {}

    for r in rows:
        frame = int(float(r["Frame_ID"]))
        tod = (frame - first_frame) / FRAME_RATE_HZ
        x = float(r["X"])
        y = float(r["Y"])
        rng = math.hypot(x, y)
        az = math.degrees(math.atan2(x, y))  # marine: 0=fwd(N), +=starboard(E)
        pts = _points(r)
        p_ranges = [math.hypot(p[0], p[1]) for p in pts]
        p_az = [math.degrees(math.atan2(p[0], p[1])) for p in pts]
        p_dop = [p[3] for p in pts]
        p_snr = [p[4] for p in pts]
        # Per-plot spread -> R diagonal, "if derivable". <3 points or zero spread
        # => 0.0, which the loader replaces with per-sensor defaults. Capped so a
        # single outlier point cannot inflate the gate.
        sig_r = min(statistics.pstdev(p_ranges), 30.0) if len(p_ranges) >= 3 else 0.0
        sig_az = min(statistics.pstdev(p_az), 30.0) if len(p_az) >= 3 else 0.0
        ref_id = int(float(r["Tracking_ID"]))
        dock = r["Dock_Label"].strip()

        plots.append({
            "tod": tod, "range_m": rng, "azimuth_deg": az,
            "sigma_r_m": sig_r, "sigma_az_deg": sig_az,
            "n_cells": int(float(r["Num_Points"])),
            "amp_max": max(p_snr), "station": STATION,
            "v_doppler_mps": statistics.fmean(p_dop),
            "snr_db": statistics.fmean(p_snr),
        })
        refs.append({"tod": tod, "ref_id": ref_id, "east_m": x, "north_m": y,
                     "dock_label": dock})

        all_ranges.append(rng)
        all_az.append(az)
        all_dop.extend(p_dop)
        all_snr.extend(p_snr)
        dock_counts[dock] = dock_counts.get(dock, 0) + 1
        id_positions.setdefault(ref_id, []).append((x, y))

    # Deterministic order: by time, then range, then id (stable, seed-free).
    plots.sort(key=lambda p: (p["tod"], p["range_m"], p["azimuth_deg"]))
    refs.sort(key=lambda r: (r["tod"], r["ref_id"]))

    # Frame-rate estimate straight from the annotation clock (Time vs Frame_ID).
    span_frames = frames[-1] - frames[0]
    span_time = max(times) - min(times)
    est_rate = span_frames / span_time if span_time > 0 else 0.0

    # Largest single-reference-track position span (dynamic-value check).
    max_span = 0.0
    for pos in id_positions.values():
        if len(pos) < 2:
            continue
        for i in range(len(pos)):
            for j in range(i + 1, len(pos)):
                max_span = max(max_span, math.hypot(pos[i][0] - pos[j][0],
                                                    pos[i][1] - pos[j][1]))

    stats = {
        "n_rows": len(rows),
        "n_frames": len(frames),
        "duration_s": span_frames / FRAME_RATE_HZ,
        "est_frame_rate_hz": est_rate,
        "n_reference_ids": len({r["ref_id"] for r in refs}),
        "range_min_m": min(all_ranges), "range_max_m": max(all_ranges),
        "range_std_m": statistics.pstdev(all_ranges),
        "az_min_deg": min(all_az), "az_max_deg": max(all_az),
        "doppler_min_mps": min(all_dop), "doppler_max_mps": max(all_dop),
        "doppler_distinct": len({round(v, 3) for v in all_dop}),
        "snr_min_db": min(all_snr), "snr_max_db": max(all_snr),
        "dock_label_counts": dock_counts,
        "max_reference_track_span_m": max_span,
        "n_duplicate_labels_removed": n_dup,
    }
    return plots, refs, stats


def _guard(label: str, stats: dict) -> list[str]:
    """Fail-loud integrity checks. Returns list of violation messages."""
    v: list[str] = []
    if stats["n_rows"] < 30:
        v.append(f"too few rows ({stats['n_rows']} < 30)")
    if not (15.0 <= stats["est_frame_rate_hz"] <= 25.0):
        v.append(f"frame rate {stats['est_frame_rate_hz']:.2f} Hz outside [15,25] "
                 f"(tod derivation assumes {FRAME_RATE_HZ:.0f} Hz)")
    if not (0.5 < stats["range_min_m"] and stats["range_max_m"] <= 200.0):
        v.append(f"range extent {stats['range_min_m']:.2f}..{stats['range_max_m']:.2f} m "
                 "implausible for a short-range berthing radar (expect ~(0.5, 200])")
    if stats["range_std_m"] <= 0.5:
        v.append(f"range std {stats['range_std_m']:.3f} m too small — values look static/placeholder")
    if (stats["az_max_deg"] - stats["az_min_deg"]) <= 1.0:
        v.append("azimuth spread <= 1 deg — a single-bearing placeholder, not a scene")
    if stats["n_reference_ids"] < 2:
        v.append(f"only {stats['n_reference_ids']} reference IDs (< 2)")
    if stats["max_reference_track_span_m"] <= 0.5:
        v.append("no reference track moves > 0.5 m — positions look static/placeholder")
    bad = set(stats["dock_label_counts"]) - {"0", "1"}
    if bad:
        v.append(f"Dock_Label has non-binary values {sorted(bad)}")
    if abs(stats["doppler_min_mps"]) > 10.0 or abs(stats["doppler_max_mps"]) > 10.0:
        v.append(f"Doppler {stats['doppler_min_mps']:.2f}..{stats['doppler_max_mps']:.2f} m/s "
                 "implausible for berthing speeds (|v| <= 10)")
    if stats["doppler_distinct"] < 2:
        v.append("Doppler has < 2 distinct values — looks like a placeholder column")
    if not (0.0 < stats["snr_min_db"] and stats["snr_max_db"] <= 60.0):
        v.append(f"SNR {stats['snr_min_db']:.2f}..{stats['snr_max_db']:.2f} dB outside (0, 60]")
    return v


PLOT_COLS = [("tod", "{:.4f}"), ("range_m", "{:.4f}"), ("azimuth_deg", "{:.4f}"),
             ("sigma_r_m", "{:.4f}"), ("sigma_az_deg", "{:.4f}"), ("n_cells", None),
             ("amp_max", "{:.2f}"), ("station", None), ("v_doppler_mps", "{:.4f}"),
             ("snr_db", "{:.2f}")]
REF_COLS = [("tod", "{:.4f}"), ("ref_id", None), ("east_m", "{:.4f}"),
            ("north_m", "{:.4f}"), ("dock_label", None)]


def _to_csv(rows: list[dict], cols) -> str:
    import io
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow([h for h, _ in cols])
    for r in rows:
        w.writerow([fmt.format(float(r[h])) if fmt else str(r[h]) for h, fmt in cols])
    return buf.getvalue()


def _sha256(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _meta_text(label: str, port: str, src: str, stats: dict) -> str:
    dl = stats["dock_label_counts"]
    return (
        f"# R-BAD fixture: {label}\n"
        f"source: Labelled Buffers Data/Arrival/{port}/{src}\n"
        f"dataset: R-BAD (Zenodo 16936465, CC-BY-4.0); MDPI Electronics 14(20):4065\n"
        f"sensor: TI FMCW mmWave (IWR6843 60-64 GHz + AWR1443/1642/1843 77-81 GHz), 20 Hz\n"
        "regime: automotive/industrial mmWave FMCW, NOT marine X-band "
        "(tuning does not transfer; not a marine-radar number)\n"
        "frame: sensor body-frame Cartesian (NO ego pose in dataset) -> fixed ENU "
        "(E=X starboard, N=Y forward); anchored/moored logic untestable by construction\n"
        "labels: reference tracker (authors' Tracking_ID), NOT ground truth -> "
        "score = cross-tracker consistency only. Dock_Label binary, meaning unconfirmed.\n"
        f"duration_s: {stats['duration_s']:.1f}\n"
        f"annotation_frame_rate_hz: {stats['est_frame_rate_hz']:.2f}\n"
        f"n_plots: {stats['n_rows']}\n"
        f"n_duplicate_labels_removed: {stats['n_duplicate_labels_removed']} "
        "(exact-centroid double-labels collapsed to one plot/reference)\n"
        f"n_reference_ids: {stats['n_reference_ids']}\n"
        f"range_m: {stats['range_min_m']:.2f}..{stats['range_max_m']:.2f}\n"
        f"doppler_mps: {stats['doppler_min_mps']:.2f}..{stats['doppler_max_mps']:.2f}\n"
        f"snr_db: {stats['snr_min_db']:.2f}..{stats['snr_max_db']:.2f}\n"
        f"dock_label_counts: 0={dl.get('0', 0)} 1={dl.get('1', 0)}\n"
    )


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    rbad_root = os.path.dirname(here)  # tests/fixtures/rbad
    ap = argparse.ArgumentParser(description="Extract R-BAD labelled buffers into replay fixtures.")
    ap.add_argument("--labelled-root",
                    default=os.path.join(rbad_root, "_download", "labelled",
                                         "Labelled Buffers Data", "Arrival"),
                    help="dir containing <Port>/log_*_labeled.csv")
    ap.add_argument("--out-root", default=rbad_root,
                    help="dir to write <label>/ scenario fixtures into")
    ap.add_argument("--only", nargs="*", default=None,
                    help="optional subset of scenario labels to extract")
    args = ap.parse_args()

    selected = [s for s in SCENARIOS if args.only is None or s[0] in args.only]
    if not selected:
        print("no scenarios selected", file=sys.stderr)
        return 2

    # ---- Phase 1: derive + guard ALL buffers before writing anything. ----
    prepared = []
    failures = []
    for label, port, src in selected:
        path = os.path.join(args.labelled_root, port, src)
        if not os.path.exists(path):
            failures.append(f"{label}: source not found: {path}")
            continue
        try:
            plots, refs, stats = _derive(_load_rows(path))
        except Exception as e:  # parse/format problem is itself a fail-loud case
            failures.append(f"{label}: extraction error: {e}")
            continue
        violations = _guard(label, stats)
        if violations:
            for msg in violations:
                failures.append(f"{label}: GUARD FAILED: {msg}")
            continue
        prepared.append((label, port, src, plots, refs, stats))

    if failures:
        print("REFUSING TO WRITE — fail-loud integrity guards tripped:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    # ---- Phase 2: write (all buffers passed). ----
    checksums = []
    for label, port, src, plots, refs, stats in prepared:
        out_dir = os.path.join(args.out_root, label)
        os.makedirs(out_dir, exist_ok=True)
        artifacts = {
            "radar_plots.csv": _to_csv(plots, PLOT_COLS),
            "reference_tracks.csv": _to_csv(refs, REF_COLS),
            "meta.txt": _meta_text(label, port, src, stats),
        }
        for name, text in artifacts.items():
            with open(os.path.join(out_dir, name), "w", newline="") as fh:
                fh.write(text)
            checksums.append((f"{label}/{name}", _sha256(text)))
        print(f"wrote {label}: {stats['n_rows']} plots, {stats['n_reference_ids']} "
              f"reference IDs, {stats['duration_s']:.0f}s")

    checksums.sort()
    lines = [f"{h}  {p}" for p, h in checksums]
    with open(os.path.join(args.out_root, "CHECKSUMS.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\nsha256 checksums (also in CHECKSUMS.txt):")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
