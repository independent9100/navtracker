"""CSV serialization + checksums.

Column orders and headers are EXACT matches for navtracker's replay loaders
(loadOwnshipCsv / loadAisCsv / loadPlotCsvBodyFrame / loadCameraBearingsCsv) so
the fixtures drop straight in. truth.csv uses a new schema consumed by the
loadSimTruthCsv helper added with the C++ scenario-run.

Determinism: floats are written with fixed precision via explicit format
strings, so the same seed yields byte-identical files (the checksum contract).
"""

from __future__ import annotations

import csv
import hashlib
import io

import numpy as np

from .geo import Datum
from .truth import TruthTrack

# (header, key, format) per file. `None` format => write value verbatim (int/str).
OWNSHIP_COLS = [("unix_time", "unix_time", "{:.3f}"), ("lat", "lat", "{:.8f}"),
                ("lon", "lon", "{:.8f}"), ("heading_deg", "heading_deg", "{:.4f}")]

AIS_COLS = [("unix_time", "unix_time", "{:.3f}"), ("mmsi", "mmsi", None),
            ("lat", "lat", "{:.8f}"), ("lon", "lon", "{:.8f}"),
            ("sog_mps", "sog_mps", "{:.2f}"), ("cog_deg", "cog_deg", "{:.1f}"),
            ("nav_status", "nav_status", None), ("name", "name", None)]

RADAR_COLS = [("tod", "tod", "{:.3f}"), ("range_m", "range_m", "{:.4f}"),
              ("azimuth_deg", "azimuth_deg", "{:.4f}"),
              ("sigma_r_m", "sigma_r_m", "{:.4f}"),
              ("sigma_az_deg", "sigma_az_deg", "{:.4f}"),
              ("n_cells", "n_cells", None), ("amp_max", "amp_max", "{:.2f}"),
              ("station", "station", None)]

CAMERA_COLS = [("unix_time", "unix_time", "{:.3f}"), ("camera", "camera", None),
               ("bearing_rel_deg", "bearing_rel_deg", "{:.4f}"),
               ("sigma_deg", "sigma_deg", "{:.4f}"),
               ("confidence", "confidence", "{:.2f}"), ("u_px", "u_px", None),
               ("v_px", "v_px", None), ("w_px", "w_px", None),
               ("h_px", "h_px", None), ("frame", "frame", None)]

TRUTH_COLS = [("unix_time", "unix_time", "{:.3f}"), ("truth_id", "truth_id", None),
              ("mmsi", "mmsi", None), ("lat", "lat", "{:.8f}"),
              ("lon", "lon", "{:.8f}"), ("sog_mps", "sog_mps", "{:.4f}"),
              ("cog_deg", "cog_deg", "{:.4f}"), ("heading_deg", "heading_deg", "{:.4f}"),
              ("nav_status", "nav_status", None)]


def _fmt(row: dict, cols) -> list[str]:
    out = []
    for _, key, fmt in cols:
        v = row[key]
        out.append(fmt.format(float(v)) if fmt is not None else str(v))
    return out


def rows_to_csv(rows: list[dict], cols) -> str:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow([h for h, _, _ in cols])
    for r in rows:
        w.writerow(_fmt(r, cols))
    return buf.getvalue()


def truth_rows(datum: Datum, targets: list[TruthTrack], duration_s: float,
               period_s: float = 1.0) -> list[dict]:
    """Sample each target's complete truth at ``period_s`` for the truth CSV."""
    rows = []
    times = np.arange(0.0, duration_s + 1e-9, period_s)
    for tk in targets:
        for t in times:
            s = tk.sample(float(t))
            lat, lon = datum.enu_to_lonlat(s["e"], s["n"])
            rows.append({
                "unix_time": float(t), "truth_id": tk.mmsi, "mmsi": tk.mmsi,
                "lat": lat, "lon": lon, "sog_mps": s["sog"], "cog_deg": s["cog"],
                "heading_deg": s["heading"], "nav_status": tk.nav_status})
    rows.sort(key=lambda x: (x["unix_time"], x["truth_id"]))
    return rows


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()
