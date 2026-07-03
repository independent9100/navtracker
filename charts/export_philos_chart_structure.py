#!/usr/bin/env python3
"""Densify the charted radar-visible structure layer into a Point-feature GeoJSON
scoped to the philos clip area, for chart corroboration of live radar hazards
(Stage 1b-ii increment 6).

The loader adapters/static/GeoJsonStaticObstacles reads Point features only, but
piers/wharves in charts/geojson/radar_clutter.geojson are LineString/Polygon.
This mirrors the R4 classification script (charts/philos_cluster_classification.py):
densify every edge at DENSIFY_M metres, keep Points/MultiPoints as-is, subset to
the philos bounding box (covers both sunset_cruise inner-harbour and
close_approach Charles-basin clips), and emit Point features that the C++ loader
consumes verbatim. radar_clutter is the curated, WATLEV-filtered radar-conspicuous
layer — the physically-apt single source for "does this radar structure hazard
coincide with charted radar-visible structure". (R4's dual-layer fixed_surface AND
radar_clutter agreement was a conservative DELETION bound; corroboration only
CONFIRMS a classification, so the radar-visible layer alone is appropriate.)

Output: tests/fixtures/philos/charts/radar_structure_points.geojson (gitignored;
force-add). Regenerate with `python3 charts/export_philos_chart_structure.py`.
"""
import json
import math
import os

SRC = os.path.join(os.path.dirname(__file__), "geojson", "radar_clutter.geojson")
OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "fixtures",
                   "philos", "charts", "radar_structure_points.geojson")
DENSIFY_M = 8.0
# Philos bbox: union of sunset_cruise (inner harbour ~42.378/-71.046) and
# close_approach (Charles basin ~42.358/-71.088), with margin.
LAT_MIN, LAT_MAX = 42.35, 42.39
LON_MIN, LON_MAX = -71.10, -71.04
LAT0 = 0.5 * (LAT_MIN + LAT_MAX)
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(LAT0))


def seg_len_m(a, b):
    dlon = (b[0] - a[0]) * M_PER_DEG_LON
    dlat = (b[1] - a[1]) * M_PER_DEG_LAT
    return math.hypot(dlon, dlat)


def in_bbox(lon, lat):
    return LON_MIN <= lon <= LON_MAX and LAT_MIN <= lat <= LAT_MAX


def densify(feature, out):
    g = feature.get("geometry")
    if not g:
        return
    t = g["type"]
    co = g["coordinates"]
    rings = []
    if t == "Point":
        out.append((co[0], co[1]))
        return
    if t == "MultiPoint":
        out.extend((c[0], c[1]) for c in co)
        return
    if t == "LineString":
        rings = [co]
    elif t == "MultiLineString":
        rings = co
    elif t == "Polygon":
        rings = co
    elif t == "MultiPolygon":
        for poly in co:
            rings += poly
    for ring in rings:
        for a, b in zip(ring[:-1], ring[1:]):
            steps = max(1, int(seg_len_m(a, b) / DENSIFY_M))
            for k in range(steps + 1):
                f = k / steps
                out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))


def main():
    d = json.load(open(SRC))
    pts = []
    for ft in d["features"]:
        densify(ft, pts)
    seen = set()
    feats = []
    for lon, lat in pts:
        if not in_bbox(lon, lat):
            continue
        key = (round(lon, 6), round(lat, 6))  # dedup coincident densified points
        if key in seen:
            continue
        seen.add(key)
        feats.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": {"source_id": "radar_clutter"},
        })
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"type": "FeatureCollection", "features": feats}, open(OUT, "w"))
    print(f"wrote {len(feats)} charted structure points -> {os.path.relpath(OUT)}")


if __name__ == "__main__":
    main()
