#!/usr/bin/env python3
"""Regenerate tests/fixtures/autoferry/trondheim_harbor.geojson from OpenStreetMap.

The AutoFerry replay dataset is framed in the Piren local tangent plane
(NED origin LLA [63.4389029083, 10.39908278, 39.923]; navtracker uses ENU about
that datum). To run the land clutter prior / land-aware PDA pool on the real
AutoFerry channels we need the real Trondheim inner-harbour land polygons.

This fetches OSM coastline + water for the harbour bbox via the Overpass API,
assembles land polygons in local metres about the Piren datum (so the geometry
is validated directly against the AutoFerry ENU ground truth), and writes a
WGS84 GeoJSON FeatureCollection in the same shape as tests/fixtures/philos/
boston.geojson (Polygon features, [lon,lat] rings, outer + holes).

Requires: shapely (not a repo dependency — `pip install --target <dir> shapely`
and run with PYTHONPATH=<dir>). Network access to overpass-api.de.

Reproducibility: OSM is live, so re-running may pick up edits. The committed
fixture was generated 2026-07-03; the vessel-in-water validation below is the
invariant that must hold for any regeneration.

Data (c) OpenStreetMap contributors, ODbL (https://www.openstreetmap.org/copyright).
"""
import json, math, os, sys, urllib.request

LAT0, LON0, H0 = 63.4389029083, 10.39908278, 39.923           # Piren datum
BBOX_S, BBOX_W, BBOX_N, BBOX_E = 63.4310, 10.3820, 63.4468, 10.4162
OUT = "tests/fixtures/autoferry/trondheim_harbor.geojson"

M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))
def proj(lon, lat):   return ((lon - LON0) * M_LON, (lat - LAT0) * M_LAT)
def unproj(x, y):     return [round(LON0 + x / M_LON, 7), round(LAT0 + y / M_LAT, 7)]

OVERPASS_QL = f"""
[out:json][timeout:90];
(
  way["natural"="coastline"]({BBOX_S},{BBOX_W},{BBOX_N},{BBOX_E});
  way["natural"="water"]({BBOX_S},{BBOX_W},{BBOX_N},{BBOX_E});
  relation["natural"="water"]({BBOX_S},{BBOX_W},{BBOX_N},{BBOX_E});
  way["man_made"="pier"]({BBOX_S},{BBOX_W},{BBOX_N},{BBOX_E});
);
out geom;
"""

def fetch_osm():
    req = urllib.request.Request(
        "https://overpass-api.de/api/interpreter",
        data=OVERPASS_QL.encode(),
        headers={"User-Agent": "navtracker-research/1.0 (coastline fetch)",
                 "Content-Type": "text/plain"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.load(r)["elements"]

def build(els):
    from shapely.geometry import LineString, Polygon, Point, box
    from shapely.ops import polygonize, unary_union
    from shapely import make_valid
    def gm(e): return [proj(p["lon"], p["lat"]) for p in (e.get("geometry") or [])]

    bx0, by0 = proj(BBOX_W, BBOX_S); bx1, by1 = proj(BBOX_E, BBOX_N)
    bbox = box(bx0, by0, bx1, by1)
    coast = [LineString(gm(e)) for e in els if e["type"] == "way"
             and (e.get("tags") or {}).get("natural") == "coastline"
             and len(e.get("geometry") or []) >= 2]

    inner = []
    for e in els:
        t = e.get("tags") or {}
        if e["type"] == "way" and (t.get("natural") == "water"
                                   or t.get("disused:waterway") == "dock"):
            c = gm(e)
            if len(c) >= 3:
                inner.append(make_valid(Polygon(c if c[0] == c[-1] else c + [c[0]])))
    for e in els:
        if e["type"] == "relation" and (e.get("tags") or {}).get("natural") == "water":
            lines = [LineString([proj(p["lon"], p["lat"]) for p in m["geometry"]])
                     for m in (e.get("members") or [])
                     if m.get("role") == "outer" and m.get("geometry")
                     and len(m["geometry"]) >= 2]
            for poly in polygonize(unary_union(lines)):
                inner.append(make_valid(poly))
    inner_water = unary_union(inner) if inner else None

    noded = unary_union(coast + [bbox.boundary])
    def sea_score(face):
        v = 0
        for ls in coast:
            cs = list(ls.coords)
            for i in range(len(cs) - 1):
                ax, ay = cs[i]; bx, by = cs[i + 1]
                mx, my = (ax + bx) / 2, (ay + by) / 2
                if face.exterior.distance(Point(mx, my)) < 0.5:
                    dx, dy = bx - ax, by - ay; L = math.hypot(dx, dy) or 1.0
                    if face.contains(Point(mx - dy / L * 2.0, my + dx / L * 2.0)): v -= 1  # left = land
                    if face.contains(Point(mx + dy / L * 2.0, my - dx / L * 2.0)): v += 1  # right = sea
        return v
    sea = unary_union([f for f in polygonize(noded) if sea_score(f) > 0])
    water = unary_union([w for w in [sea, inner_water] if w is not None and not w.is_empty])
    return make_valid(bbox.difference(water))

def validate(land):
    from shapely.geometry import Point
    bad = 0; total = 0
    for sc in ("scenario2", "scenario3", "scenario13", "scenario16", "scenario17", "scenario22"):
        p = f"data/autoferry/{sc}/{sc}_groundTruth.json"
        if not os.path.exists(p): continue
        for frame in json.load(open(p)):
            for tgt in frame:
                n, e = tgt["position"][0], tgt["position"][1]   # [N,E,0] -> ENU (E,N) metres
                total += 1
                if land.contains(Point(e, n)): bad += 1
    print(f"  validation: {bad}/{total} ground-truth points on land (expect 0)")
    return total == 0 or bad == 0

def write(land):
    land = land.simplify(1.5)
    geoms = list(land.geoms) if land.geom_type == "MultiPolygon" else [land]
    feats = []
    for g in geoms:
        if g.area < 30.0: continue
        rings = [[unproj(*c) for c in g.exterior.coords]]
        rings += [[unproj(*c) for c in h.coords] for h in g.interiors]
        feats.append({"type": "Feature", "properties": {},
                      "geometry": {"type": "Polygon", "coordinates": rings}})
    fc = {"type": "FeatureCollection",
          "properties": {
              "source": "OpenStreetMap via Overpass API, (c) OpenStreetMap contributors, ODbL",
              "query_bbox_S_W_N_E": [BBOX_S, BBOX_W, BBOX_N, BBOX_E],
              "datum_lla": [LAT0, LON0, H0],
              "note": ("Trondheim inner harbour (Piren/Ravnkloa) land polygons for the "
                       "AutoFerry replay land model. Assembled from natural=coastline + "
                       "natural=water (Kanalen/Ravnkloloepet/Nidelva) in local metres about "
                       "the Piren datum, then reprojected to WGS84."),
              "generator": "tools/build_autoferry_coastline.py"},
          "features": feats}
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(fc, open(OUT, "w"))
    print(f"  wrote {OUT}: {len(feats)} polygons")

if __name__ == "__main__":
    els = fetch_osm()
    print(f"fetched {len(els)} OSM elements")
    land = build(els)
    ok = validate(land)
    if not ok:
        print("  WARNING: ground truth falls on land — check frame/datum before committing")
        sys.exit(1)
    write(land)
