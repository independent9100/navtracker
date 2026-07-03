"""
R4 — philos persistent-structure CLUSTER classification + canary table.

The 2026-07-02 chart-coverage field-check (philos_chart_coverage.py) ranked the
strongest persistent radar CELLS and cross-referenced them against the Boston
S-57 ENC. This script closes its open sub-task: group the persistent cells into
CLUSTERS (connected components), classify each with the chart as the authoritative
structure reference, and emit:

  1. a per-cluster table (centroid lat/lon, footprint, dwell, chart distances,
     class) → charts/philos_cluster_classification.csv;
  2. the KEEP / SUPPRESS canary set a later philos suppression claim must honour
     (KEEP = real craft the detector must preserve — ADR 0002; SUPPRESS =
     chart-confirmed fixed structure/aid);
  3. cluster-geometry statistics (footprint / extent / dwell distributions) that
     PARAMETERISE the Stage 1b-ii structure detector (cell size, extent floor,
     persistence bar);
  4. the return-mass CEILING: what fraction of the over-count return mass a
     perfect structure detector could remove without touching a KEEP cluster.

What the chart can and cannot decide is explicit: charted fixed structure / shore
/ aid → objective SUPPRESS; well-charted open water with NO charted structure →
almost certainly real craft (a charted harbor would show fixed structure) → KEEP,
flagged with a maps URL for a confirming visual pass; outside ENC coverage → the
chart is silent → UNKNOWN, defaults to KEEP (never suppress the unlabeled).

Method for projection/gridding/persistence mirrors philos_chart_coverage.py
(world_bearing = heading_true + az_body; 25 m cells; persistent = seen in >=2
clips OR spanning >50% of one clip, AND >75 m from any AIS fix).
"""
import csv, glob, json, math, os
from collections import defaultdict, deque
import numpy as np
from osgeo import ogr

ROOT = os.environ.get("PHILOS_ROOT", "/home/andreas/workspace/navtracker")
OUT = os.environ.get("PHILOS_COVERAGE_OUT", f"{ROOT}/charts")
SCEN_GLOB = f"{ROOT}/tests/fixtures/philos/out/*/"
GEO = f"{ROOT}/charts/geojson"
ENC = {
    "US5BOSCC": f"{ROOT}/charts/ENC_ROOT/US5BOSCC/US5BOSCC.000",
    "US5BOSCD": f"{ROOT}/charts/ENC_ROOT/US5BOSCD/US5BOSCD.000",
}

R_EARTH = 6_371_000.0
CELL_M = 25.0
AIS_EXCL_M = 75.0
DENSIFY_M = 8.0
# classification thresholds (m) — documented in the module docstring.
CHARTED_STRUCT_M = 50.0   # <= this to a charted fixed structure => SUPPRESS
CHARTED_SHORE_M = 40.0    # <= this to charted shoreline       => SUPPRESS
CHARTED_AID_M = 40.0      # <= this to a charted buoy/beacon    => SUPPRESS (aid)
KEEP_CLEAR_STRUCT_M = 100.0  # a KEEP cluster must be this far from charted struct
OWNSHIP_NEARLANE_M = 50.0    # hugging own-ship track => near-field wake/clutter


def d2r(d): return d * math.pi / 180.0


# ── load all philos clips → radar plots in lat/lon ──────────────────────────
def load_ownship(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["unix_time"]), float(r["lat"]),
                         float(r["lon"]), float(r["heading_deg"])))
    rows.sort(key=lambda x: x[0])
    return rows


def interp_own(own, t):
    if t <= own[0][0]: return own[0][1:]
    if t >= own[-1][0]: return own[-1][1:]
    lo, hi = 0, len(own) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if own[m][0] <= t: lo = m
        else: hi = m
    t0, la0, lo0, h0 = own[lo]; t1, la1, lo1, h1 = own[hi]
    f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    dh = ((h1 - h0 + 180) % 360) - 180
    return (la0 + f * (la1 - la0), lo0 + f * (lo1 - lo0), (h0 + f * dh) % 360)


scen_dirs = sorted(glob.glob(SCEN_GLOB))
plots, ais_ll, own_tracks, scen_names = [], [], [], []
for si, d in enumerate(scen_dirs):
    scen_names.append(os.path.basename(d.rstrip("/")))
    own = load_ownship(d + "ownship.csv")
    own_tracks.append([(r[1], r[2]) for r in own])
    with open(d + "radar_plots.csv") as f:
        for r in csv.DictReader(f):
            tod = float(r["tod"]); rng = float(r["range_m"]); az = float(r["azimuth_deg"])
            la0, lo0, hdg = interp_own(own, tod)
            tb = d2r((hdg + az) % 360)
            north = rng * math.cos(tb); east = rng * math.sin(tb)
            lat = la0 + (north / R_EARTH) * 180 / math.pi
            lon = lo0 + (east / (R_EARTH * math.cos(d2r(la0)))) * 180 / math.pi
            plots.append((si, tod, lat, lon))
    ap = d + "ais.csv"
    if os.path.exists(ap):
        with open(ap) as f:
            for r in csv.DictReader(f):
                try: ais_ll.append((float(r["lat"]), float(r["lon"])))
                except Exception: pass

print(f"clips: {len(scen_dirs)} -> {', '.join(scen_names)}")
print(f"radar plots: {len(plots)}   AIS fixes: {len(ais_ll)}")

lat_ref = sum(p[2] for p in plots) / len(plots)
lon_ref = sum(p[3] for p in plots) / len(plots)
mE = math.cos(d2r(lat_ref)) * R_EARTH * math.pi / 180.0
mN = R_EARTH * math.pi / 180.0
def to_enu(lat, lon): return ((lon - lon_ref) * mE, (lat - lat_ref) * mN)
def enu_to_ll(e, n): return (lat_ref + n / mN, lon_ref + e / mE)

# ── grid + persistent-structure cells (same rule as the coverage script) ────
cell = defaultdict(lambda: {"n": 0, "scen": set(),
                            "tod": defaultdict(lambda: [1e18, -1e18]),
                            "e": 0.0, "n_sum": 0.0})
scen_span = {}
for si, tod, lat, lon in plots:
    e, n = to_enu(lat, lon)
    ci, cj = int(math.floor(e / CELL_M)), int(math.floor(n / CELL_M))
    c = cell[(ci, cj)]; c["n"] += 1; c["scen"].add(si); c["e"] += e; c["n_sum"] += n
    tr = c["tod"][si]; tr[0] = min(tr[0], tod); tr[1] = max(tr[1], tod)
    sp = scen_span.setdefault(si, [1e18, -1e18]); sp[0] = min(sp[0], tod); sp[1] = max(sp[1], tod)

ais_enu = np.array([to_enu(la, lo) for la, lo in ais_ll]) if ais_ll else np.empty((0, 2))
def min_ais(e, n):
    if len(ais_enu) == 0: return 1e18
    return float(np.min(np.hypot(ais_enu[:, 0] - e, ais_enu[:, 1] - n)))

# persistent cells keyed by (ci,cj) with rich per-cell stats for clustering
pcells = {}
for (ci, cj), c in cell.items():
    e = c["e"] / c["n"]; n = c["n_sum"] / c["n"]
    multiscen = len(c["scen"]) >= 2
    longspan = False
    best_dwell = 0.0
    for si in c["scen"]:
        span = c["tod"][si][1] - c["tod"][si][0]
        dur = scen_span[si][1] - scen_span[si][0]
        if dur > 0:
            best_dwell = max(best_dwell, span / dur)
            if span > 0.5 * dur: longspan = True
    if (multiscen or longspan) and min_ais(e, n) > AIS_EXCL_M:
        pcells[(ci, cj)] = {"e": e, "n": n, "count": c["n"],
                            "nscen": len(c["scen"]), "dwell": best_dwell}
print(f"occupied 25m cells: {len(cell)}   persistent structure cells: {len(pcells)}")

# ── chart geometry → labelled point clouds ──────────────────────────────────
def densify_file(path, geom_filter=None):
    """Return Nx2 ENU point cloud from a geojson file (points + densified lines/polys)."""
    d = json.load(open(path)); pts = []
    for ft in d["features"]:
        if geom_filter and not geom_filter(ft["properties"]): continue
        g = ft.get("geometry")
        if not g: continue
        t = g["type"]; co = g["coordinates"]; rings = []
        if t == "Point": pts.append((co[0], co[1])); continue
        elif t == "MultiPoint": pts += [(c[0], c[1]) for c in co]; continue
        elif t == "LineString": rings = [co]
        elif t == "MultiLineString": rings = co
        elif t == "Polygon": rings = co
        elif t == "MultiPolygon":
            for poly in co: rings += poly
        for ring in rings:
            for a, b in zip(ring[:-1], ring[1:]):
                ae, an = to_enu(a[1], a[0]); be, bn = to_enu(b[1], b[0])
                L = math.hypot(be - ae, bn - an); steps = max(1, int(L / DENSIFY_M))
                for k in range(steps + 1):
                    f = k / steps
                    pts.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
    return np.array([to_enu(p[1], p[0]) for p in pts]) if pts else np.empty((0, 2))


def densify_labelled(path):
    """Fixed-structure cloud with per-point obj_class (nearest gives the class)."""
    d = json.load(open(path)); pts = []; labels = []
    for ft in d["features"]:
        cls = ft["properties"].get("obj_class", "?")
        g = ft.get("geometry")
        if not g: continue
        t = g["type"]; co = g["coordinates"]; rings = []
        if t == "Point": pts.append((co[0], co[1])); labels.append(cls); continue
        elif t == "LineString": rings = [co]
        elif t == "MultiLineString": rings = co
        elif t == "Polygon": rings = co
        elif t == "MultiPolygon":
            for poly in co: rings += poly
        for ring in rings:
            for a, b in zip(ring[:-1], ring[1:]):
                ae, an = to_enu(a[1], a[0]); be, bn = to_enu(b[1], b[0])
                L = math.hypot(be - ae, bn - an); steps = max(1, int(L / DENSIFY_M))
                for k in range(steps + 1):
                    f = k / steps
                    pts.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                    labels.append(cls)
    en = np.array([to_enu(p[1], p[0]) for p in pts]) if pts else np.empty((0, 2))
    return en, labels


struct_en, struct_lbl = densify_labelled(f"{GEO}/fixed_surface.geojson")
# The curated S-57 radar-conspicuous set (what export_obstacles.py deems a radar
# obstacle — the same layer the 2026-07-02 coverage field-check used, and closest
# to what the tracker's StaticObstacleModel would consume). Reported ALONGSIDE
# fixed_surface so the "is this charted structure?" call does not hinge on which
# layer we pick; SUPPRESS requires BOTH to agree it is charted.
radar_en = densify_file(f"{GEO}/radar_clutter.geojson")
shore_en = densify_file(f"{GEO}/land.geojson")
buoy_en = densify_file(f"{GEO}/aid_floating.geojson")
beacon_en = densify_file(f"{GEO}/aid_fixed.geojson")
own_enu = np.array([to_enu(la, lo) for tr in own_tracks for la, lo in tr])
print(f"chart clouds: fixed_surface={len(struct_en)} radar_clutter={len(radar_en)} "
      f"shore={len(shore_en)} buoy={len(buoy_en)} beacon={len(beacon_en)}")


def nearest(cloud, e, n):
    if len(cloud) == 0: return 1e18, -1
    dd = np.hypot(cloud[:, 0] - e, cloud[:, 1] - n)
    i = int(np.argmin(dd)); return float(dd[i]), i


# ── ENC polygon overlays: coverage (M_COVR) + anchorage/mooring areas ───────
def collect_polys(classes):
    u = ogr.Geometry(ogr.wkbMultiPolygon)
    for path in ENC.values():
        ds = ogr.Open(path)
        if ds is None: continue
        for cls in classes:
            lyr = ds.GetLayerByName(cls)
            if lyr is None: continue
            for ft in lyr:
                g = ft.GetGeometryRef()
                if g is None: continue
                gt = g.GetGeometryName()
                if gt == "POLYGON": u.AddGeometry(g.Clone())
                elif gt == "MULTIPOLYGON":
                    for i in range(g.GetGeometryCount()):
                        u.AddGeometry(g.GetGeometryRef(i).Clone())
    return u


cov_u = collect_polys(["M_COVR"])
moor_u = collect_polys(["ACHARE", "ACHBRT", "BERTHS", "SMCFAC", "HRBFAC",
                        "MORFAC", "MARCUL"])
def inside(u, e, n):
    lat, lon = enu_to_ll(e, n)
    pt = ogr.Geometry(ogr.wkbPoint); pt.AddPoint(lon, lat)
    return bool(u.Contains(pt))


# ── PER-CELL classification (robust; connected-component clustering over a dense
#    harbour front over-merges shore + piers + offshore anchored craft into one
#    blob, so the 25 m cell is the honest unit). SUPPRESS requires BOTH the broad
#    fixed_surface layer AND the curated radar_clutter layer to place charted
#    structure nearby — neither layer alone decides it.
def classify(s):
    if s["d_own"] < OWNSHIP_NEARLANE_M and s["dwell"] < 0.4:
        return "TRANSIENT_NEARLANE"
    charted_struct = (s["d_fixed"] <= CHARTED_STRUCT_M
                      and s["d_radar"] <= CHARTED_STRUCT_M)
    if charted_struct or s["d_shore"] <= CHARTED_SHORE_M or s["d_aid"] <= CHARTED_AID_M:
        return "SUPPRESS_CHARTED"
    if s["in_moor"] and s["d_fixed"] > 75.0:
        return "KEEP_ANCHORAGE"
    if (s["in_cov"] and s["d_fixed"] > KEEP_CLEAR_STRUCT_M
            and s["d_radar"] > KEEP_CLEAR_STRUCT_M and s["d_shore"] > KEEP_CLEAR_STRUCT_M):
        return "KEEP_INCOV_UNCHARTED"
    if not s["in_cov"]:
        return "UNKNOWN_OUTCOV"
    return "UNKNOWN_INCOV"


cells = []
for k, pc in pcells.items():
    e, n = pc["e"], pc["n"]
    d_fixed, ii = nearest(struct_en, e, n)
    fixed_cls = struct_lbl[ii] if ii >= 0 else "?"
    d_radar = nearest(radar_en, e, n)[0]
    d_shore = nearest(shore_en, e, n)[0]
    d_aid = min(nearest(buoy_en, e, n)[0], nearest(beacon_en, e, n)[0])
    d_own = nearest(own_enu, e, n)[0]
    d_ais = min_ais(e, n)
    in_cov = inside(cov_u, e, n); in_moor = inside(moor_u, e, n)
    lat, lon = enu_to_ll(e, n)
    s = {"d_fixed": d_fixed, "d_radar": d_radar, "d_shore": d_shore,
         "d_aid": d_aid, "d_own": d_own, "dwell": pc["dwell"],
         "in_cov": in_cov, "in_moor": in_moor}
    cells.append({"key": k, "e": e, "n": n, "lat": lat, "lon": lon,
                  "returns": pc["count"], "dwell": pc["dwell"],
                  "nscen": pc["nscen"], "d_fixed": d_fixed,
                  "fixed_cls": fixed_cls, "d_radar": d_radar, "d_shore": d_shore,
                  "d_aid": d_aid, "d_own": d_own, "d_ais": d_ais,
                  "in_cov": in_cov, "in_moor": in_moor, "cls": classify(s)})

cells.sort(key=lambda c: -c["returns"])
total_mass = sum(c["returns"] for c in cells)

# ── the offshore-group diagnostic that motivated per-cell + dual-layer ───────
print("\n== top-20 persistent CELLS (dual chart-layer distance resolves the layer question) ==")
print("  rank      lat        lon    ret dwl nsc d_fixed(cls)  d_radar d_shore d_aid  cov moor  CLASS")
for r, c in enumerate(cells[:20]):
    print(f"  {r+1:3d} {c['lat']:9.5f} {c['lon']:10.5f} {c['returns']:4d} "
          f"{c['dwell']:.2f} {c['nscen']:2d} {c['d_fixed']:5.0f}({c['fixed_cls']:>6s}) "
          f"{c['d_radar']:6.0f} {c['d_shore']:6.0f} {c['d_aid']:5.0f} "
          f"{int(c['in_cov'])} {int(c['in_moor'])}   {c['cls']}")

# ── return-mass split by class = the Stage 1b removable CEILING ─────────────
by_cls = defaultdict(lambda: [0, 0])
for c in cells:
    by_cls[c["cls"]][0] += 1; by_cls[c["cls"]][1] += c["returns"]
print(f"\n== return-mass split by class  (persistent cells n={len(cells)}, "
      f"mass={total_mass}) — the Stage 1b removable CEILING ==")
for cls, (nc, mass) in sorted(by_cls.items(), key=lambda kv: -kv[1][1]):
    print(f"  {cls:22s} cells={nc:4d}  return-mass={mass:6d} "
          f"({100*mass/max(1,total_mass):5.1f}%)")
supp = by_cls["SUPPRESS_CHARTED"][1]
keep = by_cls["KEEP_ANCHORAGE"][1] + by_cls["KEEP_INCOV_UNCHARTED"][1]
unk = by_cls["UNKNOWN_OUTCOV"][1] + by_cls["UNKNOWN_INCOV"][1]
print(f"  --> SUPPRESS-able (chart-confirmed structure)      : {100*supp/max(1,total_mass):5.1f}%")
print(f"  --> KEEP (real craft — detector MUST NOT touch)    : {100*keep/max(1,total_mass):5.1f}%")
print(f"  --> UNKNOWN (needs imagery; defaults to KEEP)       : {100*unk/max(1,total_mass):5.1f}%")

# ── spatial dedup for a readable canary table (merge same-class cells <=DEDUP) ─
DEDUP_M = 80.0
def dedup(subset):
    reps = []
    for c in subset:  # subset already sorted by returns desc
        if all(math.hypot(c["e"] - r["e"], c["n"] - r["n"]) > DEDUP_M for r in reps):
            reps.append(c)
    return reps
def maps_url(c): return f"https://www.google.com/maps/search/?api=1&query={c['lat']:.5f},{c['lon']:.5f}"

print("\n== SUPPRESS canaries (chart-confirmed structure — a philos claim SHOULD suppress) ==")
for c in dedup([c for c in cells if c["cls"] == "SUPPRESS_CHARTED"])[:10]:
    print(f"  {c['lat']:9.5f},{c['lon']:10.5f}  ret={c['returns']:4d}  d_fixed={c['d_fixed']:4.0f}m "
          f"d_radar={c['d_radar']:4.0f}m ({c['fixed_cls']})")
print("\n== KEEP canaries (real craft — a philos claim must NOT suppress; ADR 0002) ==")
for c in dedup([c for c in cells if c["cls"].startswith("KEEP")])[:10]:
    print(f"  {c['lat']:9.5f},{c['lon']:10.5f}  ret={c['returns']:4d}  d_fixed={c['d_fixed']:4.0f}m "
          f"d_radar={c['d_radar']:4.0f}m in_moor={int(c['in_moor'])}  {maps_url(c)}")
print("\n== UNKNOWN — needs a visual pass (coords + maps URL) ==")
for c in dedup([c for c in cells if c["cls"].startswith("UNKNOWN")])[:10]:
    print(f"  {c['lat']:9.5f},{c['lon']:10.5f}  ret={c['returns']:4d}  d_fixed={c['d_fixed']:4.0f}m "
          f"cov={int(c['in_cov'])}  {maps_url(c)}")

# ── footprint stats via connected components (REPORTED only, NOT used to
#    classify — for detector cell-size / extent-floor). Components are computed
#    per class so the over-merge across classes cannot inflate a footprint.
NBR = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]
def components(keys):
    keyset = set(keys); seen = set(); comps = []
    for k in keys:
        if k in seen: continue
        q = deque([k]); seen.add(k); comp = [k]
        while q:
            kk = q.popleft()
            for dx, dy in NBR:
                nk = (kk[0] + dx, kk[1] + dy)
                if nk in keyset and nk not in seen:
                    seen.add(nk); q.append(nk); comp.append(nk)
        comps.append(comp)
    return comps
def pct(a, p): return float(np.percentile(a, p)) if len(a) else 0.0
def comp_extent(comp):
    es = np.array([pcells[k]["e"] for k in comp]); ns = np.array([pcells[k]["n"] for k in comp])
    return 0.0 if len(comp) < 2 else float(np.max(np.hypot(es[:, None]-es[None, :], ns[:, None]-ns[None, :])))
print("\n== per-class connected-component footprint (detector cell-size / extent floor) ==")
by_class_keys = defaultdict(list)
for c in cells: by_class_keys[c["cls"]].append(c["key"])
for cls in ("SUPPRESS_CHARTED", "KEEP_INCOV_UNCHARTED", "UNKNOWN_INCOV", "UNKNOWN_OUTCOV"):
    comps = components(by_class_keys[cls])
    if not comps: continue
    sizes = np.array([len(c) for c in comps]); exts = np.array([comp_extent(c) for c in comps])
    print(f"  {cls:22s} components={len(comps):4d}  cells/comp p50/p90={pct(sizes,50):3.0f}/{pct(sizes,90):3.0f}"
          f"  extent_m p50/p90={pct(exts,50):4.0f}/{pct(exts,90):4.0f}")
print("\n== dwell / persistence distribution (detector persistence bar) ==")
for name, grp in [("SUPPRESS_CHARTED", [c for c in cells if c["cls"] == "SUPPRESS_CHARTED"]),
                  ("KEEP", [c for c in cells if c["cls"].startswith("KEEP")]),
                  ("ALL", cells)]:
    dw = np.array([c["dwell"] for c in grp]); ns = np.array([c["nscen"] for c in grp])
    print(f"  {name:18s} n={len(grp):4d}  dwell p50/p90={pct(dw,50):.2f}/{pct(dw,90):.2f}"
          f"  n_clips p50/p90={pct(ns,50):.0f}/{pct(ns,90):.0f}")

# ── classified map (visual proof of the KEEP vs SUPPRESS split) ─────────────
os.environ.setdefault("MPLCONFIGDIR", os.environ.get("MPLCONFIGDIR", OUT))
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(13, 11))
try:
    land = json.load(open(f"{GEO}/land.geojson"))
    for ft in land["features"]:
        g = ft.get("geometry")
        if not g: continue
        rings = ([g["coordinates"][0]] if g["type"] == "Polygon"
                 else ([p[0] for p in g["coordinates"]] if g["type"] == "MultiPolygon" else []))
        for ring in rings:
            en = np.array([to_enu(c[1], c[0]) for c in ring])
            if len(en) > 2: ax.fill(en[:, 0], en[:, 1], color="#eee8d5", zorder=0)
except Exception as ex:
    print("land draw skipped:", ex)
if len(struct_en): ax.scatter(struct_en[:, 0], struct_en[:, 1], s=0.6, c="#1f3a5f", alpha=0.25, zorder=1, label="charted fixed structure")
COLOR = {"SUPPRESS_CHARTED": "#2d8659", "KEEP_INCOV_UNCHARTED": "#cc2222",
         "KEEP_ANCHORAGE": "#e8820c", "UNKNOWN_INCOV": "#8888aa",
         "UNKNOWN_OUTCOV": "#555577", "TRANSIENT_NEARLANE": "#bbbbbb"}
for cls, col in COLOR.items():
    grp = [c for c in cells if c["cls"] == cls]
    if not grp: continue
    xy = np.array([(c["e"], c["n"]) for c in grp])
    sz = np.array([6 + 0.15 * c["returns"] for c in grp])
    mk = "s" if cls == "SUPPRESS_CHARTED" else ("^" if cls.startswith("KEEP") else "x")
    ax.scatter(xy[:, 0], xy[:, 1], s=sz, c=col, marker=mk, alpha=0.8, zorder=4,
               label=f"{cls} ({100*by_cls[cls][1]/max(1,total_mass):.0f}% mass)")
if len(ais_enu): ax.scatter(ais_enu[:, 0], ais_enu[:, 1], s=12, c="#7711aa", marker="v", alpha=0.6, zorder=5, label="AIS fixes")
ax.set_aspect("equal"); ax.grid(True, alpha=0.25)
ax.set_xlabel(f"East (m) from {lat_ref:.4f},{lon_ref:.4f}"); ax.set_ylabel("North (m)")
ax.set_title("philos persistent radar cells classified vs Boston S-57 ENC\n"
             f"SUPPRESS(structure) {100*supp/max(1,total_mass):.0f}% | "
             f"KEEP(craft) {100*keep/max(1,total_mass):.0f}% | UNKNOWN {100*unk/max(1,total_mass):.0f}% "
             f"— dwell does NOT separate the two")
ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
png = f"{OUT}/philos_cluster_classification.png"; plt.tight_layout(); plt.savefig(png, dpi=140); plt.close()
print(f"map: {png}")

# ── CSV deliverable ─────────────────────────────────────────────────────────
csv_path = f"{OUT}/philos_cluster_classification.csv"
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["rank", "lat", "lon", "returns", "dwell", "nscen",
                "d_fixed_m", "fixed_class", "d_radar_m", "d_shore_m", "d_aid_m",
                "d_ownship_m", "d_ais_m", "in_coverage", "in_moorarea", "class"])
    for r, c in enumerate(cells):
        w.writerow([r + 1, f"{c['lat']:.6f}", f"{c['lon']:.6f}", c["returns"],
                    f"{c['dwell']:.3f}", c["nscen"], f"{c['d_fixed']:.1f}",
                    c["fixed_cls"], f"{c['d_radar']:.1f}", f"{c['d_shore']:.1f}",
                    f"{c['d_aid']:.1f}", f"{c['d_own']:.1f}", f"{c['d_ais']:.1f}",
                    int(c["in_cov"]), int(c["in_moor"]), c["cls"]])
print(f"\nCSV: {csv_path}")
