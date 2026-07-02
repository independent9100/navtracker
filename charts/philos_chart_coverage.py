"""
philos radar persistent-structure  vs  charted static obstacles (S-57 ENC).

Question: how much of the philos "expected obstacles" (the persistent stationary
radar structure that drives the PMBM over-count) is actually present in the
parsed NOAA ENC GeoJSONs in ./charts/geojson/ ?

Method:
  1. Project radar_plots (body frame) -> world lat/lon for EVERY philos scenario,
     using the loader convention world_bearing = heading_true + az_body
     (relative-to-heading; validated in philos_clutter_analysis.py).
  2. Aggregate into 25 m ENU cells. A cell is an EXPECTED FIXED-STRUCTURE cell
     when it is (seen in >=2 independent scenario passes) OR (spans >50% of a
     single scenario's replay), AND is >75 m from any AIS vessel position
     (so moving cooperative traffic is excluded).
  3. Densify the charted radar-clutter geometry (Points/Lines/Polys) to a ~8 m
     point cloud, split fixed vs floating.
  4. For each expected cell, nearest charted-obstacle distance. "Covered" if it
     is within MATCH_R. Report coverage by cell and by radar-plot count, at
     several MATCH_R, for fixed-only and fixed+floating chart sets.
  5. Render an overlay map.
"""
import csv, glob, json, math, os
import numpy as np

ROOT = "/home/andreas/workspace/navtracker"
OUT = os.environ.get("PHILOS_COVERAGE_OUT", f"{ROOT}/charts")
SCEN_GLOB = f"{ROOT}/tests/fixtures/philos/out/*/"
CHART = f"{ROOT}/charts/geojson/radar_clutter.geojson"
LAND = f"{ROOT}/charts/geojson/land.geojson"

R_EARTH = 6_371_000.0
def d2r(d): return d*math.pi/180.0

CELL_M = 25.0
AIS_EXCL_M = 75.0
DENSIFY_M = 8.0
MATCH_RADII = [25.0, 50.0, 75.0]

# ── load all scenarios ──────────────────────────────────────────────────────
scen_dirs = sorted(glob.glob(SCEN_GLOB))
plots = []       # (scenario_idx, tod, lat, lon)
ais_ll = []      # (lat, lon)
own_tracks = []  # per scenario list of (lat,lon)
scen_names = []

def load_ownship(path):
    rows=[]
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]), float(r["heading_deg"])))
    rows.sort(key=lambda x:x[0]); return rows

def interp_own(own, t):
    if t<=own[0][0]: return own[0][1:]
    if t>=own[-1][0]: return own[-1][1:]
    lo,hi=0,len(own)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if own[m][0]<=t: lo=m
        else: hi=m
    t0,la0,lo0,h0=own[lo]; t1,la1,lo1,h1=own[hi]
    f=(t-t0)/(t1-t0) if t1>t0 else 0.0
    dh=((h1-h0+180)%360)-180
    return (la0+f*(la1-la0), lo0+f*(lo1-lo0), (h0+f*dh)%360)

for si, d in enumerate(scen_dirs):
    name = os.path.basename(d.rstrip("/")); scen_names.append(name)
    own = load_ownship(d+"ownship.csv")
    own_tracks.append([(r[1],r[2]) for r in own])
    with open(d+"radar_plots.csv") as f:
        for r in csv.DictReader(f):
            tod=float(r["tod"]); rng=float(r["range_m"]); az=float(r["azimuth_deg"])
            la0,lo0,hdg = interp_own(own, tod)
            tb = d2r((hdg+az)%360)                       # relative-to-heading
            north = rng*math.cos(tb); east = rng*math.sin(tb)
            lat = la0 + (north/R_EARTH)*180/math.pi
            lon = lo0 + (east/(R_EARTH*math.cos(d2r(la0))))*180/math.pi
            plots.append((si, tod, lat, lon))
    ap = d+"ais.csv"
    if os.path.exists(ap):
        with open(ap) as f:
            for r in csv.DictReader(f):
                try: ais_ll.append((float(r["lat"]), float(r["lon"])))
                except Exception: pass

print(f"scenarios: {len(scen_dirs)}  ->  {', '.join(scen_names)}")
print(f"total radar plots: {len(plots)}   AIS positions: {len(ais_ll)}")

# ── ENU reference = mean of all radar-plot positions ────────────────────────
lat_ref = sum(p[2] for p in plots)/len(plots)
lon_ref = sum(p[3] for p in plots)/len(plots)
mE = math.cos(d2r(lat_ref))*R_EARTH*math.pi/180.0
mN = R_EARTH*math.pi/180.0
def to_enu(lat, lon): return ((lon-lon_ref)*mE, (lat-lat_ref)*mN)

# ── grid the radar plots ────────────────────────────────────────────────────
from collections import defaultdict
cell = defaultdict(lambda: {"n":0, "scen":set(), "tod":defaultdict(lambda:[1e18,-1e18]), "e":0.0,"n_sum":0.0})
scen_span = {}
for si,tod,lat,lon in plots:
    e,n = to_enu(lat,lon)
    ci,cj = int(math.floor(e/CELL_M)), int(math.floor(n/CELL_M))
    c=cell[(ci,cj)]; c["n"]+=1; c["scen"].add(si); c["e"]+=e; c["n_sum"]+=n
    tr=c["tod"][si]; tr[0]=min(tr[0],tod); tr[1]=max(tr[1],tod)
    sp=scen_span.setdefault(si,[1e18,-1e18]); sp[0]=min(sp[0],tod); sp[1]=max(sp[1],tod)

ais_enu = np.array([to_enu(la,lo) for la,lo in ais_ll]) if ais_ll else np.empty((0,2))
def min_ais(e,n):
    if len(ais_enu)==0: return 1e18
    return float(np.min(np.hypot(ais_enu[:,0]-e, ais_enu[:,1]-n)))

expected=[]   # (e,n,count) persistent fixed-structure cells
for (ci,cj),c in cell.items():
    e=c["e"]/c["n"]; n=c["n_sum"]/c["n"]
    multiscen = len(c["scen"])>=2
    longspan=False
    for si in c["scen"]:
        span=c["tod"][si][1]-c["tod"][si][0]
        dur=scen_span[si][1]-scen_span[si][0]
        if dur>0 and span> 0.5*dur: longspan=True
    if (multiscen or longspan) and min_ais(e,n)>AIS_EXCL_M:
        expected.append((e,n,c["n"]))
expected=np.array(expected) if expected else np.empty((0,3))
print(f"occupied 25m cells: {len(cell)}   expected fixed-structure cells: {len(expected)}")

# ── densify charted obstacles to point clouds ───────────────────────────────
def densify(geojson_path, want_floating=None):
    d=json.load(open(geojson_path)); pts=[]
    for ft in d["features"]:
        g=ft.get("geometry")
        if not g: continue
        if want_floating is not None:
            persist=ft["properties"].get("radar_persistence")
            isfloat = (persist=="floating")
            if want_floating and not isfloat: continue
            if (not want_floating) and isfloat: continue
        t=g["type"]; co=g["coordinates"]
        rings=[]
        if t=="Point": pts.append(tuple(co[:2])); continue
        elif t=="MultiPoint": pts+= [tuple(c[:2]) for c in co]; continue
        elif t=="LineString": rings=[co]
        elif t=="MultiLineString": rings=co
        elif t=="Polygon": rings=co
        elif t=="MultiPolygon":
            for poly in co: rings+=poly
        for ring in rings:
            for a,b in zip(ring[:-1], ring[1:]):
                ae,an=to_enu(a[1],a[0]); be,bn=to_enu(b[1],b[0])
                L=math.hypot(be-ae,bn-an); steps=max(1,int(L/DENSIFY_M))
                for k in range(steps+1):
                    f=k/steps; pts.append(( (a[0]+f*(b[0]-a[0])), (a[1]+f*(b[1]-a[1])) ))
    # pts are (lon, lat); to_enu expects (lat, lon)
    en=np.array([to_enu(p[1],p[0]) for p in pts]) if pts else np.empty((0,2))
    return en

chart_fixed = densify(CHART, want_floating=False)
chart_float = densify(CHART, want_floating=True)
chart_all = np.vstack([chart_fixed, chart_float]) if len(chart_float) else chart_fixed
print(f"chart densified points: fixed={len(chart_fixed)} floating={len(chart_float)}")

def bbox(a):
    return (float(a[:,0].min()),float(a[:,0].max()),float(a[:,1].min()),float(a[:,1].max())) if len(a) else None
print(f"  bbox expected cells (E,E,N,N): {bbox(expected[:,:2]) if len(expected) else None}")
print(f"  bbox chart fixed         : {bbox(chart_fixed)}")

def nearest(cloud, e, n):
    if len(cloud)==0: return 1e18
    return float(np.min(np.hypot(cloud[:,0]-e, cloud[:,1]-n)))

# land (coastline) densified — the shoreline the radar mostly hits near shore
chart_land = densify(LAND)
print(f"chart densified points: land={len(chart_land)}   bbox land: {bbox(chart_land)}")

# ── coverage ────────────────────────────────────────────────────────────────
print("\n== COVERAGE of expected radar-structure cells by the chart ==")
tot_cells=len(expected); tot_plots=float(expected[:,2].sum()) if tot_cells else 0.0

def dvec(cloud):
    return np.array([nearest(cloud,e,n) for e,n,_ in expected]) if tot_cells else np.empty(0)

d_obst = dvec(chart_all)          # discrete obstacles (fixed + floating) — StaticObstacle inputs
d_fixed= dvec(chart_fixed)        # discrete fixed obstacles only
d_land = dvec(chart_land)         # charted shoreline — the land/coastline prior's domain
d_union= np.minimum(d_obst, d_land)
results={"obstacles":d_obst,"fixed":d_fixed,"land":d_land,"union":d_union}

def report(label,dists):
    print(f"\n  {label}:")
    for R in MATCH_RADII:
        cov=dists<=R; cc=int(cov.sum()); cp=float(expected[cov,2].sum()) if tot_cells else 0.0
        print(f"    within {R:4.0f} m:  cells {cc:4d}/{tot_cells} ({100*cc/max(1,tot_cells):5.1f}%)   "
              f"weighted-by-plots {100*cp/max(1,tot_plots):5.1f}%")
report("discrete obstacles only (StaticObstacle set)", d_obst)
report("charted shoreline only (land/coastline prior)", d_land)
report("UNION  obstacles + shoreline  (what the chart-driven suppression covers)", d_union)

# ── registration-offset diagnostic ──────────────────────────────────────────
# Is the moderate coverage a systematic projection offset (heading/range bias)
# or genuine spread? Find, for each expected cell within 150 m of the union
# chart cloud, the vector to its nearest chart point; look for a consistent mean.
chart_union = np.vstack([chart_all, chart_land])
def nearest_vec(cloud,e,n):
    dd=np.hypot(cloud[:,0]-e, cloud[:,1]-n); i=int(np.argmin(dd))
    return cloud[i,0]-e, cloud[i,1]-n, float(dd[i])
offs=[]
for e,n,_ in expected:
    dx,dy,dd=nearest_vec(chart_union,e,n)
    if dd<150.0: offs.append((dx,dy,dd))
offs=np.array(offs)
if len(offs):
    mean_dx=offs[:,0].mean(); mean_dy=offs[:,1].mean()
    print(f"\n== registration diagnostic (cells within 150 m of chart, n={len(offs)}) ==")
    print(f"  mean offset vector cell->chart: dE={mean_dx:+.1f} m  dN={mean_dy:+.1f} m  |mean|={math.hypot(mean_dx,mean_dy):.1f} m")
    print(f"  median nearest dist: {np.median(offs[:,2]):.1f} m")
    # coverage if we shift expected cells by the mean offset (removes any bias)
    shifted=np.array([nearest(chart_union, e+mean_dx, n+mean_dy) for e,n,_ in expected])
    for R in (25.0,50.0):
        print(f"  bias-corrected union within {R:.0f} m: {100*(shifted<=R).sum()/max(1,tot_cells):.1f}%")

# ── are the STRONG (track-forming) clusters charted? ────────────────────────
# The over-count is driven by the most persistent/dense returns. Rank expected
# cells by plot count; report union coverage among the strongest N.
order=np.argsort(-expected[:,2])
print("\n== coverage among the STRONGEST expected cells (over-count drivers) ==")
for N in (50,100,200,500):
    if N>tot_cells: continue
    idx=order[:N]; du=d_union[idx]
    print(f"  top {N:4d} by plot-count:  union within 50 m {100*(du<=50).sum()/N:5.1f}%   within 75 m {100*(du<=75).sum()/N:5.1f}%   "
          f"(median nearest {np.median(du):.0f} m)")

# ── are the strong uncharted clusters in own-ship's operating lane? ──────────
own_enu=np.array([to_enu(la,lo) for tr in own_tracks for la,lo in tr])
def near_own(e,n): return float(np.min(np.hypot(own_enu[:,0]-e, own_enu[:,1]-n)))
print("\n== strong clusters vs own-ship track (near-field-clutter test) ==")
for N in (100,500):
    idx=order[:N]
    do=np.array([near_own(expected[i,0],expected[i,1]) for i in idx])
    print(f"  top {N}: median dist to nearest own-ship track point = {np.median(do):.0f} m  "
          f"(<100 m: {100*(do<100).sum()/N:.0f}%, <200 m: {100*(do<200).sum()/N:.0f}%)")
allo=np.array([near_own(e,n) for e,n,_ in expected])
print(f"  all expected cells: median dist to own-ship track = {np.median(allo):.0f} m")

# ── anchorage / mooring AREA test (are the uncharted clusters anchored craft?) ─
from osgeo import ogr
ENC={"US5BOSCC":f"{ROOT}/charts/ENC_ROOT/US5BOSCC/US5BOSCC.000",
     "US5BOSCD":f"{ROOT}/charts/ENC_ROOT/US5BOSCD/US5BOSCD.000"}
MOOR=["ACHARE","ACHBRT","BERTHS","SMCFAC","HRBFAC","MORFAC","MARCUL"]
RESTR=["RESARE","FAIRWY","CTNARE","DMPGRD"]
def collect(classes):
    union=ogr.Geometry(ogr.wkbMultiPolygon); rings=[]
    for path in ENC.values():
        ds=ogr.Open(path)
        for cls in classes:
            lyr=ds.GetLayerByName(cls)
            if lyr is None: continue
            for ft in lyr:
                g=ft.GetGeometryRef()
                if g is None: continue
                gt=g.GetGeometryName()
                polys=[g] if gt=="POLYGON" else ([g.GetGeometryRef(i) for i in range(g.GetGeometryCount())] if gt=="MULTIPOLYGON" else [])
                for p in polys:
                    if p is None or p.GetGeometryName()!="POLYGON": continue
                    union.AddGeometry(p.Clone())
                    r=p.GetGeometryRef(0)
                    if r: rings.append(np.array([to_enu(r.GetY(k), r.GetX(k)) for k in range(r.GetPointCount())]))
    return union, rings
moor_u, moor_rings = collect(MOOR)
restr_u, restr_rings = collect(RESTR)
def inside(u, e, n):
    lon=lon_ref+e/mE; lat=lat_ref+n/mN
    pt=ogr.Geometry(ogr.wkbPoint); pt.AddPoint(lon,lat)
    return u.Contains(pt)
in_moor=np.array([inside(moor_u,e,n) for e,n,_ in expected])
in_restr=np.array([inside(restr_u,e,n) for e,n,_ in expected])
print("\n== anchorage/mooring AREA overlay (uncharted clusters = anchored vessels?) ==")
print(f"  ALL expected cells inside a charted anchorage/mooring/berth area: {100*in_moor.sum()/tot_cells:.1f}%")
for N in (100,500):
    idx=order[:N]
    print(f"  top {N} strong clusters inside anchorage/mooring area: {100*in_moor[idx].sum()/N:.0f}%   inside restricted/fairway: {100*in_restr[idx].sum()/N:.0f}%")
# strong AND uncharted-by-structure: how many explained by an anchorage area?
strong=order[:100]; unch = d_union[strong]>50.0
if unch.sum():
    print(f"  of top-100 that are >50 m from any structure ({int(unch.sum())}): {100*in_moor[strong][unch].sum()/unch.sum():.0f}% fall inside a charted anchorage/mooring area")

# ── field-check: top clusters vs BRIDGES + ENC coverage extent ──────────────
def densify_class(path, objclass):
    d=json.load(open(path)); pts=[]
    for ft in d["features"]:
        if ft["properties"].get("obj_class")!=objclass: continue
        g=ft.get("geometry")
        if not g: continue
        t=g["type"]; co=g["coordinates"]; rings=[]
        if t=="Point": pts.append((co[0],co[1])); continue
        elif t=="LineString": rings=[co]
        elif t=="MultiLineString": rings=co
        elif t=="Polygon": rings=co
        elif t=="MultiPolygon":
            for poly in co: rings+=poly
        for ring in rings:
            for a,b in zip(ring[:-1],ring[1:]):
                ae,an=to_enu(a[1],a[0]); be,bn=to_enu(b[1],b[0])
                L=math.hypot(be-ae,bn-an); steps=max(1,int(L/DENSIFY_M))
                for k in range(steps+1):
                    f=k/steps; pts.append((a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1])))
    return np.array([to_enu(p[1],p[0]) for p in pts]) if pts else np.empty((0,2))
bridge_pts=densify_class(f"{ROOT}/charts/geojson/fixed_surface.geojson","BRIDGE")
cov_u=ogr.Geometry(ogr.wkbMultiPolygon)
for path in ENC.values():
    ds=ogr.Open(path); lyr=ds.GetLayerByName("M_COVR")
    if not lyr: continue
    for ft in lyr:
        g=ft.GetGeometryRef()
        if g is None: continue
        if g.GetGeometryName()=="POLYGON": cov_u.AddGeometry(g.Clone())
        elif g.GetGeometryName()=="MULTIPOLYGON":
            for i in range(g.GetGeometryCount()): cov_u.AddGeometry(g.GetGeometryRef(i).Clone())
def in_cov(e,n):
    lon=lon_ref+e/mE; lat=lat_ref+n/mN
    pt=ogr.Geometry(ogr.wkbPoint); pt.AddPoint(lon,lat); return cov_u.Contains(pt)
print(f"\n== field-check: top-20 strong clusters (bridges? in coverage?)  (charted BRIDGE pts={len(bridge_pts)}) ==")
print("  rank      lat        lon      count  d_chart  d_bridge  in_ENC_cov")
for r,i in enumerate(order[:20]):
    e,n,cnt=expected[i]; lat=lat_ref+n/mN; lon=lon_ref+e/mE
    print(f"  {r+1:4d}  {lat:9.5f} {lon:10.5f} {int(cnt):6d} {d_union[i]:7.0f} {nearest(bridge_pts,e,n):8.0f}   {in_cov(e,n)}")
top=order[:100]
dbr=np.array([nearest(bridge_pts,expected[i,0],expected[i,1]) for i in top])
outc=np.array([not in_cov(expected[i,0],expected[i,1]) for i in top])
print(f"  top-100 within 75 m of a charted BRIDGE: {100*(dbr<=75).sum()/100:.0f}%   within 150 m: {100*(dbr<=150).sum()/100:.0f}%")
print(f"  top-100 OUTSIDE ENC chart coverage (M_COVR): {100*outc.sum()/100:.0f}%")
print(f"  top-100 EITHER near a bridge(<=150m) OR outside coverage: {100*((dbr<=150)|outc).sum()/100:.0f}%")

# ── map ─────────────────────────────────────────────────────────────────────
os.environ.setdefault("MPLCONFIGDIR", OUT)
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig,ax=plt.subplots(figsize=(12,11))
# land polygons (context)
try:
    land=json.load(open(LAND))
    for ft in land["features"]:
        g=ft.get("geometry");
        if not g: continue
        polys = g["coordinates"] if g["type"]=="Polygon" else (sum(g["coordinates"],[]) if g["type"]=="MultiPolygon" else [])
        rings = [g["coordinates"][0]] if g["type"]=="Polygon" else ([p[0] for p in g["coordinates"]] if g["type"]=="MultiPolygon" else [])
        for ring in rings:
            en=np.array([to_enu(c[1],c[0]) for c in ring])
            if len(en)>2: ax.fill(en[:,0],en[:,1],color="#eee8d5",zorder=0)
            if len(en)>1: ax.plot(en[:,0],en[:,1],color="#9c9madd"[:7] if False else "#b8b096",lw=0.4,zorder=0.5)
except Exception as ex:
    print("land draw skipped:", ex)

# anchorage / mooring / berth areas (context — could explain uncharted clusters)
for j,ring in enumerate(moor_rings):
    if len(ring)>2:
        ax.fill(ring[:,0],ring[:,1],facecolor="#c39bd3",edgecolor="#7d3c98",alpha=0.30,lw=0.8,
                zorder=1.5,label=("charted anchorage/mooring area" if j==0 else None))

# charted bridges (orange) + ENC coverage boundary (dotted)
if len(bridge_pts):
    ax.scatter(bridge_pts[:,0],bridge_pts[:,1],s=7,c="#e8820c",marker="s",alpha=0.75,
               label="charted bridges (57 features)",zorder=4.5)
_cov=[(42.30,-71.10),(42.375,-71.10),(42.375,-70.95),(42.30,-70.95),(42.30,-71.10)]
_cr=np.array([to_enu(la,lo) for la,lo in _cov])
ax.plot(_cr[:,0],_cr[:,1],color="#222",lw=1.6,ls=":",label="ENC coverage extent (lat<=42.375)",zorder=2.2)

# charted obstacles
if len(chart_fixed): ax.scatter(chart_fixed[:,0],chart_fixed[:,1],s=1.2,c="#1f3a5f",alpha=0.35,label=f"charted fixed obstacles ({len(chart_fixed)} pts)",zorder=2)
if len(chart_float): ax.scatter(chart_float[:,0],chart_float[:,1],s=14,c="#d08b00",marker="D",edgecolors="k",linewidths=0.3,alpha=0.9,label=f"charted floating buoys ({len(chart_float)})",zorder=4)

# expected radar cells classified by which chart layer explains them (@50 m)
if tot_cells:
    R=50.0
    by_obst = d_obst<=R
    by_land = (~by_obst) & (d_land<=R)
    unexpl  = ~(by_obst|by_land)
    ax.scatter(expected[by_obst,0],expected[by_obst,1],s=20,c="#2d8659",marker="s",alpha=0.85,label=f"radar structure @ discrete obstacle ({int(by_obst.sum())})",zorder=4)
    ax.scatter(expected[by_land,0],expected[by_land,1],s=16,c="#1f6fb0",marker="o",alpha=0.7,label=f"radar structure @ charted shore ({int(by_land.sum())})",zorder=3)
    ax.scatter(expected[unexpl,0],expected[unexpl,1],s=24,c="#aa3333",marker="x",linewidths=1.3,label=f"radar structure — UNCHARTED ({int(unexpl.sum())})",zorder=5)

# ownship tracks + AIS
for tr in own_tracks:
    en=np.array([to_enu(la,lo) for la,lo in tr]); ax.plot(en[:,0],en[:,1],color="#4488cc",lw=0.6,alpha=0.5,zorder=1)
if len(ais_enu): ax.scatter(ais_enu[:,0],ais_enu[:,1],s=10,c="#7711aa",marker="^",alpha=0.5,label=f"AIS vessel fixes ({len(ais_enu)})",zorder=3)

ax.set_aspect("equal"); ax.grid(True,alpha=0.25)
ax.set_xlabel(f"East (m)  from lat={lat_ref:.4f} lon={lon_ref:.4f}"); ax.set_ylabel("North (m)")
cov_pct = 100*(d_union<=50.0).sum()/max(1,tot_cells)
ax.set_title(f"Philos persistent radar structure vs Boston S-57 chart\n"
             f"{cov_pct:.0f}% of expected radar-structure cells within 50 m of a charted obstacle OR shoreline")
ax.legend(loc="upper right",fontsize=8,framealpha=0.9)
png=f"{OUT}/philos_chart_coverage.png"; plt.tight_layout(); plt.savefig(png,dpi=140); plt.close()
print(f"\nmap saved: {png}")
