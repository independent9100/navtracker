#!/usr/bin/env python3
"""
Export static obstacles from NOAA S-57 ENC cells to GeoJSON.

Input : ENC_ROOT/<CELL>/<CELL>.000  (S-57 base cells, WGS84 lon/lat)
Output: geojson/*.geojson            (EPSG:4326 FeatureCollections)

The ENC object classes are grouped into hazard buckets that match how the
navtracker static-obstacle branch consumes them (see ADR 0002):

  fixed_surface     piers/wharves, pontoons, piles, bridges, dolphins,
                    platforms, dry docks, silos, cranes, hulks, above-water
                    obstructions/wrecks -> hard things a ferry can hit and
                    that appear as persistent radar returns.
  underwater_hazard rocks, submerged wrecks/obstructions, submarine pipelines
                    -> grounding hazards, mostly not radar-visible.
  aid_fixed         beacons (fixed structures in the water).
  aid_floating      lateral/special buoys + mooring buoys -> persistent
                    (slowly drifting) radar clutter, NOT fixed obstacles.
  land              LNDARE / COALNE -> the shore / keep-clear boundary.

Everything except `land` is also merged into obstacles_all.geojson.

Run from the charts/ directory:  python3 export_obstacles.py
"""
import json
import os
from osgeo import ogr, gdal

gdal.SetConfigOption(
    "OGR_S57_OPTIONS",
    "RETURN_PRIMITIVES=OFF,RETURN_LINKAGES=OFF,LNAM_REFS=OFF,SPLIT_MULTIPOINT=ON",
)

CELLS = {
    "US5BOSCC": "ENC_ROOT/US5BOSCC/US5BOSCC.000",  # Inner Harbor / Charles R.
    "US5BOSCD": "ENC_ROOT/US5BOSCD/US5BOSCD.000",  # President Roads approach
}
OUT_DIR = "geojson"

# --- S-57 acronym -> human name (only classes we export) --------------------
CLASS_NAME = {
    "SLCONS": "shoreline construction", "PONTON": "pontoon",
    "PILPNT": "pile / post", "PYLONS": "pylon / bridge support",
    "BRIDGE": "bridge", "MORFAC": "mooring / warping facility",
    "OFSPLF": "offshore platform", "GATCON": "gate", "DAMCON": "dam",
    "DYKCON": "dyke", "DRYDOC": "dry dock", "SILTNK": "silo / tank",
    "FORSTC": "fortified structure", "CRANES": "crane",
    "BUISGL": "building", "LNDMRK": "landmark", "OILBAR": "oil barrier",
    "HULKES": "hulk", "OBSTRN": "obstruction", "UWTROC": "underwater rock",
    "WRECKS": "wreck", "PIPSOL": "submarine pipeline",
    "BCNLAT": "lateral beacon", "BCNSPP": "special-purpose beacon",
    "BOYLAT": "lateral buoy", "BOYSPP": "special-purpose buoy",
    "LNDARE": "land area", "COALNE": "coastline",
}

# hazard bucket -> ENC classes fed into it
BUCKETS = {
    "fixed_surface": ["SLCONS", "PONTON", "PILPNT", "PYLONS", "BRIDGE",
                      "MORFAC", "OFSPLF", "GATCON", "DAMCON", "DYKCON",
                      "DRYDOC", "SILTNK", "FORSTC", "CRANES", "BUISGL",
                      "LNDMRK", "OILBAR", "HULKES", "OBSTRN"],
    "underwater_hazard": ["UWTROC", "WRECKS", "PIPSOL"],
    "aid_fixed": ["BCNLAT", "BCNSPP"],
    "aid_floating": ["BOYLAT", "BOYSPP"],
    "land": ["LNDARE", "COALNE"],
}

# --- S-57 code lists (IHO S-57 App. A) --------------------------------------
WATLEV = {1: "partly submerged at HW", 2: "always dry", 3: "always submerged",
          4: "covers and uncovers", 5: "awash", 6: "subject to inundation",
          7: "floating"}
CATSLC = {1: "breakwater", 2: "groyne", 3: "mole", 4: "pier/jetty",
          5: "promenade pier", 6: "wharf/quay", 7: "training wall",
          8: "rip rap", 9: "revetment", 10: "sea wall", 11: "landing steps",
          12: "ramp", 13: "slipway", 14: "fender", 15: "solid face wharf",
          16: "open face wharf", 17: "log ramp"}
CATMOR = {1: "dolphin", 2: "deviation dolphin", 3: "bollard",
          4: "tie-up wall", 5: "post/pile", 6: "chain/wire/cable",
          7: "mooring buoy"}
CATPLE = {1: "stake", 2: "snag/stump", 3: "post", 4: "tripodal"}
CATOBS = {1: "snag/stump", 2: "wellhead", 3: "diffuser", 4: "crib",
          5: "fish haven", 6: "foul area", 7: "foul ground", 8: "ice boom",
          9: "ground tackle", 10: "boom"}
CATWRK = {1: "non-dangerous wreck", 2: "dangerous wreck",
          3: "distributed remains", 4: "wreck showing mast/masts",
          5: "wreck showing superstructure"}
CATBRG = {1: "fixed", 2: "opening", 3: "swing", 4: "lifting", 5: "bascule",
          6: "pontoon", 7: "draw", 8: "transporter", 9: "footbridge",
          10: "viaduct", 11: "aqueduct", 12: "suspension"}
CONDTN = {1: "under construction", 2: "ruined", 3: "under reclamation",
          4: "wingless", 5: "planned construction"}

CAT_FIELD = {"SLCONS": ("CATSLC", CATSLC), "MORFAC": ("CATMOR", CATMOR),
             "PILPNT": ("CATPLE", CATPLE), "OBSTRN": ("CATOBS", CATOBS),
             "WRECKS": ("CATWRK", CATWRK), "BRIDGE": ("CATBRG", CATBRG)}


def as_int(v):
    if isinstance(v, list):
        v = v[0] if v else None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def get(feat, name):
    idx = feat.GetFieldIndex(name)
    if idx < 0 or not feat.IsFieldSet(idx):
        return None
    return feat.GetField(idx)


def surface_state(cls, feat):
    """Radar-clutter view: does this feature break the sea surface?

    Returns (is_surface: bool, state: str). WATLEV drives it — an
    always-submerged structure/rock/wreck returns no radar and is NOT
    clutter, regardless of which obstacle bucket it sits in. Submarine
    pipelines are always sub-surface by definition.
    """
    if cls == "PIPSOL":
        return False, "submarine"
    wl = as_int(get(feat, "WATLEV"))
    if wl == 3:                      # always under water
        return False, WATLEV[3]
    if wl in WATLEV:                 # 1,2,4,5,6,7 all break/reach surface
        return True, WATLEV[wl]
    # no WATLEV coded: pontoons float; other charted structures are built
    # above water; treat as surface with an explicit "assumed" tag.
    return True, "assumed dry (no WATLEV)"


def build_props(cls, feat, cell, bucket):
    p = {"source_cell": cell, "obj_class": cls,
         "obj_class_name": CLASS_NAME.get(cls, cls), "hazard_group": bucket}
    name = get(feat, "OBJNAM")
    if name:
        p["name"] = name
    # category decode
    if cls in CAT_FIELD:
        fld, table = CAT_FIELD[cls]
        code = as_int(get(feat, fld))
        if code is not None:
            p["category_code"] = code
            p["category"] = table.get(code, f"code {code}")
    # water level
    wl = as_int(get(feat, "WATLEV"))
    if wl is not None:
        p["watlev_code"] = wl
        p["watlev"] = WATLEV.get(wl, f"code {wl}")
    # condition (ruined / under construction)
    cd = as_int(get(feat, "CONDTN"))
    if cd is not None:
        p["condition"] = CONDTN.get(cd, f"code {cd}")
    # depths / heights / clearances
    for src, dst in [("VALSOU", "sounding_m"), ("HEIGHT", "height_m"),
                     ("VERCLR", "vert_clearance_m"), ("VERLEN", "vert_length_m")]:
        v = get(feat, src)
        if v not in (None, 0, "0"):
            p[dst] = v
    # radar-clutter tagging
    is_surf, state = surface_state(cls, feat)
    p["surface_breaking"] = is_surf
    p["surface_state"] = state
    # floating aids (buoys, mooring buoys) drift within a watch circle;
    # everything else that breaks the surface is a fixed radar return.
    if cls in ("BOYLAT", "BOYSPP") or (
            cls == "MORFAC" and as_int(get(feat, "CATMOR")) == 7):
        p["radar_persistence"] = "floating"
    elif is_surf:
        p["radar_persistence"] = "fixed"
    else:
        p["radar_persistence"] = "none"
    p["lnam"] = get(feat, "LNAM")
    return p


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    collections = {b: [] for b in BUCKETS}
    seen = set()               # LNAM dedup across overlapping cells
    stats = {}                 # (bucket, class) -> count

    for cell, path in CELLS.items():
        ds = ogr.Open(path)
        if ds is None:
            raise SystemExit(f"cannot open {path}")
        for bucket, classes in BUCKETS.items():
            for cls in classes:
                lyr = ds.GetLayerByName(cls)
                if lyr is None:
                    continue
                lyr.ResetReading()
                for feat in lyr:
                    geom = feat.GetGeometryRef()
                    if geom is None or geom.IsEmpty():
                        continue
                    lnam = feat.GetFieldIndex("LNAM") >= 0 and feat.GetField("LNAM")
                    key = (lnam, cls)
                    if lnam and key in seen:
                        continue
                    if lnam:
                        seen.add(key)
                    props = build_props(cls, feat, cell, bucket)
                    collections[bucket].append({
                        "type": "Feature",
                        "properties": props,
                        "geometry": json.loads(geom.ExportToJson()),
                    })
                    stats[(bucket, cls)] = stats.get((bucket, cls), 0) + 1

    # write per-bucket files
    written = {}
    for bucket, feats in collections.items():
        fn = os.path.join(OUT_DIR, f"{bucket}.geojson")
        fc = {"type": "FeatureCollection",
              "name": bucket,
              "crs": {"type": "name",
                      "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
              "features": feats}
        with open(fn, "w") as f:
            json.dump(fc, f)
        written[fn] = len(feats)

    # merged obstacles (everything except land)
    merged = [ft for b, feats in collections.items() if b != "land"
              for ft in feats]
    fn = os.path.join(OUT_DIR, "obstacles_all.geojson")
    with open(fn, "w") as f:
        json.dump({"type": "FeatureCollection", "name": "obstacles_all",
                   "crs": {"type": "name",
                           "properties":
                               {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
                   "features": merged}, f)
    written[fn] = len(merged)

    # radar-clutter view: surface-breaking obstacles + aids only.
    # Drops always-submerged structures/rocks/wrecks and submarine pipelines;
    # keeps intertidal (covers/uncovers, awash) rocks/wrecks that return radar
    # at low water. Excludes land/coastline.
    clutter = [ft for b, feats in collections.items() if b != "land"
               for ft in feats if ft["properties"].get("surface_breaking")]
    fn = os.path.join(OUT_DIR, "radar_clutter.geojson")
    with open(fn, "w") as f:
        json.dump({"type": "FeatureCollection", "name": "radar_clutter",
                   "crs": {"type": "name",
                           "properties":
                               {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
                   "features": clutter}, f)
    written[fn] = len(clutter)
    clutter_by_state = {}
    clutter_by_persist = {}
    for ft in clutter:
        st = ft["properties"].get("surface_state")
        pr = ft["properties"].get("radar_persistence")
        clutter_by_state[st] = clutter_by_state.get(st, 0) + 1
        clutter_by_persist[pr] = clutter_by_persist.get(pr, 0) + 1
    dropped_submerged = len(merged) - len(clutter)

    # manifest
    manifest = {"cells": CELLS,
                "radar_clutter": {"total": len(clutter),
                                  "dropped_non_surface": dropped_submerged,
                                  "by_surface_state": clutter_by_state,
                                  "by_persistence": clutter_by_persist},
                "buckets": {b: len(fs) for b, fs in collections.items()},
                "by_class": {f"{b}/{c}": n for (b, c), n in sorted(stats.items())},
                "files": written}
    with open(os.path.join(OUT_DIR, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    # console report
    print("Wrote GeoJSON to", OUT_DIR + "/")
    for b in BUCKETS:
        print(f"  {b:18s} {len(collections[b]):5d} features")
    print(f"  {'obstacles_all':18s} {len(merged):5d} features (non-land)")
    print(f"  {'radar_clutter':18s} {len(clutter):5d} features "
          f"(surface-breaking; dropped {dropped_submerged} submerged/submarine)")
    print("   by surface_state:", clutter_by_state)
    print("   by persistence:  ", clutter_by_persist)
    print("\nBy object class:")
    for (b, c), n in sorted(stats.items()):
        print(f"  {b:18s} {c:8s} {CLASS_NAME.get(c, c):26s} {n:5d}")


if __name__ == "__main__":
    main()
