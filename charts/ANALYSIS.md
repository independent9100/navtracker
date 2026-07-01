# Boston ENC static-obstacle analysis (autoferry approach)

Source: NOAA ENC cells downloaded from https://charts.noaa.gov/ (S-57).
Parsed with GDAL 3.8.4 S57 driver. Reproduction: `python3 export_obstacles.py`.

## Cells & coverage

| Cell | Title | Extent (lon / lat) |
|---|---|---|
| US5BOSCC | Boston Inner Harbor — Charles R. & Neponset R. entrance to Old Harbor | lon −71.100…−71.025, lat 42.300…42.375 |
| US5BOSCD | President Roads to Boston Inner Harbor | lon −71.025…−70.950, lat 42.300…42.375 |

Both are **usage band 5 (harbour scale)** — the highest-detail ENC band, which
is what an autoferry approach needs. Together they tile the whole harbour from
the outer President Roads approach (US5BOSCD) through the Inner Harbor
(US5BOSCC). The autoferry corridor is fully inside this box.

## Verdict

**Yes — the charts contain the static-object information we need**, and richly
so. Every category of fixed hazard a surface ferry cares about is present, with
geometry in WGS84 lon/lat and IHO attributes (category, water level, sounding,
vertical clearance). See `geojson/` for the exported obstacles.

## What was exported (`geojson/`, EPSG:4326)

| File | Features | Content |
|---|---|---|
| `fixed_surface.geojson` | 2340 | Hard above-water structures a ferry can hit / that show on radar |
| `underwater_hazard.geojson` | 483 | Rocks, submerged wrecks/obstructions, submarine pipelines |
| `aid_fixed.geojson` | 14 | Beacons — fixed structures in the water |
| `aid_floating.geojson` | 110 | Lateral/special buoys — persistent (drifting) radar clutter, **not** fixed |
| `land.geojson` | 379 | LNDARE + COALNE — the shore / keep-clear boundary |
| `obstacles_all.geojson` | 2947 | Everything except land, merged, tagged with `hazard_group` |
| `radar_clutter.geojson` | 2635 | **Surface-breaking** obstacles + aids (`WATLEV`-filtered) — the radar-clutter set |
| `manifest.json` | — | Counts by bucket and by object class |

Features carry decoded properties: `obj_class`, `hazard_group`, `category`,
`watlev`, `sounding_m`, `height_m`, `vert_clearance_m`, `name`, `source_cell`,
`lnam` (stable S-57 feature id, used for cross-cell dedup).

### Fixed surface obstacles by class

shoreline construction 1268 (871 pier/jetty · 190 sea wall · 81 rip rap ·
38 fender · 29 wharf/quay · 14 groyne · 9 breakwater · …), pontoon 235,
pile/post 185, obstruction 105, bridge 57, silo/tank 42, building 342,
landmark 27, pylon 23, mooring facility 21, gate 12, offshore platform 8,
dry dock 4, dyke 3, crane 2, dam 2, hulk 2, oil barrier 1.

### Underwater hazards

underwater rock 427, wreck 43 (including several **dangerous wrecks**, some
`covers and uncovers` / `always submerged` with charted soundings 3–10 m),
submarine pipeline 13.

## How this maps to the static-obstacle branch (ADR 0002)

- The vessel-vs-environment split holds: `land.geojson` = the environment
  boundary (coastline); `obstacles_all.geojson` = discrete fixed hazards that
  are the `StaticObstacle` inputs, kept **separate from the coastline** as the
  ADR requires.
- `fixed_surface` + `aid_fixed` are true keep-clear / radar-persistent
  obstacles → candidate `StaticObstacle` / keep-clear geometry.
- `aid_floating` (buoys) is the persistent-radar-clutter set — relevant to the
  synthetic-clutter / over-count work, but should **not** be fed as immovable
  obstacles (buoys watch-circle drift).
- `underwater_hazard` is grounding-risk context; mostly not radar-visible, so
  it informs routing/keep-clear but not track↔obstacle association.

## Radar-clutter coverage (`radar_clutter.geojson`)

"Radar clutter" is a different lens than "hittable obstacle": it is any
**persistent radar return that can spawn a false/extra track**. The
discriminator is not the obstacle bucket but **water level (`WATLEV`)** — a
structure/rock/wreck that is *always submerged* returns no radar and is not
clutter, while a *drying* rock is a strong return at low water.

`radar_clutter.geojson` = 2635 surface-breaking features (from every non-land
bucket), with `dropped 312` always-submerged / submarine features removed
(174 SLCONS, 78 OBSTRN, 30 wrecks, 17 rocks, 13 pipelines). Each feature is
tagged:

- `surface_breaking` (bool), `surface_state` — `always dry` (943),
  `covers and uncovers` (540), `assumed dry (no WATLEV)` (1142),
  `floating` (8), `awash` (2).
- `radar_persistence` — `fixed` (2524) vs `floating` (111 = 110 nav buoys +
  1 mooring buoy). Floating aids drift within a watch circle and should be
  handled as *moving* clutter, not immovable obstacles.

**Completeness — every feature-bearing layer in both cells was audited and
classified** (see `export_obstacles.py`). Findings:

- No dedicated radar objects exist here: **no `RTPBCN` (racon), no `RADRFL`
  (radar reflector as its own object), no `RADSTA`**, and **zero features carry
  the `CONRAD` "radar conspicuous" flag** — so clutter cannot be identified from
  that attribute; it must come from the physical-structure classes above.
- `LIGHTS` (8+38), `DAYMAR` (1+3), `FOGSIG` (3) are **intentionally excluded**:
  in S-57 these are the light/mark/signal mounted *on* a support (beacon, pile,
  landmark, dolphin) that is already exported — adding them double-counts the
  same radar return. `CGUSTA` (shore station) and `FNCLNE` (land fence) are
  on-land, already inside `land`/`BUISGL`.
- **Tidal rocks decision:** the 540 `covers and uncovers` + 2 `awash` features
  (mostly Boston's drying ledges) *are* included, because they break the
  surface and clutter the radar at low water. If you want only permanently-dry
  obstacles, filter to `surface_state in {always dry, assumed dry, floating}`
  (≈2093 features) and drop the tidal ones.

## Gaps / caveats

- **No heights on most structures.** Vertical clearance is populated for
  bridges (e.g. Summer Street Bridge 1.8 m) but `height_m` is sparse — radar
  cross-section / above-water extent must be inferred from class + `watlev`.
- **Geometry granularity varies.** SLCONS is a mix of Line String (edges) and
  Polygon (footprints); consumers must handle both. Piles/rocks/buoys are
  points.
- **Buoys are floating.** Positions are the charted watch-circle centre, not a
  live position — treat as approximate.
- **Currency.** US5BOSCD edition 4, updated through 2026-01-08; US5BOSCC base
  2026-06-25. Chart notes flag uncharted submarine pipelines/cables and that
  USACE surveys do not detect all underwater features — the ENC is not a
  complete obstacle truth set.
- Only the two supplied cells are covered; anything outside the box above
  (e.g. seaward of President Roads) needs the adjacent band-4/band-5 cells.

## Files added in this directory

- `export_obstacles.py` — the S-57 → GeoJSON exporter (idempotent).
- `geojson/` — output FeatureCollections + manifest.
- `ENC_ROOT/` — extracted ENC cells (from the two NOAA zips).
