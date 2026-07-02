# Boston ENC static-obstacle analysis (autoferry approach)

Source: NOAA ENC cells downloaded from https://charts.noaa.gov/ (S-57).
Parsed with GDAL 3.8.4 S57 driver. Reproduction: `python3 export_obstacles.py`.

## Cells & coverage

| Cell | Title | Extent (lon / lat) |
|---|---|---|
| US5BOSCC | Boston Inner Harbor — Charles R. & Neponset R. entrance to Old Harbor | lon −71.100…−71.025, lat 42.300…42.375 |
| US5BOSCD | President Roads to Boston Inner Harbor | lon −71.025…−70.950, lat 42.300…42.375 |
| US5BOSDC | Boston Inner Harbor to Mystic River and Chelsea River | Inner Harbor → Mystic/Chelsea R. (north arm) |

All three are **usage band 5 (harbour scale)** — the highest-detail ENC band,
which is what an autoferry approach needs. Together they tile the whole harbour
from the outer President Roads approach (US5BOSCD) through the Inner Harbor
(US5BOSCC) and up the north arm into the Mystic and Chelsea Rivers (US5BOSDC).
The autoferry corridor is fully inside this box. Features that overlap between
cells are de-duplicated by stable S-57 `LNAM` id, so the counts below are the
union, not a sum with double-counting.

## Verdict

**Yes — the charts contain the static-object information we need**, and richly
so. Every category of fixed hazard a surface ferry cares about is present, with
geometry in WGS84 lon/lat and IHO attributes (category, water level, sounding,
vertical clearance). See `geojson/` for the exported obstacles.

## What was exported (`geojson/`, EPSG:4326)

| File | Features | Content |
|---|---|---|
| `fixed_surface.geojson` | 3382 | Hard above-water structures a ferry can hit / that show on radar |
| `underwater_hazard.geojson` | 531 | Rocks, submerged wrecks/obstructions, submarine pipelines |
| `aid_fixed.geojson` | 16 | Beacons — fixed structures in the water |
| `aid_floating.geojson` | 114 | Lateral/special buoys — persistent (drifting) radar clutter, **not** fixed |
| `land.geojson` | 470 | LNDARE + COALNE — the shore / keep-clear boundary |
| `obstacles_all.geojson` | 4043 | Everything except land, merged, tagged with `hazard_group` |
| `radar_clutter.geojson` | 3613 | **Surface-breaking** obstacles + aids (`WATLEV`-filtered) — the radar-clutter set |
| `manifest.json` | — | Counts by bucket and by object class |

Features carry decoded properties: `obj_class`, `hazard_group`, `category`,
`watlev`, `sounding_m`, `height_m`, `vert_clearance_m`, `name`, `source_cell`,
`lnam` (stable S-57 feature id, used for cross-cell dedup).

### Fixed surface obstacles by class

shoreline construction 1789 (1207 pier/jetty · 272 sea wall · 134 rip rap ·
59 fender · 38 wharf/quay · 15 groyne · 10 ramp · …), pontoon 393,
pile/post 278, obstruction 125, bridge 78, silo/tank 76, building 505,
landmark 33, pylon 26, mooring facility 25, gate 18, offshore platform 20,
dry dock 4, dyke 3, crane 2, dam 3, hulk 2, fortified structure 1,
oil barrier 1.

### Underwater hazards

underwater rock 443, wreck 73 (including several **dangerous wrecks** — 46
dangerous, 26 showing superstructure, 1 showing masts — some
`covers and uncovers` / `always submerged` with charted soundings 3–10 m),
submarine pipeline 15.

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

`radar_clutter.geojson` = 3613 surface-breaking features (from every non-land
bucket), with `dropped 430` always-submerged / submarine features removed
(261 SLCONS, 94 OBSTRN, 42 wrecks, 18 rocks, 15 pipelines). Each feature is
tagged:

- `surface_breaking` (bool), `surface_state` — `always dry` (1371),
  `covers and uncovers` (577), `assumed dry (no WATLEV)` (1652),
  `floating` (11), `awash` (2).
- `radar_persistence` — `fixed` (3498) vs `floating` (115 = 114 nav buoys +
  1 mooring buoy). Floating aids drift within a watch circle and should be
  handled as *moving* clutter, not immovable obstacles.

**Completeness — every feature-bearing layer in all three cells was audited and
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
- **Tidal rocks decision:** the 577 `covers and uncovers` + 2 `awash` features
  (mostly Boston's drying ledges) *are* included, because they break the
  surface and clutter the radar at low water. If you want only permanently-dry
  obstacles, filter to `surface_state in {always dry, assumed dry, floating}`
  (≈3034 features) and drop the tidal ones.

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
  2026-06-25; US5BOSDC base 2025-12-02. Chart notes flag uncharted submarine
  pipelines/cables and that USACE surveys do not detect all underwater
  features — the ENC is not a complete obstacle truth set.
- Only the three supplied cells are covered; anything outside the box above
  (e.g. seaward of President Roads) needs the adjacent band-4/band-5 cells.

## Files added in this directory

- `export_obstacles.py` — the S-57 → GeoJSON exporter (idempotent).
- `geojson/` — output FeatureCollections + manifest.
- `ENC_ROOT/` — extracted ENC cells (from the three NOAA zips).
