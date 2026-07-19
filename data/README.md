# navtracker validation data

Real-world maritime sensor recordings for validating the tracker against
the synthetic harness in `tests/scenario/`. Each subdirectory holds a
representative sample of one source. The data **payload** is **not** committed
(see `.gitignore`); this README — the license/provenance manifest, the record of
which datasets are non-commercial and what was removed and why — **is**
git-tracked (W6.3.3), so edits go through a branch like any other doc.

Total on-disk for the bundled samples: ~2.3 GB.

**Status legend.** *Fetched* = present on disk and consumed by the build/tests.
*Staging-only* = present on disk but **not** consumed by the C++ test suite
(safe to prune to reclaim disk; re-fetchable via the per-source commands below).
*Form-gated* / *REMOVED* as noted per row.

| Source       | Sensors                                | License        | Sample size | Status      |
| ------------ | -------------------------------------- | -------------- | ----------- | ----------- |
| `dlr/`       | X-band radar (polar HDF5) + AIS (CSV)  | CC-BY-4.0      | 1.4 GB      | Fetched     |
| `pohang/`    | Marine radar (PNG) + GPS + AHRS        | CC-BY-NC-4.0   | —           | **REMOVED 2026-07-12** (license NO-GO — non-commercial, same class as MOANA/GFW; user directive; zero C++ tests consumed it; re-fetchable from source if ever re-licensed) |
| `marinecadastre/` | AIS (CSV)                         | US public      | 277 MB      | **Staging-only** (no C++ test consumes it) |
| `dma/`       | AIS (CSV)                              | DMA open data  | 266 MB      | **Staging-only** (no C++ test consumes it; the DMA *date-format* handling in `AisCsvReplayAdapter` is unrelated to this dir) |
| `stonesoup/` | Solent AIS demo + Stone Soup framework | MIT            | 56 MB       | **Staging-only** (framework clone; used only by the offline `tools/stonesoup_gospa_crosscheck.py`, not the build) |
| `kystverket/`| Norway live AIS (GeoJSON snapshots)    | NLOD           | 15 MB       | **Staging-only** (no C++ test consumes it) |
| `philos/`    | AIS + radar + EO (R/V Philos)          | CC-BY-NC-SA-4.0| —           | Form-gated  |
| `autoferry/` | radar + lidar + EO + IR (milliAmpere)  | CC-BY-4.0      | 7 MB        | Fetched     |

---

## AutoFerry milliAmpere — heterogeneous multi-sensor MTT (`autoferry/`)

Source: https://github.com/Autoferry/sensor_fusion_dataset
Paper: Helgesen, Vasstein, Brekke, Stahl, "Heterogeneous multi-sensor
tracking for an autonomous surface vehicle in a littoral environment",
*Ocean Engineering* 252 (2022) 111168.

The NTNU `milliAmpere` autonomous passenger-ferry sensor rig (Simrad 4G
radar, Velodyne VLP-16 lidar, 5× FLIR Blackfly EO, 5× FLIR Boson IR)
recorded against **RTK-GNSS ground truth** (10 mm / 0.05°) on three
reference targets (Havfruen, Gunnerus, Jetboat) across two environments:
open water (env 1) and an urban channel (env 2).

**Why it matters.** This is navtracker's best *true-ground-truth*,
multi-target, heterogeneous real-world benchmark — strictly better than
the AIS-as-truth philos/haxr replays. The labelled scenarios exercise the
exact hard cases the synthetic harness only approximates: target
obscuration / sensor-shadow, merged measurements, track merging, track
continuation, high-speed overtake, and maneuvers. The paper's tracker is
VIMMJIPDA — the JIPDA upgrade on navtracker's own roadmap — giving a
published RMSE/GOSPA/track-break baseline to compare against.

**Format.** Per-scenario folder with `<label>_detections.json` +
`<label>_groundTruth.json` (and a redundant `.mat`). Detections are in an
ownship-fixed Piren NED frame (origin LLA `[63.4389029083, 10.39908278,
39.923]`); the loader maps NED→ENU by planar axis swap. Active sensors
(lidar=1, radar=2) carry 2×M `[norths, easts]` points → Position2D;
passive (IR=3, EO=4) carry 1×M bearings → Bearing2D. Loader:
`adapters/replay/AutoferryJsonReplay.{hpp,cpp}`; bench wrapper
`defaultAutoferryScenarios()` in `adapters/benchmark/ReplayScenarioRun.cpp`.

**Sample on disk.** 9 scenarios with GT coverage — env 1: 2,3,4,5,6;
env 2: 13,16,17,22. ~7 MB JSON.

**Fetch command (all 9 scenarios):**
```bash
base="https://raw.githubusercontent.com/Autoferry/sensor_fusion_dataset/main"
for s in 2 3 4 5 6 13 16 17 22; do
  mkdir -p data/autoferry/scenario$s
  curl -sSL -o data/autoferry/scenario$s/scenario${s}_detections.json \
    "$base/scenario$s/scenario${s}_detections.json"
  curl -sSL -o data/autoferry/scenario$s/scenario${s}_groundTruth.json \
    "$base/scenario$s/scenario${s}_groundTruth.json"
done
```

**Current limitations (read before drawing conclusions).**
1. **Sensor selection.** The loader emits radar+lidar (Position2D) always
   and EO/IR (Bearing2D) when `AutoferryLoadOptions::include_bearings=true`
   (the bench AutoFerry scenarios enable it; the synthetic-default is off).
   Passive bearings only *refine* tracks — the track-birth paths drop a
   non-gating bearing (`canInitiateTrack`) rather than seed an
   unobservable-range position, matching the paper's active-only
   initialization (§4.4.1). Note: naive four-sensor fusion currently
   *degrades* accuracy because ~27% of EO/IR detections are clutter and the
   tracker lacks outlier-robust updates — see the AutoFerry section in
   `docs/baselines/README.md`.
2. **JPDA full-enumeration degrades under clutter.** Real scans carry up
   to ~24 detections (≈2 targets + clutter); the `O(M^T)` joint-event
   enumeration OOMs once clutter spawns extra tracks. `JpdaAssociator` now
   caps enumeration (1e6 events) and falls back to greedy hard assignment
   per overflowing cluster (`AssociationResult::overflow_fallback`). On
   these scenarios the JPDA configs therefore partly behave as GNN — the
   honest fix is cluster decomposition + an EHM solver (see
   `sota-roadmap.md`).
3. **No GT velocity.** Ground truth is position-only, so `sog_rmse_mps` /
   `cog_rmse_deg` are not meaningful here (same caveat as the other
   replays). OSPA / position RMSE / continuity / ID-switches are valid.

---

## DLR HAXR — Hamburg port X-band radar + AIS (`dlr/`)

DOI: `10.5281/zenodo.19824555`. Published Feb 2026 by DLR Institute of
Communications and Navigation (Meinert, Moreira).

This is the open-data successor to the request-only Fowdur, Baum &
Heymann 2021 dataset cited in `docs/algorithms/sota-roadmap.md`. 13
overlapping shore-mounted maritime radar stations along the Elbe at
Hamburg port, hourly UTC chunks. Anonymised AIS-derived vessel position
reports per station serve as track ground truth.

**Sensor stack.**
- Radar: polar (range, azimuth, amplitude, time) X-band, per-station
  rotation cycles, HDF5.
- AIS: CSV per station per hour, columns
  `tod, uid, range_m, azimuth_deg` — already projected into each
  station's body frame.
- Station positions: `stations.csv`, columns `station, x_m, y_m`,
  shared local Cartesian frame.

**Sample on disk.** 3 stations × 3 hours = 18 sensor files (9 HDF5,
9 CSV) plus `stations.csv`, `manifest.json`, `demo.py`.
- Stations: `kattwyk`, `seemannshoeft`, `parkhafen`.
- Hours: 08, 09, 11 UTC.

**Why it matters for navtracker.** Real radar + co-temporal ground-truth
labels in radar-sensor frame. Closest open analogue to the
multi-radar / AIS-as-truth setup that exposes the tracker's
range/bearing branch on real noise statistics.

**Fetch command (single station-hour):**
```bash
curl -sSL -o data/dlr/kattwyk_08-UTC.hdf5 \
  https://zenodo.org/api/records/19824555/files/kattwyk_08-UTC.hdf5/content
curl -sSL -o data/dlr/kattwyk_08-UTC.csv \
  https://zenodo.org/api/records/19824555/files/kattwyk_08-UTC.csv/content
```

**Scale up.** 81 files total (13 stations × 3 hours × 2 modalities +
metadata), 97 GB across all versions. Loop over the station list in
`stations.csv` to expand to the full multi-station picture; the
companion `haxr` Python package on PyPI handles HDF5 cycle segmentation.

---

## Pohang Canal Dataset (`pohang/`)

KAIST MORIN lab, AWS Open Data sponsorship. AWS S3 bucket:
`s3://pohang-canal-dataset` (`us-west-2`, no AWS account needed).
Direct HTTP via `https://pohang-canal-dataset.s3.us-west-2.amazonaws.com/`.

A 7.5 km canal/port/coastal route through Pohang, South Korea, recorded
from a single moving vessel with the closest-to-navtracker sensor mix in
the open: marine radar, GPS+RTK, AHRS, plus stereo / IR / 6-direction
omnicam / LiDAR for completeness. Per-object track ground truth via
PoLaRIS (arXiv:2412.06192) is the planned extension.

**6 sequences.** `pohang00..pohang05`; `pohang01` and `pohang05` are
night-time.

**Sample on disk.** `pohang00` (daytime canal) and `pohang02` (daytime
inner port), each with three small zips:
- `radar.zip` (175–180 MB): `radar/images/*.png` PPI scans +
  `radar/timestamp.txt` + `radar/timestamp_deg.txt` for ownship heading
  per scan.
- `navigation.zip` (21–25 MB): `navigation/gps.txt`, `ahrs.txt`,
  `baseline.txt` — ownship pose time series.
- `calibration.zip` (3 KB): per-sensor extrinsics.

**Why it matters.** Real marine radar against real GPS+AHRS ownship pose
on a moving vessel, two scenario geometries. Validates the
range/bearing/EKF path under genuine antenna rotation + ego-motion +
canal multipath. The image-domain radar can be converted to plot
detections via simple threshold + connected components (or fed into our
existing plot-style `Measurement` builder once a per-pixel range/azimuth
mapping is recovered from the calibration).

**Fetch command (one sequence, the navigation-relevant subset):**
```bash
SEQ=pohang00
B=https://pohang-canal-dataset.s3.us-west-2.amazonaws.com
for f in calibration.zip navigation.zip radar.zip; do
  curl -sSL -o data/pohang/${SEQ}/${f} ${B}/${SEQ}/${f}
done
```

**LiDAR via fixture-then-cleanup.** We deliberately do not keep raw
LiDAR archives on disk. The `tests/fixtures/pohang_lidar/` fixture
fetches `lidar_port.zip` (~12 GB), runs DBSCAN to produce a
plots CSV, then **deletes the zip and any unzipped point clouds**.
The post-cleanup state of `data/pohang/pohang00/lidar/` is empty;
the resulting plots CSV lives under `tests/fixtures/pohang_lidar/out/`.
See that fixture's `run_and_cleanup.sh` for the canonical workflow.

**Scale up.** Skip `stereo.zip` (~110 GB/sequence), `omni.zip` (~70 GB).
`infrared.zip` is 5.9 GB/sequence — drop in only if you want to extend
bearing-only EO/IR validation. List the full bucket with:
```bash
curl -s 'https://pohang-canal-dataset.s3.us-west-2.amazonaws.com/?list-type=2'
```

License: **CC-BY-NC-4.0**. Non-commercial only.

---

## MarineCadastre — US coastal AIS (`marinecadastre/`)

NOAA Office for Coastal Management. **US public domain.**

Daily AIS bulk archives for US coastal waters since 2009, distributed as
zipped CSV. Modern years (2024–) are also on AWS as cloud-optimised
GeoParquet but the zip→CSV pipeline is the simplest replay path.

**Sample on disk.** `AIS_2024_01_01.zip` (277 MB compressed, ~3 GB CSV).
Columns: `MMSI, BaseDateTime, LAT, LON, SOG, COG, Heading, VesselName,
IMO, CallSign, VesselType, Status, Length, Width, Draft, Cargo,
TransceiverClass`.

**Why it matters.** Hundreds-of-targets simultaneous AIS density,
dropout statistics, real MMSI distributions, real ship-type and length
data for feature-aided association (see `sota-roadmap.md` honourable
mention).

**Fetch command:**
```bash
curl -sSL -o data/marinecadastre/AIS_YYYY_MM_DD.zip \
  https://coast.noaa.gov/htdata/CMSP/AISDataHandler/YYYY/AIS_YYYY_MM_DD.zip
```

**Scale up.** Drop the date; the URL template generates 12 months ×
~28 days of archives per year, ~150–300 MB each.

---

## DMA — Danish AIS (`dma/`)

Danish Maritime Authority. Primary distribution at `https://web.ais.dk/`
which was 502'ing during this fetch — we used the figshare mirror by
Anita Graser (DOI `10.6084/m9.figshare.11577543`). Same source data,
permanent URL.

**Sample on disk.**
- `dk_csv_20170701.7z` (152 MB → 2.0 GB CSV `aisdk_20170701.csv`).
- `dk_csv_20180101.7z` (114 MB → 1.6 GB CSV `aisdk_20180101.csv`).
- Schema is the standard DMA AIS dump (timestamp, MMSI, lat, lon, SOG,
  COG, heading, ship-type, length, beam, draft, destination, ...).

**Why it matters.** Dense Baltic / North-Sea ferry and short-sea
traffic — qualitatively different traffic mix from the US coastal set.

**Fetch command (figshare mirror):**
```bash
curl -sSL -o data/dma/dk_csv_20170701.7z \
  https://ndownloader.figshare.com/files/26782631
curl -sSL -o data/dma/dk_csv_20180101.7z \
  https://ndownloader.figshare.com/files/26782541
# Full January 2017 (3.8 GB compressed):
# curl -sSL -o data/dma/dk_csv_jan2017.7z \
#   https://ndownloader.figshare.com/files/20894946
# Extract:
7z x data/dma/dk_csv_20170701.7z -odata/dma/
```

**Scale up.** When `web.ais.dk` returns, daily archives 2014–present are
at `https://web.ais.dk/aisdata/aisdk-YYYY-MM-DD.zip`. Until then,
figshare has Jan 2017 (31 days), 2017-07-01, and 2018-01-01.

---

## Stone Soup framework + Solent AIS demo (`stonesoup/`)

Dstl Stone Soup framework, MIT-licensed, full source clone. Provides
scenario generators, OSPA / GOSPA metrics, GLMB / JPDA / PHD reference
trackers, and an out-of-the-box real-world demo for the Solent estuary
(UK).

**Sample on disk.**
- Full Stone Soup repo at `stonesoup/Stone-Soup/` (~56 MB).
- Solent AIS demo data: `docs/demos/SolentAIS_20160112_130211.csv`
  (18 624 records, columns `Time, MMSI, Latitude_degrees,
  Longitude_degrees, COG_degrees, SOG_knots`). Real Solent harbor AIS,
  ~10 minutes from 2016-01-12.
- Demo driver: `docs/demos/AIS_Solent_Tracker.py`.

**Why it matters.** Two uses.
1. **Replay** the Solent demo through `AisNmeaAdapter` (after column
   re-mapping) for an externally-authored real-world AIS scenario.
2. **Metric cross-validation** — score our tracks with
   `stonesoup.metricgenerator.OSPAMetric` and confirm
   `core/scenario/Ospa.hpp` agrees. The simplest hedge against our
   metric having a bug.

**Fetch command:**
```bash
git clone --depth 1 --filter=blob:limit=10m \
  https://github.com/dstl/Stone-Soup.git data/stonesoup/Stone-Soup
pip install stone-soup
```

---

## Kystverket — Norway live AIS (`kystverket/`)

Norwegian Coastal Administration. **Norwegian Licence for Open
Government Data (NLOD).**

Two access paths:
- **Historical bulk** (`ais-public.kystverket.no/ais-download/`) is a
  request form — specify time window, ship type, bounding box; CSV or
  GeoParquet emailed back. Not auto-fetchable.
- **Realtime GeoJSON** (`kystdatahuset.no/ws/api/ais/realtime/geojson`) —
  no auth, returns one snapshot of all current Norwegian-waters
  positions per call. Polling this builds a longitudinal sample.

**Sample on disk.** 5 realtime snapshots ~20 s apart on 2026-06-04
~18:11Z, each ~3 MB GeoJSON FeatureCollection. Per-feature properties:
`mmsi, imo, ship_name, ship_type, ais_class, callsign, length, breadth,
draught, cog, speed, true_heading, status, maneuvre, destination,
date_time_utc`, plus a 2-point `LineString` geometry.

**Why it matters.** The only continuous-poll real-world AIS source in
the bundle. Feeds a soak test that runs without operator action and
exposes the tracker to genuine arrival/departure dynamics and MMSI
reassignments over hours/days.

**Fetch / poll commands:**
```bash
# Single snapshot:
curl -sSL -o data/kystverket/realtime_$(date -u +%Y%m%dT%H%M%SZ).json \
  https://kystdatahuset.no/ws/api/ais/realtime/geojson

# 1 Hz poll for an hour:
for i in $(seq 1 3600); do
  T=$(date -u +%Y%m%dT%H%M%SZ)
  curl -sSL -o data/kystverket/rt_${T}.json \
    https://kystdatahuset.no/ws/api/ais/realtime/geojson
  sleep 1
done
```

Other useful Kystverket endpoints (all GET, no auth):
- `/ws/api/ais/realtime/vessels-by-type` — fleet composition snapshot.
- `/ws/api/anchorage/areas` — port-area polygons.
- `/ws/api/voyage/infographics/arrivals-per-day` — traffic stats.

POST endpoints (require simple JSON body, no auth): historical position
lookup by MMSI + time, bbox + time, polygon + time. See
`https://kystdatahuset.no/ws/swagger/v1/swagger.json`.

---

## MIT Sea Grant R/V Philos (`philos/`) — NOT FETCHED

CC-BY-NC-SA-4.0, distributed only through a download form at
`https://seagrant.mit.edu/auvlab-datasets-marine-perception-2-3/`.
No public S3 / direct URL. The form asks for name, email,
organisation, and intended use; download URL is emailed back.

**The two datasets to request** (both 2022-11-07, R/V Philos in Boston):
- `philos_2022_11_07_ais_ferry_near.tar.gz` (1.0 GB, ~20 s) —
  the canonical AIS-projected-on-radar example,
  ferry *Frederick Nolan*.
- `philos_2022_11_07_ais_ferry_far.tar.gz` (0.9 GB) — distant
  vessel variant.

Each ROS bag contains AIS, marine radar imagery, and centre-camera
video. Once obtained, extraction does not require a ROS install — the
`rosbags` (or `bagpy`) Python package reads `.bag` files standalone.

**To request:** fill the form on the URL above with the project context
("evaluation of multi-sensor maritime fusion tracker — academic
research"). Place the downloaded `.tar.gz` files in this directory; the
README will not change.

---

## Quick provenance / size summary

```
data/dlr/                     1.4 GB  CC-BY-4.0       18 files
data/pohang/pohang00/         195 MB  CC-BY-NC-4.0    3 zips
data/pohang/pohang02/         200 MB  CC-BY-NC-4.0    3 zips
data/marinecadastre/          277 MB  US public       1 zip (=> ~3 GB CSV)
data/dma/                     266 MB  DMA open        2 7z (=> ~3.6 GB CSV)
data/stonesoup/                56 MB  MIT             Stone Soup repo + Solent CSV
data/kystverket/               15 MB  NLOD            5 GeoJSON snapshots
data/philos/                       —  CC-BY-NC-SA-4.0 request-only
```

Last fetched: 2026-06-04 by `/data/README.md`.
