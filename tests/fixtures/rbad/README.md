# R-BAD berthing fixtures (committed source; data local-only)

Fixtures for a **fixed-frame, label-scored berthing replay** derived from the
**R-BAD** dataset (Radar-Based Berthing-Aid Dataset).

- **Dataset:** Zenodo record `16936465` (doi:10.5281/zenodo.16936465),
  **CC-BY-4.0**. Paper: MDPI *Electronics* 14(20):4065 (2025).
- **Sensor:** four TI FMCW **mmWave** chips — IWR6843 (60–64 GHz) +
  AWR1443/1642/1843 (77–81 GHz), 20 Hz, short range (≤ ~56 m observed).
- **Platform:** an operational Ro-Ro/passenger ferry; 69 h across 13 ports.

## ⚠️ Regime caveat (governs every claim)

This is an **automotive/industrial mmWave FMCW** sensor class, **not marine
X-band**. It corroborates the berthing *scene* on a new sensor; it is **not** a
third marine-radar geography, and philos/HAXR tuning is **not** expected to
transfer. No number from these fixtures is a marine-radar result.

## Two structural facts (both baked into how the fixtures are used)

1. **No ego pose anywhere** in the dataset (no GPS/IMU/heading file; detections
   are sensor body-frame Cartesian). We replay it as a **fixed-frame relative-
   tracking** scene: own-ship is the origin, body frame ≡ a fixed ENU frame
   (E = X starboard, N = Y forward). Consequence: nothing is world-stationary in
   a body frame, so the anchored/moored static-hazard logic (ADR 0002) is
   **untestable here by construction** — its non-firing is not evidence.
2. **The labels are a reference tracker, not ground truth.** The clusters and
   `Tracking_ID` are the authors' own onboard clustering/tracking pipeline.
   Scoring against them is a **cross-tracker consistency** measure (reported
   `vs_reference_tracker`), never an accuracy claim. `Dock_Label` (binary 0/1)
   meaning is unconfirmed (paper bot-blocked) → distributions only, assert
   nothing on it.

## What is committed vs local

Committed (source): `generator/*.py`, this `README.md`, `requirements.txt`.
**Not** committed (multi-MB, gitignored, produced on demand): the downloaded
`_download/` archive and the per-scenario `rbad_*/` fixture dirs.

## Obtaining the data + regenerating fixtures

```sh
# 1. Download + unzip ONLY the small labelled buffers (~31 MB); the 31.6 GB raw
#    archive is NOT needed (it is video-dominated — see the deferred route below).
mkdir -p tests/fixtures/rbad/_download && cd tests/fixtures/rbad/_download
curl -L -o Labelled_Buffers_Data.zip \
  "https://zenodo.org/api/records/16936465/files/Labelled%20Buffers%20Data.zip/content"
# expect md5 894a55b05552c57b85e60535fe433e85
unzip -q Labelled_Buffers_Data.zip -d labelled

# 2. Extract the replay fixtures (stdlib only; fails loud on integrity violations)
cd -  # back to repo root
python tests/fixtures/rbad/generator/extract_rbad.py
```

Output per scenario dir `rbad_<port>_<n>/`:

| file | columns | role |
|---|---|---|
| `radar_plots.csv` | `tod,range_m,azimuth_deg,sigma_r_m,sigma_az_deg,n_cells,amp_max,station,v_doppler_mps,snr_db` | detections fed to the tracker — one plot per clustered object at its centroid. First 8 cols are navtracker's standard plot schema; `v_doppler_mps`+`snr_db` are carried forward as **columns only** (first in-hand per-detection Doppler). |
| `reference_tracks.csv` | `tod,ref_id,east_m,north_m,dock_label` | the authors' reference-tracker trajectories + binary label; the **consistency reference**, not truth. |
| `meta.txt` | — | provenance + per-buffer stats + the caveats above. |

`CHECKSUMS.txt` (sha256) is written at the fixture root. Re-runs are
byte-identical (fixed float precision + explicit sort).

## Deferred route (preserved for free)

The 31.6 GB **Raw Aggregated Frames** archive's only added value is the synced
MP4 video. Download it **only** if a berthing-scene result ever needs
**independent kinematic truth** — then commission a manual video-label pass
(the philos R8 workflow). Until that trigger, it stays undownloaded.

## Scope

6 arrival approaches across 2 ports (~28 min of 1 Hz labelled data), a
representative subset of the ~121 min of labelled arrivals available locally
across 13 ports. This is a **reality-check arm, not a tuning target** — no
config is tuned to this dataset.
