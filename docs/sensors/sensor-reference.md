# Maritime Sensor Data Reference

**Date:** 2026-05-28 · **Status:** Living document

Per-sensor reference for what each input provides, in what units, at what rate, with what error characteristics — the information the fusion engine needs to build normalized `Measurement`s with correct covariances and to fuse identity/attributes. Figures are typical / standards-based starting points; calibrate `R` from real data per installation.

Legend: **R** = measurement covariance to assign · **rate** = nominal update interval · **frame** = native reference frame.

---

## 1. AIS (Automatic Identification System)

Cooperative transponder broadcast. Identity-rich but only for vessels that carry and enable it; absent on non-cooperative/small craft; spoofable. Standard: **ITU-R M.1371-5**; message structure per IEC 62320 / NavCen.

### Data provided
- **Dynamic** (Msg 1/2/3 Class A, Msg 18/19 Class B): latitude, longitude (WGS-84), **SOG** (speed over ground), **COG** (course over ground), **true heading** (Class A; often absent Class B), rate of turn (Class A), navigational status, position-accuracy flag, timestamp (UTC second).
- **Static/Voyage** (Msg 5 Class A, Msg 24 Class B): **MMSI**, IMO number, name, call sign, **ship type**, **dimensions** (A/B/C/D → length, beam, antenna offset), draught, destination, ETA.

### Units
Position degrees (WGS-84); SOG knots (0.1 kn resolution); COG degrees (0.1°); heading degrees (1°); ROT °/min.

### Update rate (dynamic)
- **Class A** (SOTDMA), speed/maneuver dependent:
  - anchored/moored: ~3 min · 0–14 kn: ~10 s · 0–14 kn turning: ~3⅓ s · 14–23 kn: ~6 s · 14–23 kn turning: ~2 s · >23 kn: ~2 s.
- **Class B** (CS-TDMA): ~30 s when >2 kn; ~3 min when ≤2 kn.
- **Static** (both): ~6 min.

### Error / covariance characteristics
- Position derives from the vessel's own GNSS: typically ~10 m CEP, but **position-accuracy flag** distinguishes high (<10 m, DGNSS) vs low (>10 m). Set `R` from the flag.
- SOG/COG GNSS-derived; COG noisy at low speed (set large COG `R` when SOG small).
- Heading from gyro/compass if present; may be default/unavailable (511° = not available).
- Latency: report timestamp is UTC second of position fix — usually reliable; account for transmission/aggregation delay.

### Identity content
Strongest identity source: **MMSI** (primary external key), name, call sign, type, dimensions → feed attribute fusion. Use MMSI as an association *hint*, validated by kinematic plausibility (anti-spoof, §gating).

### Failure modes / gotchas
- Non-cooperative targets emit nothing → no AIS track.
- Spoofing / wrong manual entry (type, dimensions, even position). Plausibility-gate before trusting.
- Class B sparse and may lack heading.
- "Position accuracy" flag often optimistic.
- Lat/lon → ENU conversion needed; watch ±180° meridian and pole edge cases (rare operationally).

---

## 2. Navigation radar / ARPA

Active radar with onboard Automatic Radar Plotting Aid producing tracks. All-weather, non-cooperative detection. Standard: **IMO Res. A.823(19)** (ARPA performance). Can be **multiple radars** (X-band ~9 GHz for resolution, S-band ~3 GHz for weather penetration); high-res W-band (76–77 GHz) for harbor.

### Data provided
- Per ARPA track: **track ID**, target **range** and **bearing** (relative to own-ship), derived **course** and **speed** (over ground or through water depending on stabilization), **CPA/TCPA**. Sometimes target length/size estimate from echo extent.
- Or, if no ARPA: raw plots/detections (range, bearing, amplitude) — would need our own plot-to-track, currently out of scope (we assume ARPA tracks).

### Units
Range meters/NM; bearing degrees relative (convert to true using own-ship heading); speed knots; course degrees true.

### Update rate
Tied to antenna rotation: typically **~2–3 s/scan** (X-band ~24–45 rpm). Track state refreshes once per scan.

### Error / covariance characteristics (A.823(19), 95%)
- Range accuracy: within **~30–50 m or ~1% of range**.
- Bearing accuracy: within **~1–2°** (note bearing error → cross-range error grows linearly with range: σ_crossrange ≈ range · σ_bearing). Model `R` in polar (range, bearing) then transform — anisotropic, range-dependent ellipse.
- Steady-state course/speed accurate only after ~1–3 min of stable tracking.

### Identity content
ARPA track ID (use as association hint **within one radar**; not stable across radars or after target swap). No vessel identity (no MMSI/name). Possible coarse size.

### Failure modes / gotchas
- **Target swap**: two close/crossing targets exchange identities → track ID unreliable across encounters; do not treat ARPA ID as ground truth.
- Sea/rain clutter → false tracks, missed small targets/buoys.
- Range/bearing are **relative to own-ship** → require accurate own-ship position + heading at measurement time to geo-reference. Antenna offset from CCRP matters.
- ARPA output is already filtered: feeding as a measurement is a pseudo-measurement (inflate `R`, correlated noise) — revisit track-level fusion if biased (spec §13).
- Multiple radars may both report the same target in one cycle → relaxes the one-measurement-per-track assumption.

---

## 3. EO/IR camera

Electro-optical (visible) and/or infrared imaging with an onboard detector/tracker. Good for classification and bearing; range only with extra capability.

### Data provided
- **Bearing** (azimuth; possibly elevation) to detected object — always available.
- **Range / size** only if equipped: stereo, laser rangefinder, or monocular size-based estimate (low confidence). User noted a *camera with range tracker* is possible.
- Object **classification** / type (vessel, buoy, person-in-water, ...), confidence.
- Possibly a per-object **stable track ID** from the camera's own tracker (use as hint; not cross-sensor).

### Units
Bearing degrees relative to camera boresight → convert to true using own-ship heading + camera mount orientation + (if PTZ) pan/tilt. Range meters when available.

### Update rate
High when active (video framerate, e.g. 10–30 Hz) but detections may be throttled (e.g. 1–10 Hz). Often the highest-rate sensor.

### Error / covariance characteristics
- Bearing: typically sub-degree to a few degrees depending on optics/calibration; small angular `R` but **range unobservable** from bearing alone → bearing-only measurement (large/unbounded range uncertainty until fused with another sensor). This is the case where a particle/UKF may help (spec §11.2).
- Range (if present): stereo accuracy degrades ~quadratically with distance; monocular size-based estimate is biased and low-confidence — large `R`.
- Classification confidence feeds attribute fusion, not kinematics.

### Identity content
No MMSI. Provides type/classification and size cues for attribute fusion. Camera track ID is a short-lived hint only.

### Failure modes / gotchas
- Fails in fog/low light (EO) — IR mitigates but lower resolution; sun glare, spray, horizon clutter.
- Bearing depends critically on accurate mount calibration + own-ship attitude (roll/pitch) and PTZ angles at capture time → needs precise time sync with own-ship nav.
- Bearing-only updates need care in association/estimation (don't let an unbounded range collapse a track).

---

## 4. Own-ship navigation (GPS/IMU/compass)

Not a target sensor — the reference that converts relative (range/bearing) measurements into the common geo/ENU frame. Errors here propagate into *every* relative measurement.

### Data provided
- **Position** (GNSS, WGS-84), **SOG/COG**, **true heading** (gyro/satellite compass), **attitude** (roll/pitch/yaw from IMU/AHRS), heave (optional), UTC time.

### Units
Position degrees; speed knots/m·s; heading/attitude degrees.

### Update rate
GNSS ~1–10 Hz; IMU/attitude 10–100+ Hz; compass ~10–50 Hz. Generally higher than target sensors.

### Error / covariance characteristics
- GNSS position ~1–10 m (better with DGNSS/RTK); heading accuracy depends on source (satellite compass ~0.1–0.5°, gyro drift over time).
- Attitude critical for camera/long-range radar bearing geo-referencing; small heading error → large cross-range error at distance.

### Failure modes / gotchas
- GNSS outage/multipath in harbors; dropouts → must extrapolate own-ship for a bounded time and flag affected target tracks (spec §8).
- Time sync between own-ship nav and other sensors is essential — apply per-sensor clock offsets.
- Lever-arm offsets (antenna/IMU/radar/camera positions relative to CCRP) should be compensated.

---

## 5. Extension sensors (not in initial scope; design accommodates)

| Sensor | Provides | Rate | Notes / when to add |
|--------|----------|------|---------------------|
| **LiDAR** | Range + bearing (+ elevation), shape/size, very accurate at short range | 5–20 Hz | Harbor / close-range, small-object detection. Poor in rain/fog; limited range. New adapter → `Measurement` (accurate, small `R`, short range). |
| **Additional radars** | As §2, different band/location | per scan | Multi-radar fusion; relaxes one-measurement-per-track assumption; needs cross-radar association. |
| **SAR / satellite** | Wide-area detections, position | minutes–hours | Shore/wide-area surveillance; high latency → stresses time-buffer; mostly non-real-time. |
| **RF detection (non-AIS)** | Bearing / emitter ID | varies | Detect non-cooperative emitters; bearing-only like EO/IR. |
| **Acoustic / sonar** | Bearing/range (subsurface) | varies | Different target class (subsurface); likely separate track type. |

---

## 6. Cross-cutting modeling notes

- **Polar vs Cartesian noise**: radar/LiDAR/camera errors are naturally in (range, bearing). Build `R` in polar, then either use a nonlinear measurement model (EKF Jacobian) or convert to Cartesian with the proper transformed (anisotropic, range-dependent) covariance. Do **not** assume isotropic Cartesian noise for these.
- **Bearing-only sensors** (EO/IR, RF) make range unobservable from a single sensor → the estimator/association must tolerate large/undefined range uncertainty until cross-sensor fusion constrains it.
- **Relative measurements** (radar/camera/LiDAR) require own-ship pose at the *measurement timestamp* — interpolate own-ship nav to that time; budget own-ship error into `R`.
- **Identity hints** (MMSI, ARPA/camera track IDs) accelerate association but are not authoritative: validate against kinematics before locking; never use as the fusion primary key.
- **Rates differ by ~3 orders of magnitude** (camera ~10 Hz vs AIS static ~6 min) → the time-ordered buffer and per-track predict-to-measurement-time design must handle wildly asynchronous inputs.

---

## Sources
- [ITU-R M.1371 / AIS message types — USCG NavCen](https://www.navcen.uscg.gov/types-of-ais)
- [Class A Position Report (Msg 1/2/3) — NavCen](https://www.navcen.uscg.gov/ais-class-a-reports)
- [Class B Position Report (Msg 18) — NavCen](https://www.navcen.uscg.gov/ais-class-b-reports)
- [IALA Guideline 1082 — Overview of AIS](https://www.navcen.uscg.gov/sites/default/files/pdf/IALA_Guideline_1082_An_Overview_of_AIS.pdf)
- [IMO Res. A.823(19) — ARPA performance standards](https://imorules.com/IMORES_A823.19.html)
- [Automatic Identification System — Wikipedia](https://en.wikipedia.org/wiki/Automatic_identification_system)
