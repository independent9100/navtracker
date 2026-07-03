# Implementer prompt — camera boat detection → bearing-only fixtures

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: 2026-07-03 discussion (opens the camera-corroboration axis,
ADR 0001 A3 / D5 context; serves the 1b-ii KEEP-guard later — this ticket is
fixture generation + wiring proof ONLY).

---

You are working in the navtracker repo (C++17 maritime multi-sensor fusion
tracker; read `CLAUDE.md` first — hexagonal architecture, TDD, documentation
standards). Your task: build an **offline pipeline that detects boats in the
philos camera frames and turns them into bearing-only measurement fixtures**,
plus the minimal C++ wiring proof that the tracker consumes them.

## Why (context you need, do not re-litigate)

- The philos clips we use for replay benchmarks are radar-only; the labelled
  ones carry zero AIS. Camera bearings make them genuinely multi-sensor.
- The eventual consumer is the Stage 1b-ii corroboration KEEP-guard ("a
  boat-shaped detection at the bearing of a persistent radar cell is KEEP
  evidence"). That integration is **not** in this ticket. You produce the
  fixtures and prove the bearing-only channel wires; nothing more.
- Camera **distance/range is out of scope** — decided. Bearings only.
- The tracker natively supports bearing-only: `MeasurementModel::Bearing2D`
  (one angle + 1×1 covariance in rad²), handled via nonlinear h(x)=atan2 in
  EKF/UKF. `canInitiateTrack(Bearing2D) == false` by design — a bearing can
  update/corroborate a track, never create one. Do not change that.

## Inputs on disk

- Frames: `vids/<clip_dir>/{center,left,right}_camera/<unix_time>.jpg`
  (~12 Hz, epoch-second filenames — same timebase as the fixture CSVs).
  Clip-dir → fixture-dir mapping:
  - `vids/philos_2022_11_07_ais_ferry_near` → `tests/fixtures/philos/out/ais_ferry_near`
  - `vids/philos_2021_10_28_ sunset_cruise` (note the space) → `out/sunset_cruise`
  - `vids/prodromos_2021_10_29_close_approach` → `out/close_approach`
- Per-clip fixtures: `tests/fixtures/philos/out/<clip>/ownship.csv`
  (`unix_time,lat,lon,heading_deg`), `radar_plots.csv`, and for
  ais_ferry_near also `ais.csv`.
- Existing extraction scripts to match in style: `tests/fixtures/philos/
  extract_section.py`, `extract_radar.py` (Python venv; see the fixtures
  README for the venv setup).

## Scope guards (hard rules)

1. **Clips in scope:** `ais_ferry_near` (calibration + validation),
   `sunset_cruise`, `close_approach`. **Excluded:** `car_carrier_near`
   (its ownship/heading fixture is corrupt until ticket R8.8's re-extraction
   lands — running on it would produce rotated garbage) and the three
   held-out clips `sailboats_busy`, `almost_cross`, `ais_ferry_far`
   (held-out protocol: nothing touches them until the 1b-ii detector is
   frozen).
2. **Offline only.** The detector (YOLO or similar) runs in a Python script
   that emits CSV fixtures. No new C++/Conan dependency, no runtime
   inference anywhere in `core/`, `adapters/`, or `app/`.
3. **No range.** The fixture carries bearings; never emit or estimate
   target distance from the camera.
4. **Absence asymmetry (document it where the fixture format is documented):**
   a camera detection is evidence a boat exists at that bearing; the absence
   of a detection is NEVER evidence of absence (fog, night, sunset glare,
   detector misses). Downstream consumers may use presence for KEEP, must
   never use absence for SUPPRESS.
5. **Circularity rule (document it too):** the existence labels in
   `tests/fixtures/philos/labels/` were derived from these same videos.
   Camera detections may be used to test fusion *mechanics* and as a
   corroboration channel — never as truth for accuracy claims against those
   labels (sensor and truth would share a source).
6. **Fail-loud data integrity** (lesson from R8.8): before projecting
   anything, assert the clip's ownship fixture has ≥1 Hz sampling and >1
   distinct heading value; assert frame timestamps overlap the ownship time
   range. Abort with a clear message on violation — never emit a fixture
   from suspect inputs.
7. **Determinism:** pin the detector model (exact weights file + version in
   the README and script), fix the confidence threshold in the script, no
   randomness. Committed CSVs are the interface; a re-run with the pinned
   setup must reproduce them. Never hand-edit an emitted fixture.

## The work

### 1. Detector pass (Python, offline)

New script `tests/fixtures/philos/extract_camera_bearings.py`:

- Run an off-the-shelf detector (suggest ultralytics YOLO, COCO `boat`
  class; small model is fine — CPU inference over ~1–2k frames/camera is
  acceptable) over all three RGB cameras of each in-scope clip.
- Expect weak recall in hard light (sunset_cruise is literally sunset).
  That's acceptable — see absence asymmetry. Record confidence per
  detection; do not chase recall with threshold games.
- Output per frame: camera id, unix_time (from filename), bbox (px),
  confidence.

### 2. Pixel→bearing calibration (the only real math — document it)

- Model per camera: hull-relative bearing = yaw_offset + f(pixel column).
  Start linear (narrow-FOV approximation): `bearing_rel = a + b·u`. Fit
  a, b per camera.
- Calibration source: `ais_ferry_near` has AIS. For frames where the AIS
  ferry is the plausible match (predicted bearing from AIS position +
  ownship pose falls inside the camera's rough FOV, and a detection
  exists near it), pair detection pixel column ↔ true bearing computed
  from AIS lat/lon and ownship position, converted to hull-relative using
  `heading_deg`. Robust-fit (e.g. RANSAC or iterative reweighting) — some
  pairings will be wrong boats.
- Hold back ~30% of correspondences; report median and p90 absolute
  residual on the held-back set. Target: median ≤ 2°, p90 ≤ 4°. If a
  linear model can't reach that, try adding a mild quadratic term before
  concluding the cameras need real intrinsics — and if it still fails,
  STOP and report rather than shipping a bad calibration.
- Commit the calibration as a small JSON next to the script
  (per-camera a, b, residual stats, fit date, correspondence count).
  The 2021/2020 clips were shot from different vessels (philos vs
  prodromos) — a center-camera calibration from philos 2022 does NOT
  automatically transfer. Emit fixtures for a clip only when either
  (a) the same vessel+camera calibration applies, or (b) you can
  bootstrap a coarse yaw_offset from a video-verified correspondence
  (e.g. the close_approach collider, the sunset_cruise ferry — see
  `tests/fixtures/philos/labels/` and the R8.6/R8.7 sections of
  `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md`);
  document which path each clip used and its expected σ. If neither
  works for a clip, emit nothing for it and say so — a wrong bearing
  fixture is worse than none.

### 3. Fixture emission

`tests/fixtures/philos/out/<clip>/camera_bearings.csv` with columns:

```
unix_time,camera,bearing_rel_deg,sigma_deg,confidence,u_px,v_px,w_px,h_px,frame
```

- `bearing_rel_deg` is **hull-relative** (calibrated), NOT true bearing.
  Rationale (same principle as the TTM-over-TLL rule in
  `docs/sensors/sensor-reference.md` §2): keep the raw observation; let
  the C++ loader compose own-ship heading at load time so heading-bias
  machinery can apply later.
- `sigma_deg`: per-detection 1-σ = calibration residual σ ⊕ a bbox-width
  term (wide/near boats have sloppier centers). Document the formula.

### 4. C++ wiring proof (TDD)

- Small loader in `adapters/replay/` (match `PlotCsvReplayAdapter`
  conventions): `camera_bearings.csv` + `OwnShipProvider` →
  `Bearing2D` Measurements (value = true bearing in rad from
  hull-relative + heading at `poseAtOrBefore(t)`; covariance = σ² in
  rad²; `sensor_position_enu` = ownship at t; sensor kind EO/IR;
  source_id = camera id). Drop rows with no available pose (count and
  log them).
- Unit test: parsing, heading composition, wrap handling, the drop rule.
- One replay smoke test: feed `ais_ferry_near` radar plots + camera
  bearings through the tracker; assert (mechanics only) that at least one
  track receives a Bearing2D update and that no track is *initiated* by a
  camera measurement. No accuracy assertions (circularity rule).

### 5. Docs + bookkeeping

- Extend `tests/fixtures/philos/README.md`: pipeline, model pin, venv,
  calibration method with the four-part standard (math / assumptions /
  rationale / ways to improve), the absence-asymmetry and circularity
  rules, per-clip calibration provenance.
- One paragraph in `docs/sensors/sensor-reference.md` §3 (EO/IR): the
  philos camera-bearing fixture exists, its σ, its role
  (corroboration-only).
- Eval-log entry: detection counts per clip/camera, calibration residuals,
  dropped-row counts, runtime.
- `docs/learning/` is NOT required by this ticket (offline test tooling;
  the learning chapter comes with the camera-corroboration channel in
  core). Note that explicitly in your handoff summary so it isn't seen as
  an omission.

## Acceptance

1. Calibration residuals on held-back AIS correspondences: median ≤ 2°,
   p90 ≤ 4° (report the numbers; STOP and report if unreachable).
2. `camera_bearings.csv` committed for every in-scope clip that passed the
   calibration-provenance rule; each with the data-integrity guards green.
3. C++ loader + unit tests + smoke test green; full suite green; no new
   Conan/C++ dependency; determinism test stays green.
4. Docs per §5. Handoff summary states: per-clip detection counts, which
   calibration path each clip used, residual stats, and any clip you
   refused to emit (with reason).

Work TDD where there is C++ (watch each test fail first). For the Python
side, include a `--self-test` mode or small pytest covering the bearing
math (pixel→bearing→true round-trip with a synthetic calibration).
