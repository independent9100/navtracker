# Implementer prompt — camera-chain vs operator: YOLO pipeline graded against the R8.8 human labels

Status: ready to hand off. Paste everything below the line. Origin: named
follow-up from the 2026-07-06 R8.8 occlusion labelling pass — the camera→
bearing pipeline was deliberately NOT used for labelling (circularity: those
labels test camera corroboration). Now run the machine over the same clip and
grade it against the human labels. Direction is one-way: THE HUMAN LABELS
GRADE THE MACHINE, never the reverse — no label is edited because YOLO
disagrees.

---

You are working in the navtracker repo (read `CLAUDE.md` — parallel-work +
fixture-trap notes). Mostly Python against local data; use a worktree for
anything committed (`git worktree add ../navtracker-camchain -b camera-chain-comparison`).
Budget ~half day to 1 day.

## Step 1 — calibration check (gate for everything else)

The pinned pipeline (`tests/fixtures/philos/extract_camera_bearings.py` +
`camera_bearing_calibration.json`) was calibrated on the 2022 rig
(ais_ferry_near, residual 0.45°/1.32°) and transferred to 2021 (sunset). The
car_carrier clip is the 2020 rig — and its bag ships its own intrinsics
(`vids/philos_2020_10_22_car_carrier_near/metadata/camera_cal_files.tar.gz`).
Check whether the 2020 intrinsics match the calibrated assumption; if they
differ, use the bag's own intrinsics and document the swap. If neither is
usable (the close_approach/prodromos refusal precedent), STOP and report —
do not emit bearings from an uncalibrated camera.

## Step 2 — run the pinned chain on car_carrier_near

Center + left cameras (the labelled ones), pinned YOLO weights + environment
(the checksummed-drift-guard discipline from the camera-bearing eval-log
entry applies: record weights + env + output checksums).

## Step 3 — the comparison (the deliverable)

Against `tests/fixtures/philos/labels/car_carrier_near_labels.csv`, per label
row: did the chain detect an object at the matching bearing during the row's
window? Report per row: hit / partial / miss, with bearing residual where
hit. The rows that matter most:
- `carrier_gl_a/b` — a hull filling the frame: does YOLO handle it or
  fragment it? (Detector behavior at extreme scale.)
- `unknown_w860` + `yacht_moored_2` — small targets at ~860 m: detection
  range limit in practice.
- THE SHADOW INTERVAL (t 50–85): the carrier physically blocks the camera's
  view of the yachts too. The chain should report NOTHING there for them —
  confirm it does, and state the lesson explicitly: the machine's silence
  during occlusion is "not observed", never "observed empty" (the operator's
  shakiness caveat, now measured for the machine too).
- `sail_pair` / `sail_third` / `sail_close_end` — small sails: hit rate vs
  the operator's eyes.

## Step 4 — annotated frames (yes — but LOCAL only)

Render ~8–12 frames with YOLO boxes + the label-row bearings overlaid
(the key moments: t≈0, 30, 45, 60, 80, 95, 110, 118). These make the
comparison reviewable by the operator at a glance. LICENSE BOUNDARY: philos
imagery is research-scoped — annotated frames stay LOCAL
(`tests/fixtures/philos/out/car_carrier_near/chain_comparison/`, gitignored),
listed with checksums in the eval-log entry. Committed deliverables are the
comparison TABLE + eval-log entry only (`docs/baselines/2026-07-08_camera_chain_vs_operator.md`).

## Acceptance

1. Calibration decision documented (which intrinsics, why, residual if
   measurable against a known geometry).
2. Per-label-row hit/partial/miss table + bearing residuals; the shadow-
   interval silence check; committed baseline doc + dated eval-log entry
   with all checksums (weights, outputs, annotated frames).
3. Annotated frames rendered locally; NOT committed.
4. No label edited; no chain parameter tuned to this clip — misses are
   FINDINGS about the chain's limits (that's the value), listed for the
   arbiter.
5. Stop-and-report: Step-1 calibration unusable; or the chain's output is
   so degraded on the 2020 rig that per-row comparison is meaningless
   (report the degradation instead).
