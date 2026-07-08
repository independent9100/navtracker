# Camera chain vs operator — pinned YOLO pipeline graded against the R8.8 human labels (car_carrier_near)

Date: 2026-07-08. **Measurement / comparison only** — no default touched, no
algorithm changed, no chain parameter tuned to this clip, **no label edited**.
Ticket: `docs/superpowers/plans/2026-07-08-camera-chain-comparison-ticket.md`.

Direction (hard rule): the **human labels grade the machine, never the reverse**.
Misses and disagreements are FINDINGS about the chain (and, here, about the
labels' internal consistency) — listed for the arbiter, not acted on.

License boundary: philos imagery is research-scoped. The annotated frames stay
**local** (`tests/fixtures/philos/out/car_carrier_near/chain_comparison/frames/`,
gitignored); only this doc + the eval-log entry are committed. Checksums below.

## Clip facts (read first)

| | `car_carrier_near` |
|---|---|
| Bag | `philos_2020_10_22_car_carrier_near` (**2020 rig**), Boston Inner Harbor |
| Duration | 120.99 s (unix 1603390014.90 → 1603390135.89) |
| Cameras graded | center (19351369) + left (19066296) — the labelled ones |
| Frames | 1451 / camera @ ~12 Hz |
| AIS | **none** (2020 campaigns carry no AIS receiver) |
| Radar | `radar_plots.csv`, 5117 returns @ ~50 Hz, body-frame azimuth |
| Ownship | 8739 rows @ 72 Hz, heading quaternion-derived, integrity PASS |
| Labels | `labels/car_carrier_near_labels.csv` (R8.8 occlusion pass, 9 rows) |

## Step 1 — calibration decision (the gate) — **PASS**

**Intrinsics: use the 2020 bag's own cal files** (`camera_cal_files.tar.gz`), not
the committed 2022 calibration. The 2020 files carry the **same camera serials**
(center 19351369, left 19066296, right 19110492) as the 2022 rig — same physical
cameras, recalibrated 2020-08-21 vs 2022-04-19 — so swapping to the rig-matched
intrinsics is the principled choice. Numeric drift 2022→2020:

| Cam | fx 2022 → 2020 | cx 2022 → 2020 | bearing impact |
|---|---|---|---|
| center | 1500.63 → 1459.22 (−2.8%) | 662.85 → 661.25 | ~0.3° mid-frame, ~0.6° at FOV edge |
| left | 1462.44 → 1440.40 (−1.5%) | 613.54 → 605.37 | similar |

(The 2020 left & right YAMLs are byte-identical to each other — the dataset gave
both flank cameras one cal. Only center + left are graded, so this is moot here.)

**Boresight: transfer the committed 2022 center `yaw_offset` (2.29217°),
extrinsic-propagated to left (−45.264° → −42.971°).** 2020 has **no AIS**, so an
independent RANSAC refit (the `ais_ferry_near` method) is impossible. Same serials
⇒ same mount ⇒ the boresight should transfer; the residual risk is the 2020
heading being quaternion-derived (a documented ~2° convention residual) where 2022
used a direct heading topic.

**Independent validation against radar (non-circular).** The C++ replay adapter
recovers a radar world bearing as `heading + azimuth_deg`, validated on
`ais_ferry_near` to match AIS to ~1–6°; i.e. radar `azimuth_deg` is **hull-relative**
(0 = bow, CW), directly comparable to the camera's `bearing_rel`. For every camera
detection we took the nearest radar return in angle within the camera FOV (±0.25 s,
±12° association, **no offset fitted**) and looked at the signed residual
`camera_bearing − radar_azimuth`:

| Camera | n | signed median | signed mean | abs median | abs p90 |
|---|---|---|---|---|---|
| center | 948 | **+0.31°** | −0.14° | 3.53° | 9.66° |
| left | 555 | **+0.08°** | −0.39° | 4.98° | 9.87° |

The signed medians are ~**0°** for both cameras: the transferred 2022 boresight has
**no measurable systematic bias** on the 2020 rig. The ~3.5–5° absolute spread is
camera↔radar association + parallax (radar range-centroid vs camera bbox-centre on
extended targets) + timing — not boresight error. **Gate passes; the camera is
calibrated well enough to grade.** (Radar ≠ camera ≠ labels, so this validation is
free of the labelling circularity.)

## Step 3 — per-label grading

"Chain verdict" = did a YOLO boat detection land within **8°** of the label's
stated position bearing during its window, in the camera whose FOV contains that
bearing? `cov` = matched frames / in-FOV opportunity frames. `ambig` = fraction of
matched frames where **another** label also sits within 8° at the same time
(co-bearing → attribution is not unique). `obj size` = median matched-box area as
% of the 1280×1024 frame (a size sanity check on *what* was matched).

| region_id | label | window s | cam(s) | verdict | cov | resid med/p90° | ambig | obj size | reading |
|---|---|---|---|---|---|---|---|---|---|
| carrier_gl_a | KEEP_VESSEL | 40–80 | center | hit* | 0.89 | 4.12 / 7.4 | 0.00 | 0.14% | **FALSE ATTRIBUTION** — matched a small (0.14%) distant sailboat at +14° stbd, **not the carrier**. Label coord is wrong (Finding 1). |
| carrier_gl_b | KEEP_VESSEL | 80–110 | center | hit* | 0.99 | 4.61 / 7.65 | 0.09 | 0.37% | same — small starboard object, not the carrier. |
| unknown_w860 | KEEP_VESSEL | whole | center,left | partial* | 0.36 | 2.55 / 6.84 | **1.00** | 3.08% | every match is the **carrier** crossing the yacht bearing (100% co-bearing, box size = carrier). Independent 860 m detection ≈4% (center). |
| yacht_moored_2 | KEEP_VESSEL | whole | center,left | partial* | 0.32 | 2.28 / 6.47 | **1.00** | 3.07% | co-located with `unknown_w860`; same carrier co-bearing story. |
| yacht_exit_port | KEEP_VESSEL | 0–12 | left | **hit** | 0.94 | 3.93 / 6.98 | 0.00 | 0.6% | **clean, unambiguous** — small motor yacht exiting to port, left cam. |
| portq_object | SUPPRESS_STRUCTURE | 0–60 | — | n/a | 0.00 | — | — | — | never in any camera FOV — **consistent** with the label ("out of camera FOV; radar-only"). Camera correctly silent. |
| sail_pair | KEEP_VESSEL | 6–121 | center,left | hit | 0.59 | 4.35 / 7.32 | 0.37 | 1.35% | detected; ~⅓ of frames co-bearing with another label. |
| sail_third | KEEP_VESSEL | 40–110 | center,left | hit | 0.55 | 5.24 / 7.6 | 0.48 | 0.14% | small/distant; ~half the matches co-bearing ambiguous. |
| sail_close_end | KEEP_VESSEL | 107–121 | center | hit | 0.92 | 4.91 / 7.62 | 0.23 | 0.22% | detected late-clip; mostly unambiguous. |

`*` = verdict is confounded — read the "reading" column and Findings 1–2. The raw
verdict is what the ticket's literal recipe produces; the reading is what the data
actually shows.

### The carrier, correctly (radar + camera consistent)

Grading `carrier_gl_a/b` by their stated lat/lon is meaningless (Finding 1). The
**actual** carrier is the dominant large box in the **left** camera on the **port**
side — matching the operator's text ("enters left cam ~20 s; crossing port side")
and the radar port track:

| t s | 28 | 38 | 48 | 60 | 72 | 84 | 96 |
|---|---|---|---|---|---|---|---|
| bearing (hull-rel) | −22° | −27° | −32° | −41° | −52° | −57° | −62° |
| box width px | 295 | 364 | 397 | 538 | 578 | 504 | 227 |
| frame area % | 2.6 | 3.7 | 3.7 | 5.1 | 10.7 | 7.4 | 4.3 |

Over t 20–110: **32 frames** with a big (>2% area) box; **fragmentation (≥2 big
boxes) in only 4 frames (12.5%)**; **max 11.7% frame area**. So:

- **YOLO handles the carrier well at this scale** — a single confident box through
  the crossing, minimal fragmentation. (t60 left frame: one clean box on the hull,
  "NYK LINE" legible.)
- **It does NOT "fill the frame."** The ticket's premise ("a hull filling the
  frame") doesn't hold — the carrier crosses at 143–269 m and subtends ≈½ the frame
  width, ~10% area at closest. Extreme-scale fragmentation was **not** observed.
- It leaves **all** camera FOV when it swings astern (|bearing| > 69°): for a
  stretch around t 30–70 the closest large radar returns are abeam/aft where no
  camera looks — a **geometry** blind spot, not a detector miss.

## The shadow interval (t 50–85) — silence check — **confirmed**

The moored yachts (`unknown_w860`, ~860 m) sit at hull-relative −29°…−38° here
(left FOV). During the shadow the carrier crosses their bearing:

| t s | 50 | 55 | 60 | 65 | 70 | 75 | 80 | 85 |
|---|---|---|---|---|---|---|---|---|
| yacht bearing | −29° | −31° | −33° | −30° | −32° | −31° | −31° | −38° |
| left det @ bearing | carrier (W415, 3.7%) | carrier (W454, 4.9%) | carrier (W538, 5.1%) | none | none | none | none | none |

t 50–60: the only detection at the yacht bearing is the **large carrier box** (the
occluder). t 65–85: **nothing** (carrier has moved past to −41°+; the yacht is too
small/occluded to fire). The moored yacht is **never independently detected** in
the shadow. **Lesson, now measured for the machine as it was caveated for the
operator: the chain's silence during occlusion is "not observed", never "observed
empty".** A downstream consumer must not read the camera's non-detection here as
evidence the yacht is gone (it is radar-shadowed and camera-occluded by the same
carrier).

## 860 m detection-range limit

`unknown_w860` at ~860 m is in the center FOV only early (t 0–20, 264 in-FOV
frames) and is detected in ~**4%** of them. The two moored white yachts are
visible to the operator's eye the whole clip but are at/beyond YOLO-nano's reliable
range: independent (non-carrier) detection is sparse. Where the chain *does* fire
at a small distant target the **bearing is tight** (`unknown_w860` matched-residual
median 2.55°, the best of any row) — the range limit is a **recall** limit, not a
bearing-accuracy limit.

## Findings for the arbiter

1. **`carrier_gl_a/b` label coordinates are internally inconsistent (upstream R8.8
   labelling bug).** Both are tagged source `nearestbig{60,95}` (the radar
   nearest-big return at t=60/95), but their lat/lon (42.3598, −71.0387) back-project
   to +8…+16° **starboard**, whereas the actual radar nearest-big <300 m at t=60 is
   at −137° **port** — implied position (42.3561, −71.0355), ~400 m away. The label
   **text** ("crossing port side / left cam") is correct and matches radar + camera;
   the label **coordinates** are not. Grading by those coordinates matches a small
   starboard sailboat instead of the carrier. **Per the hard rule the label is NOT
   edited** — flagged here for the arbiter to correct upstream.
2. **Bearing-only + point-position labels cannot attribute co-bearing detections.**
   The port/port-bow sector is crowded (carrier, 2+ sails, 2 moored yachts, exiting
   yacht). `unknown_w860`/`yacht_moored_2` score "partial" only because the carrier
   crosses their bearing (ambig = 1.00). A per-row "hit" means "the chain saw
   *something* at that bearing then", which is meaningful for well-separated rows
   (`yacht_exit_port`, `sail_close_end`) and confounded for the crowded sector. This
   is a limit of the grading method, not of the chain.
3. **Carrier: single clean box, no extreme-scale fragmentation, ~12% max area** (not
   frame-filling). YOLO-nano handles the near car carrier fine.
4. **860 m recall limit**: small distant moored yachts detected ~4% of in-FOV frames
   (bearing tight when detected). A detection-range finding, not a bearing finding.
5. **Occlusion silence is "not observed"** — confirmed in the shadow interval.
6. **`portq_object` (SUPPRESS_STRUCTURE) is correctly never seen by the cameras** —
   consistent with its "out of FOV, radar-only" label.

No chain parameter was tuned; the misses/confounds above are the value.

## Reproduce / checksums

Pinned detector (unchanged from the 2026-07-03 camera-bearing eval-log pin):
`yolov8n.pt` sha256 `f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`,
ultralytics 8.4.87, torch 2.12.1+cpu, opencv 5.0.0, numpy 2.4.4, python 3.12.6,
CPU, `conf=0.25 imgsz=1280 class=8(boat)`, augment off. The clip is registered with
the pinned `extract_camera_bearings.py` at **runtime** (CLIP_MAP injection) so the
pinned script stays byte-identical; center + left only.

| artifact (all under `tests/fixtures/philos/`, gitignored/local) | sha256 |
|---|---|
| `out/car_carrier_near/_camera_detections.csv` (center 3086 + left 1591) | `9a24b313a876cccaa1f144163d4a0292709760bf38b1abbbf4749e7cc5222908` |
| `out/car_carrier_near/chain_comparison/final_results.json` | `17674633e4ca6667b08f09e5a287f4d8b5eb4244f1a8b9201b8d054311888e14` |
| `out/car_carrier_near/chain_comparison/comparison_results.json` | `70904afbf23d1c523455a89b515a5f4aac0e69151ea0a090544020c3016f72e4` |
| driver `run_chain_car_carrier.py` | `26b51c36888f71b78fc38c0b9a07cdb2c36fe4e3dee5d6cd4a2052a138250bc2` |
| `compare_chain_car_carrier.py` | `5c2f508a8919d30332d513c3938bb4f8076b6d0b1df6efd4aa72c04daeea918f` |
| `final_compare.py` | `46f045dc284de56e6822976c6316b6d6897445f94f0b26051fa9b30f2b5ca677` |
| `render_frames_car_carrier.py` | `3273e72e05e4c026d33e4a798cdcf2399bd5a8a6cf9c3792b503cfccd364657d` |
| `diagnose_chain_car_carrier.py` | `8e16da95ccd54c5f9885456bfcf841ef2a1be149f761096b15b6517ede2653ff` |

Annotated frames (LOCAL only, NOT committed) —
`out/car_carrier_near/chain_comparison/frames/`:

| frame | sha256 |
|---|---|
| t000_center.jpg | `691814563879c5d7bf5e3844b4a957068352d1e18e4cfe405522f50866308650` |
| t000_left.jpg | `0df035cf780c8f5560b21b9a19046729375722b0a2d6147016caae6e5b2f2612` |
| t030_left.jpg | `ae8bf049bcced65c124e462c4a79f29ad809cb57534cb6e9a2584e0ebbfec4bd` |
| t045_left.jpg | `33a03f5c01d066187cfd8d1da22c6f976ab589030c2cce8c060cde03da96ccd6` |
| t060_center.jpg | `5076163fe50029cd0f7768297250b3c1c1806b00892a60c473428c744f20910c` |
| t060_left.jpg | `4fc96f847299b5c7686e8b160a099e3c2d665eaccdd3b6062768a623adb05b57` |
| t080_left.jpg | `6f0bca6ecff832a66ee0a73f43cb4466bda6317ccd300e41009dd01008171597` |
| t095_center.jpg | `9aedb99e94b68c0bccfe432aa6f8a9ea5dc609723369704f2e4fc96db40d9a88` |
| t110_center.jpg | `9280d8706d495d3d4849b7769fd5dc152a513b5f8eb36818e1ffa134a3f05e05` |
| t110_left.jpg | `f2d6c61179d8945179dfe668c0024aaa76ce9516efdbbc37a608b8af4d604698` |
| t118_center.jpg | `17c9a99c7ca773c40326efe1a3756711fc9fb8db8454ff69401351977e513dab` |
