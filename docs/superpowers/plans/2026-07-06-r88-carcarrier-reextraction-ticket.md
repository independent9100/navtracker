# Implementer prompt — R8.8 code half: car_carrier_near re-extraction + fail-loud extractor guard

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: R8.8 in `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md`
(L561–597). Scope here is the CODE HALF ONLY — steps 1, 2, 3, 5 of that
section. Step 4 (the occlusion video/label pass) is a separate user+analyst
session that this work unblocks; do NOT attempt any video labelling.

---

You are working in the navtracker repo (C++17 core, but this ticket is almost
entirely Python fixture tooling + docs; read `CLAUDE.md` first). Mission:
repair the broken `car_carrier_near` philos extraction, make the extractor
fail loudly on the failure class that produced it, and redo the one analysis
that consumed the corrupted data.

## The data-integrity bug you are fixing (established 2026-07-03)

`tests/fixtures/philos/out/car_carrier_near/` (from the 2020-10-22 bag) has
`heading_deg = 0.000` CONSTANT and only 26 own-ship rows over 120 s. Cause:
`extract_section.py`'s topic lists don't match the 2020 bag layout, so it
fell back to the sparse `/gnss` topic and emitted a heading placeholder.
Consequence: every world-projected radar return from this clip is rotated
about own-ship by the true heading (GPS track suggests course ≈ 300°, so the
rotation is large). The other six clips were verified fine (dense rows, real
headings) — but only by a one-off manual sweep; nothing guards this today.

The bag itself carries everything needed: `/filter/positionlla` +
`/sensor/gps/fix` (~60–70 Hz position) and `/filter/quaternion` /
`/imu/data` (~60 Hz attitude → yaw).

## Ground rules

- All of `tests/fixtures/` is LOCAL-ONLY (gitignored, multi-GB). Committed
  artifacts are: the extractor script changes, tests if any, docs, eval-log
  entry (with emitted-file checksums as the tracked drift-guard — the
  camera-bearing precedent).
- The bags are on this machine — `tests/fixtures/philos/README.md` and
  `batch_extract.sh` document layout and the extraction venv.
- This clip is IN-SAMPLE (the held-out set was sailboats_busy /
  almost_cross / ais_ferry_far — duty discharged, but car_carrier_near was
  never held out, so no pre-registration ceremony applies).
- Measurement/tooling only: no tracker code, no defaults.

## Tasks (in order)

### 1. Extractor fix + fail-loud guard (`tests/fixtures/philos/extract_section.py`)

- Position fallbacks: `/filter/positionlla`, `/sensor/gps/fix` — prefer the
  densest NavSatFix-compatible topic present in the bag.
- Heading fallback: yaw from `/filter/quaternion` (or `/imu/data`) when none
  of the named heading topics exist. Mind conventions: quaternion yaw →
  degrees true, same convention as the existing heading path (verify against
  a clip with a known-good heading topic: re-extract one of the six good
  clips with the fallback FORCED and compare headings — they should agree to
  a few degrees; document the residual).
- **Fail-loud guard (the actual point):** after extraction, assert
  (a) >1 distinct heading value, (b) own-ship row rate ≥ 1 Hz over the clip
  span. Violation = hard error naming the clip and the offending series —
  never emit placeholder rows. This is the trap that silently produced a
  rotated clip; the guard makes the failure class impossible to repeat.

### 2. Re-extract `car_carrier_near`

Expect ~7k own-ship rows with real headings (course ≈ 300°). Sanity checks
to record: (a) heading series is dynamic and consistent with the GPS course;
(b) a quick before/after projection check — world-projected radar returns
should now sit sensibly relative to the shore/chart instead of rotated
(one plot or a numbers-based check both fine; this was the visible symptom).
Run the guard over ALL SEVEN clips (re-extraction not needed for the good
six — running the guard against their existing outputs is enough) so the
integrity sweep is now mechanical, not manual.

### 3. Redo the R4 chart-analysis contribution

The corrupted clip's cells contaminated the R4 chart-corroboration analysis.
Re-run `tests/fixtures/philos/charts/philos_chart_coverage.py` on the
re-extracted clip and re-check the in-coverage UNKNOWN cell at
(42.3583, −71.0464) — previously supported by this clip + ais_ferry_far
(only the latter currently valid). Report whether the UNKNOWN survives,
moves, or dissolves with corrected geometry; update the R4 numbers wherever
they are quoted if they shift (say where you looked either way).

### 4. Eval-log entry (step 5 of the R8.8 section)

Dated entry: the integrity finding (constant-heading placeholder → rotated
clip; extractor fallbacks + guard now in place), the re-extraction result +
checksums of the new `car_carrier_near` outputs, the chart re-check outcome,
and the standing AIS note: the 2020/2021 philos campaigns carry NO AIS at
all (receiver absent) — the AIS-veto's real-data validation must come from
HAXR, not philos.

## Acceptance

1. Extractor fallbacks + fail-loud guard shipped; guard demonstrated to
   PASS on all seven clips post-fix and to FAIL (loudly, clearly) on the
   old broken output (keep a copy aside to prove it — then delete).
2. `car_carrier_near` re-extracted: ~7k rows, dynamic headings, projection
   sanity check recorded.
3. Chart re-check done, outcome + any updated numbers reported.
4. Eval-log entry with checksums. Suite green (should be untouched — this
   is fixture tooling; say so explicitly).
5. Handoff states: the clip is now READY for the occlusion labelling
   session (step 4 of R8.8, user + analyst), and names anything the session
   should know (e.g., exact clip duration, when the carrier is closest,
   any extraction quirks worth flagging to the operator).
6. Budget ~1 day. If the 2020 bag layout fights the fallbacks for more than
   ~2 h (missing topics, unexpected frames), stop and report what the bag
   actually contains instead of forcing a guess.
