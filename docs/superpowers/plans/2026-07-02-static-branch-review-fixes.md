# Static-Obstacle Branch — Review Findings & Fix Tickets (2026-07-02)

Source: deep review of the static-object approach (ADR 0002 line): decision
docs, code, bench evidence, Herrmann 2025, and a 2020–2026 literature sweep.
Overall verdict: **the approach is validated — keep following it.** These
tickets close the gaps the review found. Each is self-contained: an
implementer can pick one up cold.

All items are **Cl-3** (north-star: `docs/algorithms/comparison-baselines.md`).

**Prerequisite — RESOLVED 2026-07-02 (commits 3ee491f + 3aa9c58 on master):**
the `addAnchoredBoats` truth-sort bug (unsorted `Scenario.truth` →
`groupTruth` fragmentation) is fixed: additive builders now `sortTruthByTime`
(exact `Timestamp` comparator), contract tests added
(`HarborCompleteTruth.TruthIsTimeSortedIntoFortyCompleteGroups`), a latent
same-family instance in the HAXR replay path fixed, and the harbor baseline
re-measured. **Corrected "before" for R5/R6: card_err_mean +11.64,
gospa_mean 50.63, gospa_false 2362, gospa_missed 34, lifetime_ratio 0.974**
(the old +13.32 / 0.92 numbers were invalid; see eval-log 2026-07-02
correction). R5/R6 are unblocked.

Measurement discipline for every ticket that touches behaviour: A/B in the
bench, numbers into `docs/algorithms/evaluation-log.md`, promotion per the
north-star process. No ticket ships on assertion alone.

---

## R1 — Birth-floor blackout: soft suppression can compose into a hard no-birth zone

**Problem.** The keep-clear buffer is soft *only in isolation*. The birth
scale `(1−c_land)·(1−c_static)` (`core/pmbm/PmbmTracker.hpp:815-827`)
multiplies into the adaptive-birth intensity (`core/pmbm/PmbmTracker.cpp:497-502`),
and the resulting `r_new` is then compared against
`min_new_bernoulli_existence` (`core/pmbm/PmbmTracker.cpp:963-966`; 0.05 in
the shipped `imm_cv_ct_pmbm_land` / `_static` configs). Worked example: near
shore (c_land = 0.5) inside a charted obstacle's keep-clear buffer
(c_static = 0.9) → scale 0.05 → r_new ≈ 0.048 < 0.05 → **the Bernoulli is
never created, every scan, forever**. Under `birth_existence_target = 0.1`
configs it is worse (r_new ≈ 0.011). This silently re-creates the hard
no-birth zone that ADR 0002 forbids ("the keep-clear buffer is soft-only —
a real vessel passing close still births") for exactly the anchored-vessel
case the ADR protects. No test exercises land + obstacle both active; no
test exercises a *stationary* target inside a buffer (the integration-test
vessel transits at 10 m/s).

**Suggested fix.** Make the existence floor a *pre-suppression* check
(ADR 0001's parked "A2" idea): a birth candidate is dropped by the floor only
if its **unsuppressed** existence would already be below
`min_new_bernoulli_existence`; if it passes, the Bernoulli is created with
the suppressed r_new even when that lands below the floor. Rationale: the
floor exists to prune numerically irrelevant births; geography is the ramp's
job, and the ramp must stay soft. Hard-drop (scale < 0, i.e. either prior
above its hard gate) remains the only absolute kill. Tiny suppressed
Bernoullis will mostly get pruned by the existing pruning thresholds — but a
*persistent* anchored boat accumulates evidence across scans and can confirm,
which is the desired behaviour.

**Acceptance.**
- New test: land + obstacle both active, candidate in the overlap band →
  Bernoulli exists with r_new = scale × r_unsuppressed (no floor kill).
- New end-to-end test: zero-velocity target inside a keep-clear buffer (and
  inside a soft shore band) births and reaches Confirmed within a bounded
  number of scans.
- Existing null-model / flag-off bit-identical tests stay green.
- A/B on philos + shore_clutter_* + dense_clutter: expect ≈ neutral (the
  suppression *magnitude* is unchanged; only the kill-vs-tiny-Bernoulli
  behaviour changes). If philos card_err regresses materially, record and
  escalate — do not silently re-tune.
- Amend ADR 0001 status: A2 adopted (see also R7.5).

---

## R2 — Stage 1b feed: label from the true assignment, per-sensor; keep the learned map's influence birth-only

**Problem.** The `feed_clutter_map` spike (`core/pmbm/PmbmTracker.cpp:1495-1562`)
reconstructs which measurement each Bernoulli claimed by
nearest-neighbour-at-same-timestamp (`:1510-1522`) instead of using the actual
assignment computed during association (it exists in `enumerateChildren`, then
is discarded). There is no sensor filter — an AIS-updated Bernoulli can
"claim" a radar return at the same timestamp — and the Bearing2D fallback
compares `sensor_position_enu` to the track mean, which is geometrically
meaningless for claiming. Failure mode: in close-pair geometry two Bernoullis
resolve to the same nearest return, so a real target return is fed as
weight-1.0 clutter at a real target's position.

**CORRECTION (2026-07-02, measured):** this ticket originally attributed the
dense_clutter regression (lifetime 0.90 → 0.26) to the NN-claim heuristic.
An A/B after implementing the true-assignment fix shows dense_clutter is
**unchanged (0.26)** — the heuristic is a real correctness bug (mislabeling in
close-pair and cross-sensor cases) but NOT the regression's cause. The actual
cause is what the eval log (2026-07-01) already said: **correctly-claimed
returns are fed at weight 1−r**, so in dense clutter a real target with
depressed existence feeds mostly-clutter mass at its own position → λ_C rises
there → existence drops further → death spiral. The cure is the
persistence+extent gate — i.e. the planned Stage 1b `LiveOccupancyModel`
(persistent AND extended → suppress; scattered noise never qualifies), not
anything inside this ticket. Separately, the loop is self-referential: learned λ_C lowers
r_new at an anchorage while a boat is still unconfirmed, delaying
confirmation, which keeps its returns labelled as clutter (literature-known
risk; the standard mitigation is a bootstrap — estimate the clutter field
robustly, feed a matched filter — not a single closed loop; cf. Vo, Vo,
Hoseinnezhad & Mahler, arXiv:1507.06397).

**Suggested fix.**
1. During the update, record for the max-weight global hypothesis each
   Bernoulli's claimed measurement index for the current scan (e.g. a
   `last_claimed_meas_index` set alongside `last_update`, or a per-hypothesis
   assignment vector). The feed loop consumes that instead of the NN
   reconstruction.
2. Restrict claims to the same `(SensorKind, MeasurementModel)` bundle as the
   return being labelled.
3. Delete the Bearing2D positional fallback (bearing-only measurements cannot
   initiate anyway).
4. Scope the learned occupancy's influence to the **birth channel only**
   (the Stage 1b `LiveOccupancyModel` design already does this —
   `birthSuppression`, not λ_C). Keep `feed_clutter_map`'s λ_C coupling a
   default-off spike flag and deprecate it once `LiveOccupancyModel` lands.
5. Minor: `!scan.empty()` (`:1503`) means empty scans never decay the map —
   either call observe with an empty bundle or document the asymmetry.

**Acceptance.**
- Unit test: two confirmed tracks + two returns in close-pair geometry →
  neither real return is fed with weight 1.0.
- Unit test: AIS-updated Bernoulli + radar return at the same timestamp →
  the radar return is not claimed cross-sensor.
- Determinism test for the feed stays green.
- ~~A/B: dense_clutter lifetime restored to ≈ 0.90~~ — **superseded by the
  correction above**: this fix does not (and cannot) restore dense_clutter;
  that gate belongs to Stage 1b's persistence+extent layer. R2's acceptance
  is the correctness tests + the negative A/B recorded in the eval log.

---

## R3 — Extent-only discrimination is an interim rule: document it and pre-build its failure-mode gates

**Problem.** Stage 1b discriminates KEEP (compact) from SUPPRESS (persistent
AND extended). Two documented failure modes (admitted in the Stage 1b design
but with no scenario coverage planned): a **large anchored ship** (~200 m,
non-AIS) is extended → wrongly suppressed (dangerous); a **single compact
pile/dolphin** is compact → tracked as a vessel (cardinality pollution, not
dangerous). The literature solves this with *classification*, not geometry:
Dalhaug, Stahl, Mester & Brekke 2025 (arXiv:2502.18368) exclude anything
boat-shaped from the static map via camera instance segmentation, and measured
both failure directions (a pre-trained model classified a floating dock as two
boats). They also measured that **ENC-only static layers under-cover real
harbour structure and still yield false tracks** — independent confirmation of
our ~1/3 chart-coverage finding.

**Suggested fix (docs + scenarios now, algorithm in 1b-ii).**
1. Add Dalhaug et al. 2025 (arXiv:2502.18368) to `docs/references/` (PDF +
   README row) — it is the closest published system to our Stage 1b/2 and is
   not yet in the repo.
2. Amend the Stage 1b design + ADR 0002 staging: extent is explicitly an
   **interim** discriminator; 1b-ii requirements are chart corroboration
   (S-57 `SMCFAC`/`PONTON`/`MORFAC`/`ACHARE` overlay), AIS corroboration
   (nav-status 1/5, Message 21 AtoN incl. virtual), and later camera
   classification.
3. Add two gate scenarios now (they may fail under 1b-i — record the numbers
   as known limitations, do not gate 1b-i on them): (a)
   `harbor_large_anchored_ship` — an extended zero-velocity truth target
   (multi-point returns spanning ~150 m) that must be KEPT; (b)
   `harbor_compact_dolphin` — a single fixed point return with no truth whose
   phantom track is measured (SUPPRESS target for 1b-ii).

**Acceptance.** Reference added; design/ADR amended; both scenarios exist,
run deterministically, and their 1b-i numbers are recorded in the eval log as
the 1b-ii "before".

---

## R4 — Reconcile the phantom-cluster attribution; commit the field-check evidence

**Problem.** The doc chain contradicts itself. ADR 0002 context and design
spec §14.10 (`2026-05-28-maritime-sensor-fusion-design.md:449-450`) say the
philos over-count is "fixed infrastructure, **not real boats**" (raw-radar
motion test: zero non-AIS movers). The Stage 1b design says it is "a **mix**"
including real anchored boats (operator Google-Maps field check). Both are
dated 2026-07-01; neither cross-references the other. Meanwhile the strongest
quantitative evidence exists **only in the uncommitted working tree**
(`charts/philos_chart_coverage.py` + PNG): top-100 strongest clusters — 0%
within 75 m of a charted bridge, **32% outside ENC coverage entirely**, ~63%
unexplained by any chart layer, median 180 m from the own-ship track; 0% in
charted anchorage areas. The unexplained residual is the largest single slice
of the remaining over-count and is currently attributed by narrative, not
measurement.

**Suggested fix.**
1. Commit the working-tree `charts/philos_chart_coverage.py` + PNG; add an
   eval-log entry with the field-check numbers (bridges, ENC-coverage,
   top-20 cluster table).
2. Amend spec §14.10 and ADR 0002 context wording to the supportable claim:
   "persistent uncharted structure dominates; at least one cluster is
   visually confirmed anchored boats; a 20 s clip cannot exclude sub-1 m/s
   moored craft — treat as a mixture."
3. Targeted investigation task: overlay the top-20 unexplained clusters
   (starting with the dominant group at ~42.3585 N / −71.0875 E and the
   outside-ENC group at ~42.376–42.379 N / −71.046 E) on satellite imagery /
   raw radar; classify each as structure / moored craft / own-ship-wake or
   sea clutter. Output: one eval-log table. This decides how much of the
   residual Stage 1b can actually crush (a persistent-occupancy layer will
   not remove non-persistent near-lane sea clutter).

**Acceptance.** Charts committed; eval-log entry exists; both docs amended
with cross-references; classification table for the top-20 recorded.

---

## R5 — Measure Stage 1a: the charted layer shipped with zero measured benefit

**Problem.** North-star row (Stage 1a) says it plainly: "No A/B bench
improvement measured yet — no fixture has charted hazards." The S-57 layer's
value currently rests entirely on the offline coverage analysis.

**Suggested fix.** (Prerequisite: harbor truth-sort fix + re-baseline.)
1. Add `harbor_charted_pier`: identical to `harbor_complete_truth` but the
   pier is *charted* — implement via the existing `ScenarioRun` static-
   obstacle hook (`syntheticObstacles()` / `static_obstacles_geojson_path`,
   wired in `core/benchmark/Sweep.cpp`) as a line of `StaticObstacle`s along
   the pier (footprint ≈ 10 m, keep-clear ≈ 50 m).
2. A/B `imm_cv_ct_pmbm` vs `imm_cv_ct_pmbm_static` on it. Expected: pier
   phantom births hard-dropped → card_err / gospa_false collapse; the three
   anchored boats (≥ 650 m away) byte-identical.
3. Real-data A/B: philos replay with the Boston ENC obstacle GeoJSON
   (`charts/geojson/`) loaded as the static-obstacle model. Expected
   improvement bounded by the coverage analysis (~⅓ of persistent structure);
   record whatever it is honestly.

**Acceptance.** Both A/Bs in the eval log; north-star Stage 1a row updated
from "no measurement" to the measured deltas.

---

## R6 — Harbor gate variant: boat-near-pier adjacency (the case philos actually contains)

**Problem.** `harbor_complete_truth` keeps its anchored boats ≥ 650 m from
the pier, so the geometry that motivated this whole effort — real boats
moored *next to* structure (your Google-Maps cluster) — is never stressed.
The current yardstick proves extent separability under generous spacing only.
It is easy on every known weak axis of the discriminator.

**Suggested fix.** Add `harbor_boat_near_pier`: one anchored zero-velocity
truth boat 30–60 m off the pier line — inside the charted pier's keep-clear
buffer (R5 variant) and inside the future live-occupancy neighbourhood
(Stage 1b). Gate: the boat's lifetime_ratio ≥ the open-water anchored boats'
value, while pier suppression still holds. This is also where R1's fix is
proven end-to-end (soft buffer + floor must not compose into a kill), and it
becomes a mandatory scenario in the M2/Stage-1b promotion gate alongside
`harbor_complete_truth`.

**Acceptance.** Scenario exists (deterministic, truth-sorted, contract test);
baseline measured under `imm_cv_ct_pmbm`, `_static`, `_land`; added to the
M2 gate definition in `docs/algorithms/synthetic-clutter-bench.md`.

---

## R7 — Housekeeping batch (one PR, no behaviour change intended except 7.2)

1. **`soft_max` vs hard-gate invariant is unenforced across objects.**
   `StaticObstacleParams::soft_max` (`core/static/StaticObstacleModel.hpp:17-21`)
   must stay strictly below `PmbmTracker::Config::static_obstacle_hard_gate`
   (0.95, `PmbmTracker.hpp:407`) or the whole buffer silently becomes a hard
   no-birth zone. Fix: clamp `soft_max` to ≤ 0.9 in the model constructor
   (documented contract: "birthSuppression returns 1.0 only inside the hard
   footprint") + unit test; optionally a wiring-time check in `Sweep.cpp`.
2. **GeoJSON adapter validation** (`adapters/static/GeoJsonStaticObstacles.cpp`):
   non-numeric coordinates currently *throw* mid-parse (`:79-80`) instead of
   skipping, contradicting the header contract; no lat/lon range check; no
   radius-sign check; raw `nlohmann::json::parse_error` escapes. Fix:
   per-feature try/catch → skip + count; validate lat ∈ [−90, 90],
   lon ∈ [−180, 180], radii ≥ 0; wrap parse errors in the documented
   `std::runtime_error`. Tests for each rejection path.
3. **Hazard-id collisions share hysteresis state.** `staticHazardId` hashes
   rounded position + category (`core/output/StaticHazardOutput.cpp:7-20`);
   co-located ENC records (wreck + obstruction at the same point) collide,
   and `StaticHazardEvaluator::inside_` is keyed by that id — one obstacle's
   Exit can suppress another's Enter. Fix: key `inside_` by obstacle index
   (stable within a run) while events keep `hazard_id`; optionally mix
   `source_id` into the hash when non-empty. Test: two obstacles at the same
   rounded position + category → independent enter/exit events.
4. **Datum-recenter wiring is undocumented for library consumers.** Nothing
   outside the bench registers the obstacle model as a datum sink
   (`Sweep.cpp:367-373` skips it deliberately — fixed datum). A consumer
   using auto-recenter gets a silently stale ENU cache. Fix: add the
   `provider.registerDatumSink(&obstacle_model)` snippet to CLAUDE.md's
   datum-sink example and the library docs; note the Sweep exception.
5. **ADR 0001 is stale.** It still reads as a blanket "no-birth zone kept"
   decision, but `imm_cv_ct_pmbm_land` (gate 0.05, no birth pin — ADR
   0002:70-73) relaxed it and is now the recommended config. Fix: add an
   "Amended 2026-07-02" note narrowing the decision's scope to
   `coverage_land`, pointing to `pmbm_land`, and recording A2's adoption
   when R1 lands.
6. **(Optional, defer)** `StaticObstacleModel::birthSuppression` is an O(N)
   scan per birth candidate (`StaticObstacleModel.hpp:44-58`). Fine for
   fixtures; a real ENC load (thousands of features) × per-measurement birth
   candidates warrants a uniform-grid spatial hash. Do this only when a
   real chart load lands (R5.3 will tell).

**Acceptance.** Each sub-item has its test; full suite green; no bench
deltas (bit-identical A/B on philos + synthetics for 7.1–7.4).

---

## Suggested order

1. **R7.1–7.3 + R2** (small, unblock Stage 1b correctness) →
2. **R1** (anchored-vessel safety; measured A/B) →
3. **R5 + R6** (after the truth-sort re-baseline; makes Stage 1a honest and
   the M2 gate meaningful) →
4. **R4 + R3 + R7.4–7.5** (evidence + docs; R4's cluster classification
   ideally before Stage 1b promotes, so we know what the layer can and
   cannot crush).

---

## R8 — Video-derived existence labels: fixture, label-aware philos metrics, binary gates (added 2026-07-03)

**Context.** The R4 video pass on `sunset_cruise` (operator + frame/radar
cross-reference, recorded in this session's eval-log addendum) resolved the
open UNKNOWN clusters and produced the first time-coincident ground truth for
philos. Headline findings: the clip contains **zero AIS** and ≥4 real moving
vessels; the two strongest "phantom" candidates were real vessels (a ferry
with a natural **stop→go transition at t≈90 s**, and a loitering vessel whose
returns **cease at t≈94 s while still in radar view**). Video labels must
become machine-checkable data — but NOT kinematic truth.

**Hard rule: never convert these labels into `TruthSample`s.** Video gives
existence + rough position + time window, not per-second positions.
Fabricating kinematic truth would be circular (positions derived from the
same radar the tracker consumes) and would corrupt GOSPA's localisation
term. The AIS-derived truth stays untouched; labels are a NEW category
("existence/region labels").

### R8.1 — Label fixture

Create `tests/fixtures/philos/labels/sunset_cruise_labels.csv`, schema:

```
region_id,source_rank,lat,lon,radius_m,t_start_s,t_end_s,label,evidence,confidence,notes
```

(`t_*` relative to clip start = first ownship timestamp 1635458136.03;
empty t = whole clip.) Entries from the 2026-07-03 video pass:

| region | lat, lon | radius | window (s) | label | evidence | conf |
|---|---|---|---|---|---|---|
| ferry_v1_a (rank11) | 42.378286, −71.046405 | 40 m | 0–98 | KEEP_VESSEL | video+radar+cell-timeline | high |
| ferry_v1_b (rank16) | 42.378730, −71.046373 | 40 m | 88–116 | KEEP_VESSEL | video+radar+cell-timeline | high |
| loiterer_v2 (rank202) | 42.379817, −71.046039 | 50 m | 10–94 | KEEP_VESSEL | returns cease t≈94 while in view; final frame empty; big ferry displaced starboard | med-high |
| midriver_grp (ranks 39/45/82) | 42.3766, −71.0432 | 80 m | — | SUPPRESS_STRUCTURE | E–W spread, 10–15 m from uncharted SLCONS; satellite pending | med |
| astern_blob | 42.3746, −71.0482 | 120 m | — | SUPPRESS_STRUCTURE | extended (n_cells ≈2400); out of camera FOV; satellite pending | med |
| ranks 84/95 | 42.3747, −71.0446 | 60 m | — | UNKNOWN | — | — |

Also add the chart-derived anchorage canary from the R4 CSV (42.3585,
−71.0877, KEEP) for the clips that cover it (`close_approach`,
`almost_cross`, `sailboats_busy`). Format is per-clip; HAXR gets the same
format when labeled.

### R8.2 — Label-aware philos metric decomposition

Keep raw `gospa_false` untouched (historical comparability). ADD columns to
the philos bench output: `false_on_suppress` (false mass whose track falls in
a SUPPRESS region during its window — the true phantom mass mitigation should
shrink), `tracks_on_keep` (confirmed-track mass/count in KEEP regions — must
NOT shrink; a drop = deleting real vessels), `false_unlabeled` (remainder).
This makes philos un-gameable: a config that "wins" by deleting ferries shows
a falling `tracks_on_keep`, which is a regression by definition.

### R8.3 — Binary gates

1. **Canary test per labeled clip:** for each KEEP region, assert ≥1
   confirmed track within `radius_m` during the window; for each SUPPRESS
   region (once any suppression mechanism is active), assert no long-lived
   confirmed track persists there. Pass/fail, no scores.
2. **Stop→go regression fixture:** extract `sunset_cruise` t≈60–120 s.
   Assert: a confirmed track exists on the ferry (r11→r16 regions), keeps a
   **stable track_id** through the stop→go transition at t≈90 s, and reports
   motion (SOG above threshold) by t≈110–116. This is the real-data
   instance of the ADR 0002 amendment's rule-3 gate; the synthetic variant
   stays alongside.

### R8.4 — Detector design input (1b-ii): the observed-empty discriminator

Both resolved UNKNOWNs were cracked by the same signature: **occupancy that
ceases while the cell is still inside radar coverage** = vessel departed;
structure never goes quiet while observable. Pull the Stage-2 "unknown vs
observed-empty" distinction forward into the 1b-ii detector as
**coverage-aware decay**: only decay cell evidence when the cell was
observable and returned empty; never decay for lack of coverage. Update the
detector design doc + `docs/learning/` chapter accordingly (CLAUDE.md
four-part standard applies).

### R8.5 — Bookkeeping

Eval-log addendum for 2026-07-03 must record: zero-AIS finding, the
stop→go event, the observed-empty discriminator, the full label table, and
the r202 resolution method (plot-time distribution per cell). Update the
philos metric-interpretation note in `docs/algorithms/comparison-baselines.md`
(philos = partial truth: AIS positions + existence labels; decomposed
reporting; canaries binding). Same labeling method applies to HAXR when it
becomes the churn testbed.

**Acceptance.** Fixture committed + loader; decomposition columns in the
philos bench output with a unit test; both binary gates green under
`imm_cv_ct_pmbm_land` (no suppression active → SUPPRESS assertions vacuous,
KEEP + stop→go must pass TODAY — they document current-behaviour safety);
docs updated.

**Shipped 2026-07-03 (R8.1–R8.3):** fixture
`tests/fixtures/philos/labels/sunset_cruise_labels.csv` + loader
`core/benchmark/ExistenceLabel` (unit-tested, `tests/benchmark/
test_existence_label.cpp`). Decomposition + both gates in
`tests/replay/test_philos_sunset_labels.cpp` — sunset_cruise is zero-AIS/no
radar-truth, so it is run through `imm_cv_ct_pmbm_land` directly (empty truth ⇒
no Sweep GOSPA slices) and scored against the labels: `tracks_on_keep=1633`,
`false_on_suppress=3070`, `false_unlabeled=18295`; all four KEEP canaries covered
(≤3.6 m); stop→go holds a stable id (id 13) across the t≈90 transition and reports
2.89 m/s late. Eval-log entry added. **R8.4 (observed-empty / coverage-aware
decay) and R8.5 bookkeeping remain** — R8.4 folds into increment 6 (the corrobo-
ration detector); R8.5's comparison-baselines philos-metric note pending.

### R8.6 — `close_approach` video pass: KEEP_MIXED labels, real-data CPA fixture, density stress (added 2026-07-03)

Second operator video pass (frames verified against radar/fixture geometry;
environment identified: Charles River sailing basin, regatta-density
dinghies). Clip: 80 s, start unix 1635536559.0, **zero AIS**, densest scene
in the dataset (~330–550 plots per 10 s).

1. **New label value `KEEP_MIXED`** (extend `ExistenceLabel` loader): region
   contains vessels AND structure. Semantics: **presence-gated** — a track OR
   an emitted hazard satisfies; never a strict must-be-tracks assertion; any
   departure from the region must become a track (the coverage-aware-decay /
   observed-empty mechanism is what detects departures). Create
   `tests/fixtures/philos/labels/close_approach_labels.csv`:

   | region | lat, lon | radius | window | label | evidence |
   |---|---|---|---|---|---|
   | sailing_dock (R4 ranks 1–2) | 42.35853, −71.08768 | 70 m | whole clip | KEEP_MIXED | frame t≈5 s right camera: float/dock lined with ~25 berthed dinghies + 3–4 crewed dinghies sailing beside it |
   | far_bank_line (ex-UNKNOWN shore group) | 42.3570, −71.0837 | 80 m | whole clip | KEEP_MIXED | end-of-clip center frame: far-bank small-craft line in exactly this direction; cells persistent across recording days ⇒ fixed floats/moorings; satellite pending for dock-vs-moorings detail only |

   Apply the same KEEP_MIXED semantics to the anchorage canary entries for
   `almost_cross` / `sailboats_busy` (same region, chart-derived until their
   own video passes).

2. **Real-data CPA/collision fixture (first of its kind).** A ~4 m club
   sailing dinghy (2 crew) closes and makes contact (or near-contact) at
   t≈61 s (unix ≈1635536620); approach window t≈30–65 s; a second dinghy
   crosses close ahead at the same moment. Radar plot data confirms returns
   on the collider down to 17 m (small returns, n_cells 4–29, amp ≥74).
   Assertions: (a) a confirmed track exists on the collider during the
   approach; (b) `CpaEvaluator` fires `Entered` for it with usable lead time
   before contact. This is the first real-data collision-alarm test.

3. **Sensor-doc note (also corrects an earlier in-session speculation):**
   the 15 m plot-extraction floor is a real blind zone ONLY in the final
   seconds before contact; the radar demonstrably tracked the collider to
   17 m. Record in `docs/sensors/sensor-reference.md` alongside the
   intensity-≥64 threshold note. Camera-channel motivation remains
   (identity/classification), not detection-at-range.

4. **Label-scored replay for `close_approach`** exactly as done for
   `sunset_cruise` in 9fde7ad (zero-AIS clip → run `imm_cv_ct_pmbm_land`,
   score `tracks_on_keep` / `false_on_suppress` / `false_unlabeled` against
   the labels). This clip is the standing **KEEP-stress benchmark**: any
   suppression mechanism must hold `tracks_on_keep` flat here.

5. **R4 accounting update (eval log, one line):** the largest in-coverage
   UNKNOWN group reclassifies to KEEP_MIXED → the honestly-suppressible
   ceiling shrinks below the previous ~50%; cite the frame evidence.

**Coverage-descriptor decision (R8.4 amendment, decided 2026-07-03):** the
feed's optional coverage descriptor carries **(sensor ENU, max range, azimuth
sector)** from day one; a disc is the degenerate sector [0°, 360°). Rationale:
interface churn is the expensive part, and disc-only coverage is wrong exactly
on philos (per-burst sweeps ⇒ in-range-but-out-of-sector cells would decay
wrongly, invalidating the mechanism's own validation). Wiring is staged:
synthetics pass the full circle (bit-identical to universal decay); philos
derives each burst's sector from that burst's plot azimuth span plus
conservative padding (self-estimated, no configured sector, portable to
HAXR). Under-estimated coverage is the safe error direction (no decay ⇒
hazards persist). Order within increment 6: coverage-aware decay first (has
real-data validation targets: loiterer cessation t≈94 and ferry-vacated cells
in `sunset_cruise`), then AIS veto (validates in synthetics/HAXR only — all
labeled philos clips are zero-AIS), then chart corroboration.

**Shipped 2026-07-03 (R8.6 items 1/4/5):** `KEEP_MIXED` label class added to
`core/benchmark/ExistenceLabel` (presence-gated; loader + fixture-load unit
tests in `tests/benchmark/test_existence_label.cpp`). Fixture
`tests/fixtures/philos/labels/close_approach_labels.csv` (2 video-verified
KEEP_MIXED regions: `sailing_dock` = R4 ranks 1–2, `far_bank_line` = ex-UNKNOWN
shore group). The label-scored replay harness was extracted to
`tests/replay/PhilosLabelReplay.hpp` (shared `runClip`/`decompose`); sunset_cruise
is bit-identical after the refactor. **close_approach KEEP-stress baseline**
(imm_cv_ct_pmbm_land, 880 scans, `tests/replay/test_philos_close_approach_labels.cpp`):
`tracks_on_keep = 5570`, `false_on_suppress = 0`, `false_unlabeled = 15182`; both
KEEP_MIXED canaries COVERED (`sailing_dock` 0.14 m, `far_bank_line` 1.40 m). This
clip is now the standing KEEP-stress benchmark. Eval-log entry + R4 ceiling
correction added. Full suite 925/925. **Deferred (independent, do not gate the
coverage-aware-decay work): item 2 (real-data CPA/collision fixture) + item 3
(15 m plot-floor sensor-doc note) trail as a separate task; the
`almost_cross`/`sailboats_busy` chart-derived anchorage canaries wait for a video
pass confirming the anchorage is in each clip's FOV (avoided a fake gate).**

### R8.7 — `ais_ferry_near` video pass: labels for THE bench scenario (added 2026-07-03)

Third operator video pass, frame- and geometry-verified. Clip: 20 s, start
unix 1667846694.0, own-ship **berthed/static** (moves ~15 m total), 23 AIS
vessels of which exactly 1 in radar range (mmsi 367074170, the ferry the
center camera holds). This is the bench "philos" scenario — every historical
gospa number (63.1, the 63→52 spike, the MHT comparison) was measured here —
and the persistent radar mass around the berth is dock structure PLUS
berthed pleasure boats: mixed regions, not pure clutter.

Create `tests/fixtures/philos/labels/ais_ferry_near_labels.csv`:

| region | lat, lon | radius | window | label | evidence |
|---|---|---|---|---|---|
| marina_dock_line | 42.3722, −71.0545 | 120 m | whole clip | KEEP_MIXED | right camera t≈10 s: floating docks + a dozen+ berthed motor yachts, footbridge/wave attenuator; matches top persistent cells (42.3718–42.3725 band, 90–165 m, full-clip t-span). Refine the boundary from the cells when loading. |
| left_pier | (left-camera pier; derive from cells W/SW of own-ship) | ~60 m | whole clip | SUPPRESS_STRUCTURE | operator left-camera observation: bare pier, no boats |
| south_pier_band | 42.3666, −71.0550 | 80 m | whole clip | SUPPRESS_STRUCTURE | persistent full-clip cells at 480 m; beyond camera range — satellite double-check optional |
| south_far_band | 42.3628, −71.0556 | 80 m | whole clip | SUPPRESS_STRUCTURE | persistent full-clip cells at ~900 m; satellite optional |

Notes:
- The AIS ferry needs NO label — it is inside the AIS truth set already.
- Run the label-scored decomposition on this clip like the others; this
  makes `tracks_on_keep` / `false_on_suppress` meaningful retroactively on
  the exact scenario the north-star cites. On this clip the decomposition is
  the difference between "suppressed pier returns" and "deleted a moored
  yacht."
- **Decay-test bonus:** own-ship static ⇒ coverage sectors barely move —
  the natural "own-ship stationary" sanity case for coverage-aware decay
  (cells outside swept sectors must not decay; marina cells accumulate
  cleanly). Add as a cheap assertion in the decay tests.
- Clip-level: confirms ADR 0002's re-reading of philos as a
  clutter-rejection-while-berthed test.

Video-pass status: `sunset_cruise` done (R8.1), `close_approach` done
(R8.6), `ais_ferry_near` done (this section). Next queued: `sailboats_busy`
(KEEP-stress cross-validation of the Charles-basin regions from a different
recording day), then `almost_cross` / `car_carrier_near` / `ais_ferry_far`
(the last two mainly for the 42.3583,−71.0464 in-coverage UNKNOWN, likely
satellite-resolvable instead).

**R8.6 STATUS (2026-07-03, commit 1d55d00, suite 925/925):** items 1/4/5
SHIPPED — `KEEP_MIXED` label class (TDD), `close_approach_labels.csv` (both
video-verified regions; column-count guard test), shared `PhilosLabelReplay`
harness (sunset re-run bit-identical: 1633/3070/18295, canaries 0.17–3.6 m,
stop→go id stable @2.89 m/s), close_approach KEEP-stress baseline under
`imm_cv_ct_pmbm_land` (880 scans): **tracks_on_keep = 5570,
false_on_suppress = 0 (no SUPPRESS regions labeled in this clip),
false_unlabeled = 15182; both KEEP_MIXED canaries covered (0.14 m / 1.40 m)**
— this is the standing "hold tracks_on_keep flat" gate. R4 ceiling
correction in eval-log. Items 2 (CPA/collision fixture) + 3 (sensor-doc
min-range note) deferred as an independent task; anchorage canaries for
`almost_cross`/`sailboats_busy` pending FOV-confirming passes (correctly not
asserted unverified). NEXT: R8.4 coverage-aware decay — descriptor on
`ScanObservation` (today carries no sensor pose/coverage), populated by the
`feed_clutter_map` producer (`PmbmTracker.cpp` ~L1664), consumed in
`LiveOccupancyModel::observe()`; absent descriptor ⇒ universal decay
(synthetics bit-identical); under-estimated sector = safe direction.

### R8.8 — `car_carrier_near` re-extraction (broken heading) + occlusion pass (added 2026-07-03)

**Data-integrity finding.** `car_carrier_near` (2020-10-22 bag) has
`heading_deg = 0.000` constant and only 26 own-ship rows over 120 s — the
extractor's topic lists don't match the 2020 bag layout, so it fell back to
the sparse `/gnss` topic (26 fixes) and emitted a heading placeholder. Every
world-projected radar return from this clip is therefore **rotated about
own-ship by the true heading** (GPS track suggests course ≈300°, so the
rotation is large). Integrity sweep confirms all six other clips are fine
(dense rows, real headings). The bag itself carries everything needed:
`/filter/positionlla` + `/sensor/gps/fix` (~60–70 Hz position) and
`/filter/quaternion` / `/imu/data` (~60 Hz attitude → yaw).

1. **Extractor fix** (`tests/fixtures/philos/extract_section.py`): add
   position fallbacks `/filter/positionlla`, `/sensor/gps/fix` (prefer the
   densest NavSatFix-compatible topic) and heading fallback = yaw from
   `/filter/quaternion` (or `/imu/data`), applied when none of the named
   heading topics exist. Guard: after extraction, assert >1 distinct heading
   value and row rate ≥1 Hz — fail loudly instead of emitting placeholders
   (this is the trap that silently produced a rotated clip).
2. **Re-extract `car_carrier_near`**; expect ~7k own-ship rows with real
   headings.
3. **Invalidate + redo this clip's contribution to the R4 chart analysis**
   (its cells were rotated): re-run `charts/philos_chart_coverage.py` after
   re-extraction; re-check the in-coverage UNKNOWN at 42.3583, −71.0464,
   which was previously supported by this clip + `ais_ferry_far` (only the
   latter is currently valid).
4. **Then the video/label pass** (operator + frame verification): purpose is
   the **occlusion archetype** — a large car carrier close to own-ship
   shadows moored vessels behind it; test data for whether coverage-aware
   decay's sector model needs an LOS/shadow guard (don't decay cells
   shadowed by a strong closer return) or whether shadow-induced
   observed-empty false-fires stay negligible. Clip is in-sample (holdout
   remains `sailboats_busy` / `almost_cross` / `ais_ferry_far`).
5. Eval-log entry for the integrity finding + the AIS note: the 2020/2021
   campaigns carry NO AIS at all (receiver absent) — the AIS-veto's
   real-data validation must come from HAXR, not philos.

---

## R9 — Cooperative+radar readiness (pre-real-test; reviewed 2026-07-04)

Context: cooperative (fleet-partner GNSS fixes, `SensorKind::Cooperative`,
`platform_id`) + radar will be the first real deployment test. Review of the
cooperative paths against this week's changes: core is INTACT — 28 tests
green on the current tree (identity gate, cooperative-only retirement,
AIS-as-cooperative activity, bus determinism); none of the week's work
(coverage decay, labels, camera loader, pose sorted-insert) touches its
code paths. Three items to close BEFORE the real test:

1. **Latent: cooperative leaks into the occupancy layer (only when the
   detector is ON).** Cooperative fixes are Position2D/canInitiateTrack, so
   in `feed_clutter_map` they (a) feed as full-weight clutter when
   unclaimed, and (b) with `estimate_coverage_sector` on, the cooperative
   bundle self-estimates a meaningless "swept wedge" (fleet-partner
   bearings from `sensor_position_enu`=origin) that is UNIONED into the
   decay footprint — decay evidence over cells nothing observed:
   over-claiming, the unsafe direction (same family as the multi-cluster
   bug). Exposure needs occupancy detector + coverage flag both ON (both
   default OFF today). Fix: exclude non-scanning SensorKinds (Cooperative,
   Ais) from `cov_sensor` coverage estimation; and when the corroboration
   veto lands, key it on Cooperative as well as AIS ("cooperative-known
   platform must track, never suppress" — strongest possible
   discriminator under the ADR 0002 amendment).

2. **Scenario-test gap: nothing fuses Cooperative + radar end-to-end** —
   the exact shape of the first real test. Add one scenario test: fleet
   partner (cooperative fixes ~10 s) + radar plots on the same vessel +
   clutter; assert (a) ONE track, not a cooperative/radar dual; (b)
   `platform_id` carried; (c) ID stable through a cooperative dropout
   while radar corroborates — and NO cooperative-timeout retirement in
   that regime (verified in code: retirement sits inside the
   no-surveillance-opportunity branch, `PmbmTracker.cpp` ~L668–695, so
   radar coverage suppresses it — pin this with a test); (d) retirement
   DOES fire when both channels go silent past
   `cooperative_stale_timeout_sec`.

3. **Docs: the real-test wiring recipe.** The retirement behavior in (2c)
   depends on `use_sensor_activity` + `DeclaredSensorActivity` profiles
   for BOTH the radar and the cooperative channel; without profiles the
   legacy fallback behaves differently. The integration guide currently
   notes ISensorActivity only in passing — add a cooperative-channel
   section (fits the guide's keep-in-sync rule; hand to the guide's
   doc-bug follow-up pass).

---

## R10 — Remote-track ingestion (shore/VTS feed as pseudo-measurements) [queued 2026-07-04, AFTER R9]

Context: target deployment sensor suite is (1) remote station sending
TRACKS, (2) cooperative vessels sending their own positions, (3) camera —
possibly with a distance sensor, (4) radar, (5) AIS. Items 2–5 are
supported today (camera+distance = the existing range/bearing path via
`makeMeasurementFromRelativeBearing` with `SensorKind::EoIr` — no new work,
just a guide note). Item 1 is the gap this ticket closes. Stance (design
spec §13, same as ARPA from day one): another tracker's output is a
**pseudo-measurement** — filtered, correlated, someone else's lifecycle
artifacts — never an independent observation.

**Do.**
1. `SensorKind::RemoteTrack` — so sensor defaults, activity profiles, and
   the occupancy exclusions key on it. MUST be excluded from
   `cov_sensor` coverage estimation and treated like Cooperative/Ais in
   the R9 item-1 fix (a shore feed is a non-scanning source; its
   "coverage wedge" would be meaningless — same over-claiming trap).
2. Small adapter (`adapters/remote_track/`): remote track update →
   `Position2D` (or `PositionVelocity2D`; velocity opt-in, extra
   suspicion) with:
   - **R inflation** (config, default ×2–3 on the remote system's stated
     covariance; pessimistic default when none stated);
   - **rate thinning** (config max update rate per remote track id —
     consecutive filtered outputs are correlated, not independent);
   - `hints.sensor_track_id` = remote track id (per-source scope — one
     `source_id` per remote STATION, so multi-station feeds stay
     disjoint); `hints.mmsi` passed through when the remote system
     carries it.
3. **Circular-AIS rule documented where the adapter is documented:** if
   raw AIS and an AIS-fusing shore feed are both wired, the same
   transmission arrives twice — pick one path per vessel or inflate for
   the correlation. This is a deployment decision the adapter cannot make
   silently; it warns (log/diagnostic) when both channels carry the same
   MMSI.
4. `DeclaredSensorActivity` profile: surveillance channel with the remote
   station's coverage area, so miss-math knows where the feed should see.
   Registration-bias machinery (backlog #9/#13) applies per source_id for
   fixed alignment errors — verify wiring, don't rebuild.
5. Tests: adapter unit tests + ONE fusion scenario shaped like the real
   deployment — remote tracks + radar + AIS (+ cooperative) on the same
   vessel: single track, no dual from the remote feed, ID stable when the
   remote feed drops or swaps its own ids, R9-style no-retirement while
   radar corroborates.
6. Integration-guide entry (keep-in-sync rule applies): when to use it,
   the three pseudo-measurement rules, the circular-AIS warning, and the
   camera-with-distance pointer (range/bearing path, not a new sensor
   kind).

**Explicitly NOT in scope:** covariance intersection / measurement
decorrelation (proper track-to-track fusion). Deferred per spec §13 with
a measured trigger: revisit when a real shore feed shows bias/overconfidence
that R-inflation + thinning cannot price (NEES/consistency check on the
fusion scenario is the detector for this).

**Acceptance.** SensorKind + adapter + exclusions + warning shipped;
fusion scenario green; NEES sanity on the scenario recorded in the
eval-log; guide entry present (drift-guard will enforce the config
struct); north-star row updated.
