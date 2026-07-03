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
