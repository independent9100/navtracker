# navtracker — Maritime Multi-Sensor Fusion Tracker: Design Spec

**Date:** 2026-05-28
**Status:** Draft for review
**Authors:** Andreas + Claude

---

## 1. Overview

navtracker fuses outputs from multiple maritime sensors into a single, authoritative set of vessel **tracks**. It is *not* a raw detector (no radar plot extraction or image-level object detection). It consumes the higher-level outputs that sensors already produce — AIS position reports, ARPA radar tracks, EO/IR camera detections, own-ship navigation — and produces one fused, de-duplicated, continuously maintained track picture.

The defining problem is **multi-target tracking (MTT) with data association**: deciding which incoming piece of data belongs to which target among many, spawning new tracks, maintaining them, and deleting stale ones — while exploiting sensor-provided identifiers (MMSI, ARPA track ID) when present but never depending on them.

## 2. Goals and Non-Goals

### Goals
- Fuse heterogeneous sensor outputs into one authoritative track set.
- Full multi-target data association; opportunistic use of sensor IDs as hints.
- Every track carries a **stable, unique internal track ID** independent of external identity.
- Single core engine runs both **real-time (onboard)** and **deterministic replay** of recorded logs.
- Per-track outputs: fused kinematic state + covariance, identity/classification/size, lifecycle + provenance, derived CPA/TCPA.
- Pluggable data-association and estimator strategies so algorithms can be evaluated and swapped.
- Rigorous documentation of math, assumptions, decision rationale, and improvement paths.

### Non-Goals (this project)
- Raw detection (radar signal processing, image object detection).
- COLREGS / collision-avoidance *decision-making* (we expose CPA/TCPA; deciding maneuvers is downstream).
- Sensor hardware drivers / transport (adapters consume already-decoded messages or log records).
- UI / visualization (we expose a track API; rendering is a separate consumer).
- **Emitting fixed infrastructure (piers, breakwaters, shoreline) as vessel
  tracks.** The point-target vessel tracker owns *vessels* (including static
  ones — anchored/moored/drifting); fixed environment is handled by a separate
  static-obstacle branch and is *not* a vessel track. The derived CPA/TCPA
  therefore covers own-ship × vessel-track only, not collision against
  infrastructure. See §14.10 and ADR 0002.

## 3. Key Decisions (with rationale)

| # | Decision | Chosen | Rejected alternatives | Why |
|---|----------|--------|-----------------------|-----|
| D1 | Fusion topology | **Centralized, measurement-level** | Track-to-track (decentralized); Hybrid | Single source of truth → trivially yields a stable unique track ID even for non-cooperative targets; statistically near-optimal use of information. Sensor IDs become hints/attributes, not the fusion key. |
| D2 | Time base | **Message-timestamp driven** | Wall-clock driven | Makes live and replay use the identical engine; enables deterministic, reproducible tests. |
| D3 | Common coordinate frame | **Local tangent plane (ENU) about a configurable datum** | Geodetic lat/lon directly; ECEF | Linear-ish kinematics over operational ranges; simple covariances; avoids lat/lon distortion in filters. Geodetic conversion happens only at the boundary. |
| D4 | Late/out-of-order data default | **Drop beyond reorder window + log** | Retrodiction/OOSM update | Simpler, deterministic baseline. Retrodiction listed as a documented improvement (§11). |
| D5 | Baseline data association | **Global Nearest Neighbor (GNN)** | JPDA, MHT | Predictable cost, good enough to validate the pipeline; pluggable so JPDA/MHT can be evaluated later. |
| D6 | Baseline estimator | **EKF** (IMM-ready) | UKF, Particle filter, linear KF | Handles nonlinear range/bearing measurement models; cheap; pluggable. IMM/particle to be evaluated for maneuvering targets. |
| D7 | Architecture style | **Clean architecture + hexagonal (ports & adapters)** | Layered monolith | Keeps sensor formats and I/O at the edges; domain core has zero I/O dependencies; strategies are ports → highly testable and swappable. |
| D8 | Sensor reference | **Separate document** | Inline in this spec | It's a living reference that will grow per sensor; keeping it standalone keeps this spec stable. |

> Scale/performance envelope is intentionally **undecided**. The design keeps association and estimator on the hot path swappable so the system can be tuned once real load is known.

## 4. Architecture (clean / hexagonal)

```
                       ┌──────────────────────────── Domain core (no I/O) ────────────────────────────┐
 driving adapters      │                                                                               │   driven adapters
 ┌───────────────┐     │  ┌───────────┐  ┌───────────┐  ┌──────────────┐  ┌───────────┐  ┌──────────┐ │   ┌────────────┐
 │ AIS adapter   │──┐  │  │ Normalize │  │ Time-order │ │ Data         │  │ Estimator │  │ Track    │ │   │ Track sink │
 │ ARPA adapter  │──┼─▶│─▶│ + geo-proj │─▶│ buffer    │─▶│ association  │─▶│ (per track)│─▶│ manager  │─┼──▶│ (API/log)  │
 │ EO/IR adapter │──┤  │  └───────────┘  └───────────┘  └──────────────┘  └───────────┘  └──────────┘ │   └────────────┘
 │ own-ship nav  │──┘  │                       ▲ ports:  IDataAssociator   IEstimator     IMotionModel  │
 └───────────────┘     │                       └ attribute/identity fusion + CPA/TCPA derivation ──────┘
        ▲ port: ISensorAdapter                                                                          │
        └──────────────────────────────────────────────────────────────────────────────────────────────┘
```

- **Domain core**: tracks, estimators, association, track management, attribute fusion. Pure C++, no I/O, no sensor formats. Depends only on abstractions.
- **Ports (interfaces)**: `ISensorAdapter`, `ITrackSink`, `IDataAssociator`, `IEstimator`, `IMotionModel`, `IClock`/time-source.
- **Adapters**: concrete sensor parsers and output sinks; implement ports; live outside the core.
- **Dependency rule**: source dependencies point inward. Adapters depend on the core; the core never depends on adapters.

### Module layout (proposed)
```
navtracker/
  core/            # domain: types, tracker, estimators, association, track mgmt   (no I/O)
  ports/           # interfaces (ISensorAdapter, IEstimator, IDataAssociator, ...)
  adapters/        # ais/, arpa/, eoir/, ownship/, sinks/                          (I/O, formats)
  app/             # composition root: wires adapters+strategies, run/replay modes
  tests/           # gtest unit + scenario/replay tests, metrics harness
  docs/            # specs, sensor reference, algorithm docs (math/assumptions/...)
```

## 5. Core Domain Types

- **`Measurement`** — normalized sensor output handed to the core:
  - `timestamp` (source time), `source_id`, `sensor_kind`
  - `frame` (ENU about datum), value subset (position, and/or range/bearing, and/or velocity)
  - measurement covariance `R`
  - optional association hints: `mmsi`, `sensor_track_id`
  - optional attributes: vessel type, dimensions, classification, confidences
- **`Track`** — authoritative fused entity:
  - `track_id` (stable internal primary key, never reused)
  - kinematic state `x`, covariance `P`, motion-model tag
  - lifecycle status (Tentative / Confirmed / Coasting / Deleted)
  - attribute set (identity, type, dimensions, classification) each with confidence + provenance
  - provenance: contributing sources, update history
  - derived: CPA, TCPA vs own-ship
- **Strategy ports**: `IDataAssociator`, `IEstimator`, `IMotionModel`.
- **Edge ports**: `ISensorAdapter` (produces `Measurement`s), `ITrackSink` (consumes track updates).

## 6. Data Flow & Determinism

1. Adapters decode native input → raw typed reports, stamped with source time.
2. Normalizer projects each report into the ENU frame using the latest own-ship nav; builds `Measurement` with covariance `R`.
3. Time-ordered buffer releases measurements in timestamp order within a bounded reorder window (configurable). Beyond the window → drop + log (D4).
4. For each released measurement: predict all tracks to its timestamp → associate (gating + assignment) → update matched tracks → unmatched measurements feed track initiation.
5. Track manager runs lifecycle transitions; attribute fusion merges identity/type/size; CPA/TCPA derived.
6. Track sink emits the updated track set.

**Determinism:** the engine advances only on measurement timestamps. Replay feeds logged messages with original stamps through the same buffer → identical results given identical input ordering. Live mode differs only in *where* messages come from and the real-time release of the buffer.

## 7. Track Lifecycle

`Tentative → Confirmed → Coasting → Deleted`
- **Initiation**: unmatched measurement seeds a Tentative track.
- **Confirmation**: M-of-N detections or score threshold (documented, configurable).
- **Coasting**: no association this cycle → predict-only; covariance grows.
- **Deletion**: timeout or score below floor. `track_id` retired, never reused.
- Every transition recorded in provenance.

## 8. Error Handling & Edge Cases

- **Missing/late own-ship nav** → cannot project relative measurements; extrapolate own-ship for a bounded time, flag affected tracks, else hold.
- **Identity conflicts** (two MMSIs on one track, or one MMSI on two tracks) → documented resolution policy; kinematic track remains primary.
- **Per-sensor clock skew** → configurable time offset per source applied at normalization.
- **Implausible/spoofed AIS** → plausibility gating before attribute fusion (kinematic consistency check).
- **Boundary validation**: adapters validate parse/units/NaN. Internal stages trust their inputs (validated-at-the-edge).

## 9. Testing Strategy (gtest)

- **Unit tests** per stage and per strategy, written against the port interfaces (mocks/fakes for ports).
- **Scenario / replay tests**: synthetic ground-truth scenarios — crossing, overtaking, head-on, AIS dropout, non-cooperative (no-MMSI) target, sensor clock skew — assert track continuity, **ID stability**, and accuracy vs. truth.
- **Metrics harness**: track-accuracy / OSPA-style metrics to *quantitatively compare* estimator and association choices later (directly serves the "what to test next" goal).
- Determinism test: same log replayed twice → identical track output.
- Build: CMake + Conan; gtest pulled via Conan; tests run in CI-friendly fashion.

## 10. Documentation Standard (project-wide rule)

Every non-trivial algorithmic component is documented with a fixed four-part template:

1. **Math** — state/measurement models, equations, covariances, coordinate conventions.
2. **Assumptions** — what must hold for the math to be valid.
3. **Rationale** — why this approach over the considered alternatives.
4. **Ways to improve / what to test next** — concrete candidate alternatives and the experiment to evaluate them.

This template is enforced via CLAUDE.md and applied to the algorithm sections below and all future ones.

## 11. Baseline Algorithm Documentation

### 11.1 State & motion model (baseline: Constant Velocity, ENU 2D)

**Math.** State `x = [px, py, vx, vy]ᵀ` in ENU meters / m·s⁻¹. Discrete CV model over step `Δt`:

```
x_k = F(Δt) x_{k-1} + w,   F = [[1,0,Δt,0],[0,1,0,Δt],[0,0,1,0],[0,0,0,1]]
Q = process noise from continuous white-noise acceleration with PSD q  (standard CV Q(Δt))
```

**Assumptions.** Targets move ~constant velocity between updates; maneuvers absorbed by process noise `q`; flat-Earth ENU valid over operational range; 2D (surface) motion.

**Rationale.** CV is the standard, robust baseline; minimal state; nonlinearity lives only in measurement models. Chosen over constant-acceleration (overfits) and coordinated-turn (premature) for a first cut.

**Ways to improve / test next.** IMM mixing CV + coordinated-turn for maneuvering vessels; tune/learn `q` per target class; 3D state if needed.

### 11.2 Measurement models

**Math.**
- AIS / position-type: linear `z = H x`, `H = [[1,0,0,0],[0,1,0,0]]` (and velocity rows when COG/SOG present). Gaussian `R` from reported/assumed accuracy.
- EO/IR bearing-only and radar range/bearing: nonlinear `z = h(x)` (range `√(px²+py²)`, bearing `atan2(py,px)` relative to own-ship after frame transform); linearized via Jacobian `H = ∂h/∂x` for the EKF.

**Assumptions.** Gaussian, zero-mean measurement noise; covariances `R` known/estimable per sensor (see sensor reference); own-ship pose known at measurement time.

**Rationale.** EKF linearization is adequate for the mild nonlinearity of range/bearing at operational geometry; far cheaper than UKF/particle.

**Ways to improve / test next.** UKF for stronger nonlinearity / bearing-only geometry; particle filter for highly non-Gaussian/multimodal cases (e.g. bearing-only before range converges); per-sensor `R` calibration from data.

### 11.3 Data association (baseline: GNN)

**Math.** Mahalanobis gating `d² = ν ᵀ S⁻¹ ν` (innovation `ν`, innovation covariance `S = HPHᵀ + R`) with χ² gate; assignment minimizes total gated cost (e.g. auction/Hungarian). Sensor ID hints (MMSI, ARPA track ID) bias/lock association when present and plausible.

**Assumptions.** ≤1 measurement per track per sensor per cycle; gating covariances trustworthy; clutter modest.

**Rationale.** Deterministic, bounded cost, easy to reason about; sufficient to validate fusion before investing in probabilistic association.

**Ways to improve / test next.** JPDA for dense/ambiguous clutter; MHT for deferred decisions; learned/feature-aided association using attributes (size, type).

## 12. Phasing & Deliverables

**Phase 0 (this phase — docs only, no engine code):**
- This design spec.
- `CLAUDE.md` — stack (C++17/Conan/CMake), conventions, hexagonal/clean-architecture invariants, the §10 documentation template, testing-with-gtest expectations.
- Sensor-data reference doc — per sensor: data fields, units, update rates, error/covariance characteristics, identity content, failure modes, gotchas.

**Phase 1+ (later, via implementation plan):** project skeleton (CMake/Conan, gtest) → core domain types → ports → adapters → baseline GNN+EKF tracker → scenario/replay test harness + metrics. Then algorithm evaluation (IMM/UKF/particle, JPDA/MHT) using the metrics harness.

## 13. Open Questions / To Revisit

- Datum strategy for the ENU origin: fixed datum vs. own-ship-referenced vs. periodic re-origin for long transits.
- Exact confirmation/deletion policy parameters (M-of-N vs. score-based) — decide during Phase 1 with scenario data.
- Whether ARPA tracks ever need true track-level fusion (covariance intersection) rather than pseudo-measurement treatment — revisit if measurement-level proves biased.
- Multi-radar overlap handling (same target from two radars in one cycle) — refine association's one-measurement-per-track assumption.

## 14. Future Improvements (deferred)

Items captured here so we do not lose them. None are scheduled; each is a
candidate project in its own right.

### 14.1 Sensor pose on measurements

Add `sensor_position_enu` (and eventually `sensor_orientation`) to
`Measurement` so the range/bearing/bearing-only h(x) families can be
evaluated against an arbitrary sensor pose instead of the implicit ENU
origin. Defaults to `(0,0)` so existing tests and adapters are unaffected.

**Unlocks.** True bearing-only fusion with parallax (own-ship motion makes
range observable); shore-based or off-origin sensors; multi-platform fusion
(own-ship + escort + UAV).

**Cost.** ~50 lines + tests in `MeasurementModels` and the adapter layer.
Independent of, but prerequisite to, 14.2.

### 14.2 Rigid-body platform / target geometry

Two coupled extensions:

1. **Own-ship sensor offsets.** Configurable `(dx, dy)` per sensor relative
   to the navigation reference point (GPS antenna). Adapters apply the
   offset when populating `sensor_position_enu`. Today every sensor is
   implicitly co-located at the GPS antenna; the error is bounded by the
   longest offset (~50 m on a 100 m platform). Sub-percent at open-water
   ranges; meaningful at harbor / docking ranges.

2. **Target geometry into association + CPA.** `TrackAttributes` already
   carries `length_m` and `beam_m`; today they are populated by adapters
   and never consumed. Wire them into:
   - **CPA / collision metrics** — replace `‖center_A − center_B‖` with a
     bow/beam-aware closest-approach calculation.
   - **Association strength** — radar return size consistency with claimed
     vessel length as an extra association feature alongside Mahalanobis
     gating.
   - **Outlier rejection** — a measurement that "jumps" by less than half
     the hull length is likely a different return from the same ship, not
     a tracking glitch.

**Unlocks.** Honest collision-avoidance metrics; stronger association at
close range; better lifecycle decisions when measurement spread is
comparable to vessel size.

**Cost.** Medium. Sensor-offset config + adapter changes ~100 lines.
Geometry-aware CPA + association feature is a separate, larger change.

### 14.3 Extended object tracking

Promote target extent (length, beam, orientation) into the dynamic state
rather than carrying it as a passive attribute. State grows to roughly
`[px, py, vx, vy, ψ, L, B]`; motion model needs leeway and heading-vs-
velocity-direction constraints; measurement models must handle radar
extent reports and camera bounding boxes natively.

**Unlocks.** Tracking maneuvering large vessels accurately at close range
(tugs, anchor handling, low-speed dock approaches); confidence on size
from multi-look fusion; native handling of extended-return sensors.

**Cost.** Substantial — its own project. Predicate: 14.1 and 14.2 in place
so that the simpler representations have been exhausted first.

### 14.4 Bearing-only scenarios with own-ship motion

Once 14.1 is in, build a scenario harness that emits bearing-only
measurements from a moving sensor frame. This is the smallest meaningful
scenario where a particle filter should provably outperform EKF / UKF
(banana-shaped posterior during the parallax convergence window). Until
14.1 lands, bearing-only from a stationary sensor at the ENU origin is
range-unobservable and all three filters tie at the prior-range uncertainty
(see `docs/algorithms/evaluation-log.md` entry of 2026-06-01).

### 14.5 Close-range precision sensors

Add support for sensors whose error budget is dramatically tighter than
nav-class radar at sub-kilometre ranges. The classes worth wiring:

1. **Laser rangefinder** — range σ ~1 m and bearing σ ~0.05° at 0–500 m
   when manually aimed or auto-tracked on a designated target. Existing
   `RangeBearing2D` model fits with no math change; only a new adapter is
   needed.
2. **Docking / close-range radar** (e.g. Furuno DRS-NXT short range) —
   sub-metre range and tighter bearing than nav radar at <100 m. Same
   measurement model, separate adapter so its `R` reflects the tighter
   spec.
3. **Stereo / RGB-D camera** — range + bearing at <50 m for visually
   detected vessels. Same `RangeBearing2D`; the adapter extracts depth
   from disparity or the camera's depth channel.
4. **Forward-looking sonar** — range to surface/sub-surface obstacles.
   Needs a new `Range2D` measurement model (range only, no bearing) and
   its own adapter.

**Unlocks.** Harbor pilotage / docking / anchor-handling / escort-tug
operations where target proximity is what matters and nav radar's 50 m
range σ swamps the posterior. The tighter `R` dominates the fusion at
close range, so the gain shows up automatically without changing the
estimator.

**Cost.** Per sensor class: ~80 lines for the adapter, plus `Range2D`
infrastructure if forward-looking sonar is in scope (~50 lines for the
measurement model and its tests). `SensorKind` already has `Lidar`;
add `LaserRangefinder`, `DockingRadar`, `StereoCamera`, and `Fls` when
the corresponding adapters are wired.

**Dependency.** Only meaningful after 14.1 (sensor pose on measurements).
A laser rangefinder mounted on the bow with the GPS antenna 30 m aft
would otherwise have its precise return co-located at the GPS receiver,
defeating the precision. 14.2 (own-ship sensor offsets) further refines
this for multi-sensor platforms. Without 14.1 the close-range sensors
will appear to "miss" the target by their own mount offset.

### 14.6 CPA / TCPA — basic point-mass version

Already implemented in `core/collision/Cpa.hpp` as
`computeCpa(track_a, track_b, t_ref)`. Closed-form CV math; clamps past
TCPA to 0 and reports current distance when diverging.

This is the **point-mass, deterministic** version. Real collision-avoidance
systems need uncertainty (§14.7) and hull geometry (§14.8) to be honest.
Pairwise enumeration across all confirmed tracks and an own-ship-aware
"CPA to me" path are also pending.

### 14.7 CPA uncertainty propagation

The deterministic CPA scalar is a point estimate. Each track carries a
4×4 covariance; both contribute uncertainty to the CPA distance and TCPA.
For a useful collision-warning system the right output is
**`P(CPA < threshold)`** within some lookahead window, not a single
CPA number.

Three implementation paths, in order of fidelity:

1. **Linearization.** Jacobian of `(cpa_distance, tcpa)` w.r.t. each
   track's `(p, v)`; propagate `J Σ Jᵀ` to get a 2×2 covariance over
   `(cpa, tcpa)`. Cheap, valid when uncertainty is small relative to the
   geometry's nonlinearity. Closed-form derivatives are tractable.
2. **Monte-Carlo.** Sample N draws from each track's Gaussian, compute
   CPA per draw, build the empirical distribution. Always correct,
   slow. Good baseline for validating (1).
3. **Sigma-point / unscented.** UKF-style sigma points through the CPA
   function. Better-than-linear nonlinearity capture, much cheaper than
   Monte-Carlo. Likely the production sweet spot.

**Unlocks.** Honest collision-warning thresholds; risk-tiered alerts
(e.g., "85% chance CPA < 100 m within 5 min"); IMM/PF tracks where the
posterior on velocity is genuinely non-Gaussian and the linearization
breaks down.

**Cost.** Closed-form Jacobians (1): ~80 lines + tests. Monte-Carlo (2):
~40 lines + tests. Sigma-point (3): ~120 lines + tests. Plus a new
output schema (`CpaDistribution { mean_cpa, std_cpa, mean_tcpa,
std_tcpa, prob_collision }`) for downstream.

### 14.8 Hull-aware closest-approach

`CpaResult.cpa_distance_m` today is **center-to-center**. Real
collision-avoidance wants **bow-to-hull** or **hull-to-hull** distance,
which requires modeling each vessel as an oriented 2D rectangle (length,
beam, heading). Existing infrastructure: `TrackAttributes.length_m` and
`TrackAttributes.beam_m` are already populated by AIS but never consumed.

**Math.** Two oriented rectangles → minimum point-to-rectangle or
rectangle-to-rectangle distance via the separating axis theorem (SAT) or
GJK. For maritime vessels with axis-aligned hull frames relative to
heading, SAT is the right primitive: project both rectangles onto each
axis (4 axes total for two rectangles), find the maximum gap, that's the
distance. Time-of-closest-approach becomes a piecewise problem because
the minimum can occur at any pair of vertices or edges over time.

**Unlocks.** CPA values that mean what they sound like to a watch
officer; correct warnings when a 300 m container ship's bow is 50 m from
a 12 m fishing boat at the closest point of approach (today reports 150 m
center-to-center, which sounds safe).

**Cost.** Moderate. SAT primitive: ~100 lines + tests. Sweep-CPA for two
oriented rectangles in linear motion: ~150 lines + tests (the geometry
gets more interesting). Heading state needed (from velocity or from AIS).

**Dependency.** §14.2 (target geometry attributes wired into the
calculation). §14.7 (uncertainty) becomes more complex on top of this —
deferred until the deterministic hull-aware version is in place.

### 14.9 Heading errors and gyro drift in own-ship pose

Today every relative-bearing sensor (ARPA TTM in "relative" mode, EO/IR
bearing) is rotated into the ENU frame using the latest
`OwnShipPose.heading_true_deg`. The adapter does
`bearing_true = bearing_relative + own_ship_heading`. **This treats own-ship
heading as if it were known exactly**, which it isn't:

1. **Instantaneous bias.** A 1° gyro error on a target at 5 km range
   creates an 87 m cross-track position bias on the projected measurement.
   That's larger than typical nav-radar bearing noise — so heading error
   dominates the actual measurement error budget at long range.
2. **Drift over time.** Gyro bias of even 0.01°/s accumulates to 36° over
   an hour. AIS doesn't catch this (the AIS receiver doesn't know your
   heading); only an absolute reference (GPS course-over-ground,
   dual-antenna heading, magnetic compass with corrected declination)
   can. Without it, the entire fused track picture rotates around
   own-ship slowly and OSPA on synthetic scenarios — where heading is
   exact — wouldn't catch it.

Three things real maritime trackers do that we don't:

1. **Heading-error contribution to measurement R.** For any sensor that
   used own-ship heading in its projection, the adapter should add
   `(range × σ_heading)²` to the cross-track component of `R`. This
   makes the tracker correctly down-weight long-range relative bearings
   when heading is uncertain. Today `R` reflects only the sensor's own
   noise.
2. **Heading bias as a state.** Estimate gyro bias online from
   comparison of dead-reckoned vs GPS-COG / fix-derived heading; apply
   the bias correction to subsequent measurement projections.
   Conceptually a one-state extension of the tracker (or a separate
   own-ship navigation filter that feeds corrected heading downstream).
3. **Sensor orientation on the measurement.** Extension of §14.1: not
   just *where* was the sensor but *which way was it pointing* at
   measurement time. For bearings reported in body frame, the sensor's
   pointing matters; for bearings already true-North-referenced, it
   doesn't (own-ship heading absorbed the rotation).

**Unlocks.** Honest measurement covariances at long range, where
heading-induced cross-track error dominates sensor-intrinsic noise.
Detectability of gyro drift through the tracker (currently invisible
because synthetic scenarios assume perfect heading). Multi-platform
scenarios where each platform has its own gyro/heading source.

**Cost.** Heading-error R contribution: ~50 lines in
`adapters/util/Projection.cpp` plus a way to pass `σ_heading` from the
adapter config. ~50 lines. Heading bias as a state in a small
own-ship filter: ~200 lines. Sensor orientation field: ~30 lines + the
same threading work as §14.1.

**Dependency.** Useful in isolation but most impactful once §14.1 is in
heavy use (which it now is for bearing-only). Stacks cleanly with §14.2
(sensor offsets) and §14.5 (close-range precision sensors where heading
error at short range is small but not zero).

### 14.10 Static objects: track vessels, map the environment

**Decision of record: ADR 0002.** This section is the design sketch; the
scope decision and its rationale live in the ADR.

**The insight (measured, 2026-07-01; attribution refined 2026-07-02, R4).** The
chronic PMBM over-count on the philos replay — the phantom tracks the land prior
fights — is **dominated by persistent uncharted structure, and is a mixture, not
purely fixed infrastructure.** Checked against the *raw* radar (raw plots from an
offline intensity-threshold + DBSCAN chain, **not** ARPA tracks): all returns are
near-field (≤ ~976 m), only 1 of 23 AIS vessels is in radar range, and the ~1,940
non-AIS returns are persistent, extended, fixed structure (`n_cells` up to 6181;
piers/breakwaters). A motion test found zero coherent non-AIS *moving* boats — but
a 20 s clip **cannot exclude sub-1 m/s moored/drifting craft**, and an operator
Google-Maps field check confirmed at least one persistent cluster is anchored
boats. So the honest claim is: **persistent uncharted structure dominates; a
minority are moored craft indistinguishable from structure in 20 s — treat as a
mixture.** Either way philos is a **clutter-rejection / false-positive** test
where the "clutter" is overwhelmingly static environment. (The chart-coverage
field check — top-100 clusters 0 % within 75 m of a charted bridge, ~32 % outside
ENC coverage entirely, ~63 % unexplained by any chart layer — is the R4 evidence;
see `charts/philos_chart_coverage.{py,png}` and eval-log 2026-07-01/-02.)

**The scope rule: vessel-vs-environment, not moving-vs-stationary.**
- **Static-but-real vessels** (anchored/moored/drifting) are **in scope** as
  tracks. They are hazards, may get underway, and are already representable (the
  output contract's *"Confirmed stationary; direction undefined"* state; CV CPA
  reduces correctly for v=0). "Static" is a track attribute, never a delete
  reason.
- **Fixed infrastructure** is **out of scope as vessel tracks** — extended,
  velocity-less, chart-known; a point-target CV filter and center-to-center CPA
  are structurally wrong for it (feeding it to the point tracker *is* the
  over-count).

**Why it lives here (not downstream).** navtracker is the fusion hub — it holds
every sensor, the chart, own-ship pose, and the confirmed dynamic tracks. For an
**uncharted** static obstacle (a pillar on no chart) nothing downstream has more
information. So static-obstacle handling is a **separate branch inside
navtracker**, not another system.

**Architecture.** A distinct estimator (occupancy / evidential grid + chart
priors), *not* the same PMBM asked to do double duty:

```
 sensors + own-ship + chart ─┬─▶ vessel MTT (PMBM/MHT) ──▶ vessel tracks ─┐
                             │        ▲    │ 1−r labels                    ├─▶ world model ─▶ CPA / hazards
                             └─▶ static-obstacle branch ──▶ static hazards ┘
                                  (occupancy + chart prior)  ▲ birth prior ┘
```

Cross-feed both ways: vessel tracks label returns already explained by a
confident target (weight `1 − r`) so the static branch does not double-count
them; the static-occupancy estimate is a **vessel-birth prior** that suppresses
phantom tracks. This is the honest generalization of today's land prior (a
*chart-based* static prior) + clutter map (a crude *live* static-occupancy
estimator).

**Caveat — we move (the shore-station shortcut does not port).** Herrmann et
al. 2025 (arXiv:2508.16169), the closest published system, remove land with a
*precomputed median mask in a fixed geographic frame* — cheap, and correct
**only because their radar is a stationary shore station**. navtracker rides
own-ship, so any data-derived static layer must accumulate in a **world / geo
frame via own-ship pose**, not the radar body frame (a body-frame mask would
smear as the platform moves). This is exactly why the stage-2 static estimate is
a geo-referenced occupancy grid, not a borrowed body-frame mask. It also means
the *chart* prior (`StaticObstacle` / coastline, already geo-referenced) is the
right thing to lead Stage 1 with.

**Interfaces.**
- Keep `CoastlineGeometry` / `ILandModel` for **land** (a birth-suppression
  region).
- Add a separate **`StaticObstacle`** input for discrete charted hazards
  (point / small polygon + keep-clear radius + type/depth/lit/real-or-virtual
  AtoN attributes) — *not* folded into the coastline (folding creates tiny
  no-birth zones around each obstacle that suppress passing vessels, and loses
  attributes). **Schema-align it to the ENC / S-101 hazard ontology** (`CATOBS`,
  `WATLEV`, `VALSOU`/depth, `EXPSOU`, lit, real/synthetic/virtual-AtoN flag,
  positional-uncertainty buffer radius) so ENCs and AIS AtoN (Message 21) ingest
  directly. See ADR 0002 "Prior art & validation."
- A **static-hazard output** layer, combined with vessel tracks for the
  operator picture and the collision layer.

**Staging.**
1. **Stage 1a — SHIPPED 2026-07-01** (`StaticObstacle` chart input as birth
   prior + hazard output; see plan
   `docs/superpowers/plans/2026-07-01-static-obstacle-stage1.md` and ADR 0002).
   **Stage 1b — OPEN:** wire PMBM to feed the clutter-map primitive
   (dominant-hypothesis `1 − r` labeling, the parked design) and **emit it as a
   static-occupancy hazard**, not a hidden λ_C tweak.
2. **Stage 2 — OPEN:** a full Bayesian/evidential occupancy grid; a **stationary
   IMM mode** (low-PSD / zero-velocity pseudo-measurement) so moored tracks stay
   tight and CPA gets a clean stationary-hazard flag; **sensor/chart-aware
   near-shore birth** (ADR 0001 A3) to reliably initiate near-shore static
   *vessels*.

**Unlocks.** A clean vessel picture *and* static-obstacle awareness (incl.
uncharted) from one system; collapses the land prior + clutter map + philos
over-count into one coherent capability; closes the anchored-vessel safety gap
(a moored vessel near shore that today can be suppressed — ADR 0001).

**Cost.** Stage 1 is modest (chart input + reframe the existing clutter-map
primitive). Stage 2 (occupancy grid, stationary mode) is a substantial new
subsystem — a roadmap item, not a quick add.

**Dependency.** The sensor/chart-aware near-shore discriminator needs EO/IR or
AIS coverage near shore (ADR 0001 A3). Extended-object handling for large single
static returns overlaps §14.3 (EOT) and should reuse it rather than duplicate.

### 14.11 Populate `TrackOutput.contributing_sources` for PMBM (consumer-surface consistency) — RESOLVED 2026-07-15 (POPULATE)

**RESOLVED — POPULATE (arbiter ruling).** Implemented on branch
`pmbm-contributing-sources`: PMBM now fills `Track::contributing_sources` from the
F2-truthful claimed-source channel, cumulative and deduplicated, matching the
flat/MHT semantics (a sibling `bernoulli_sources_` map fed at the same
`last_claimed_meas_index`-gated site as `contribution_history_`, read in
`refreshAggregatedTracks`). Attributes-only: the R3 two-class A/B is byte-identical
on every accuracy/state/consistency metric (18 PMBM configs × 26 scenarios). The F2
invariant extends to the output field (`test_pmbm_contribution_provenance` +
`test_t2t_live_pedigree_content` PMBM-backed pin, both teeth-proven). Pedigree
stays diagnostics-only in fusion math. See
`docs/baselines/2026-07-15_pmbm_contributing_sources.md` and `pmbm-design.md`
§3.6.1. The original decision record follows.

**Observed 2026-07-15 (F2 provenance cycle).** `Track::contributing_sources` —
the per-track list of sensor streams that fed a track, and the field the T2T
self-adapter (`NavtrackerSource`) turns into a `SourcePedigree` — is populated by
the flat/MHT pipeline (per-update from the associated measurement's `source_id`)
but is left **empty by PMBM**, the deployable tracker. PMBM instead maintains the
richer `recent_contributions` (`SourceTouch`) provenance vector, which the F2 fix
made truthful. Nobody had flagged the gap because the field was distrusted until
F2 landed (§10 Rider B).

**The small decision.** Whether to derive `TrackOutput.contributing_sources` for
PMBM tracks from the now-truthful `recent_contributions`:
- **Leave empty (honest).** A PMBM-sourced `ExternalTrack` carries an
  all-`Unknown` pedigree → T2T independence conservatively falls back to
  `PossiblyCorrelated`. Costs fused-covariance tightness, never correctness.
- **Populate (useful).** Distinct sensors on a PMBM track would surface in the
  output attribute list and let the T2T self-adapter emit a genuine
  `ProvablyIndependent` verdict for disjoint-source PMBM tracks — the same
  benefit the flat/MHT path already gets.

**Cost.** Small: fold the claimed-source ids into `contributing_sources` in
`refreshAggregatedTracks` (dedup, alive-id semantics already present). Pedigree
stays diagnostics-only either way, so this is a usefulness improvement, not a
correctness fix. **Done** (see the RESOLVED banner above) — the "populate" arm was
chosen and shipped attributes-only.
