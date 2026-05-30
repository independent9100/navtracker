# Pipeline & Time

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec sections 6 and 7.

## 1. Reorder buffer (`ReorderBuffer`)

**Math/Logic.** Track `latest = max(time pushed)`. On `push(m)`: if
`m.time < latest − window` drop (count); else insert into a time-ordered
container, update `latest`. On `drain()`: pop in time order everything with
`time ≤ latest − window`.

**Assumptions.** Source timestamps are trusted; `window` ≥ worst expected
reorder skew; one-shot drain per cycle.

**Rationale.** Decouples message arrival from processing; gives live and replay
the same release semantics so the engine is deterministic (spec D2, D4).

**Ways to improve / test next.** Per-source latency calibration; OOSM /
retrodiction update instead of dropping; bounded total size with overflow
drop; multi-sensor scan grouping inside the window.

## 2. Single-measurement orchestration (`Tracker`)

**Math/Logic.** For each released `Measurement z`:
1. `predictAll(estimator, z.time)`.
2. `result = associator.associate(tracks, {z})`.
3. If matched: `estimator.update(track, z)`; `recordHit`; `noteObservation`;
   add `z.source_id` to provenance if absent.
4. Else: `estimator.initiate(z)` → `add` to manager.
5. Maintenance: for every track, if `z.time − last_observation > miss_timeout`
   then `recordMiss` (Coasting → Deleted via the lifecycle state machine).

**Assumptions.** One measurement per `process` call; ≤1 track matches a given
measurement; `predict` is a no-op for `dt ≤ 0`; the source-of-truth time base
is `Measurement.time`, never wall-clock.

**Rationale.** Single-message processing matches the asynchronous, multi-rate
reality of the sensor mix (spec §6). A timeout-based miss policy avoids the
classic mistake of marking every unrelated track as missed on every message,
which would shred healthy tracks.

**Ways to improve / test next.** Group simultaneous measurements into per-time
scans for joint association; OOSM retrodiction; pre-association MMSI/sensor-
track-ID hint locking; batch updates when multiple sensors agree.
