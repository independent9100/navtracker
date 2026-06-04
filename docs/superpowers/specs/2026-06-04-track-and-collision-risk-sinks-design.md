# Track Lifecycle & Collision-Risk Sinks Design

**Date:** 2026-06-04
**Status:** Approved

This spec adds two new event-based ports — `ITrackSink` (track lifecycle) and `ICollisionRiskSink` (CPA-derived collision alerts) — and a `CpaEvaluator` that walks track pairs each cycle and emits risk events with hysteresis. Together they turn the library from pull-based to push-based for the two consumer concerns operators actually care about.

## 1. Problem

The library has accumulated rich state — track lifecycle in `TrackManager`, CPA-with-uncertainty math (`computeCpaWithUncertainty`) including `P(CPA < d_threshold)`, own-ship synthesis (`synthesizeOwnShipTrack`) — but no way to push events from it. A consumer who wants:

- to know when a new track appeared or an old one was deleted (a UI rendering tracks; a logger keeping replay diffs), OR
- to be alerted when a track-pair's CPA probability crosses a configured risk threshold (the operator console / collision alarm),

today has to poll `mgr.tracks()` every cycle, diff against their own prior snapshot, and re-run `computeCpaWithUncertainty` themselves for every (own-ship, track) pair. That's a lot of glue per consumer for state the library already has.

This spec adds the push surface.

## 2. Architecture

```
                                     +-----------------+
   measurement                       |  composition    |
       v                              |  root           |
   +---------+      +-----------+   +-+------+         |
   | Tracker |----->|TrackManager|--|TrackSink|--->    | consumers
   +---------+      +-----------+   +--------+         |  (UI, logger, …)
       ^                  ^                            |
       |    every cycle   |                            |
       |                  v                            |
   OwnShipProvider     CpaEvaluator --> CollisionRiskSink |
                       (walks pairs)                   |
                                     +-----------------+
```

- `TrackManager` becomes the single source of lifecycle events. It gains a nullable `ITrackSink*` and fires `onTrackInitiated / Confirmed / Updated / Deleted` on the transitions it already detects internally.
- `Tracker` notifies `TrackManager` of state-update transitions via a new `recordUpdated(id, t)` method, so `onTrackUpdated` flows through the same channel as the other lifecycle events.
- `CpaEvaluator` is a standalone class wired by the composition root. Each call to `evaluate(t)` synthesizes own-ship as a track, loops over confirmed tracks, computes `CpaPrediction`, and fires risk events with hysteresis (enter / update / exit).

No production-path semantics change. All sinks are nullable; null = no-op = today's behavior.

## 3. `ITrackSink`

```cpp
namespace navtracker {

struct TrackLifecycleEvent {
  TrackId id;
  Timestamp time;
  TrackStatus status;     // status AT the moment the event fires
};

class ITrackSink {
 public:
  virtual ~ITrackSink() = default;
  virtual void onTrackInitiated(const TrackLifecycleEvent&) = 0;
  virtual void onTrackConfirmed(const TrackLifecycleEvent&) = 0;
  virtual void onTrackUpdated(const TrackLifecycleEvent&) = 0;
  virtual void onTrackDeleted(const TrackLifecycleEvent&) = 0;
};

}  // namespace navtracker
```

**Event semantics:**

| Method | Fires when |
|---|---|
| `onTrackInitiated` | `TrackManager::add` returns a new id. `status = Tentative`. |
| `onTrackConfirmed` | `TrackManager::recordHit` transitions `Tentative → Confirmed`. Fires exactly once per track lifetime. |
| `onTrackUpdated` | `Tracker` succeeded in an `estimator.update(tr, z)` call. Fires every successful update — high rate, but consumers can ignore based on filter (e.g., status check). Provides "track state may have changed" notification. |
| `onTrackDeleted` | `TrackManager::recordMiss` reached the delete threshold and the track is about to be erased. Status reflects what it was before erasure. Fires before erasure so the consumer can inspect via `mgr.tracks()` one last time if needed. |

**Cardinality:** zero or one sink per `TrackManager`. Setter is `setTrackSink(ITrackSink*)`. To fan out, the consumer implements a multiplexing sink itself.

**Lifetime:** the manager stores the raw pointer; caller owns lifetime. Standard pattern (matches `IDatumChangeSink`, `IBearingInnovationSink`).

## 4. `ICollisionRiskSink`

```cpp
namespace navtracker {

enum class CollisionRiskTransition {
  Entered,   // P crossed enter_probability from below
  Exited,    // P fell below exit_probability OR track was deleted
  Updated,   // still risky; per-cycle state refresh
};

struct CollisionRiskEvent {
  CollisionRiskTransition transition;
  TrackId other;             // the non-own-ship track in the pair
  Timestamp time;
  CpaPrediction prediction;  // full CPA prediction at this moment
};

class ICollisionRiskSink {
 public:
  virtual ~ICollisionRiskSink() = default;
  virtual void onCollisionRisk(const CollisionRiskEvent& event) = 0;
};

}  // namespace navtracker
```

**Transition semantics** (per-pair state machine, default `NotRisky`):

| State (before) | Condition | Transition fired | State (after) |
|---|---|---|---|
| NotRisky | `P >= enter_probability` | Entered | Risky |
| NotRisky | `P < enter_probability` | (none) | NotRisky |
| Risky | `P >= exit_probability` | Updated (if `emit_updates`) | Risky |
| Risky | `P < exit_probability` | Exited | NotRisky |
| Risky | track no longer in `mgr.tracks()` | Exited | (state dropped) |

Hysteresis: `enter_probability >= exit_probability` (config-enforced). Default `0.5 / 0.3`.

`Updated` events are gated by `cfg_.emit_updates` (default false) to avoid spamming consumers. Operators who want a live console feed turn it on.

## 5. `CpaEvaluator`

```cpp
namespace navtracker {

struct CpaEvaluatorConfig {
  double d_threshold_m{500.0};
  double enter_probability{0.5};
  double exit_probability{0.3};
  bool   evaluate_tentative{false};   // by default only Confirmed tracks
  bool   emit_updates{false};         // per-cycle Updated events when Risky
};

class CpaEvaluator {
 public:
  CpaEvaluator(const TrackManager& manager,
               const OwnShipProvider& provider,
               CpaEvaluatorConfig cfg = {});

  void setSink(ICollisionRiskSink* sink) { sink_ = sink; }

  // Run one evaluation pass over the current confirmed tracks. Composition
  // root decides the cadence; typically called after Tracker.process.
  // If no own-ship pose is available, the call is a no-op.
  void evaluate(Timestamp t);

  // Diagnostics.
  std::size_t entered() const { return n_entered_; }
  std::size_t exited()  const { return n_exited_; }
  std::size_t updated() const { return n_updated_; }
  std::size_t riskyPairs() const { return state_.size(); }

 private:
  const TrackManager& manager_;
  const OwnShipProvider& provider_;
  CpaEvaluatorConfig cfg_;
  ICollisionRiskSink* sink_{nullptr};

  // Pair state: a track id is present here iff its last evaluation was Risky.
  std::unordered_set<std::uint64_t> state_;

  std::size_t n_entered_{0};
  std::size_t n_exited_{0};
  std::size_t n_updated_{0};
};

}  // namespace navtracker
```

**Logic of `evaluate(t)`:**

1. If `provider.latest()` empty → no-op.
2. Synthesize own-ship as a `Track` via `synthesizeOwnShipTrack(*pose, t, provider)`.
3. For each track in `manager_.tracks()` matching the status gate (`Confirmed` only by default):
   - Compute `pred = computeCpaWithUncertainty(own, track, t, cfg_.d_threshold_m)`.
   - Apply the per-pair state machine (§4).
   - If a transition fires AND `sink_ != nullptr`, dispatch.
4. For each id in `state_` that wasn't seen in step 3 (track deleted or fell out of status gate): fire `Exited`, drop from `state_`.

**Diverging tracks:** evaluated like any other. If the pair is past CPA but still close enough to push `P >= enter_probability`, the operator gets the alert. The CPA math's `is_diverging` flag is exposed via `pred` for the consumer to display, not filtered here.

## 6. Tracker integration

`Tracker` already mutates `Track` via `estimator_.update(tr, z)`. Add one call afterward:

```cpp
estimator_.update(tr, z);
manager_.recordUpdated(tr.id, z.time);   // NEW
```

This is the only Tracker change. The new manager method:

```cpp
void TrackManager::recordUpdated(TrackId id, Timestamp t) {
  if (sink_ == nullptr) return;
  const int i = index(id);
  if (i < 0) return;
  TrackLifecycleEvent e{id, t, tracks_[i].status};
  sink_->onTrackUpdated(e);
}
```

Note: the manager does NOT do any state mutation in `recordUpdated`. The `recordHit` call (which the Tracker also makes) still handles M-of-N counter bookkeeping and confirm transitions. `recordUpdated` is purely an event fire.

## 7. TrackManager integration

`TrackManager` grows:

```cpp
void setTrackSink(ITrackSink* sink) { sink_ = sink; }
```

And fires events at the right points:

| Method | Event |
|---|---|
| `add(track, first_observation)` | `onTrackInitiated(id, first_observation, Tentative)` |
| `recordHit(id)` when `hits >= confirm_hits_` and the status was Tentative on entry | `onTrackConfirmed(id, lastObservation(id), Confirmed)` |
| `recordMiss(id)` when `misses >= delete_misses_` | `onTrackDeleted(id, lastObservation(id), prev_status)` BEFORE erasing |

`onTrackUpdated` is plumbed from `recordUpdated` (§6).

## 8. Assumptions

1. Sinks are stateless from the manager/evaluator's perspective. The library never reads them back. Consumers can hold state inside the sink for fan-out or filtering.
2. Sink callbacks are synchronous — fired on the calling thread. Consumers that need async dispatch implement that inside the sink.
3. The composition root calls `CpaEvaluator::evaluate(t)` at whatever cadence makes sense; nothing automatic. Typical: once per `Tracker::process()` call, using `z.time` as the eval time.
4. `synthesizeOwnShipTrack` returns a track with id `TrackId{0}` (sentinel). Evaluator skips any track with `id.value == 0` to avoid pairing own-ship with itself — only relevant if a buggy consumer adds id 0 to the manager.
5. Events fire **after** the underlying state mutation completes (e.g., `onTrackConfirmed` after the status flip, `onTrackDeleted` BEFORE erasure — only deviation, called out above).

## 9. Out of scope (deferred)

- **Inter-target CPA** (track-vs-track collision risk between two non-own-ship targets). This spec only evaluates own-ship × each track. Generalization is straightforward but multiplies event volume by N².
- **Per-event severity classification** beyond probability. Could later add `severity = f(P, tcpa, range)` for operator-friendly grading.
- **Subscriber-driven evaluation cadence.** Today the composition root calls `evaluate(t)`; an alternative is automatic eval after every Tracker.process. Adding that later is a one-line wire-up; we keep the explicit call for clarity.
- **CollisionRisk event for own-ship itself going risky with multiple targets simultaneously.** Each pair is independent. A "multi-threat" indicator can be derived by the consumer.
- **Snapshot semantics on `onTrackDeleted`.** Today the event carries `TrackId + Timestamp + status` only. Including the last known kinematic state would force a deep copy on every delete. Consumers can subscribe to `onTrackUpdated` to track the latest state per id themselves.

## 10. Validation

### 10.1 `ITrackSink` lifecycle (`tests/tracking/test_track_sink.cpp`)

- **U-LIFE-1:** Add a track → `onTrackInitiated` fires once with `Tentative` status.
- **U-LIFE-2:** `recordHit` to confirm-threshold → `onTrackConfirmed` fires once; subsequent hits don't refire.
- **U-LIFE-3:** `recordMiss` to delete-threshold → `onTrackDeleted` fires once BEFORE erasure (verified by sink looking up `mgr.tracks()` size at fire time).
- **U-LIFE-4:** Tracker `process` matching a confirmed track → `onTrackUpdated` fires.
- **U-LIFE-5:** Null sink → no crashes on any of the above.

### 10.2 `CpaEvaluator` hysteresis (`tests/collision/test_cpa_evaluator.cpp`)

- **U-CPA-1:** Two tracks heading toward collision (own-ship east at 5 m/s, other west at 5 m/s, on collision course) → at t=t_close, `evaluate(t)` fires `Entered`.
- **U-CPA-2:** After `Entered`, repeated `evaluate(t)` at same instant should NOT refire Entered (idempotent).
- **U-CPA-3:** Tracks pass each other, `P` drops below exit threshold → `Exited` fires.
- **U-CPA-4:** Track gets deleted while Risky → `Exited` fires for that pair on the next `evaluate(t)`.
- **U-CPA-5:** `emit_updates = true` → while Risky, each `evaluate(t)` fires `Updated`.
- **U-CPA-6:** No own-ship pose pushed → `evaluate(t)` is a no-op; no events.
- **U-CPA-7:** `evaluate_tentative = false` (default): Tentative tracks are skipped even if Risky-looking.

### 10.3 Backward compat regression

All existing tracking / pipeline / collision tests must pass unchanged with default null sinks.

## 11. Files

| Action | Path |
|---|---|
| Create | `ports/ITrackSink.hpp` |
| Create | `ports/ICollisionRiskSink.hpp` |
| Modify | `core/tracking/TrackManager.hpp/.cpp` — sink, event firing in add/recordHit/recordMiss; new `recordUpdated` |
| Modify | `core/pipeline/Tracker.cpp` — call `recordUpdated` after successful update (both `process` and `processBatch` hard branch) |
| Create | `core/collision/CpaEvaluator.hpp/.cpp` |
| Create | `tests/tracking/test_track_sink.cpp` |
| Create | `tests/collision/test_cpa_evaluator.cpp` |
| Modify | `CMakeLists.txt` — wire new core sources and test sources |

## 12. Rationale

| Decision | Considered | Chosen | Why |
|---|---|---|---|
| Single ITrackSink with 4 methods | 4 separate sink interfaces | One | Lifecycle is a coherent concept; fan-out done by the consumer. |
| TrackManager owns lifecycle events; Tracker proxies updates via `recordUpdated` | Tracker has its own sink | Manager-centric | Single source of truth for lifecycle state; matches existing `recordHit/recordMiss` pattern. |
| Standalone `CpaEvaluator` (not auto-fired from Tracker) | Auto-fire after each `Tracker.process` | Standalone | Composition root may want different cadence (e.g. CPA every 5th cycle for cost); explicit is clearer. |
| Hysteresis via `enter_probability` and `exit_probability` | Single threshold | Hysteresis | Avoids event chatter on noisy P near threshold. |
| `Updated` events behind `emit_updates` flag (default off) | Always fire | Behind flag | Default-off prevents spam; consumers that need a live feed opt in. |
| Per-pair state via `unordered_set<uint64_t>` | Map to richer state | Set | Boolean "is risky" is all we need; smaller and faster. |
| `evaluate_tentative=false` default | True | False | Operators care about confirmed targets; tentative tracks are pre-decision noise. Opt-in to evaluate them. |
| Own-ship synthesized per evaluation | Cache as a member | Per-call | Pose changes between cycles; recompute is cheap; caching adds invalidation complexity. |
| Sinks are nullable, default null | Constructor argument | Nullable | Backward compat; matches existing sink patterns; composition-root wiring is optional. |
| `onTrackDeleted` fires BEFORE erasure | After (with snapshot) | Before | Consumer can still inspect `mgr.tracks()` if they want last-known state; no deep copy in the event. |

## 13. Acceptance

- Full suite green (target: existing + ~12 new tests across the two new test files).
- Lifecycle events fire on the right transitions; no event refires for the same logical transition.
- CPA hysteresis prevents event chatter; Exited fires both on probability drop AND on track deletion.
- Backward compat: all prior tracking/pipeline/collision tests pass unchanged.
- No changes to `IHeadingBiasProvider`, `IBearingInnovationSink`, `Measurement`, `OwnShipPose`, or any consumer of the existing pull-based `mgr.tracks()` accessor.

## 14. What changes in consumer experience

- A UI consumer subscribes to `ITrackSink`, draws on `onTrackInitiated/Confirmed`, redraws on `onTrackUpdated`, removes on `onTrackDeleted`. No polling.
- An operator alarm subscribes to `ICollisionRiskSink`, plays a sound on `Entered`, dismisses on `Exited`. Tuned via `enter_probability`/`exit_probability` to match the operator's risk tolerance.
- A replay log writer subscribes to both sinks and emits a deterministic event stream alongside the existing pull-based state, useful for offline diff and regression testing.
- Library users who never call `setTrackSink` or `setSink` on the evaluator: zero behavior change.
