# ReorderBuffer End-to-End Validation Design

**Date:** 2026-06-04
**Status:** Approved
**Scope:** A — `ReorderBuffer` stays an *optional* library primitive. No production-path wiring. Validation proves the primitive's claims end-to-end with realistic clock skew.

## 1. Problem

`core/pipeline/ReorderBuffer` currently has two trivial unit tests and zero scenario or integration coverage. The system claim "live and replay use the same core; replay of a log must be deterministic" (CLAUDE.md §invariant 4) is *only* defensible if a consumer who pipes skewed inputs through the buffer gets the same deterministic, accurate output every time. That guarantee is currently unproven.

This spec adds a realistic-skew scenario test and a small `SkewInjector` sim utility so the guarantee is mechanically checked.

## 2. Architecture (no production-path changes)

```
sim emitters → Measurement stream                       (truth-ordered)
              ↓
        SkewInjector (per-sensor lag + jitter, seeded)   (arrival-ordered)
              ↓
        ReorderBuffer(window=W)                          (truth-ordered again)
              ↓
        Tracker                                          (same as baseline)
```

The injector and buffer live *outside* `Tracker`. The baseline path bypasses both. Consumers who never see skew never pay for ordering.

## 3. SkewInjector — math and contract

### 3.1 Math

Given a measurement with `Measurement.time = t_truth` and `Measurement.sensor = k`:

- `arrival_time(m) = t_truth + lag_k + jitter_k` where:
  - `lag_k` is the constant per-sensor offset for sensor kind `k`;
  - `jitter_k ~ Uniform(-J_k, +J_k)` drawn from a single seeded `std::mt19937_64`;
  - all draws are deterministic in (seed, ingestion order).

Measurements are emitted in **arrival-time order** (stable sort on `arrival_time`, ties broken by original ingestion index). **`Measurement.time` is never mutated** — it remains the truth timestamp the buffer sorts on.

### 3.2 Profile table

| `SensorKind` | `lag` (s) | `jitter` (±s) |
|---|---|---|
| `Ais` | 0.50 | 1.00 |
| `ArpaTtm`, `ArpaTll` | 0.05 | 0.05 |
| `EoIr` | 0.15 | 0.05 |
| `OwnShip` | 0.00 | 0.02 |
| `Lidar`, `Unknown` | 0.00 | 0.00 |

Profile is a value type (`SkewProfile`) with `std::array<std::pair<double,double>, 7>` indexed by `SensorKind`. Consumers can override per kind.

### 3.3 Assumptions

- Truth timestamps survive transport unmodified (no clock drift inside the sensor; the buffer relies on this).
- Skew is bounded by `max(lag_k + J_k) < W` where `W` is the `ReorderBuffer` window; otherwise late-arrival drops are *expected* and asserted.
- Jitter is symmetric and independent per measurement — adequate for VDL contention and frame-bus latency at the scenario timescales we test (tens of seconds). Burst-correlated lag is out of scope.
- The injector is deterministic in (seed, input order). Replaying the same sim with the same seed yields the same arrival sequence.

### 3.4 Rationale

- **Stable sort on `arrival_time` (not a sample-and-drop queue)**: the realistic transport model is "every packet arrives, just out of order." Drop is the buffer's job, not the injector's.
- **Per-sensor `lag + uniform jitter` (not Gaussian, not Poisson)**: matches the observed envelope of maritime feeds well enough to surface ordering bugs without claiming a calibrated noise model. Pessimistic `J_AIS = 1.0 s` covers the worst VDL bursts we've seen in practice.
- **Seeded RNG, not platform RNG**: required for the determinism invariant.
- **Lives in `sim/`, not `core/`**: it's a test-only stimulus, not a runtime component, and `sim/` is already excluded from the library-consumer link targets.

### 3.5 Ways to improve / what to test next

- **Burst-correlated lag** — model VDL contention as a Markov chain over (idle / burst-of-K) states. Would surface drop policies under realistic AIS storms.
- **Per-source-id skew** — today we key on `SensorKind`. Adding `source_id` keying lets us model two AIS receivers with different lags.
- **Calibrated profiles** — replace pessimistic defaults with measured distributions when sea-trial logs land.
- **Statistical late-drop heuristic in `ReorderBuffer`** — instead of a hard window, drop on `P(late | observed history) > threshold`. Worth comparing OSPA against the fixed-window baseline.

## 4. Invariants asserted by the e2e test

In strict order, each as a separate `EXPECT_*` or assertion:

### I-1. Determinism under skew (the headline)

Same `(scenario, skew seed, window)` run twice produces byte-identical track outputs. We compare the `ScenarioResult.steps` sequence: same step count, same per-step timestamp, same per-step `TrackSnapshot` set (id + position equal to `Eigen` exactness — not within tolerance, *equal*).

### I-2. Drop accounting

For a measurement `m` with `arrival_time(m) - t_truth > W`, the buffer must report it dropped. Concretely we sweep two windows per scenario:

- **Comfortable window** `W = 4·max_jitter + 2·max_lag`. With the profile above that is `W = 4·1.0 + 2·0.5 = 5.0 s`. Expect `dropped == 0`.
- **Tight window** `W = 0.5·max_lag = 0.25 s` (deliberately under the AIS lag floor). Expect `dropped == ground_truth_late_count`, where ground truth is computed by re-running the injector and counting `arrival_time - t_truth > W`.

### I-3. Accuracy parity vs. ordered baseline

For the **comfortable-window** run:

- Position RMSE within `max(5% relative, 0.5 m absolute)` of the un-skewed baseline.
- Velocity RMSE within `max(5% relative, 0.05 m/s absolute)` of the baseline.
- Track ID assignment sequence (the ordered list of `(creation_time, track_id, truth_id_associated)`) is identical to the baseline.

RMSE is computed by `core/scenario/Metrics.hpp` aggregations, extended with a small helper if needed.

### I-4. Monotonic drain output

Every call to `ReorderBuffer.drain()` returns a vector that is non-decreasing in `Measurement.time`. Asserted inside the test harness on every drain call across the scenario.

## 5. Scenarios

Two existing scenario shapes, re-driven through the skew path:

- **Crossing** (mirrors `tests/scenario/test_crossing.cpp`): two targets crossing at right angles, 40 ticks, dense observations from all sensor kinds.
- **AIS dropout** (mirrors `tests/scenario/test_ais_dropout.cpp`): single target with an AIS gap from t=5s to t=12s. Exercises the case where dropped AIS must not be conflated with late AIS.

Per scenario × per window setting, three runs:

1. **Baseline.** Ordered, no buffer. Reference numbers for I-3.
2. **Skewed-buffered run #1.** Through `SkewInjector(seed=42)` → `ReorderBuffer(W)` → `Tracker`. Asserts I-2, I-3, I-4.
3. **Skewed-buffered run #2 (replay).** Same as #2, fresh state. Compared byte-identically to #2 for I-1.

## 6. Files

| Action | Path | Notes |
|---|---|---|
| Create | `sim/SkewInjector.hpp` | Public API + `SkewProfile` |
| Create | `sim/SkewInjector.cpp` | Stable-sort emit, seeded RNG |
| Create | `tests/sim/test_skew_injector.cpp` | Unit tests: determinism, order preservation per sensor, ground-truth lateness |
| Create | `tests/scenario/test_reorder_buffer_e2e.cpp` | The three runs × two scenarios |
| Modify | `core/pipeline/ReorderBuffer.hpp` | Expand header doc to the four-part standard (math/assumptions/rationale/improvements) |
| Modify | `CMakeLists.txt` | Register `SkewInjector` in `navtracker_sim`; register new test sources |

No changes to `Tracker`, no changes to adapters, no changes to `Measurement`.

## 7. Determinism guards (implementation notes)

- `SkewInjector` uses `std::mt19937_64` with a 64-bit seed constructor argument. No `std::random_device`, no clock-based seeding.
- Stable sort, not `std::sort`, so equal arrival times preserve insertion order. (Otherwise `std::sort` could pick different orderings on different libcs.)
- `std::uniform_real_distribution<double>` is **not** portable across STLs. To keep tests bit-exact across platforms we draw integer jitter ticks from `mt19937_64` and convert: `jitter = (rng() % (2*N+1) - N) * (J_k / N)` with `N = 1024`. This trades a fixed quantization for cross-platform reproducibility, well below the timescales the buffer cares about.
- `Timestamp` arithmetic stays in `int64_t` nanos throughout — no float drift in arrival times.

## 8. Documentation deliverables

- `sim/SkewInjector.hpp` header doc — four-part: math (§3.1), assumptions (§3.3), rationale (§3.4), ways to improve (§3.5).
- `core/pipeline/ReorderBuffer.hpp` header doc — currently one line, expand to four-part: math (window cutoff, monotonic drain), assumptions (truth-time integrity, bounded skew), rationale (why fixed-window over alternatives), ways to improve (statistical drop, per-source windows, reorder-on-drain).

## 9. Out of scope (deferred, listed so we don't forget)

- Wiring `ReorderBuffer` into `Tracker` or any adapter (Scope A excludes this).
- Per-source windows.
- Pose interpolation across skew (Layer-3 of the time-mismatch work — already deferred in `2026-06-03-library-friendliness-design.md`).
- Statistical late-drop heuristic in the buffer itself.
- Calibrated skew profiles from real logs.

## 10. Acceptance

This work is done when:

- Full suite green (currently 342/342 → target 342 + new tests).
- The four invariants I-1..I-4 pass on both scenarios.
- `ReorderBuffer.hpp` and `SkewInjector.hpp` carry the four-part doc.
- No changes to production-path code (Tracker, adapters, Measurement) beyond doc.
