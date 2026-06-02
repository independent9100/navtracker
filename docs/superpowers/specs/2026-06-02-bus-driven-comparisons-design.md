# Bus-Driven Estimator/Associator Comparisons Design

**Date:** 2026-06-02
**Status:** Approved for implementation
**Related:**
- `docs/superpowers/specs/2026-06-01-simulated-sensor-bus-design.md` (SimulatedSensorBus, just shipped)
- `docs/algorithms/evaluation-log.md` (current eval-log with multi-seed sweep results)
- `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md` §14.9 (heading errors, still deferred)

## 1. Goal

Re-run the four head-to-head estimator/associator comparisons from our prior multi-seed sweep — JPDA vs GNN, IMM-3 vs single-mode CV, PF vs EKF (bearing-only), MHT vs JPDA — but with measurement streams produced by `SimulatedSensorBus` instead of synthetic direct-Measurement builders. Answer: do the demonstrated wins survive realistic multi-sensor noise, cadence variation, and adapter geometry?

Secondary goal: a fairer comparison metric. The bus emits ~10× more measurements than direct-Measurement scenarios; per-measurement OSPA over-weights track-init transients. We add a per-window OSPA helper so the metric counts world-state accuracy at a fixed cadence, independent of how many sensors fire in that window.

Out of scope: heading errors / gyro drift (§14.9 still deferred), new motion models, new estimators, new sensor types, new scenarios. v1 is exclusively about re-running the existing wins through the bus.

## 2. Prior wins to re-test

From `evaluation-log.md` after the multi-seed sweep:

| Comparison | Prior result (direct-Measurement, 20 seeds) | Bus expectation |
|---|---|---|
| JPDA vs GNN (clutter crossing) | JPDA wins: 45.32 ± 0.44 vs 47.32 ± 0.11 OSPA; ID switches 2.45 vs 9.90 | Should hold — JPDA's edge is clutter handling, which the bus now models on ARPA |
| IMM-3 vs CV (maneuvering) | IMM-3 wins: 4.81 ± 0.59 vs 5.67 ± 0.91 OSPA | Should hold — IMM advantage is mode-switching, independent of sensor model |
| PF vs EKF (bearing-only moving) | PF directional: 180 ± 125 vs 213 ± 125 (overlapping CIs) | Likely still directional; bus changes the source of bearings (EO/IR) but not the underlying geometry |
| MHT vs JPDA (clutter crossing) | Retracted: 1.97 ± 1.9 both (no demonstrated win) | Re-confirm retraction OR catch a context where MHT recovers |

## 3. Architecture invariants honoured

- **Test-layer-only.** All new code lives under `sim/`, `core/scenario/`, and `tests/sim/`. No changes to estimators, associators, or trackers.
- **Determinism.** Each comparison loops 20 seeds (range 201..220 — same as prior sweep). For each seed, the bus produces a byte-identical Scenario, and the two trackers consume it in deterministic order. Same-seed re-run yields identical aggregate.
- **Hexagonal.** Bus extensions stay in `sim/`. Metrics extension stays in `core/scenario/`. No leaks into the algorithmic core.

## 4. Infrastructure changes

### 4.1 `ManeuveringTrajectory`

`sim/TruthTrajectory.hpp` gains a new concrete `ITruthTrajectory`:

```cpp
class ManeuveringTrajectory final : public ITruthTrajectory {
 public:
  // Three-leg trajectory: straight at `velocity` from `start` for
  // `straight_duration_s`, then a constant-rate turn `omega_rad_s` for
  // `turn_duration_s` (positive = left turn in ENU), then straight for
  // another `straight_duration_s` at the post-turn heading and speed.
  // t0 anchors the first leg's start.
  ManeuveringTrajectory(Eigen::Vector2d start,
                        Eigen::Vector2d velocity,
                        double straight_duration_s,
                        double turn_duration_s,
                        double omega_rad_s,
                        Timestamp t0);
  TruthState eval(Timestamp t) const override;
 private:
  // implementation
};
```

**Math** (eval at relative time `τ = t - t0`):

- `0 ≤ τ < straight_duration_s`: `p = start + velocity * τ`, `v = velocity`.
- `straight_duration_s ≤ τ < straight_duration_s + turn_duration_s` (turn leg): let `τ' = τ - straight_duration_s`, `p_turn_start = start + velocity * straight_duration_s`, `θ = atan2(velocity.y, velocity.x)`, `r = |velocity| / |omega_rad_s|`. Position rotates around the centre `p_turn_start + r * [-sin(θ), cos(θ)] * sign(omega)`; velocity at `τ'` has magnitude `|velocity|` and angle `θ + omega_rad_s * τ'`.
- After the turn leg: `p = p_turn_end + v_post * (τ - turn_end)`, `v = v_post`.

`eval` clamps `τ < 0` to leg-1 start, `τ` past the end of leg 3 to constant motion continuation.

**Test** (`tests/sim/test_maneuvering_trajectory.cpp`) covers: t=0 returns start/velocity; end-of-leg-1 returns straight-line position; mid-turn position is on the arc; end-of-turn velocity has rotated by `omega * turn_duration_s`; post-turn position is linear in `velocity_post`.

### 4.2 ARPA clutter

`sim/ArpaEmitter.hpp` extends `ArpaEmitterConfig`:

```cpp
struct ArpaEmitterConfig {
  // existing fields...
  int clutter_per_rotation{0};      // mean N of uniform false alarms per rotation
  double clutter_min_range_m{50.0}; // clutter drawn in [min, max] range × [0, 360) bearing
};
```

Per emission cycle (at each rotation slot for any configured target), after emitting the real detection, draw `Poisson(clutter_per_rotation)` clutter detections — uniform range in `[clutter_min_range_m, max_range_m]`, uniform bearing in `[0, 360)` — and emit each as a `$RATTM` sentence with `arpa_track_num = 0` (or a designated clutter ID; for v1 use 0). Clutter measurements carry the same `range_std_m` / `bearing_std_deg` as real detections.

**Important determinism note:** the clutter loop must execute *exactly once per rotation slot* regardless of how many real targets fire. To preserve this, the emitter draws clutter on a separate fixed cadence — keyed off `cfg.rotation_dt_s` and a single `next_clutter_emit_` timestamp (analogous to EO/IR's single `next_emit_`). This decouples clutter from target presence and keeps RNG draws stable across configs.

**Test** (`tests/sim/test_arpa_clutter.cpp`): with `clutter_per_rotation = 5` and no real targets, over a 30 s run at 3 s rotation, expect ~50 clutter detections (10 rotations × Poisson(5) average); per-detection range falls within `[min, max]`; bearings cover `[0, 360)`.

### 4.3 Per-window OSPA

`core/scenario/Metrics.hpp` gains:

```cpp
struct PerWindowOspa {
  double mean;                       // average over windows
  double stddev;                     // sample stddev across windows
  std::vector<double> per_window;    // raw per-window means
};

// Group ScenarioStep entries by floor((step.time - t0) / window_dt_s).
// For each window, compute mean OSPA across the steps it contains.
// Empty windows are dropped (not counted toward the average).
PerWindowOspa computePerWindowOspa(const std::vector<ScenarioStep>& steps,
                                   Timestamp t0,
                                   double window_dt_s,
                                   double ospa_cutoff);
```

**Rationale.** Mean of means: each window contributes equally regardless of how many measurements fell in it. A bus tick with 4 sensors firing simultaneously produces 4 ScenarioStep entries; per-window OSPA averages them into one number per window, then averages windows uniformly. This is what we want when comparing across cadences.

**Test** (`tests/sim/test_per_window_ospa.cpp`): with steps at t = 0.1, 0.2, 1.1, 1.2, 1.3, 2.5 and OSPA values [10, 12, 5, 5, 8, 20], `window_dt_s = 1.0`, expect three windows with means [11, 6, 20] and overall mean = 12.33.

## 5. Comparison tests

Each comparison test file is structured the same way:

```cpp
namespace {

struct BusSweepResult {
  PerWindowOspa ospa;
  int total_id_switches;  // summed over seeds
};

BusSweepResult sweep(/*ctor fn for tracker A*/,
                     /*scenario config*/,
                     std::uint32_t seed_base, std::size_t n_seeds);

}  // namespace

TEST(BusComparison, JpdaVsGnnClutterCrossing) {
  const auto a = sweep(makeJpdaTracker, /*config*/{}, 201, 20);
  const auto b = sweep(makeGnnTracker, /*config*/{}, 201, 20);

  // Headlines printed to test output, gtest-style.
  std::cout << "JPDA  per-window OSPA: " << a.ospa.mean << " ± " << a.ospa.stddev
            << "  ID switches: " << a.total_id_switches << '\n';
  std::cout << "GNN   per-window OSPA: " << b.ospa.mean << " ± " << b.ospa.stddev
            << "  ID switches: " << b.total_id_switches << '\n';

  // Soft assertion that JPDA still wins on at least one of the two metrics.
  EXPECT_TRUE(a.ospa.mean <  b.ospa.mean
           || a.total_id_switches < b.total_id_switches)
      << "JPDA neither beats GNN on OSPA nor on ID-switch count";
}
```

**Specifics per comparison:**

| Test | Scenario builder (via bus) | Tracker A | Tracker B |
|---|---|---|---|
| `BusComparison.JpdaVsGnnClutterCrossing` | Crossing trajectories + ARPA `clutter_per_rotation = 5` | JPDA + EKF + CV | GNN + EKF + CV |
| `BusComparison.Imm3VsCvManeuvering` | `ManeuveringTrajectory` straight-turn-straight | IMM-3 (CV + CT pos / CT neg) | EKF + CV |
| `BusComparison.PfVsEkfBearingOnlyMoving` | Existing bearing-only-moving (reused from `test_bus_regression`) | PF (200 particles) | EKF + CV |
| `BusComparison.MhtVsJpdaClutterCrossing` | Same as JPDA test | MHT TOMHT | JPDA |

Seeds: 201..220 in every test (matches prior sweep). Each `sweep` constructs a fresh tracker per seed, runs the bus, runs `runScenario`/`runScenarioBatched` as appropriate, accumulates per-window OSPA and ID-switch count, returns aggregates.

**Test assertion philosophy:** these are *finding-tracking* tests, not pass/fail correctness gates. The `EXPECT_TRUE(a wins on at least one metric)` is loose; the real output is the printed numbers, which feed the eval-log update. If a test "fails" (loses on both metrics) we update the eval-log with a retraction, not silence the test.

## 6. Eval-log update

After all four comparison tests pass (or the assertion loosens to allow a retraction to be recorded), append a section to `docs/algorithms/evaluation-log.md`:

```markdown
## Bus-driven confirmation pass (2026-06-02)

Re-ran the four winning comparisons through SimulatedSensorBus to test
whether direct-Measurement wins survive realistic multi-sensor noise.
Metric: per-window OSPA (1 s windows) + cumulative ID-switch count
over 20 seeds (range 201..220, matching the prior sweep).

| Comparison | Direct-measurement (prior) | Bus-driven (this pass) | Verdict |
|---|---|---|---|
| ... fill in from test output ... |

Notes:
- Bus injects ARPA clutter (Poisson mean=5/rotation), 1 Hz GPS, SOTDMA AIS,
  10 Hz EO/IR with bearing+range. Heading bias deferred to §14.9.
- Per-window OSPA differs from prior per-measurement OSPA — the windows
  are 1 s wide and uniform-averaged, not weighted by the bus's higher
  measurement rate. Direct comparison to prior numbers is illustrative,
  not strict.
```

(Actual numbers filled from test output during execution.)

## 7. Implementation order

8 tasks, TDD throughout:

1. `ManeuveringTrajectory` + test.
2. ARPA clutter knob + test.
3. `perWindowOspa` helper + test.
4. JPDA vs GNN bus comparison (clutter crossing).
5. IMM-3 vs CV bus comparison (maneuvering).
6. PF vs EKF bus comparison (bearing-only-moving).
7. MHT vs JPDA bus comparison (clutter crossing).
8. Eval-log update with table of all four results.

## 8. Decisions recorded

| Decision | Choice | Why |
|---|---|---|
| Clutter source | ARPA emitter knob (not standalone emitter) | Real clutter lives on radar; one knob is simpler than a new component |
| Maneuver model | Straight-turn-straight (matches `buildManeuveringTargetScenario`) | Constant-rate turn is degenerate; mode transitions are the IMM's testbed |
| Window size for per-window OSPA | 1 s (matches `truth_sample_dt_s`) | One truth snapshot per window; smaller windows reintroduce cadence weighting |
| Test assertion strength | Soft (loses on both metrics = explicit failure) | These tests track findings; the eval-log is the artefact, not the green checkmark |
| Seed range | 201..220 (20 seeds, identical to prior sweep) | Direct-comparable; user-confirmed scope |
| Heading errors | Deferred to §14.9 follow-up | Keeps this plan focused on "do wins survive sensors" alone |
| Scenarios | Reuse prior winning scenarios (extended where needed) | User-confirmed scope; no new scenarios in v1 |
