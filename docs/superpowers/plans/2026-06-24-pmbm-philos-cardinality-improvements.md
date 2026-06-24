# PMBM philos cardinality improvements — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce PMBM's over-counting (spurious/duplicate confirmed tracks) on the real-world `philos` replay so its GOSPA/OSPA stop regressing vs MHT, without regressing the autoferry wins — and make the philos benchmark itself measure tracker quality more honestly.

**Architecture:** PMBM's RFS core is mathematically sound; the defects live in its *detection / birth / existence* model. We fix the birth side (over-confident births), the death side (a mathematically-wrong miss-probability that is currently load-bearing), the clutter side (shore returns), give PMBM the coverage channel MHT already has, and finally fix two benchmark-validity issues (AIS-only truth; three-cutoff metric reporting).

**Tech Stack:** C++17 (no later standard), CMake + Conan, GoogleTest. Hexagonal architecture — `core/` has zero I/O; loaders live in `adapters/`.

---

## READ FIRST — Why `compute_miss_pD` is "critical" but is NOT a standalone fix

You will see in the review that I marked **two PMBM defects as the worst**:

1. **`compute_miss_pD` non-dedup** (`core/pmbm/PmbmTracker.cpp:552-567`) — *critical, math-wrong.*
2. **Detection vs misdetection use inconsistent P_D in one update** — *high, math-wrong.*

…and yet neither appears as its own "go fix this" item. Here is the plain-language reason.

**What the bug is.** When a target is *not* seen in a scan, PMBM lowers that track's "I still exist" probability. The amount it lowers depends on the detection probability `P_D`. The correct model says: *one sensor sweep = one chance to detect*. The buggy code instead multiplies the penalty **once for every blip in the scan**. A radar sweep that produces 10 blips is treated as "the target dodged detection 10 separate times," so the existence probability is hammered ~10× too hard (effective `P_D` jumps from 0.07 to ~0.5–0.8). The code comment literally says *"The math is wrong."*

**Why it's critical.** It is wrong **and** the whole tracker is leaning on it. It is a load-bearing crack: this over-aggressive death penalty is the *only* thing currently stopping PMBM from drowning in phantom tracks.

**Why you must NOT "just fix it."** This was already tried (`PHILOS-T1`, `PHILOS-T2` in the eval log). Dedup-ing the penalty so the math is correct makes philos **+92% GOSPA worse**, not better. Reason: PMBM mints too many new tracks in the first place (over-confident births — Task 1) and has no proper way to kill the bad ones. The wrong penalty was secretly doing that killing. Remove the wrong brake and the car (phantom count) runs away.

**Analogy.** It's a load-bearing wall built out of code. The wall is cracked and against code (the math is wrong). You can't just knock it down — the roof (cardinality control) sits on it. You first build a proper support beam (correct births + a real existence/cardinality control), *then* replace the cracked wall. Knocking it out alone collapses the building (+92%).

**So the fix exists — it's Task 2** — but it is a *bundle*, not a one-liner: correct the births (Task 1), add per-vessel identity (Task 2a), correct the miss-probability (Task 2b), and add a legitimate cardinality control to replace the brake (Task 2c) — **all measured together**. The "inconsistent P_D" defect (#2) is the *same root cause*: once Task 2b makes the miss-branch use one `P_D` per sensor, the detect-branch (which already uses one `P_D`) and the miss-branch automatically agree. It needs no separate task.

---

## How the tasks fit together (dependency graph & ordering)

```
Task 1  Principled birth existence (target r_new)      ← do FIRST, cheap, standalone
Task 6  Independent truth loader (radar_truth.csv)     ← do EARLY, cheap, may change the verdict
Task 7  GOSPA decomposition + coverage reporting       ← do EARLY, makes every later A/B legible
Task 3  Radar clutter map into PMBM                    ← standalone, attacks shore-return births
Task 2  Correct miss-P_D BUNDLE (2a+2b+2c)             ← needs Task 1 landed first; the big campaign
Task 4  PMBM coverage/visibility channel               ← larger; the principled version of Task 2c
Task 5  Per-track-hypothesis refactor                  ← largest; separate multi-day spec already exists
```

Recommended order: **1 → 6 → 7 → 3 → 2 → (4, 5 later)**. Tasks 1, 3, 6, 7 are independent and each shippable alone. Task 2 is a tuning campaign that assumes Task 1 is in. Tasks 4–5 are larger follow-ups.

**Why 6 and 7 are early and high value:** Task 6 may show that some of PMBM's "phantoms" are *real radar targets missing from the AIS-only truth* — i.e. the regression is partly a scoring artifact. Task 7 makes "is GOSPA moving because of misses or false tracks?" visible, so every A/B after it is interpretable instead of a single opaque number.

---

## Shared protocol — how to build and bench philos

All tasks use this. Run from the project root (`/home/andreas/workspace/navtracker`).

**Build the bench (incremental):**
```bash
cmake --build build --target navtracker_bench_baseline -j
```
> If the build fails with a sandbox/permission error (e.g. `~/.conan2` read-only, "Operation not permitted"), re-run the **same** command with the sandbox disabled.

**Run only philos for the PMBM configs (fast: K=1 + MHT ~1 min; K=3 ~6 min):**
```bash
./build/bench/navtracker_bench_baseline \
  --scenario-filter philos \
  --config-filter pmbm \
  --out "$TMPDIR" --run-id philos_probe
```
- `--scenario-filter` / `--config-filter` are **substring** matches.
- `--config-filter pmbm` runs the 3 PMBM configs (`imm_cv_ct_pmbm`, `_adapt`, `_adapt_k3`) on philos. Give any **new** config a label containing a unique token (e.g. `imm_cv_ct_pmbm_birthtarget`) and filter on that token to run just it.
- Output CSV lands in `$TMPDIR/<run-id>...csv` (long format: `config,scenario,seed,metric,value,unit`).

**Reference numbers to beat (philos, from `docs/baselines/pmbm_vs_mht_review_20260624.md`):**
| config | gospa_mean | ospa_mean | pos_rmse_m | mean est tracks/tick |
|---|---:|---:|---:|---:|
| `imm_cv_ct_mht` (target) | **69.43** | 424.61 | 25.58 | ~19.5 |
| `imm_cv_ct_pmbm_adapt` (baseline to beat) | 82.63 | 454.80 | 12.90 | ~28.85 |
| truth | — | — | — | ~11.35 |

**Success for a PMBM change = philos `gospa_mean` drops below 82.63 (ideally toward 69.43) while autoferry does not regress.** Autoferry guard (run before promoting any change to a shipped config):
```bash
./build/bench/navtracker_bench_baseline \
  --scenario-filter autoferry --config-filter <your-token> \
  --out "$TMPDIR" --run-id autoferry_guard
```
PMBM currently beats MHT by ~42% GOSPA on autoferry-unanchored; a change that recovers philos but loses that is not shippable as canonical (document it as an ablation instead).

---

## Global Constraints

- **C++17 only** — no later standard.
- **Hexagonal:** `core/` and `ports/` have **zero I/O**. CSV loaders go in `adapters/`. PMBM math stays in `core/pmbm/`.
- **Determinism:** the replay determinism test must stay green (same input → identical output). No wall-clock, no RNG in the hot path.
- **Algorithm-doc standard (REQUIRED):** any change to PMBM math (Tasks 1, 2, 4) must update the four sections (Math / Assumptions / Rationale / Ways-to-improve) in `docs/algorithms/pmbm-design.md` **and** the plain-English chapter `docs/learning/23-pmbm.md`.
- **Config defaults are backward-compatible:** every new `PmbmTracker::Config` field defaults to the legacy behavior (a new knob set to 0/false/empty must be bit-identical to today). Unit tests of bare predict/update must not change.
- **No new third-party dependency** without adding it through Conan and noting it.

---

### Task 1: Principled birth existence (target r_new)

**Reasoning.**
PMBM decides how confident a *newly born* track is with `r_new = λ_birth / (λ_birth + λ_C)` (`PmbmTracker.cpp:462-463`), where `λ_C` is the clutter density and `λ_birth` is a tuning scalar. The intent (Reuter 2014, and the Config comment at `Config.cpp:561-572`) is that new tracks start with a **small** `r_new ≈ 0.09` and *earn* confidence over the next few detections. That keeps clutter from instantly becoming a confirmed track.

The bug: `λ_birth` is a fixed **absolute** number (`1e-5`), tuned when `λ_C ≈ 1e-4`. But `λ_C` is a property of the sensor *and the scenario*. On philos it is **2.7e-6 (radar)** and **1e-9 (AIS)** — 8× to 10000× smaller. Plug those in: `r_new = 1e-5/(1e-5+2.7e-6) = 0.79` for radar and `≈1.0` for AIS. So on philos **every** ungated radar blip (including the persistent shore/structure returns the descriptor warns about) is born *already above* `confirm_threshold = 0.5` and emitted as Confirmed in one scan. That is the over-counting engine.

The fix is to make the design intent — *a small constant `r_new`* — actually hold regardless of scenario, by deriving `λ_birth` from the live `λ_C` so `r_new` hits a fixed target. A per-sensor *scalar* `λ_birth` (already plumbed but untuned, `PHILOS-T11`) cannot do this because one scalar still can't be right for both philos `λ_C` and autoferry `λ_C`. A *ratio/target* parameterization can: set `λ_birth = (r*/(1−r*))·λ_C` and `r_new` is exactly `r*` everywhere.

**Why this isn't already-rejected:** the rejected `PHILOS-T3` sweep varied the absolute scalar `{1e-3,1e-4,1e-5}` *against a single `λ_C`*. This task changes the **parameterization** so `r_new` is `λ_C`-invariant — a different lever.

**Files:**
- Modify: `core/pmbm/PmbmTracker.hpp` (add Config field near line 164-165, by `lambda_birth`).
- Modify: `core/pmbm/PmbmTracker.cpp:455-463` (`buildAdaptiveBirthCandidates`).
- Modify: `core/benchmark/Config.cpp` (add a probe config; ~after the `imm_cv_ct_pmbm_adapt` block at line 583).
- Test: `tests/pmbm/test_pmbm_tracker_update.cpp` (add a case; mirror the estimator/Config boilerplate already in that file).
- Docs: `docs/algorithms/pmbm-design.md`, `docs/learning/23-pmbm.md`.

**Interfaces:**
- Produces: `PmbmTracker::Config::birth_existence_target` (double, default 0.0 = legacy). When `> 0`, the adaptive-birth path yields `r_new == birth_existence_target` for every initiable measurement, independent of `λ_C`.

- [ ] **Step 1: Write the failing test.** In `tests/pmbm/test_pmbm_tracker_update.cpp`, add:

```cpp
TEST(PmbmTrackerUpdate, BirthExistenceTargetIsClutterInvariant) {
  // r_new must equal birth_existence_target regardless of clutter_intensity.
  for (double lambda_c : {1e-9, 2.7e-6, 1e-4}) {
    auto est = makeTestEstimator();  // same helper the other tests use
    pmbm::PmbmTracker::Config cfg;
    cfg.adaptive_birth = true;
    cfg.measurement_driven_birth = false;
    cfg.smart_birth_skip_existing = false;
    cfg.min_new_bernoulli_existence = 0.0;  // don't gate the birth away
    cfg.clutter_intensity = lambda_c;       // no detection model → this is λ_C
    cfg.birth_existence_target = 0.2;
    pmbm::PmbmTracker tracker(*est, cfg);

    Measurement z = makePosition2DMeasurement(/*x=*/100.0, /*y=*/50.0,
                                              Timestamp::fromSeconds(1.0));
    tracker.processBatch({z});

    double r_max = 0.0;
    for (const auto& h : tracker.density().mbm)
      for (const auto& b : h.bernoullis)
        r_max = std::max(r_max, b.existence_probability);
    EXPECT_NEAR(r_max, 0.2, 1e-6) << "lambda_c=" << lambda_c;
  }
}
```
> If `makeTestEstimator` / `makePosition2DMeasurement` aren't the exact helper names in that file, use whatever the neighboring tests use to build an `IEstimator` and a Position2D `Measurement` — copy their setup verbatim.

- [ ] **Step 2: Run it, verify it fails.**
```bash
cmake --build build --target navtracker_pmbm_tests -j && \
  ./build/tests/navtracker_pmbm_tests --gtest_filter='*BirthExistenceTargetIsClutterInvariant*'
```
Expected: FAIL — `birth_existence_target` doesn't compile yet (unknown field), or `r_max` ≈ 1.0/0.79 not 0.2.
> The test target name may differ; find it with `grep -rn pmbm tests/CMakeLists.txt` or `ctest -N | grep -i pmbm`.

- [ ] **Step 3: Add the Config field.** In `core/pmbm/PmbmTracker.hpp`, immediately after `double lambda_birth = 1e-3;` (line ~165), add:

```cpp
    // Clutter-invariant birth existence (Task 1, 2026-06-24). When > 0,
    // the adaptive-birth path derives λ_birth from the per-measurement
    // λ_C so the new-target existence is exactly r_new = this value,
    // independent of the sensor/scenario clutter density:
    //   λ_birth = (r*/(1−r*))·λ_C  ⇒  r_new = λ_birth/(λ_birth+λ_C) = r*.
    // Fixes the philos over-confident-birth bug: a fixed *absolute*
    // λ_birth gives r_new ≈ 0.79 (radar) / ≈1.0 (AIS) on philos because
    // λ_C there is 2.7e-6 / 1e-9, not the 1e-4 the scalar was tuned for.
    // 0 = legacy (use lambda_birth / lambda_birth_per_sensor). Typical
    // 0.1–0.3 so real targets ramp via posterior over later detections.
    double birth_existence_target = 0.0;
```

- [ ] **Step 4: Implement in the birth path.** In `core/pmbm/PmbmTracker.cpp`, replace the `λ_birth` selection block at lines 455-463 with:

```cpp
    double lambda_birth = cfg_.lambda_birth;
    if (cfg_.birth_existence_target > 0.0) {
      // Clutter-invariant: choose λ_birth so r_new == target for this z.
      const double r = cfg_.birth_existence_target;
      lambda_birth = (r / (1.0 - r)) * lambda_z;
    } else if (!cfg_.lambda_birth_per_sensor.empty()) {
      auto it = cfg_.lambda_birth_per_sensor.find(z.sensor);
      if (it != cfg_.lambda_birth_per_sensor.end()) {
        lambda_birth = it->second;
      }
    }
    cand.rho_target = lambda_birth;
    cand.rho_total = lambda_birth + lambda_z;
```
> `lambda_z` is already computed just above (line 402-404) as the per-measurement `λ_C`. This keeps `r_new = rho_target/rho_total = lambda_birth/(lambda_birth+lambda_z) = target` exactly.

- [ ] **Step 5: Run the test, verify it passes.**
```bash
cmake --build build --target navtracker_pmbm_tests -j && \
  ./build/tests/navtracker_pmbm_tests --gtest_filter='*BirthExistenceTargetIsClutterInvariant*'
```
Expected: PASS for all three `lambda_c` values.

- [ ] **Step 6: Add a probe bench config.** In `core/benchmark/Config.cpp`, after the `imm_cv_ct_pmbm_adapt` block (ends ~line 583), add a copy that sets the target:

```cpp
  // Task 1 probe: clutter-invariant birth existence. Same as
  // imm_cv_ct_pmbm_adapt but r_new is pinned to a target instead of a
  // fixed absolute λ_birth — fixes the philos over-confident-birth bug.
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_birthtarget";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.adaptive_k_best = false;
      cfg.k_best_per_hypothesis = 1;
      cfg.lambda_birth = 1e-5;          // ignored when target > 0
      cfg.birth_existence_target = 0.1; // <-- the knob under test
      cfg.min_new_bernoulli_existence = 0.05;
      return cfg;
    };
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
```

- [ ] **Step 7: Bench philos and sweep the target.** Build + run:
```bash
cmake --build build --target navtracker_bench_baseline -j
./build/bench/navtracker_bench_baseline --scenario-filter philos \
  --config-filter birthtarget --out "$TMPDIR" --run-id t1_philos
```
Read the philos `gospa_mean` / `ospa_mean` / `pos_rmse_m` for `imm_cv_ct_pmbm_birthtarget` from the CSV. Repeat with `birth_existence_target ∈ {0.05, 0.1, 0.2, 0.3}` (edit Step 6, rebuild, rerun).
**Decision rule:** keep the target value with the lowest philos `gospa_mean` that is **< 82.63**. If none beats 82.63, Task 1 alone is insufficient — its real payoff is as the *births* half of the Task 2 bundle; record the numbers and proceed.

- [ ] **Step 8: Autoferry guard + docs.** Run the autoferry guard command (shared protocol) for `--config-filter birthtarget`. Confirm autoferry `gospa_mean` is not worse than `imm_cv_ct_pmbm_adapt`. Update the four doc sections in `docs/algorithms/pmbm-design.md` (birth model) and `docs/learning/23-pmbm.md`.

- [ ] **Step 9: Commit.**
```bash
git add core/pmbm/PmbmTracker.hpp core/pmbm/PmbmTracker.cpp core/benchmark/Config.cpp \
        tests/pmbm/test_pmbm_tracker_update.cpp docs/algorithms/pmbm-design.md docs/learning/23-pmbm.md
git commit -m "PMBM: clutter-invariant birth existence target (philos over-birth fix)"
```

---

### Task 2: Correct the misdetection model — the BUNDLE (2a + 2b + 2c)

**Reasoning.**
This is the fix for the *critical* `compute_miss_pD` defect explained at the top. It MUST be done as a bundle and measured as a bundle — `PHILOS-T1/T2` proved the dedup alone gives **+92% GOSPA**. The bundle has three parts:

- **2a — per-vessel identity for the source-aware gate.** `should_misdetect` (`PmbmTracker.cpp:489-499`) is meant to skip the death penalty for a track when *no sensor that ever saw it* is in this scan. It keys on `source_id`, but every AIS vessel is loaded as `source_id="ais"` (`AisCsvReplayAdapter.cpp:143`), so it can't tell vessels apart. We add a **separate per-vessel identity** (from `Measurement::hints.mmsi`, already populated) used *only* by PMBM's gate. **Do NOT change the AIS `source_id` string** — MHT's correct miss-dedup (`TrackTree.cpp:177-186`) collapses all `"ais"` to one detection opportunity, and per-vessel `source_id` would silently break MHT (finding MHT-2). Keep `source_id="ais"`; add identity alongside.
- **2b — per-scan dedup of `compute_miss_pD`.** Compute the effective miss-`P_D` **once per distinct `(sensor, model, source_id)`** present in the scan, not once per measurement. This is the textbook "one sweep = one detection opportunity" model and makes the detect-branch and miss-branch use the same `P_D` (fixes the inconsistency defect too).
- **2c — a legitimate cardinality control to replace the brake.** Once 2b removes the over-penalty, you need a *correct* way to stop phantoms: raise `output_existence_floor` and/or `r_min`, tighten `min_new_bernoulli_existence`, possibly shorten `idle_halflife_sec`. Task 1 (correct births) is the precondition that makes this tractable.

**Files:**
- Modify: `core/types/Track.hpp` (add `std::optional<std::uint32_t> vessel_id;` to `struct SourceTouch`, ~line 55-58).
- Modify: `core/pmbm/PmbmTracker.cpp` — touch population (~line 1206-1209), `should_misdetect` (485-499), `compute_miss_pD` (552-567).
- Modify: `core/benchmark/Config.cpp` — a bundle probe config.
- Test: `tests/pmbm/test_pmbm_tracker_update.cpp`.
- Docs: `docs/algorithms/pmbm-design.md`, `docs/learning/23-pmbm.md`.

**Interfaces:**
- Produces: `Track::SourceTouch::vessel_id` (optional uint32). `PmbmTracker::Config::source_aware_identity` (bool, default false = legacy channel-keyed gate).

**Step group 2a — per-vessel identity (do first, commit on its own):**

- [ ] **2a.1 Write the failing test.** A PMBM tracker with `source_aware_identity=true`, two AIS Bernoullis with different `vessel_id`s; feed a scan containing only vessel A's broadcast (with `hints.mmsi = A`); assert vessel B's existence is **unchanged** (its identity not in scan) while vessel A's updates. (Mirror the construction in existing tests; set `hints.mmsi` on the measurements.)

- [ ] **2a.2 Run, verify fail.** (B decays today because the gate sees `"ais"` present.)

- [ ] **2a.3 Add the field.** In `core/types/Track.hpp`, inside `struct SourceTouch` (after `std::string source_id;`):
```cpp
    std::optional<std::uint32_t> vessel_id;  // per-vessel identity (e.g. MMSI),
                                             // used by PMBM source-aware gate.
```
(Ensure `#include <optional>` and `#include <cstdint>` are present.)

- [ ] **2a.4 Populate it at the touch site.** In `core/pmbm/PmbmTracker.cpp` where the touch is built (~line 1206, after `touch.source_id = best->source_id;`):
```cpp
      touch.vessel_id = best->hints.mmsi;  // per-vessel identity when present
```

- [ ] **2a.5 Add the Config flag + identity-aware gate.** In `PmbmTracker.hpp` near `source_aware_misdetection` add `bool source_aware_identity = false;`. In `PmbmTracker.cpp`, extend the scan-source collection and `should_misdetect` (lines 485-499):
```cpp
  std::set<std::string> scan_sources;
  std::set<std::uint32_t> scan_vessels;
  if (cfg_.source_aware_misdetection) {
    for (const auto& z : scan) {
      scan_sources.insert(z.source_id);
      if (cfg_.source_aware_identity && z.hints.mmsi.has_value())
        scan_vessels.insert(*z.hints.mmsi);
    }
  }
  auto should_misdetect = [&](BernoulliId id) {
    if (!cfg_.source_aware_misdetection) return true;
    auto it = contribution_history_.find(id);
    if (it == contribution_history_.end() || it->second.empty()) return true;
    for (const auto& touch : it->second) {
      if (cfg_.source_aware_identity && touch.vessel_id.has_value()) {
        if (scan_vessels.count(*touch.vessel_id)) return true;  // this vessel is in-scan
        continue;  // identity known but absent → this touch gives no coverage
      }
      if (scan_sources.count(touch.source_id)) return true;     // channel fallback
    }
    return false;
  };
```

- [ ] **2a.6 Run test, verify pass. Commit** (`PMBM: per-vessel identity for source-aware misdetection (behind flag)`).

**Step group 2b — per-scan dedup of `compute_miss_pD`:**

- [ ] **2b.1 Add a flag** `bool dedup_miss_pd = false;` to `PmbmTracker::Config`.
- [ ] **2b.2 Rewrite `compute_miss_pD`** (`core/pmbm/PmbmTracker.cpp:552-567`) to dedup by `(sensor,model,source_id)` when the flag is on:
```cpp
  auto compute_miss_pD = [&](const Bernoulli& b) {
    if (!detection_model_) return cfg_.probability_of_detection;
    if (b.mean.size() < 2) return cfg_.probability_of_detection;
    const Eigen::Vector2d b_xy = b.mean.head<2>();
    double survive = 1.0;
    bool any_coverage = false;
    if (cfg_.dedup_miss_pd) {
      // Textbook: one detection opportunity per distinct sensor channel
      // that surveyed this scan, NOT one per return.
      std::set<std::tuple<SensorKind, MeasurementModel, std::string>> seen;
      for (const auto& z : scan) {
        auto key = std::make_tuple(z.sensor, z.model, z.source_id);
        if (!seen.insert(key).second) continue;  // already counted this channel
        const double pD_s = detection_model_->missDetectionProbability(
            z.sensor, z.model, b_xy, z.sensor_position_enu, z.source_id);
        if (pD_s > 0.0) { any_coverage = true; survive *= (1.0 - pD_s); }
      }
    } else {
      for (const auto& z : scan) {  // legacy per-measurement (buggy) path
        const double pD_s = detection_model_->missDetectionProbability(
            z.sensor, z.model, b_xy, z.sensor_position_enu, z.source_id);
        if (pD_s > 0.0) { any_coverage = true; survive *= (1.0 - pD_s); }
      }
    }
    return any_coverage ? (1.0 - survive) : 0.0;
  };
```
(Add `#include <tuple>` and `#include <set>` if not present.)

- [ ] **2b.3 Unit test:** a 5-return single-sensor radar scan with `P_D=0.07` must give effective miss-`P_D == 0.07` with `dedup_miss_pd=true` (not `1−0.93^5≈0.30`). Assert via the existence delta on a missed Bernoulli, or expose `compute_miss_pD` through a small test seam if needed. Commit.

> **DO NOT bench 2b on philos by itself** — it will show ~+92% GOSPA. That is expected and is the trap. It is only meaningful inside the 2c bundle.

**Step group 2c — cardinality control + the bundle A/B:**

- [ ] **2c.1 Add a bundle config** `imm_cv_ct_pmbm_bundle` in `Config.cpp` that turns on **all** of: Task 1 `birth_existence_target=0.1`, `source_aware_identity=true`, `dedup_miss_pd=true`, plus the cardinality control knobs to sweep: start `output_existence_floor=0.3`, `r_min=1e-3`, `min_new_bernoulli_existence=0.1`.
- [ ] **2c.2 Sweep** the three control knobs on philos (shared protocol, `--config-filter bundle`). Grid: `output_existence_floor ∈ {0.2,0.3,0.5}`, `min_new_bernoulli_existence ∈ {0.05,0.1,0.2}`. Track philos `gospa_mean` **and** mean est-tracks/tick (Task 7 makes the latter a metric; until then infer from `ospa`/cardinality).
- [ ] **2c.3 Decision rule:** accept the bundle setting whose philos `gospa_mean` is **lowest and < 82.63** AND whose autoferry guard (shared protocol) is **not worse** than `imm_cv_ct_pmbm_adapt`. If philos improves but autoferry regresses, ship `imm_cv_ct_pmbm_bundle` as an ablation (not canonical) and record the tradeoff in the eval log.
- [ ] **2c.4 Docs + commit.** Update `pmbm-design.md` (misdetection model now textbook-correct) and `docs/learning/23-pmbm.md`. Commit the bundle.

---

### Task 3: Wire the radar spatial clutter map into PMBM

**Reasoning.**
Half of philos's phantoms are persistent **shore / moored-structure** radar returns (`ReplayScenarioRun.cpp:78-81` says most unmatched plots are *not* Poisson clutter). A spatial clutter map learns a higher `λ_C` on the cells where those returns keep appearing. Higher `λ_C` directly lowers `r_new = λ_birth/(λ_birth+λ_C)` on shore cells, so phantoms are born weaker and don't confirm. This attacks the problem at the **birth rate**, orthogonally to Tasks 1/2, so it's lower-risk.

`ClutterMapSensorDetectionModel` already exists and is wired into MHT (`imm_cv_ct_mht_cmap`) but **into no PMBM config**. The wiring point is one hardcoded `false` in `Sweep.cpp:276`.

**Files:**
- Modify: `core/benchmark/Sweep.cpp:276` (use the config flag instead of hardcoded `false`).
- Modify: `core/benchmark/Config.cpp` (add `imm_cv_ct_pmbm_cmap` config with `use_clutter_map=true`).
- Docs: note in `docs/algorithms/pmbm-design.md` (clutter model).

**Interfaces:**
- Consumes: `Config::use_clutter_map` (bool, already exists at `Config.hpp:62`), `detectionModelFor(desc, carrier, use_clutter_map)` (already exists, `Sweep.cpp:169-184`).

- [ ] **Step 1: Make PMBM honor the flag.** In `core/benchmark/Sweep.cpp:276`, change:
```cpp
          auto det = detectionModelFor(desc, carrier, /*use_clutter_map=*/false);
```
to:
```cpp
          auto det = detectionModelFor(desc, carrier, config.use_clutter_map);
```

- [ ] **Step 2: Add the config.** In `core/benchmark/Config.cpp`, after the Task-1 / bundle configs, add a copy of `imm_cv_ct_pmbm_adapt` with `use_clutter_map=true`:
```cpp
  {
    Config c;
    c.label = "imm_cv_ct_pmbm_cmap";
    c.build_estimator = &makeImmCvCt;
    c.build_associator = &makeJpda;
    c.tracker_kind = TrackerKind::Pmbm;
    c.pmbm_config = []() {
      auto cfg = makePmbmConfig();
      cfg.adaptive_birth = true;
      cfg.k_best_per_hypothesis = 1;
      cfg.lambda_birth = 1e-5;
      cfg.min_new_bernoulli_existence = 0.05;
      return cfg;
    };
    c.use_clutter_map = true;  // radar position map suppresses shore births
    c.build_sensor_bias_estimator = []() {
      return std::make_shared<SensorBiasEstimator>();
    };
    configs.push_back(std::move(c));
  }
```

- [ ] **Step 3: Build, bench philos.**
```bash
cmake --build build --target navtracker_bench_baseline -j
./build/bench/navtracker_bench_baseline --scenario-filter philos \
  --config-filter pmbm_cmap --out "$TMPDIR" --run-id t3_philos
```
**Decision rule:** keep if philos `gospa_mean` < 82.63 and autoferry guard (shared protocol) not worse. The clutter map is data-driven, so also confirm the determinism test still passes (`ctest -R determinism` or the equivalent name from `ctest -N`).

- [ ] **Step 4: Commit.**
```bash
git add core/benchmark/Sweep.cpp core/benchmark/Config.cpp docs/algorithms/pmbm-design.md
git commit -m "PMBM: allow radar spatial clutter map (suppress shore-return births on philos)"
```

---

### Task 4: Give PMBM a coverage / visibility channel (sensor-activity port)

**Reasoning.**
The deepest structural gap: MHT has an explicit IPDA+VIMM **visibility** channel that distinguishes "target not detected because it's gone" from "target not detected because the sensor wasn't looking there / wasn't active this scan." PMBM's `compute_miss_pD` only sees the returns that *did* arrive (`PmbmTracker.cpp:558`), so a sensor that is **in coverage but silent this scan** (between sweeps, blind sector) contributes no misdetection at all — and conversely the `idle_halflife_sec` hack (`PmbmTracker.cpp:510-522`) is an ad-hoc non-Bayesian stand-in for the same missing concept. This is the principled version of Task 2c: instead of tuning floors, model coverage honestly.

This is a **design + code task**, larger than 1–3. It introduces a port telling the tracker which sensors were *active and in-coverage* at a scan time, so a quiet-but-active in-coverage sensor correctly contributes one misdetection.

**Files:**
- Create: `ports/ISensorActivity.hpp` — interface: `bool isActive(SensorKind, const std::string& source_id, Timestamp) const;` and coverage already comes from `ISensorDetectionModel::missDetectionProbability` (range/sector).
- Modify: `core/pmbm/PmbmTracker.{hpp,cpp}` — in the miss branch, after the per-scan dedup (Task 2b), additionally fold in any sensor that `isActive` at this scan time and covers the Bernoulli's position but produced no matching return.
- Modify: `adapters/benchmark/ReplayScenarioRun.cpp` — supply a philos activity model (radar active continuously over its sweep cadence; AIS modeled as a per-vessel broadcast, i.e. NOT a surveillance sensor → contributes no misdetection for *other* vessels, which is the correct cure for issue 2 at the model level).
- Docs: `docs/algorithms/pmbm-design.md`, `docs/learning/23-pmbm.md` (new visibility subsection + a coverage diagram), `docs/learning/00-index.md` if a new concept chapter is warranted.

**Reasoning detail for the executor.** The key modeling statement: **AIS is not a surveillance sensor.** A surveillance sensor (radar) that is active and pointed at a region either detects a present target (with `P_D`) or misses it (with `1−P_D`) — so a silent-but-active radar *should* contribute a misdetection. AIS is per-vessel broadcast: vessel B's silence says nothing about vessel A, so AIS should contribute a misdetection for a track **only** when *that track's own vessel* was expected to broadcast and didn't. Encoding this in the activity port removes the need for both the `source_id="ais"` hack and the `idle_halflife` hack.

- [ ] **Step 1:** Write the port `ports/ISensorActivity.hpp` (zero I/O; pure interface). Document the surveillance-vs-broadcast distinction in the header.
- [ ] **Step 2:** Add `setSensorActivity(const ISensorActivity*)` to `PmbmTracker` (nullable; null = today's behavior).
- [ ] **Step 3 (TDD):** Unit test — a radar that `isActive` and covers a Bernoulli but has no return this scan applies exactly one misdetection; an AIS "sensor" never applies a misdetection to a *different* vessel's Bernoulli. Write the test, see it fail, implement the miss-branch fold, see it pass.
- [ ] **Step 4:** Supply the philos activity model in `ReplayScenarioRun.cpp`; bench philos; A/B vs the Task-2 bundle. **Decision rule:** if it matches or beats the bundle with *fewer* tuning knobs, prefer it (it lets you drop `idle_halflife_sec` and the `source_aware_*` hacks). Autoferry guard before promotion.
- [ ] **Step 5:** Docs (four-section update + learning chapter/diagram) + commit.

> Scope note: this is multi-day. Do Tasks 1–3 (and ideally 2) first; only reach for Task 4 if the floor-tuning bundle (2c) proves too fragile across scenarios.

---

### Task 5: Per-track-hypothesis structural refactor (largest; spec already exists)

**Reasoning.**
PMBM stores Bernoullis as a *flat list per global hypothesis*. MATLAB MTT-master (and the literature) store *per-track lists of single-target hypotheses*. The flat structure is why K=3 needs the `cross_parent_birth_id_cache` patch and why several output-merge gates couldn't separate phantom alt-births from legitimate close-parallel tracks (`PHILOS-T8/T9/T10`). This refactor is the principled fix for the K>1 anchored regressions and would also make the lineage of every Bernoulli explicit (so phantom births are identifiable by construction, not by tuning).

This already has a written design: **`docs/superpowers/specs/2026-06-23-pmbm-phase9-per-track-hypotheses.md`** and a feature flag stub `use_per_track_hypotheses` (`PmbmTracker.hpp:332`).

**This task = execute that spec.** It is ~1000 LOC, 3–5 days, and out of scope for a quick philos fix. Do it only after Tasks 1–3 land and only if you want K=3 to be canonical.

- [ ] **Step 1:** Read `docs/superpowers/specs/2026-06-23-pmbm-phase9-per-track-hypotheses.md` end to end.
- [ ] **Step 2:** Write a dedicated implementation plan from that spec (re-invoke the writing-plans skill). Do **not** inline it here — it is its own multi-task project.

---

### Task 6: Independent truth loader (`radar_truth.csv`) — highest information value

**Reasoning.**
Two benchmark-validity problems make philos's numbers hard to trust:
1. **AIS is both the measurement and the truth** (`ReplayScenarioRun.cpp:107-138`). Scoring a track against the same AIS sample that just updated it is a leak that *flatters* PMBM's localization (the −49% pos_rmse is optimistic, `WIRE-5`).
2. **Truth is AIS-only (~11 vessels), but the radar legitimately sees more** (shore aside, non-cooperative vessels). GOSPA charges those legitimate detections as "false tracks." Part of PMBM's "over-count" may be **correct tracking of real targets that aren't in the truth set.**

There is an **independent, radar-derived truth already in the fixture**: `tests/fixtures/philos/out/ais_ferry_near/radar_truth.csv` (columns `tod,uid,range_m,azimuth_deg`, `uid` = MMSI). It "exists but has no loader" (`docs/baselines/README.md:290`). Wiring it gives an *independent* truth (kills the leak) and a *fuller* truth (kills the spurious false-track charge for real radar targets). This single experiment can tell you how much of the regression is real vs. a scoring artifact — which should arguably gate how much effort Tasks 2/4 deserve.

**Files:**
- Create: `adapters/replay/RadarTruthCsvReader.{hpp,cpp}` — load `radar_truth.csv` body-frame range/azimuth, project to ENU via `OwnShipProvider` (mirror `loadPlotCsvBodyFrame` in `adapters/replay/PlotCsvReplayAdapter.cpp`), emit `std::vector<TruthSample>` with `truth_id = uid`, `position = projected ENU`, `time = tod`.
- Modify: `adapters/benchmark/ReplayScenarioRun.cpp` — in `PhilosScenarioRun::generate`, behind a member flag (e.g. `truth_source_`), build truth from radar_truth (independent) or AIS∪radar_truth (fuller) instead of AIS-only.
- Modify: `adapters/benchmark/ReplayScenarioRun.cpp::defaultReplayScenarios()` — optionally register a second philos variant `philos_radartruth`.
- Test: `tests/replay/` — a loader test that the projected ENU positions are finite and within the fixture's range bound (~1 km), and `truth_id`s match the known MMSIs.

**Interfaces:**
- Produces: `std::vector<TruthSample> loadRadarTruthCsv(const std::string& path, OwnShipProvider& provider);` (in `namespace navtracker::replay`). `TruthSample` is `{Timestamp time; uint64_t truth_id; Eigen::Vector2d position;}` (`core/scenario/Truth.hpp:14-17`).

- [ ] **Step 1: Write the loader test (failing).** In `tests/replay/test_radar_truth_loader.cpp`:
```cpp
TEST(RadarTruthLoader, ProjectsBodyFrameToEnuWithinRange) {
  const char* kOwnship = "tests/fixtures/philos/out/ais_ferry_near/ownship.csv";
  const char* kRadarTruth = "tests/fixtures/philos/out/ais_ferry_near/radar_truth.csv";
  if (!std::ifstream(kRadarTruth)) GTEST_SKIP() << "fixture absent";
  auto poses = navtracker::replay::loadOwnshipCsv(kOwnship);
  ASSERT_FALSE(poses.empty());
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);
  auto truth = navtracker::replay::loadRadarTruthCsv(kRadarTruth, provider);
  ASSERT_FALSE(truth.empty());
  for (const auto& t : truth) {
    EXPECT_TRUE(t.position.allFinite());
    EXPECT_LT(t.position.norm(), 5000.0);   // within a few km of datum
    EXPECT_NE(t.truth_id, 0u);              // uid parsed
  }
}
```

- [ ] **Step 2: Run, verify fail** (loader doesn't exist). Build the replay test target (`grep -rn radar_truth tests/CMakeLists.txt` — you'll need to add the new test file to `CMakeLists.txt` near the other `tests/replay/` entries, e.g. by `tests/replay/test_philos_ospa.cpp` at `CMakeLists.txt:277`).

- [ ] **Step 3: Implement the loader.** Create `adapters/replay/RadarTruthCsvReader.{hpp,cpp}`. Copy the CSV-parsing + body-frame projection from `adapters/replay/PlotCsvReplayAdapter.cpp::loadPlotCsvBodyFrame` (it already turns `range_m, azimuth_deg` + ownship pose into ENU). Difference: emit a `TruthSample` (not a `Measurement`), set `truth_id = uid`, `position = projected ENU`, `time = Timestamp::fromSeconds(tod)`. **Watch the azimuth convention** — reuse the exact same convention `loadPlotCsvBodyFrame` uses for `radar_plots.csv` (same producer, same convention).

- [ ] **Step 4: Run test, verify pass.**

- [ ] **Step 5: Wire the truth-source flag.** In `PhilosScenarioRun`, add a member `enum class TruthSource { AisOnly, RadarOnly, Union } truth_source_{TruthSource::AisOnly};` and in `generate()` build `s.truth` accordingly (still run `resampleTruthToClock(s.truth, 1.0, 30.0)` afterward). Register a `philos_radartruth` variant in `defaultReplayScenarios()` using `RadarOnly` (independent truth, kills the leak).

- [ ] **Step 6: Bench both philos variants for MHT and PMBM.**
```bash
cmake --build build --target navtracker_bench_baseline -j
./build/bench/navtracker_bench_baseline --scenario-filter philos \
  --config-filter "imm_cv_ct" --out "$TMPDIR" --run-id t6_truth
```
Compare PMBM-vs-MHT `gospa_mean` on `philos` (AIS truth) vs `philos_radartruth` (independent/fuller truth).
**Interpretation:** if PMBM's GOSPA gap vs MHT **shrinks or flips** under the independent truth, a large part of the "regression" was the AIS-only truth penalizing real radar tracks — that reframes Tasks 2/4 priority. Record this in the eval log either way.

- [ ] **Step 7: Commit.**
```bash
git add adapters/replay/RadarTruthCsvReader.hpp adapters/replay/RadarTruthCsvReader.cpp \
        adapters/benchmark/ReplayScenarioRun.cpp tests/replay/test_radar_truth_loader.cpp CMakeLists.txt
git commit -m "bench: independent radar-derived truth for philos (kills AIS truth/measurement leak)"
```

---

### Task 7: GOSPA decomposition + coverage reporting (makes every A/B legible)

**Reasoning.**
Today philos reports one opaque `gospa_mean`. You cannot tell whether a change helped by *finding more real tracks* or hurt by *adding phantoms* — both move GOSPA the same direction. GOSPA decomposes exactly (Rahmathullah 2017) into **localization + missed + false** sub-costs. Emitting those three, plus a per-tick **cardinality error** (`|est| − |truth|`) and a **coverage ratio**, turns every later A/B (Tasks 1–4) from guesswork into a clear read. This also de-conflates the three-cutoff reporting issue (`MET-3`): you'll see directly that "better RMSE, worse GOSPA" is misses, not localization.

**Files:**
- Modify: `core/scenario/Gospa.{hpp,cpp}` — add an overload returning the decomposition (keep the scalar one for back-compat).
- Modify: `core/benchmark/Metrics.{hpp,cpp}` — compute & store `gospa_localization`, `gospa_missed`, `gospa_false`, `card_err_mean` (mean `|est|−|truth|`).
- Modify: `core/benchmark/Sweep.cpp::emit` (~line 94-110) — emit the new metric rows.
- Test: `tests/scenario/` GOSPA test — a hand-built case where 1 matched (d known), 1 missed, 1 false; assert the three sub-costs equal the closed form.

**Interfaces:**
- Produces: `struct GospaComponents { double total, localization, missed, false_; int n_missed, n_false; };` and `GospaComponents gospaComponents(truth, est, cutoff, p, alpha);`.

- [ ] **Step 1: Failing test.** In the GOSPA test file (find it: `grep -rln gospa tests/`), add a case with truth = `{(0,0),(100,0)}`, est = `{(3,0),(500,500)}`, `c=20,p=2,alpha=2`. Closed form: one match at d=3 (`loc = 3^2 = 9`), one missed truth `(100,0)` (`c^2/alpha = 200`), one false est `(500,500)` (`c^2/alpha = 200`). Assert `localization≈9`, `missed≈200`, `false_≈200`, `n_missed==1`, `n_false==1`.
- [ ] **Step 2: Run, verify fail** (overload doesn't exist).
- [ ] **Step 3: Implement `gospaComponents`** in `Gospa.cpp` by reusing the existing Hungarian augmented-cost solve (`Gospa.cpp:33-50`) and bucketing each assigned cell: a `[0:n,0:m]` cell → localization; a truth→dummy (`cost==miss`) → missed; an est→dummy → false. The scalar `gospaGreedy` becomes `gospaComponents(...).total` (DRY — have the old function call the new one and `pow(total,1/p)`).
- [ ] **Step 4: Run test, verify pass.**
- [ ] **Step 5: Plumb through Metrics + Sweep::emit.** Add the four metric rows (`gospa_localization`, `gospa_missed`, `gospa_false`, `card_err_mean`) alongside the existing `gospa_mean` emission. Keep all existing rows unchanged (back-compat for downstream tooling).
- [ ] **Step 6: Bench philos, eyeball the decomposition** for `imm_cv_ct_mht` vs `imm_cv_ct_pmbm_adapt` — you should see PMBM's extra GOSPA is almost entirely the `gospa_false` term (confirming over-count). Commit.
- [ ] **Step 7: Commit.**
```bash
git add core/scenario/Gospa.hpp core/scenario/Gospa.cpp core/benchmark/Metrics.hpp \
        core/benchmark/Metrics.cpp core/benchmark/Sweep.cpp tests/scenario/<gospa_test_file>
git commit -m "bench: emit GOSPA localization/missed/false decomposition + cardinality error"
```

---

## Self-Review checklist (run after implementing each task)

1. **Determinism:** replay-determinism test still green (`ctest -N | grep -i determ`).
2. **Back-compat:** every new Config field defaults to legacy; bare predict/update unit tests unchanged; existing baseline CSVs reproduce for untouched configs.
3. **Hexagonal:** no I/O leaked into `core/` or `ports/`; loaders only in `adapters/`.
4. **Doc standard:** math changes (Tasks 1,2,4) updated the four sections in `pmbm-design.md` + the learning chapter.
5. **Autoferry guard:** any change proposed for a *canonical* config measured on autoferry and not regressing the ~42% PMBM win.
6. **Type consistency:** `birth_existence_target`, `source_aware_identity`, `dedup_miss_pd`, `vessel_id`, `gospaComponents`, `TruthSource` named identically wherever referenced.

## Why these don't re-tread rejected ground (so you don't waste a cycle)

Already tried and rejected (`PHILOS-T1…T11` in `evaluation-log.md`): miss-P_D dedup **in isolation** (+92%); the source-consistency fix in isolation (+92%); the **absolute** `λ_birth` sweep `{1e-3,1e-4,1e-5}`; the PPP-coverage birth gate; `idle_halflife` retuning alone; K=3; output-Bhattacharyya merges; the alt-birth lineage gate; `k_best_dominance_log_gap`. **None of the tasks above repeats these:** Task 1 changes the birth *parameterization* (not the absolute scalar); Task 2 lands the dedup **only as a bundle** with the new birth + per-vessel identity + a real cardinality control (the explicitly-named untried path); Task 3 wires an existing clutter map that was **never wired to PMBM**; Tasks 4/6/7 are new mechanisms / benchmark-validity work.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-24-pmbm-philos-cardinality-improvements.md`. Two execution options:

1. **Subagent-Driven (recommended)** — a fresh subagent per task, with review between tasks.
2. **Inline Execution** — execute tasks in this session with checkpoints.

Which approach (and which task to start — Task 1 is the cheapest, highest-leverage starting point)?
