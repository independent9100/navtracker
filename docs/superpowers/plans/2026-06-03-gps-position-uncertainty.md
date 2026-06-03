# GPS Position Uncertainty Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Budget own-ship GPS position uncertainty in projected measurements (R-inflation by `σ²_GPS · I`), populated from NMEA GGA's HDOP × UERE and from sim. Bias estimator picks up `(σ_GPS / r)²` as a measurement-noise floor.

**Architecture:** `OwnShipPose` gains a `position_std_m` field. `projectRangeBearingToEnu` takes a new `sigma_gps_pos_m` param and adds `σ²_GPS · I` to its output covariance. ARPA/EOIR adapters read the pose field and pass it through. The std propagates to the bias estimator via `Measurement.sensor_position_std_m` → `Track::SourceTouch.own_position_std_m` → `AisArpaPairObservation.own_position_std_m`, where it folds into `σ²_v` as `(σ_GPS / r)²`.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-03-gps-position-uncertainty-design.md`. The numbered sections in tasks below refer to that spec.

---

## Task 1: Extend `projectRangeBearingToEnu` with σ_GPS

**Files:**
- Modify: `adapters/util/Projection.hpp`
- Modify: `adapters/util/Projection.cpp`
- Modify: `tests/adapters/util/test_projection.cpp` (extend; also update existing callers to pass 0.0)

### Why

Per spec §4.1 and §5.4: add `sigma_gps_pos_m` parameter (positioned right after `sigma_heading_rad`), add `σ²_GPS · I` to the output covariance. Single math change covers ARPA TTM and EO/IR uniformly. Existing tests must pass 0.0 to remain green.

### Steps

- [ ] **Step 1: Header signature change**

Modify `adapters/util/Projection.hpp` to add the new parameter:

```cpp
PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       double sigma_gps_pos_m,
                                       const Eigen::Vector2d& own_ship_pos_enu);
```

- [ ] **Step 2: Write failing tests**

Append to `tests/adapters/util/test_projection.cpp`:

```cpp
TEST(ProjectionTest, GpsStdAddsToCovariance) {
  // Zero range/bearing/heading noise, only GPS noise. Output covariance
  // should equal sigma_gps^2 * I exactly.
  const double sigma_gps = 5.0;
  const auto out = projectRangeBearingToEnu(1000.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            sigma_gps,
                                            Eigen::Vector2d::Zero());
  const double s2 = sigma_gps * sigma_gps;
  EXPECT_NEAR(out.cov(0, 0), s2, 1e-9);
  EXPECT_NEAR(out.cov(1, 1), s2, 1e-9);
  EXPECT_NEAR(out.cov(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(out.cov(1, 0), 0.0, 1e-9);
}

TEST(ProjectionTest, GpsStdComposesWithExistingNoise) {
  // Baseline: normal range/bearing noise, no GPS, no heading. Then add
  // GPS std and expect output cov to equal baseline + sigma_gps^2 * I.
  const double sigma_gps = 3.0;
  const auto base = projectRangeBearingToEnu(1500.0,
                                             1.0,
                                             50.0,
                                             1.0 * (3.14159265358979323846 / 180.0),
                                             0.0,
                                             0.0,
                                             Eigen::Vector2d::Zero());
  const auto with_gps = projectRangeBearingToEnu(1500.0,
                                                 1.0,
                                                 50.0,
                                                 1.0 * (3.14159265358979323846 / 180.0),
                                                 0.0,
                                                 sigma_gps,
                                                 Eigen::Vector2d::Zero());
  const double s2 = sigma_gps * sigma_gps;
  EXPECT_NEAR(with_gps.cov(0, 0) - base.cov(0, 0), s2, 1e-6);
  EXPECT_NEAR(with_gps.cov(1, 1) - base.cov(1, 1), s2, 1e-6);
  EXPECT_NEAR(with_gps.cov(0, 1) - base.cov(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(with_gps.cov(1, 0) - base.cov(1, 0), 0.0, 1e-9);
}
```

- [ ] **Step 3: Update existing test callers**

Any existing call in `tests/adapters/util/test_projection.cpp` to `projectRangeBearingToEnu(...)` with 6 arguments needs to become 7 by inserting `0.0` for `sigma_gps_pos_m` right after `sigma_heading_rad`. Grep `projectRangeBearingToEnu` in the test file and update each call.

- [ ] **Step 4: Run tests, verify failure**

```
cmake --build build --target navtracker_tests 2>&1 | tail -5
```
Expected: compile fails because `Projection.cpp` doesn't implement the new signature, and `ArpaAdapter.cpp` / `EoIrAdapter.cpp` still pass 6 args.

This is the failing state. Move on without committing it; the next steps will fix the compile.

- [ ] **Step 5: Implement in `Projection.cpp`**

Modify the function definition to accept `sigma_gps_pos_m`. In the existing implementation, after computing `cov_enu = J * R * J.transpose()`, append:

```cpp
cov_enu(0, 0) += sigma_gps_pos_m * sigma_gps_pos_m;
cov_enu(1, 1) += sigma_gps_pos_m * sigma_gps_pos_m;
```

- [ ] **Step 6: Update existing non-test callers**

Grep for callers outside `tests/`:
```
grep -rn "projectRangeBearingToEnu" adapters/ sim/ core/
```
Two production callers: `adapters/arpa/ArpaAdapter.cpp` (TTM branch) and `adapters/eoir/EoIrAdapter.cpp`. Insert `0.0` for the new argument in both — Tasks 4 and 5 will replace it with the real value, but Task 1 just keeps the build green.

- [ ] **Step 7: Build + run targeted tests**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R ProjectionTest --output-on-failure
```
Expected: all ProjectionTests PASS.

- [ ] **Step 8: Run full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: full suite green (247/247 was the pre-change count; should still be 247 since the 2 new tests are net additions inside `ProjectionTest`). The count is now 249.

- [ ] **Step 9: Commit**

```
git add adapters/util/Projection.hpp adapters/util/Projection.cpp \
        adapters/arpa/ArpaAdapter.cpp adapters/eoir/EoIrAdapter.cpp \
        tests/adapters/util/test_projection.cpp
git commit -m "projection: sigma_gps_pos_m additive sigma^2 I on output cov"
```

---

## Task 2: Add `position_std_m` to `OwnShipPose`; sim populates it

**Files:**
- Modify: `adapters/own_ship/OwnShipProvider.hpp`
- Modify: `sim/OwnShipEmitter.cpp`
- Test: `tests/sim/test_own_ship_emitter.cpp` (extend)

### Why

Per spec §5.1 and §5.3: the pose carries `σ_GPS`. Sim writes the configured `gps_pos_std_m` straight into the emitted pose so the tracker side gets full sim parity. NMEA path comes in Task 3.

### Steps

- [ ] **Step 1: Add the field**

Modify `adapters/own_ship/OwnShipProvider.hpp` — append `double position_std_m{0.0};` to `OwnShipPose`:

```cpp
struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
};
```

- [ ] **Step 2: Write failing test**

Append to `tests/sim/test_own_ship_emitter.cpp`:

```cpp
TEST(OwnShipEmitterTest, EmittedPoseCarriesGpsStd) {
  // Configure gps_pos_std_m = 5; emit one tick; verify the OwnShipProvider's
  // latest pose has position_std_m == 5.
  sim::OwnShipEmitterConfig cfg;
  cfg.gps_pos_std_m = 5.0;
  // ... reuse the existing fixture pattern in this file to build the emitter,
  // emit one EmitContext, then read provider.latest()->position_std_m.
  // Assert EXPECT_DOUBLE_EQ(provider.latest()->position_std_m, 5.0);
}
```

Fill in the body using existing test patterns in `tests/sim/test_own_ship_emitter.cpp` for emitter construction.

- [ ] **Step 3: Run test, expect fail**

```
ctest --test-dir build -R OwnShipEmitterTest.EmittedPoseCarriesGpsStd --output-on-failure
```
Expected: FAIL — emitter doesn't set `position_std_m`.

- [ ] **Step 4: Implement in `OwnShipEmitter`**

In `sim/OwnShipEmitter.cpp`, find where the emitter constructs the pose pushed through the NMEA adapter. Two paths in current code:

The emitter feeds NMEA sentences through `OwnShipNmeaAdapter`, which then calls `provider.update(pose)`. So we can't directly set `position_std_m` on the pose unless we either:
- (a) Encode it into the NMEA stream (no standard sentence carries it; non-starter).
- (b) After emitting NMEA, directly call `provider.update` with an augmented pose that already has the field set. Cleaner — sim already has direct access to the provider.

Simplest implementation: after the existing NMEA emit, call `provider.update(...)` with a pose copy whose `position_std_m = cfg_.gps_pos_std_m`. The emitter holds a reference to the adapter, not the provider — see if it has the provider. If not, add a hook.

A different cleaner option: pass `gps_pos_std_m` through to the NMEA adapter via a setter, and the adapter sets it on the pose it ultimately publishes via `provider.update`. This keeps the data path through the adapter cleanly.

**Implementation choice:** look at how `OwnShipEmitter` currently injects noise (it adds to lat/lon before formatting NMEA — see `sim/OwnShipEmitter.cpp:31-32`). The clean fix is to extend `OwnShipNmeaAdapterConfig` (Task 3 will add it; for Task 2, just add a hook). Actually the simplest path: after the NMEA `ingest` call, the adapter calls `provider.update`; we extend `OwnShipNmeaAdapter` with a setter `void setNextPositionStd(double sigma)` that the emitter calls each cycle just before feeding NMEA, and the adapter stores it into the pose before `provider.update`.

If that creates too much surface area, alternative: extend `OwnShipNmeaAdapterConfig` (Task 3 anyway) to include a `position_std_m_override` for sim use; the emitter sets it via a setter on the adapter. Pick whichever is cleanest given the actual code in `OwnShipNmeaAdapter.cpp`.

- [ ] **Step 5: Run test, expect pass**

```
ctest --test-dir build -R OwnShipEmitterTest.EmittedPoseCarriesGpsStd --output-on-failure
```

- [ ] **Step 6: Run full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 250/250 green.

- [ ] **Step 7: Commit**

```
git add adapters/own_ship/OwnShipProvider.hpp \
        adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp \
        sim/OwnShipEmitter.cpp \
        tests/sim/test_own_ship_emitter.cpp
git commit -m "own-ship: position_std_m on pose; sim populates from cfg.gps_pos_std_m"
```

---

## Task 3: `OwnShipNmeaAdapter` parses HDOP via UERE

**Files:**
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.hpp`
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.cpp`
- Test: `tests/adapters/own_ship/test_own_ship_nmea.cpp` (extend)

### Why

Per spec §4.3 and §5.2: NMEA GGA's HDOP (field 8 per NMEA 0183) is converted to a σ via `σ = HDOP × UERE`, where UERE is a config field (default 5 m). If HDOP is absent or non-positive, leave `position_std_m` at 0.

### Steps

- [ ] **Step 1: Add config**

Modify `adapters/own_ship/OwnShipNmeaAdapter.hpp`:

```cpp
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
};

class OwnShipNmeaAdapter {
 public:
  explicit OwnShipNmeaAdapter(OwnShipProvider& provider,
                              OwnShipNmeaAdapterConfig cfg = {});
  // ... existing public surface
 private:
  OwnShipProvider& provider_;
  OwnShipNmeaAdapterConfig cfg_;
  // ...
};
```

Defaulted-ctor keeps existing call sites compiling unchanged.

- [ ] **Step 2: Write failing tests**

Append to `tests/adapters/own_ship/test_own_ship_nmea.cpp`:

```cpp
TEST(OwnShipNmeaAdapterTest, ParsesHdopFromGga) {
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.uere_m = 5.0;
  OwnShipNmeaAdapter adapter(provider, cfg);

  // Build a GGA sentence with HDOP = 1.2 in field 8 (the standard position).
  // Reuse existing test scaffolding for valid GGA composition.
  const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,545.4,M,46.9,M,,*47";
  adapter.ingest(gga, Timestamp::fromSeconds(0.0));

  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position_std_m, 6.0, 1e-9);  // 1.2 * 5.0
}

TEST(OwnShipNmeaAdapterTest, AbsentHdopLeavesStdAtZero) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});

  // Build a GGA sentence with empty HDOP field.
  const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,,545.4,M,46.9,M,,*47";
  adapter.ingest(gga, Timestamp::fromSeconds(0.0));

  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->position_std_m, 0.0);
}
```

Use the test file's existing helpers for valid checksum/format if it uses any; otherwise the checksum just needs to match. If the existing tests skip checksum validation, the placeholder `*47` is fine.

- [ ] **Step 3: Run, expect fail**

```
ctest --test-dir build -R "OwnShipNmeaAdapterTest" --output-on-failure
```
Expected: the two new tests fail because the adapter doesn't yet read HDOP.

- [ ] **Step 4: Implement HDOP parsing**

In `adapters/own_ship/OwnShipNmeaAdapter.cpp`, in the GGA-handling branch, after parsing existing fields, read field index 7 (zero-based) — this is HDOP per NMEA 0183:

```cpp
double hdop = 0.0;
if (parsed->fields.size() > 7 && !parsed->fields[7].empty()) {
  hdop = std::strtod(parsed->fields[7].c_str(), nullptr);
}
const double sigma = hdop > 0.0 ? hdop * cfg_.uere_m : 0.0;
pose.position_std_m = sigma;
```

Index alignment: confirm with the existing GGA parsing in this file. NMEA GGA fields after `$` and the formatter are: time(0), lat(1), N/S(2), lon(3), E/W(4), fix(5), nsats(6), hdop(7), alt(8), m(9), geoid(10), m(11), age(12), refid(13). If the local parser strips the formatter from the fields list (so `fields[0]` is "time"), HDOP is at index 7.

- [ ] **Step 5: Run, expect pass**

```
ctest --test-dir build -R "OwnShipNmeaAdapterTest" --output-on-failure
```
Expected: all PASS, including the two new ones.

- [ ] **Step 6: Run full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 252/252 green.

- [ ] **Step 7: Commit**

```
git add adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp \
        tests/adapters/own_ship/test_own_ship_nmea.cpp
git commit -m "own-ship-nmea: parse HDOP from GGA, populate pose.position_std_m via UERE"
```

---

## Task 4: ARPA adapter reads `pose.position_std_m`

**Files:**
- Modify: `adapters/arpa/ArpaAdapter.cpp`
- Test: `tests/adapters/arpa/test_arpa_adapter.cpp` (extend)

### Why

Per spec §6.1: in the TTM branch, read `own_opt->position_std_m` and pass it as the new `sigma_gps_pos_m` argument to `projectRangeBearingToEnu` (replacing the `0.0` placeholder from Task 1). TLL stays unchanged.

### Steps

- [ ] **Step 1: Write failing tests**

Append to `tests/adapters/arpa/test_arpa_adapter.cpp`:

```cpp
TEST(ArpaAdapterTest, InflatesCovarianceFromOwnShipGpsStd) {
  // Same TTM scenario as the existing baseline test, but provider
  // updated with pose.position_std_m = 5. Compare covariance to a
  // baseline (pose.position_std_m = 0) and expect +25 on the diagonal,
  // unchanged off-diagonal.
  // Reuse the test file's fixtures for building TTM + own-ship pose.
}

TEST(ArpaAdapterTest, TllUnaffectedByGpsStd) {
  // Set pose.position_std_m = 5, feed a TLL sentence; expect covariance
  // identical to the baseline TLL covariance.
}
```

Fill in the bodies using the test file's existing TTM/TLL fixtures.

- [ ] **Step 2: Run, expect fail**

```
ctest --test-dir build -R "ArpaAdapterTest.InflatesCovarianceFromOwnShipGpsStd|ArpaAdapterTest.TllUnaffectedByGpsStd" --output-on-failure
```

- [ ] **Step 3: Implement**

In `adapters/arpa/ArpaAdapter.cpp`, in the TTM branch, replace the `0.0` placeholder from Task 1 with:

```cpp
const double sigma_gps_pos = own_opt->position_std_m;
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad_corrected,
                             50.0, 1.0 * kDeg2Rad,
                             sigma_heading_eff,
                             sigma_gps_pos,
                             own_xy);
```

TLL branch stays at 0 (no GPS effect on absolute lat/lon positioning).

- [ ] **Step 4: Run, expect pass**

```
ctest --test-dir build -R "ArpaAdapterTest" --output-on-failure
```

- [ ] **Step 5: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 254/254 green.

- [ ] **Step 6: Commit**

```
git add adapters/arpa/ArpaAdapter.cpp tests/adapters/arpa/test_arpa_adapter.cpp
git commit -m "arpa: TTM projection inflates covariance from pose.position_std_m"
```

---

## Task 5: EO/IR adapter reads `pose.position_std_m`

**Files:**
- Modify: `adapters/eoir/EoIrAdapter.cpp`
- Test: `tests/adapters/eoir/test_eoir_adapter.cpp` (extend)

### Why

Per spec §6.2: same pattern as Task 4, for EO/IR detections.

### Steps

- [ ] **Step 1: Write failing test**

Append to `tests/adapters/eoir/test_eoir_adapter.cpp`:

```cpp
TEST(EoIrAdapterTest, InflatesCovarianceFromOwnShipGpsStd) {
  // Mirror Task 4's ARPA test using CameraDetection inputs.
}
```

- [ ] **Step 2: Run, expect fail**

```
ctest --test-dir build -R "EoIrAdapterTest.InflatesCovarianceFromOwnShipGpsStd" --output-on-failure
```

- [ ] **Step 3: Implement**

In `adapters/eoir/EoIrAdapter.cpp`, replace the `0.0` placeholder from Task 1 with `own_opt->position_std_m` and pass into `projectRangeBearingToEnu`.

- [ ] **Step 4: Run, expect pass**

```
ctest --test-dir build -R "EoIrAdapterTest" --output-on-failure
```

- [ ] **Step 5: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 255/255 green.

- [ ] **Step 6: Commit**

```
git add adapters/eoir/EoIrAdapter.cpp tests/adapters/eoir/test_eoir_adapter.cpp
git commit -m "eoir: projection inflates covariance from pose.position_std_m"
```

---

## Task 6: Propagate σ_GPS through `Measurement` → `Track::SourceTouch`

**Files:**
- Modify: `core/types/Measurement.hpp` (add field)
- Modify: `core/types/Track.hpp` (extend SourceTouch)
- Modify: `core/pipeline/Tracker.cpp` (populate field)
- Modify: `adapters/arpa/ArpaAdapter.cpp` and `adapters/eoir/EoIrAdapter.cpp` (set on emitted Measurement)
- Test: `tests/pipeline/test_tracker.cpp` (extend)

### Why

Per spec §7: the bias estimator needs to know the σ_GPS that was in effect at the time of an ARPA projection. Carry it on `Measurement.sensor_position_std_m`; `Tracker` copies it into `SourceTouch.own_position_std_m`. Default 0 means "no floor known" — backward compatible.

### Steps

- [ ] **Step 1: Extend `Measurement`**

Append to `core/types/Measurement.hpp` inside the `Measurement` struct:

```cpp
double sensor_position_std_m{0.0};
```

- [ ] **Step 2: Extend `Track::SourceTouch`**

Append to `core/types/Track.hpp` inside the `SourceTouch` struct:

```cpp
double own_position_std_m{0.0};
```

- [ ] **Step 3: Adapters set the field**

In `ArpaAdapter.cpp` and `EoIrAdapter.cpp` TTM/EOIR branches, immediately after constructing the `Measurement m`, set:

```cpp
m.sensor_position_std_m = own_opt->position_std_m;
```

This way the σ_GPS that was used in the projection also rides along with the Measurement for downstream consumers.

- [ ] **Step 4: Tracker populates SourceTouch**

In `core/pipeline/Tracker.cpp`, in all three SourceTouch-populating branches (single-msg `process`, hard-branch `processBatch`, soft-branch `processBatch` over `gated`), add to each `touch` after setting other fields:

```cpp
touch.own_position_std_m = z.sensor_position_std_m;  // or gz.sensor_position_std_m in soft branch
```

- [ ] **Step 5: Write failing test**

Append to `tests/pipeline/test_tracker.cpp`:

```cpp
TEST(TrackerTest, PropagatesOwnPositionStdToSourceTouch) {
  // Mirror the existing RecordsRecentContributionsOnFusion test but
  // set z.sensor_position_std_m = 4.2; assert the SourceTouch on the
  // fused track has own_position_std_m == 4.2.
}
```

- [ ] **Step 6: Run, expect pass**

```
ctest --test-dir build -R "TrackerTest.PropagatesOwnPositionStdToSourceTouch" --output-on-failure
```

- [ ] **Step 7: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 256/256 green.

- [ ] **Step 8: Commit**

```
git add core/types/Measurement.hpp core/types/Track.hpp core/pipeline/Tracker.cpp \
        adapters/arpa/ArpaAdapter.cpp adapters/eoir/EoIrAdapter.cpp \
        tests/pipeline/test_tracker.cpp
git commit -m "types/tracker: carry sensor_position_std_m through Measurement and SourceTouch"
```

---

## Task 7: Bias estimator folds σ_GPS floor into σ²_v

**Files:**
- Modify: `core/bias/HeadingBiasEstimator.hpp` (extend `AisArpaPairObservation`)
- Modify: `core/bias/HeadingBiasEstimator.cpp` (add floor term in `observe`)
- Modify: `core/bias/AisArpaPairExtractor.cpp` (propagate from `SourceTouch.own_position_std_m`)
- Test: `tests/bias/test_heading_bias_estimator.cpp` (extend)
- Test: `tests/bias/test_ais_arpa_pair_extractor.cpp` (extend)

### Why

Per spec §4.2 and §7: the bias estimator's measurement-noise σ²_v gains a `(σ_GPS / r)²` term so the estimator's variance doesn't collapse below a physically achievable floor in poor-GPS conditions.

### Steps

- [ ] **Step 1: Extend the observation struct**

In `core/bias/HeadingBiasEstimator.hpp`, append to `AisArpaPairObservation`:

```cpp
double own_position_std_m{0.0};   // owns-ship GPS sigma at observation time
```

- [ ] **Step 2: Extend the extractor**

In `core/bias/AisArpaPairExtractor.cpp`, when emitting an observation, set:

```cpp
obs.own_position_std_m = arpa->own_position_std_m;
```

(The ARPA touch carries it because Task 6 has the adapter setting it via `Measurement.sensor_position_std_m`.)

- [ ] **Step 3: Write failing extractor test**

Append to `tests/bias/test_ais_arpa_pair_extractor.cpp`:

```cpp
TEST(AisArpaPairExtractorTest, PropagatesOwnPositionStdFromTouch) {
  Track tr;
  auto ais  = makeTouch(SensorKind::Ais,     tAt(10.0), Eigen::Vector2d(1000.0, 0.0));
  auto arpa = makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0));
  arpa.own_position_std_m = 3.0;
  tr.recent_contributions.push_back(ais);
  tr.recent_contributions.push_back(arpa);
  const auto pairs = extractPairs({tr}, tAt(10.0));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_DOUBLE_EQ(pairs[0].own_position_std_m, 3.0);
}
```

- [ ] **Step 4: Implement the σ_v floor in the estimator**

In `core/bias/HeadingBiasEstimator.cpp` inside `observe(...)`, compute the ARPA range and add the floor:

```cpp
const Eigen::Vector2d arpa_rel_for_range = obs.arpa_target_position_enu - obs.own_position_enu;
const double r_arpa = arpa_rel_for_range.norm();
double sigma_v2 =
    obs.arpa_bearing_std_rad * obs.arpa_bearing_std_rad
    + (obs.ais_position_std_m * obs.ais_position_std_m) / (r_ais * r_ais);
if (r_arpa > 1.0) {
  const double term = obs.own_position_std_m / r_arpa;
  sigma_v2 += term * term;
}
```

Then the existing scalar KF update uses the new `sigma_v2`. Make sure `r_ais` is still used for the AIS term (existing); the new term uses `r_arpa`.

- [ ] **Step 5: Write failing estimator test**

Append to `tests/bias/test_heading_bias_estimator.cpp`:

```cpp
TEST(HeadingBiasEstimatorTest, GpsFloorPreventsOverConvergence) {
  // True bias = 1 deg. Inject 100 pairs with sigma_arpa = 0.5 deg,
  // sigma_ais_pos = 0.0001 m (negligible), own_position_std_m = 13.0 m,
  // range = 1500 m. The dominant noise source is now the GPS floor at
  // 13/1500 ≈ 0.005 rad ≈ 0.29 deg.
  //
  // After 100 updates, asymptotic variance should be ≈ (0.29 deg)^2 / 100
  // ≈ (0.029 deg)^2, much smaller than the floor we tested in the
  // original "GatingDelaysPublication" but bounded by the GPS-floor.
  //
  // Simpler assertion: with own_position_std_m = 0, estimator's P_b
  // after N updates is X; with own_position_std_m = 13 m, P_b is > X.
}
```

Use a comparison-against-baseline pattern (a/b same seed, only the floor differs).

- [ ] **Step 6: Run, expect pass on all new tests**

```
ctest --test-dir build -R "HeadingBiasEstimatorTest|AisArpaPairExtractorTest" --output-on-failure
```

- [ ] **Step 7: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 258/258 green.

- [ ] **Step 8: Commit**

```
git add core/bias/HeadingBiasEstimator.hpp core/bias/HeadingBiasEstimator.cpp \
        core/bias/AisArpaPairExtractor.cpp \
        tests/bias/test_heading_bias_estimator.cpp tests/bias/test_ais_arpa_pair_extractor.cpp
git commit -m "bias: fold (sigma_gps / r)^2 into sigma_v^2 of the heading bias update"
```

---

## Task 8: GPS sweep + eval-log

**Files:**
- Create: `tests/sim/test_bus_gps_sweep.cpp`
- Modify: `tests/sim/BusComparisonHelpers.hpp` (add knob + builder per scenario; minor)
- Modify: `CMakeLists.txt`
- Modify: `docs/algorithms/evaluation-log.md`

### Why

Per spec §10: close-range (ClutterCrossing, ~200 m) sweep across σ_GPS ∈ {0, 0.1, 1, 5} m × 20 seeds × two rows (R-off vs R-on) is the headline; long-range (BearingOnlyMoving, ~1500 m) probe is the sanity check that GPS inflation correctly does little at long range. Capture results into the eval-log.

### Steps

- [ ] **Step 1: Add the knob**

Append to `tests/sim/BusComparisonHelpers.hpp`:

```cpp
struct GpsSweepKnob {
  double sigma_gps_m{0.0};   // injected by OwnShipEmitter
  bool   r_inflation_on{false};  // pass through to adapter projection
};
```

Wait — the inflation is already automatic when `pose.position_std_m` is non-zero (set from sim cfg). To produce the "R-off" row, we need to keep sim injection but suppress the inflation in projection. The cleanest path: introduce a flag on the OwnShipEmitter cfg, `bool report_gps_std{true}`, that suppresses setting `pose.position_std_m` while keeping the lat/lon noise injection (so the noise still corrupts projections but the adapter doesn't know to budget for it). This produces the apples-to-apples R-off vs R-on comparison.

So: extend `sim::OwnShipEmitterConfig` with `bool report_gps_std{true};`. In the emitter, only set `position_std_m` on the pose when this is true.

- [ ] **Step 2: Add scenario builders**

Add `runBusClutterCrossingWithGps(seed, GpsSweepKnob)` and `runBusBearingOnlyMovingWithGps(seed, GpsSweepKnob)` to `BusComparisonHelpers.hpp`, mirroring the existing `*WithHeading` builders but setting `own_cfg.gps_pos_std_m = knob.sigma_gps_m;` and `own_cfg.report_gps_std = knob.r_inflation_on;`.

- [ ] **Step 3: Write the sweep test**

Create `tests/sim/test_bus_gps_sweep.cpp` modeled on `tests/sim/test_bus_heading_sweep.cpp`:

```cpp
// Two TESTs: BusGpsSweep.ClutterCrossing, BusGpsSweep.BearingOnlyMoving.
// For each: sigma_gps in {0.0, 0.1, 1.0, 5.0} m, seed 201..220, two rows
// per cell:
//   row_a = sigma_gps injected, R-inflation off
//   row_b = sigma_gps injected, R-inflation on
// Report mean per-window OSPA and id-switch count.
// SUCCEED-only.
```

- [ ] **Step 4: Run, capture numbers**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R BusGpsSweep --output-on-failure 2>&1 | tee /tmp/gps_sweep.txt
```

- [ ] **Step 5: Append to eval-log**

In `docs/algorithms/evaluation-log.md`, append a new section "## GPS position uncertainty (2026-06-03)" with two tables and a verdict paragraph. Tables use the same column layout as the §14.9 / bias-estimator tables for visual consistency:

```markdown
## GPS position uncertainty (2026-06-03)

**Setup.** Injects own-ship GPS position noise via sim::OwnShipEmitter::gps_pos_std_m; ARPA/EOIR adapters inflate measurement covariance by sigma_gps^2 * I when the pose reports the std (R-on row) and skip the inflation when it doesn't (R-off row — apples-to-apples noise but unmodeled). 20 seeds (201..220), EKF + GNN.

### ClutterCrossing (close range, ~200 m)
| sigma_gps (m) | row | OSPA mean +/- stddev | id_sw_mean |
| ... fill from /tmp/gps_sweep.txt ... |

### BearingOnlyMoving (long range, ~1500 m, sanity probe)
| sigma_gps (m) | row | OSPA mean +/- stddev | id_sw_mean |
| ... fill ... |

### Verdict
<3-5 sentences: at close range, R-on cuts OSPA materially as sigma_gps grows; at long range, the effect is in the noise — exactly as predicted by the (range x sigma_gps) inverse-of-heading gradient. Reference where it fits next to the 14.9 and bias-estimator results.>
```

- [ ] **Step 6: Commit**

```
git add tests/sim/test_bus_gps_sweep.cpp tests/sim/BusComparisonHelpers.hpp \
        sim/OwnShipEmitter.hpp sim/OwnShipEmitter.cpp \
        CMakeLists.txt docs/algorithms/evaluation-log.md
git commit -m "sim+eval: GPS sweep close-range and long-range probe with eval-log"
```

---

## Done criteria

- All 8 tasks committed.
- Full suite green at the new test count.
- Eval-log has a populated "GPS position uncertainty (2026-06-03)" section with concrete numbers and verdict.
- Spec §11 (Ways to improve) is the home for future Approach-2 work; no shim or scaffolding for it lands in this plan.
