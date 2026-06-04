# NMEA Multi-Heading-Source Wiring Design

**Date:** 2026-06-04
**Status:** Approved
**Predecessor:** `2026-06-04-multi-heading-sources-bias-design.md` (v3 estimator)

This spec lands the NMEA-side wiring deferred from v3 §6. The estimator surface stays unchanged; this work adds sentence parsing, talker-ID-based routing, and dispatch from `OwnShipNmeaAdapter` to `HeadingBiasEstimator`.

## 1. Problem

v3 delivered three new `observe()` overloads on `HeadingBiasEstimator` but no production data flows into them — the unit/permutation tests drive the estimator with synthetic structs. Real NMEA feeds carry the relevant data (`HDG`, `RMC`, `HDT` from different talkers) but `OwnShipNmeaAdapter`:

- doesn't parse `HDG` at all,
- always routes `HDT` as the gyrocompass (no talker-ID discrimination),
- parses `RMC` for SOG/COG only, dropping the magnetic-variation fields.

There's also no path from the adapter to `HeadingBiasEstimator` — even if the data were present, nobody would dispatch.

## 2. Architecture (path A — adapter holds the estimator)

`OwnShipNmeaAdapter` holds an optional `HeadingBiasEstimator*` set via `setHeadingBiasEstimator(...)`. The adapter is the natural dispatcher because it already knows the per-sentence semantics (e.g., COG only exists in `RMC`, mag heading only in `HDG`). When the estimator pointer is null, parsing happens but no dispatch — exactly the existing behavior, with the additional new fields just sitting on `OwnShipPose` for whoever wants them.

```
                          OwnShipNmeaAdapter
                         +------------------+
                         |  parse + route   |
                         |  + gyro history  |
                         |  + dispatch      |
                         +------------------+
                          |        |       |
                  HDG/RMC |        |HDT(gyro)  HDT(GPS)
                          v        v       v
                         OwnShipProvider  (pose with new fields)
                         |
                         | optional pointer
                         v
                    HeadingBiasEstimator
                    (observe overloads)
```

No new ports. `OwnShipPose` grows three optional-style fields (signaled by sentinel values to keep the struct flat for ABI sanity, see §4.4).

## 3. Sentence changes

### 3.1 `$xxHDG` — new parser

Format: `$<talker>HDG,<heading>,<deviation>,<dev_dir>,<variation>,<var_dir>*hh`

- `heading` — magnetic heading, degrees true (i.e., uncorrected for deviation and variation).
- `deviation`, `dev_dir` — per-vessel magnetic deviation in degrees and `E`/`W` direction.
- `variation`, `var_dir` — geographic variation in degrees and `E`/`W` direction.

Adapter behavior:
- Compute `magnetic_heading_corrected = heading + signed_deviation` where `signed_deviation = +deviation if dev_dir=="E" else -deviation` (the standard "easterly correction is positive" convention).
- Populate `pose.magnetic_heading_deg = magnetic_heading_corrected`.
- Populate `pose.magnetic_heading_std_deg` from config default `cfg_.magnetic_heading_sigma_deg` (default 0.5°).
- Populate `pose.magnetic_variation_deg = signed_variation` (same sign convention).
- Update internal `latest_known_variation_deg_` cache (used as fallback for variations not carried in subsequent HDG / mag sentences from other paths).

Required fields: `heading`, `var_dir` (used as the "variation present" gate; if `var_dir` empty, mag observation will only fire via the variation fallback chain).

### 3.2 `$xxRMC` — extract variation

Existing RMC parsing extracts SOG (field 6) and COG (field 7). Extend to extract:
- `magvar_deg = strtod(field[9])` if non-empty
- `magvar_dir = field[10]` (E/W); `signed = +deg if E else -deg`

Cache `latest_known_variation_deg_` from this source as well. When both HDG and RMC report variation in the same time window, the more recent one wins.

Also: when a fresh RMC arrives AND the estimator is wired, construct `GyroVsGpsCogObservation` (see §5.2).

### 3.3 `$xxHDT` — talker-ID routing

The adapter's config grows:
```cpp
std::unordered_set<std::string> gps_heading_talkers;  // default empty
```

When `parsed->talker` is in the set, route as **GPS true heading**:
- Populate `pose.gps_true_heading_deg`.
- Populate `pose.gps_true_heading_std_deg` from config default (default 0.5°).
- Do NOT touch `pose.heading_true_deg` (the gyro).
- If estimator wired: construct `GyroVsGpsHeadingObservation`.

When `parsed->talker` is NOT in the set, route as **gyro** (existing behavior):
- Populate `pose.heading_true_deg`.
- Update the internal heading-rate ring buffer (§5.1).
- Do NOT dispatch (the gyro IS the reference, not an observation of it).

**Default is empty set** to preserve backward compatibility — every existing test that sends `$GPHDT,...` continues to treat it as the gyro reference. Operators with multi-antenna receivers opt in by adding their talker IDs to the config set.

## 4. `OwnShipPose` field additions

Three new fields. All signaled by `std::nan("")` for "not present":

```cpp
struct OwnShipPose {
  // ... existing fields ...
  double gps_true_heading_deg{std::nan("")};
  double gps_true_heading_std_deg{0.0};
  double magnetic_heading_deg{std::nan("")};
  double magnetic_heading_std_deg{0.0};
  double magnetic_variation_deg{std::nan("")};
};
```

Why NaN and not `std::optional<double>`: keeps the struct trivially copyable and ABI-stable; matches the pattern used by `position_std_m{0.0}` ("default = unknown"); avoids the `optional<double>` size penalty (16 bytes vs 8 on most platforms). Consumers check via `std::isnan(...)`.

## 5. Dispatcher logic inside `OwnShipNmeaAdapter`

### 5.1 Gyro heading history (small ring) for COG gate

The adapter keeps a tiny ring buffer of the last N `(Timestamp, gyro_deg)` pairs for computing `gyro_rate_rad_per_s`:

```cpp
struct GyroSample { Timestamp t; double heading_rad; };
std::array<GyroSample, 4> gyro_history_;
std::size_t gyro_history_count_{0};
std::size_t gyro_history_head_{0};
```

When a gyro HDT arrives, the new sample is pushed. The rate at observation time `t` is computed from the two most recent samples bracketing or just preceding `t`, with the angle difference wrap-corrected:

```cpp
double computeGyroRate(Timestamp t, double max_dt_s = 2.0) const;
```

Returns `0.0` if fewer than 2 samples, or if the latest sample is older than `max_dt_s` before `t`.

### 5.2 Dispatch points

| Sentence | Adapter action |
|---|---|
| Gyro HDT | Push to ring buffer; update `pose.heading_true_deg`. No dispatch. |
| GPS HDT (configured talker) | Look up gyro at `t` (via `OwnShipProvider.poseAtOrBefore(t)`); if available, `observe(GyroVsGpsHeadingObservation{...})`. |
| RMC | If estimator wired AND gyro is fresh in the ring buffer: `observe(GyroVsGpsCogObservation{...})` with `gyro_rate_rad_per_s` computed via §5.1 and `gps_cog_std_rad` from config. |
| HDG | Apply variation-fallback chain (§5.3) to choose `variation_rad`. If estimator wired AND gyro fresh: `observe(GyroVsMagneticObservation{...})`. If variation not resolvable, do not dispatch (M1 gate). |

For each dispatch, the gyro reading comes from the **adapter's own ring buffer** (not from `OwnShipPose.heading_true_deg` of the latest pose, because a freshly-arrived RMC may pre-date the next gyro HDT). The dispatch only fires if the ring buffer has a sample within `cfg_.gyro_max_age_s` (default 2.0 s) of the observation time.

### 5.3 Variation fallback chain

When emitting `GyroVsMagneticObservation`, the variation source priority is:
1. The current HDG sentence's own variation field (if present and `var_dir` non-empty).
2. The adapter's cached `latest_known_variation_deg_` (last seen via either HDG or RMC).
3. The config-supplied `cfg_.magnetic_variation_fallback_deg` (default `std::nan("")` = none).
4. Else: skip — emit no observation. Counter increments.

## 6. Config additions

```cpp
struct OwnShipNmeaAdapterConfig {
  // ... existing fields ...

  // Multi-heading-source wiring (v3 NMEA).
  std::unordered_set<std::string> gps_heading_talkers{};  // empty by default
  double gps_heading_sigma_deg{0.5};
  double magnetic_heading_sigma_deg{0.5};
  double gps_cog_sigma_deg{1.0};
  double magnetic_variation_fallback_deg{std::nan("")};
  double gyro_max_age_s{2.0};
};
```

## 7. Diagnostics on `OwnShipNmeaAdapter`

Counters exposed for tests and operator dashboards:

```cpp
std::size_t dispatchedGpsHeading() const;
std::size_t dispatchedGpsCog()     const;
std::size_t dispatchedMagnetic()   const;
std::size_t skippedMagNoVariation() const;
std::size_t skippedGyroStale()      const;
```

## 8. Out of scope (deferred)

- **WMM integration** for variation lookup when no sentence provides it. Operator-supplied fallback only.
- **Deviation table** for magnetic compass. HDG's per-sentence deviation field is honored; nothing else.
- **Receiver-type discriminators** beyond talker ID. If an operator has two `GP*HDT` sources where one is gyro-fed and one is multi-antenna, they need to pre-split the streams.
- **`$xxHEH`, `$xxTHS`, `$xxVHW`, `$xxGSA`** and other less-common sentence variants. Add as needed.
- **JPDA-soft bearing emit** (still deferred from v2).
- **Push-based pose sink** (`IOwnShipPoseSink`). Not needed for this work since the adapter dispatches directly.

## 9. Validation

### 9.1 Sentence parsing tests (`tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp`)

- **N-HDG-1:** `$IIHDG,123.5,1.0,E,3.0,W*..` populates `pose.magnetic_heading_deg=124.5`, `pose.magnetic_variation_deg=-3.0`.
- **N-HDG-2:** Empty deviation/dev_dir → heading uncorrected; variation correctly signed.
- **N-HDG-3:** Empty variation → `pose.magnetic_variation_deg` stays NaN; adapter caches nothing for fallback from this sentence.
- **N-RMC-1:** RMC with variation field populated updates the adapter's variation cache (verifiable by a subsequent HDG without its own variation that dispatches successfully).
- **N-HDT-1 (default config):** `$GPHDT,...` routes as gyro (backward compat). `pose.heading_true_deg` updated.
- **N-HDT-2 (configured `{"GP"}`):** `$GPHDT,...` routes as GPS true heading. `pose.gps_true_heading_deg` updated, `pose.heading_true_deg` untouched.

### 9.2 Dispatch tests (`tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp`)

For each dispatch path, wire a fresh `HeadingBiasEstimator`, push sentences, assert the right `observe(...)` was called via the estimator's diagnostic counters.

- **D-HDG-1:** Push `$IIHDT` (gyro) then `$IIHDG` → estimator's `acceptedMagnetic()` increments by 1.
- **D-HDG-2:** `$IIHDG` alone (no prior gyro) → no dispatch; `skippedGyroStale()` increments.
- **D-HDG-3:** HDG with no variation, no cache, no fallback → no dispatch; `skippedMagNoVariation()` increments.
- **D-HDG-4:** HDG with config fallback variation → dispatches even when sentence has no variation.

- **D-COG-1:** `$IIHDT` then `$IIRMC,A,...,sog>3,cog,...,variation,W` → `acceptedGpsCog()` increments.
- **D-COG-2:** Low-SOG RMC → estimator's `rejectedCogBySog()` increments (gate caught it).

- **D-HDG-RMC:** `$IIRMC` carries variation, then `$IIHDG` with empty variation → adapter uses cached → dispatches. Verifies the fallback chain.

- **D-GPSHDT-1:** Config `{"GP"}`. `$GPHDT` arrives, no prior gyro → no dispatch; `skippedGyroStale()`.
- **D-GPSHDT-2:** Config `{"GP"}`. `$IIHDT` (gyro) then `$GPHDT` (GPS hdg) → `acceptedGpsHeading()` increments.

### 9.3 Backward-compat regression

Re-run the existing `tests/adapters/own_ship/test_own_ship_nmea.cpp` suite unmodified. All current tests must pass — the default `gps_heading_talkers={}` ensures no behavior change for existing `$GPHDT` consumers.

## 10. Files

| Action | Path |
|---|---|
| Modify | `adapters/own_ship/OwnShipProvider.hpp` — three new `OwnShipPose` fields |
| Modify | `adapters/own_ship/OwnShipNmeaAdapter.hpp` — config additions, estimator pointer, history ring, diagnostic counters |
| Modify | `adapters/own_ship/OwnShipNmeaAdapter.cpp` — HDG parser, talker-ID HDT routing, RMC variation forwarding, dispatch logic |
| Create | `tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp` — sentence parsing tests |
| Create | `tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp` — dispatch tests |
| Modify | `CMakeLists.txt` — register new tests |

## 11. Rationale

| Decision | Considered | Chosen | Why |
|---|---|---|---|
| Adapter dispatches directly | New `IOwnShipPoseSink` + separate dispatcher | Adapter | Per-sentence semantics are in the adapter; reconstructing them from a pose event loses information. |
| NaN-signaled optional fields | `std::optional<double>` | NaN | Keeps `OwnShipPose` trivially copyable; matches existing `position_std_m{0.0}` pattern; smaller. |
| Empty default for `gps_heading_talkers` | GNSS default `{"GP","GN","GL","GA"}` | Empty | Backward compat with existing `$GPHDT`-as-gyro tests; opt-in is honest about installation variance. |
| Adapter-local ring for gyro rate | Pull from `OwnShipProvider` history | Adapter ring | The adapter is the only producer of gyro samples; tighter coupling than touching the provider's history. |
| `gyro_max_age_s=2.0` default | 5 s, 0.5 s | 2 s | At 1 Hz gyro rate this allows one missed update; tighter than that risks spurious staleness when the sentence rate is irregular. |
| Variation fallback chain | Always require explicit variation | Three-level | Real-world feeds vary; operator pragmatism beats math purity here. |

## 12. Acceptance

- Full suite green (currently 385 → target +~12 new tests across the two new files).
- Backward-compat test set in §9.3 unchanged.
- All dispatch paths fire exactly once per applicable sentence; diagnostic counters reconcile.
- No changes to `HeadingBiasEstimator` API; no changes to `IHeadingBiasProvider`; no changes to `Tracker`.

## 13. Operator behavior after this lands

- An operator wiring `HeadingBiasEstimator` via `adapter.setHeadingBiasEstimator(&est)` gets automatic dispatch from whichever NMEA sentences their installation provides. No glue code beyond the pointer.
- Talker-ID config tells the adapter which `HDT`s are gyro vs. GPS-derived; default empty preserves today's behavior.
- Variation fallback covers operators whose HDG strips the variation field, those who only get variation from RMC, and those with a known geographic variation from chart data.
