# NMEA Multi-Heading-Source Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire NMEA-side dispatch for the v3 multi-heading-source bias observations. Parse `HDG`, route `HDT` by talker ID, forward RMC variation, and dispatch the right `HeadingBiasEstimator::observe(...)` overload per sentence. Backward-compatible defaults.

**Architecture:** `OwnShipNmeaAdapter` holds an optional `HeadingBiasEstimator*`. Three new NaN-signaled fields on `OwnShipPose`. Adapter keeps a small gyro-HDT ring buffer for rate computation. Variation fallback chain (HDG → RMC cache → config → reject).

**Spec:** `docs/superpowers/specs/2026-06-04-nmea-multi-heading-wiring-design.md`.

---

### Task 1 — Add NaN-signaled fields to OwnShipPose

**Files:**
- Modify: `adapters/own_ship/OwnShipProvider.hpp`

- [ ] **Step 1: Extend OwnShipPose**

In `adapters/own_ship/OwnShipProvider.hpp`, modify the `OwnShipPose` struct. After the existing `bool velocity_is_valid{false};` line, before the closing brace, add:

```cpp
  // Multi-heading-source fields (v3 NMEA wiring). NaN = not present.
  double gps_true_heading_deg{std::nan("")};
  double gps_true_heading_std_deg{0.0};
  double magnetic_heading_deg{std::nan("")};
  double magnetic_heading_std_deg{0.0};
  double magnetic_variation_deg{std::nan("")};
```

Add `#include <cmath>` near the existing includes if not already present (for `std::nan`).

- [ ] **Step 2: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_nmea 2>&1 | tail -3`
Expected: clean (no consumers touch the new fields yet).

- [ ] **Step 3: Commit**

```bash
git add adapters/own_ship/OwnShipProvider.hpp
git commit -m "feat(own_ship): NaN-signaled fields on OwnShipPose for v3 NMEA wiring"
```

---

### Task 2 — Config + estimator pointer + gyro history scaffolding

**Files:**
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.hpp`

- [ ] **Step 1: Forward-declare the estimator**

Near the top, after the existing includes, add:

```cpp
#include <array>
#include <cstddef>
#include <string>
#include <unordered_set>
```

Forward-declare in the navtracker namespace before the `OwnShipNmeaAdapterConfig` struct:

```cpp
class HeadingBiasEstimator;
```

- [ ] **Step 2: Extend config**

At the end of `OwnShipNmeaAdapterConfig`, before the closing brace, add:

```cpp
  // Multi-heading-source wiring (v3 NMEA). Empty default for talkers
  // preserves backward compat with $GPHDT-as-gyro consumers.
  std::unordered_set<std::string> gps_heading_talkers{};
  double gps_heading_sigma_deg{0.5};
  double magnetic_heading_sigma_deg{0.5};
  double gps_cog_sigma_deg{1.0};
  double magnetic_variation_fallback_deg{std::nan("")};
  double gyro_max_age_s{2.0};
```

(Top of file may need `#include <cmath>` if not present.)

- [ ] **Step 3: Add setter, diagnostic getters, and members to the class**

In `class OwnShipNmeaAdapter` public section, after `void setPositionStd(double sigma_m);`:

```cpp
  // Optional. When non-null, the adapter dispatches the appropriate
  // HeadingBiasEstimator::observe(...) overload for each parsed
  // HDG / GPS-talker HDT / RMC. See spec §5.2.
  void setHeadingBiasEstimator(HeadingBiasEstimator* estimator) {
    bias_estimator_ = estimator;
  }

  // Diagnostics.
  std::size_t dispatchedGpsHeading() const { return d_gps_hdg_; }
  std::size_t dispatchedGpsCog()     const { return d_cog_; }
  std::size_t dispatchedMagnetic()   const { return d_mag_; }
  std::size_t skippedMagNoVariation() const { return skip_mag_var_; }
  std::size_t skippedGyroStale()      const { return skip_stale_; }
```

In the private section, after the existing members, add:

```cpp
  HeadingBiasEstimator* bias_estimator_{nullptr};

  // Gyro HDT ring (last 4 samples) for rate computation.
  struct GyroSample { Timestamp t; double heading_rad{0.0}; };
  std::array<GyroSample, 4> gyro_history_{};
  std::size_t gyro_history_count_{0};
  std::size_t gyro_history_head_{0};

  // Variation cache (last value seen from HDG or RMC; deg, signed).
  double cached_variation_deg_{std::nan("")};

  // Diagnostic counters.
  std::size_t d_gps_hdg_{0};
  std::size_t d_cog_{0};
  std::size_t d_mag_{0};
  std::size_t skip_mag_var_{0};
  std::size_t skip_stale_{0};
```

- [ ] **Step 4: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_nmea 2>&1 | tail -3`
Expected: clean (the class compiles even without dispatching anything yet — bias_estimator_ unused so far, but will be used in Task 3).

- [ ] **Step 5: Commit**

```bash
git add adapters/own_ship/OwnShipNmeaAdapter.hpp
git commit -m "feat(nmea): config + estimator pointer scaffolding for multi-heading wiring"
```

---

### Task 3 — HDG parser, RMC variation, HDT routing, dispatch

**Files:**
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.cpp`

- [ ] **Step 1: Add includes**

At the top of the file, add:

```cpp
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/bias/HeadingBiasObservations.hpp"
```

- [ ] **Step 2: Add small helpers in the anonymous namespace**

In the anonymous namespace (after `kDegToRad`), append:

```cpp
double signedFromDir(double magnitude, const std::string& dir) {
  if (dir == "E") return magnitude;
  if (dir == "W") return -magnitude;
  return std::nan("");
}

double wrapDegToPi(double a) {
  // Convert degrees in [0,360) (or beyond) to radians in (-pi, pi].
  double rad = a * kDegToRad;
  constexpr double kPi = 3.14159265358979323846;
  while (rad > kPi) rad -= 2.0 * kPi;
  while (rad <= -kPi) rad += 2.0 * kPi;
  return rad;
}
```

- [ ] **Step 3: Implement gyro-history helpers as private methods**

Above the existing `OwnShipNmeaAdapter::OwnShipNmeaAdapter` constructor, add free-function helpers in the navtracker namespace (we keep them out of the class header for simplicity by making them anonymous-namespace methods that take a `const OwnShipNmeaAdapterConfig&` and a `GyroSample` array — but actually they need access to the gyro_history_, so they belong as private members). Instead, implement them as private members.

Add these declarations to `OwnShipNmeaAdapter.hpp` (in the private section, before the existing members):

```cpp
void pushGyroSample(Timestamp t, double heading_deg);
std::optional<double> latestGyroRad(Timestamp t,
                                    double max_age_s) const;
double gyroRateRadPerSec(Timestamp t, double max_dt_s) const;
```

Also add `#include <optional>` to the header if not already there.

Then in `OwnShipNmeaAdapter.cpp`, before `bool OwnShipNmeaAdapter::ingest(...)`, add:

```cpp
void OwnShipNmeaAdapter::pushGyroSample(Timestamp t, double heading_deg) {
  const std::size_t idx = (gyro_history_head_ + gyro_history_count_)
                        % gyro_history_.size();
  if (gyro_history_count_ < gyro_history_.size()) {
    gyro_history_[idx].t = t;
    gyro_history_[idx].heading_rad = wrapDegToPi(heading_deg);
    ++gyro_history_count_;
  } else {
    gyro_history_[gyro_history_head_].t = t;
    gyro_history_[gyro_history_head_].heading_rad = wrapDegToPi(heading_deg);
    gyro_history_head_ = (gyro_history_head_ + 1) % gyro_history_.size();
  }
}

std::optional<double> OwnShipNmeaAdapter::latestGyroRad(
    Timestamp t, double max_age_s) const {
  if (gyro_history_count_ == 0) return std::nullopt;
  const std::size_t last_idx =
      (gyro_history_head_ + gyro_history_count_ - 1) % gyro_history_.size();
  const auto& s = gyro_history_[last_idx];
  const double age = t.secondsSince(s.t);
  if (age < 0.0 || age > max_age_s) return std::nullopt;
  return s.heading_rad;
}

double OwnShipNmeaAdapter::gyroRateRadPerSec(Timestamp t,
                                             double max_dt_s) const {
  if (gyro_history_count_ < 2) return 0.0;
  const std::size_t n = gyro_history_count_;
  const std::size_t last_idx =
      (gyro_history_head_ + n - 1) % gyro_history_.size();
  const std::size_t prev_idx =
      (gyro_history_head_ + n - 2) % gyro_history_.size();
  const auto& a = gyro_history_[prev_idx];
  const auto& b = gyro_history_[last_idx];
  const double dt = b.t.secondsSince(a.t);
  if (dt <= 0.0 || dt > max_dt_s) return 0.0;
  constexpr double kPi = 3.14159265358979323846;
  double dh = b.heading_rad - a.heading_rad;
  while (dh > kPi) dh -= 2.0 * kPi;
  while (dh < -kPi) dh += 2.0 * kPi;
  // Suppress an unused parameter warning when t is not consulted.
  (void)t;
  return dh / dt;
}
```

- [ ] **Step 4: Update HDT branch with talker-ID routing + dispatch**

Replace the existing HDT branch:

```cpp
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    pose.heading_true_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    provider_.update(pose);
    return true;
  }
```

with:

```cpp
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    const double heading_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    const bool routed_gps =
        cfg_.gps_heading_talkers.count(parsed->talker) > 0;
    if (routed_gps) {
      pose.gps_true_heading_deg = heading_deg;
      pose.gps_true_heading_std_deg = cfg_.gps_heading_sigma_deg;
      provider_.update(pose);
      if (bias_estimator_ != nullptr) {
        const auto gyro_rad =
            latestGyroRad(t, cfg_.gyro_max_age_s);
        if (gyro_rad.has_value()) {
          GyroVsGpsHeadingObservation obs;
          obs.time = t;
          obs.gyro_rad = *gyro_rad;
          obs.gps_true_heading_rad = wrapDegToPi(heading_deg);
          obs.gps_true_heading_std_rad =
              cfg_.gps_heading_sigma_deg * kDegToRad;
          bias_estimator_->observe(obs);
          ++d_gps_hdg_;
        } else {
          ++skip_stale_;
        }
      }
    } else {
      pose.heading_true_deg = heading_deg;
      pushGyroSample(t, heading_deg);
      provider_.update(pose);
    }
    return true;
  }
```

- [ ] **Step 5: Add HDG parser branch**

Above the `return false;` at the end of `ingest`, before the existing closing brace, insert:

```cpp
  if (parsed->formatter == "HDG") {
    if (parsed->fields.empty()) return false;
    const double mag_raw_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    double dev_deg = 0.0;
    if (parsed->fields.size() > 2 && !parsed->fields[1].empty()) {
      const double dev_mag = std::strtod(parsed->fields[1].c_str(), nullptr);
      const double dev_signed = signedFromDir(dev_mag, parsed->fields[2]);
      if (!std::isnan(dev_signed)) dev_deg = dev_signed;
    }
    const double mag_corr_deg = mag_raw_deg + dev_deg;
    pose.magnetic_heading_deg = mag_corr_deg;
    pose.magnetic_heading_std_deg = cfg_.magnetic_heading_sigma_deg;

    double variation_deg = std::nan("");
    if (parsed->fields.size() > 4 && !parsed->fields[3].empty()
                                  && !parsed->fields[4].empty()) {
      const double var_mag = std::strtod(parsed->fields[3].c_str(), nullptr);
      variation_deg = signedFromDir(var_mag, parsed->fields[4]);
    }
    if (!std::isnan(variation_deg)) {
      pose.magnetic_variation_deg = variation_deg;
      cached_variation_deg_ = variation_deg;
    }
    provider_.update(pose);

    if (bias_estimator_ != nullptr) {
      const auto gyro_rad = latestGyroRad(t, cfg_.gyro_max_age_s);
      if (!gyro_rad.has_value()) {
        ++skip_stale_;
      } else {
        double var_use_deg = variation_deg;
        if (std::isnan(var_use_deg)) var_use_deg = cached_variation_deg_;
        if (std::isnan(var_use_deg)) {
          var_use_deg = cfg_.magnetic_variation_fallback_deg;
        }
        if (std::isnan(var_use_deg)) {
          ++skip_mag_var_;
        } else {
          GyroVsMagneticObservation obs;
          obs.time = t;
          obs.gyro_rad = *gyro_rad;
          obs.magnetic_heading_rad = wrapDegToPi(mag_corr_deg);
          obs.magnetic_heading_std_rad =
              cfg_.magnetic_heading_sigma_deg * kDegToRad;
          obs.magnetic_variation_rad = var_use_deg * kDegToRad;
          bias_estimator_->observe(obs);
          ++d_mag_;
        }
      }
    }
    return true;
  }
```

- [ ] **Step 6: Extend RMC branch — variation forwarding + COG dispatch**

In the existing RMC branch, after the line `rmc_buffer_.has_value = true;`, before `return true;`, insert:

```cpp
    // Forward magnetic variation if present (fields 9, 10).
    if (parsed->fields.size() > 10 && !parsed->fields[9].empty()
                                   && !parsed->fields[10].empty()) {
      const double var_mag = std::strtod(parsed->fields[9].c_str(), nullptr);
      const double var_signed = signedFromDir(var_mag, parsed->fields[10]);
      if (!std::isnan(var_signed)) cached_variation_deg_ = var_signed;
    }

    if (bias_estimator_ != nullptr) {
      const auto gyro_rad = latestGyroRad(t, cfg_.gyro_max_age_s);
      if (!gyro_rad.has_value()) {
        ++skip_stale_;
      } else {
        GyroVsGpsCogObservation obs;
        obs.time = t;
        obs.gyro_rad = *gyro_rad;
        obs.gps_cog_rad = wrapDegToPi(cog_deg);
        obs.gps_cog_std_rad = cfg_.gps_cog_sigma_deg * kDegToRad;
        obs.sog_mps = sog_m_per_s;
        obs.gyro_rate_rad_per_s = gyroRateRadPerSec(t, cfg_.gyro_max_age_s);
        bias_estimator_->observe(obs);
        ++d_cog_;
      }
    }
```

- [ ] **Step 7: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_nmea 2>&1 | tail -5`
Expected: clean.

- [ ] **Step 8: Sanity — existing NMEA tests still pass**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'OwnShipNmea' --output-on-failure 2>&1 | tail -10`
Expected: existing tests still green (default empty gps_heading_talkers + no bias estimator wired = no behavior change).

- [ ] **Step 9: Commit**

```bash
git add adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp
git commit -m "feat(nmea): HDG parser, talker-ID HDT routing, RMC variation, bias dispatch"
```

---

### Task 4 — Sentence-parsing tests

**Files:**
- Create: `tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp`

- [ ] **Step 1: Look up how the existing test file builds an NMEA string**

Look at `tests/adapters/own_ship/test_own_ship_nmea.cpp` for the `makeNmea` helper pattern (it builds a `$..` line and appends a checksum). Reuse it.

- [ ] **Step 2: Write the test file**

```cpp
#include <cmath>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

std::string makeNmea(const std::string& payload) {
  std::uint8_t cs = 0;
  for (char c : payload) cs ^= static_cast<std::uint8_t>(c);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + payload + buf;
}

Timestamp at(double s) { return Timestamp::fromSeconds(s); }

}  // namespace

TEST(NmeaMultiHeading, HdgWithDeviationAndVariationParsed) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("IIHDG,123.5,1.0,E,3.0,W"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->magnetic_heading_deg, 124.5, 1e-9);
  EXPECT_NEAR(pose->magnetic_variation_deg, -3.0, 1e-9);
}

TEST(NmeaMultiHeading, HdgWithEmptyVariationLeavesNan) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("IIHDG,90.0,,,,,"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->magnetic_heading_deg, 90.0, 1e-9);
  EXPECT_TRUE(std::isnan(pose->magnetic_variation_deg));
}

TEST(NmeaMultiHeading, GpHdtRoutesAsGyroByDefault) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->heading_true_deg, 123.5, 1e-9);
  EXPECT_TRUE(std::isnan(pose->gps_true_heading_deg));
}

TEST(NmeaMultiHeading, GpHdtRoutesAsGpsHeadingWhenConfigured) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  ASSERT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->gps_true_heading_deg, 123.5, 1e-9);
  EXPECT_DOUBLE_EQ(pose->heading_true_deg, 0.0);  // untouched
}
```

- [ ] **Step 3: Register in CMake**

Find `tests/adapters/own_ship/test_own_ship_nmea.cpp` in `CMakeLists.txt` and add `tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp` after it.

- [ ] **Step 4: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'NmeaMultiHeading' --output-on-failure 2>&1 | tail -15
```
Expected: 4/4 pass.

- [ ] **Step 5: Commit**

```bash
git add tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp CMakeLists.txt
git commit -m "test(nmea): HDG parser and talker-ID HDT routing"
```

---

### Task 5 — Dispatch tests

**Files:**
- Create: `tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <cmath>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

std::string makeNmea(const std::string& payload) {
  std::uint8_t cs = 0;
  for (char c : payload) cs ^= static_cast<std::uint8_t>(c);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + payload + buf;
}

Timestamp at(double s) { return Timestamp::fromSeconds(s); }

}  // namespace

TEST(NmeaBiasDispatch, HdgDispatchesMagneticAfterGyro) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(makeNmea("IIHDG,97.0,0.0,E,3.0,E"), at(1.1));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_EQ(adapter.dispatchedMagnetic(), 1u);
}

TEST(NmeaBiasDispatch, HdgWithoutGyroSkipsAndCounts) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDG,97.0,0.0,E,3.0,E"), at(1.0));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(adapter.skippedGyroStale(), 1u);
}

TEST(NmeaBiasDispatch, HdgWithoutVariationAndNoCacheSkips) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(makeNmea("IIHDG,97.0,,,,,"), at(1.1));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(adapter.skippedMagNoVariation(), 1u);
}

TEST(NmeaBiasDispatch, RmcVariationCachedAndUsedByLaterHdg) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  // RMC carries variation 3.0E.
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,5.0,0.0,010100,3.0,E,A"),
      at(1.5));
  // HDG without its own variation — should use cache.
  adapter.ingest(makeNmea("IIHDG,97.0,,,,,"), at(2.0));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_EQ(adapter.skippedMagNoVariation(), 0u);
}

TEST(NmeaBiasDispatch, RmcDispatchesCogWhenGyroFresh) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  // SOG = 5 m/s (>3 default); cog = 100 (matches heading).
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,9.72,100.0,010100,0.0,E,A"),
      at(1.5));
  EXPECT_EQ(est.acceptedGpsCog(), 1u);
  EXPECT_EQ(adapter.dispatchedGpsCog(), 1u);
}

TEST(NmeaBiasDispatch, RmcLowSogRejectedByEstimatorButStillDispatched) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  // SOG = 1 knot ~ 0.5 m/s (below 3.0 default gate).
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,1.0,100.0,010100,0.0,E,A"),
      at(1.5));
  EXPECT_EQ(adapter.dispatchedGpsCog(), 1u);
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogBySog(), 1u);
}

TEST(NmeaBiasDispatch, GpsHdtRoutedAndDispatchedWhenConfigured) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));  // gyro from II
  adapter.ingest(makeNmea("GPHDT,99.0,T"), at(1.1));   // GPS true heading
  EXPECT_EQ(est.acceptedGpsHeading(), 1u);
  EXPECT_EQ(adapter.dispatchedGpsHeading(), 1u);
}

TEST(NmeaBiasDispatch, GpsHdtWithoutGyroSkipsStale) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("GPHDT,99.0,T"), at(1.0));
  EXPECT_EQ(est.acceptedGpsHeading(), 0u);
  EXPECT_EQ(adapter.skippedGyroStale(), 1u);
}
```

- [ ] **Step 2: Register in CMake**

After the multi-heading test source, add `tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp`.

- [ ] **Step 3: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'NmeaBiasDispatch' --output-on-failure 2>&1 | tail -20
```
Expected: 8/8 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp CMakeLists.txt
git commit -m "test(nmea): dispatch tests for multi-heading-source bias paths"
```

---

### Task 6 — Final sweep

- [ ] **Step 1: Full suite**

Run: `cd /home/andreas/workspace/navtracker/build && ctest --output-on-failure 2>&1 | tail -3`
Expected: previous count + 12 new (4 parsing + 8 dispatch).

- [ ] **Step 2: Acceptance checklist (spec §12)**

Confirm:
- Backward-compat regression: all existing `OwnShipNmea` tests still pass (default empty `gps_heading_talkers` preserves `$GPHDT`-as-gyro behavior).
- All four dispatch paths fire exactly once per applicable sentence.
- Diagnostic counters reconcile (each dispatch ↔ counter increment).
- No changes to `HeadingBiasEstimator` API or `IHeadingBiasProvider`.
