# Sensor Adapters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Plug real sensor inputs into the fusion engine via concrete `ISensorAdapter` implementations for AIS, navigation radar/ARPA (NMEA TTM + TLL), EO/IR camera (with range), and own-ship navigation (NMEA GGA + HDT). Each adapter normalizes its native input into a `Measurement` in the ENU frame with a properly modelled covariance.

**Architecture:** Adapters live in `adapters/` (outside the domain core). They depend only on `core/types/*`, `core/geo/Datum`, the `OwnShipProvider` introduced here, the new `projection` helper, and the NMEA parser. They produce normalized `Measurement`s via the existing `ISensorAdapter::poll()` port (Plan 4). Wire-level decoding below NMEA (AIS AIVDM 6-bit, arbitrary camera protocols) is *out of scope* — the AIS and EO/IR adapters take pre-decoded record structs, leaving room for a real decoder in front of them.

**Tech Stack:** C++17 · Eigen 3.4 · GoogleTest. Builds on plans 1–4 (on `master`).

This is plan 5 of 6. Prereq: plans 1–4 merged — 44 tests pass.

**Documentation standard (CLAUDE.md):** Each algorithm carries Math / Assumptions / Rationale / Ways-to-improve. Concise notes per task; Task 8 writes `docs/algorithms/adapters.md`.

---

## Design notes

- **Adapter-as-normalizer.** Native input goes in; normalized `Measurement`s (ENU frame, SI units, sensor-appropriate covariance) come out. Identity cues fill `hints.mmsi` / `hints.sensor_track_id` when present.
- **Own-ship needed for relative sensors.** ARPA TTM (range/bearing) and EO/IR (bearing+range) are relative to own-ship. They consume an `OwnShipProvider` that exposes the latest pose. Interpolation by time is deferred.
- **Range+bearing → ENU position with proper polar covariance.** The classical polar-to-Cartesian Jacobian: range error stretches along the line-of-sight, bearing error widens cross-range (≈ range·σ_β). Implemented once in a free function and reused by ARPA TTM and EO/IR adapters.
- **TLL is already absolute.** Own-ship error has been folded in by the radar (sensor reference §2); we inflate position R to acknowledge it but cannot decompose.
- **Time supplied per ingest.** NMEA UTC fields are time-of-day only. We require the caller to pass a full `Timestamp` per ingest (matches the engine's time-driven contract from spec D2). UTC-to-Timestamp wiring lives at the I/O boundary above the adapter, out of scope here.
- **EO/IR baseline.** Bearing+range case only. Pure bearing-only is deferred to the next iteration (sensor-reference gotchas §3).

## File Structure

```
adapters/own_ship/OwnShipProvider.hpp/.cpp       latest own-ship pose
adapters/own_ship/OwnShipNmeaAdapter.hpp/.cpp    parses GGA/HDT, updates OwnShipProvider
adapters/util/Projection.hpp/.cpp                projectRangeBearingToEnu (polar->ENU)
adapters/util/Nmea.hpp/.cpp                      parse + checksum-validate NMEA 0183 sentences
adapters/ais/AisAdapter.hpp/.cpp                 record-based AIS adapter
adapters/arpa/ArpaAdapter.hpp/.cpp               NMEA TTM + TLL adapter
adapters/eoir/EoIrAdapter.hpp/.cpp               record-based EO/IR adapter (bearing+range)
docs/algorithms/adapters.md                      math/assumptions/rationale/improve
tests/adapters/own_ship/test_own_ship_provider.cpp
tests/adapters/own_ship/test_own_ship_nmea.cpp
tests/adapters/util/test_projection.cpp
tests/adapters/util/test_nmea.cpp
tests/adapters/ais/test_ais_adapter.cpp
tests/adapters/arpa/test_arpa_adapter.cpp
tests/adapters/eoir/test_eoir_adapter.cpp
tests/adapters/test_end_to_end.cpp
```

All new code outside `core/` — clean separation of adapters from domain.

## Build/test (from repo root)

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

If a command fails with `~/.conan2`/"readonly database", re-run sandboxed off. Commits: `git -c commit.gpgsign=false commit -m "..."`. No pushes.

## Current root CMakeLists.txt (post plan 4, for reference)

The `add_library(navtracker_core …)` list ends with `core/pipeline/Tracker.cpp` and `core/pipeline/ReorderBuffer.cpp`. The test exe list ends with `tests/pipeline/test_tracker.cpp`. We will add adapter sources to `navtracker_core` (since they have no third-party I/O — they're plain transforms) and add test sources to `navtracker_tests`. The existing `target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})` already exposes `adapters/` headers since they live under the repo root.

---

## Task 1: OwnShipPose + OwnShipProvider + polar→ENU projection helper

**Math.** Target ENU position from range `r`, true bearing `β` (rad, 0=N CW), and own-ship ENU position:
`east = own.east + r·sin(β)`, `north = own.north + r·cos(β)`.
Covariance via Jacobian `J = ∂(east,north)/∂(r,β) = [[sin β, r·cos β], [cos β, −r·sin β]]`; `Σ_xy = J · diag(σ_r², σ_β²) · Jᵀ`.
**Assumptions.** Own-ship pose is known *at the measurement time* (interpolation deferred); own-ship position error not folded into Σ_xy (documented improvement); flat-Earth ENU.
**Rationale.** Standard polar-to-Cartesian linearization. Captures the anisotropic ellipse (long along the LOS, wide perpendicular).
**Ways to improve.** Time interpolation of own-ship pose; fold own-ship position/heading uncertainty into Σ_xy; per-sensor range-dependent bias model.

**Files:**
- Create: `adapters/own_ship/OwnShipProvider.hpp`, `adapters/own_ship/OwnShipProvider.cpp`
- Create: `adapters/util/Projection.hpp`, `adapters/util/Projection.cpp`
- Test: `tests/adapters/own_ship/test_own_ship_provider.cpp`, `tests/adapters/util/test_projection.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

Create `tests/adapters/own_ship/test_own_ship_provider.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/own_ship/OwnShipProvider.hpp"

using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;

TEST(OwnShipProvider, StartsEmptyThenReturnsLatest) {
  OwnShipProvider p;
  EXPECT_FALSE(p.latest().has_value());
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(5.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 90.0;
  p.update(pose);
  ASSERT_TRUE(p.latest().has_value());
  EXPECT_DOUBLE_EQ(p.latest()->heading_true_deg, 90.0);
}
```

Create `tests/adapters/util/test_projection.cpp`:
```cpp
#include <cmath>

#include <gtest/gtest.h>
#include "adapters/util/Projection.hpp"

using navtracker::projectRangeBearingToEnu;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST(Projection, EastBearingPutsTargetEastOfOwnShip) {
  const Eigen::Vector2d own(100.0, 200.0);
  const auto out = projectRangeBearingToEnu(
      1000.0, kPi / 2.0, 5.0, 0.01, own);  // 90 deg = east
  EXPECT_NEAR(out.pos_enu.x(), 1100.0, 1e-9);
  EXPECT_NEAR(out.pos_enu.y(), 200.0, 1e-6);
}

TEST(Projection, CovarianceAnisotropyMatchesPolarJacobian) {
  const Eigen::Vector2d own(0.0, 0.0);
  const auto out = projectRangeBearingToEnu(
      1000.0, kPi / 2.0, 5.0, 0.01, own);  // east-pointing
  // East variance dominated by range_std^2 = 25; north by (r*sigma_b)^2 = 100.
  EXPECT_NEAR(out.cov(0, 0), 25.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 1), 100.0, 1e-6);
  EXPECT_NEAR(out.cov(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 0), 0.0, 1e-6);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append to `navtracker_core` source list:
```cmake
  adapters/own_ship/OwnShipProvider.cpp
  adapters/util/Projection.cpp
```

Append to `navtracker_tests` source list:
```cmake
  tests/adapters/own_ship/test_own_ship_provider.cpp
  tests/adapters/util/test_projection.cpp
```

- [ ] **Step 3: Verify it fails**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `adapters/own_ship/OwnShipProvider.hpp` / `adapters/util/Projection.hpp` not found.

- [ ] **Step 4: Create `adapters/own_ship/OwnShipProvider.hpp`**

```cpp
#pragma once

#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

// Time-stamped own-ship pose in WGS-84 geodetic + true heading.
struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};  // 0 = N, +CW
};

// Holds the latest own-ship pose. Interpolation by time is deferred.
class OwnShipProvider {
 public:
  void update(const OwnShipPose& pose);
  std::optional<OwnShipPose> latest() const;

 private:
  std::optional<OwnShipPose> latest_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/own_ship/OwnShipProvider.cpp`**

```cpp
#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

void OwnShipProvider::update(const OwnShipPose& pose) { latest_ = pose; }
std::optional<OwnShipPose> OwnShipProvider::latest() const { return latest_; }

}  // namespace navtracker
```

- [ ] **Step 6: Create `adapters/util/Projection.hpp`**

```cpp
#pragma once

#include <Eigen/Core>

namespace navtracker {

// Result of projecting a polar (range, bearing) measurement into ENU.
struct PointAndCov2D {
  Eigen::Vector2d pos_enu;
  Eigen::Matrix2d cov;
};

// Convert a range + true-bearing measurement (range_std/bearing_std) made
// from own_ship_pos_enu into an ENU position with anisotropic covariance.
// bearing_true_rad: 0 = north, +clockwise (compass convention).
// Own-ship position uncertainty is NOT folded in (documented improvement).
PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       const Eigen::Vector2d& own_ship_pos_enu);

}  // namespace navtracker
```

- [ ] **Step 7: Create `adapters/util/Projection.cpp`**

```cpp
#include "adapters/util/Projection.hpp"

#include <cmath>

namespace navtracker {

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       const Eigen::Vector2d& own_ship_pos_enu) {
  const double sb = std::sin(bearing_true_rad);
  const double cb = std::cos(bearing_true_rad);

  PointAndCov2D out;
  out.pos_enu.x() = own_ship_pos_enu.x() + range_m * sb;
  out.pos_enu.y() = own_ship_pos_enu.y() + range_m * cb;

  // J = d(east, north)/d(range, bearing)
  Eigen::Matrix2d J;
  J << sb,  range_m * cb,
       cb, -range_m * sb;
  Eigen::Matrix2d R;
  R << range_std_m * range_std_m, 0.0,
       0.0, bearing_std_rad * bearing_std_rad;
  out.cov = J * R * J.transpose();
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 8: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 9: Commit**

```bash
git add adapters/own_ship/OwnShipProvider.hpp adapters/own_ship/OwnShipProvider.cpp adapters/util/Projection.hpp adapters/util/Projection.cpp tests/adapters/own_ship/test_own_ship_provider.cpp tests/adapters/util/test_projection.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add OwnShipProvider and polar->ENU projection helper"
```

---

## Task 2: NMEA 0183 parser utility

**Math/Logic.** A NMEA 0183 sentence: `$<TT><FFF>,field1,field2,...*HH` where `<TT>` is a 2-char talker, `<FFF>` is the formatter, and `HH` is the 2-hex-digit XOR checksum over the bytes between `$` and `*`. Valid: starts with `$` (or `!`), contains `*`, checksum matches.
**Assumptions.** One sentence per line; ASCII only; no multi-sentence reassembly here (AIVDM is intentionally out of scope).
**Rationale.** Shared by ARPA and own-ship adapters.
**Ways to improve.** Multi-sentence reassembly; tolerant lenient mode for malformed feeds.

**Files:**
- Create: `adapters/util/Nmea.hpp`, `adapters/util/Nmea.cpp`
- Test: `tests/adapters/util/test_nmea.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/adapters/util/test_nmea.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/util/Nmea.hpp"

using navtracker::parseNmea;

TEST(Nmea, ParsesValidSentence) {
  // Example GGA with correct checksum (sample sentence).
  const auto parsed = parseNmea("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->talker, "GP");
  EXPECT_EQ(parsed->formatter, "GGA");
  ASSERT_GE(parsed->fields.size(), 4u);
  EXPECT_EQ(parsed->fields[0], "123519");
  EXPECT_EQ(parsed->fields[1], "4807.038");
}

TEST(Nmea, RejectsBadChecksum) {
  EXPECT_FALSE(parseNmea("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00").has_value());
}

TEST(Nmea, RejectsMissingDelimiters) {
  EXPECT_FALSE(parseNmea("GPGGA,foo,bar").has_value());
  EXPECT_FALSE(parseNmea("$GPGGA,foo,bar").has_value());  // no *HH
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `adapters/util/Nmea.cpp` to `navtracker_core`. Append `tests/adapters/util/test_nmea.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

`cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build 2>&1 | head -20`. Expected: `adapters/util/Nmea.hpp` not found.

- [ ] **Step 4: Create `adapters/util/Nmea.hpp`**

```cpp
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace navtracker {

struct NmeaSentence {
  std::string talker;     // e.g. "GP", "RA"
  std::string formatter;  // e.g. "GGA", "TTM", "TLL", "HDT"
  std::vector<std::string> fields;
};

// Parse a NMEA 0183 line. Returns empty optional on malformed input or
// failed checksum. Accepts both '$' (NMEA) and '!' (encapsulated, e.g. AIVDM)
// start characters.
std::optional<NmeaSentence> parseNmea(std::string_view line);

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/util/Nmea.cpp`**

```cpp
#include "adapters/util/Nmea.hpp"

#include <cstdint>

namespace navtracker {
namespace {

bool fromHex(char c, std::uint8_t& out) {
  if (c >= '0' && c <= '9') { out = static_cast<std::uint8_t>(c - '0'); return true; }
  if (c >= 'A' && c <= 'F') { out = static_cast<std::uint8_t>(10 + c - 'A'); return true; }
  if (c >= 'a' && c <= 'f') { out = static_cast<std::uint8_t>(10 + c - 'a'); return true; }
  return false;
}

}  // namespace

std::optional<NmeaSentence> parseNmea(std::string_view line) {
  if (line.size() < 8) return std::nullopt;
  if (line.front() != '$' && line.front() != '!') return std::nullopt;
  const auto star = line.find('*');
  if (star == std::string_view::npos) return std::nullopt;
  if (line.size() < star + 3) return std::nullopt;

  std::uint8_t hi = 0, lo = 0;
  if (!fromHex(line[star + 1], hi) || !fromHex(line[star + 2], lo)) return std::nullopt;
  const std::uint8_t expected = static_cast<std::uint8_t>((hi << 4) | lo);

  std::uint8_t cs = 0;
  for (std::size_t i = 1; i < star; ++i) cs ^= static_cast<std::uint8_t>(line[i]);
  if (cs != expected) return std::nullopt;

  const auto comma = line.find(',', 1);
  if (comma == std::string_view::npos || comma >= star) return std::nullopt;
  const auto header = line.substr(1, comma - 1);
  if (header.size() < 3) return std::nullopt;

  NmeaSentence s;
  s.talker = std::string(header.substr(0, 2));
  s.formatter = std::string(header.substr(2));

  std::size_t i = comma + 1;
  while (i <= star) {
    const auto next = line.find_first_of(",*", i);
    const auto end = (next == std::string_view::npos) ? star : next;
    s.fields.emplace_back(line.substr(i, end - i));
    if (end == star) break;
    i = end + 1;
  }
  return s;
}

}  // namespace navtracker
```

- [ ] **Step 6: Create the shared test helper `tests/adapters/util/NmeaTestHelpers.hpp`**

Used by Tasks 4, 5, and 7 to build valid NMEA sentences without hand-computing checksums:
```cpp
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace navtracker_test {

// Build a full NMEA 0183 sentence: prepends '$', appends '*' and the XOR
// checksum (two uppercase hex digits) computed over the body.
inline std::string makeNmea(std::string_view body) {
  std::uint8_t cs = 0;
  for (char c : body) cs ^= static_cast<std::uint8_t>(c);
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02X", cs);
  std::string out = "$";
  out.append(body.data(), body.size());
  out.push_back('*');
  out.append(hex);
  return out;
}

}  // namespace navtracker_test
```

- [ ] **Step 7: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 8: Commit**

```bash
git add adapters/util/Nmea.hpp adapters/util/Nmea.cpp tests/adapters/util/test_nmea.cpp tests/adapters/util/NmeaTestHelpers.hpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add NMEA 0183 parser with checksum validation"
```

---

## Task 3: AisAdapter (record-based)

**Math/Logic.** Position2D measurement: `value = ENU(g, datum)` from `Datum::toEnu`. Covariance: `σ²·I₂` where `σ = 10 m` if `high_accuracy` (DGNSS class), else `30 m` (sensor reference §1). Hints: `mmsi`.
**Assumptions.** Caller pre-decodes AIVDM into `AisDynamicReport`; alt is ignored (surface vessel); accuracy flag mapping is heuristic and tunable.
**Rationale.** Cooperative position with identity — the cleanest data path.
**Ways to improve.** Plausibility gate; use heading/SOG/COG via PositionVelocity2D; calibrate σ from reported accuracy more carefully.

**Files:**
- Create: `adapters/ais/AisAdapter.hpp`, `adapters/ais/AisAdapter.cpp`
- Test: `tests/adapters/ais/test_ais_adapter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/adapters/ais/test_ais_adapter.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"

using navtracker::AisAdapter;
using navtracker::AisDynamicReport;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

TEST(AisAdapter, IngestProducesPosition2DAtOrigin) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(100.0);
  r.mmsi = 211000000u;
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.high_accuracy = true;
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  const Measurement& m = out[0];
  EXPECT_EQ(m.sensor, SensorKind::Ais);
  EXPECT_EQ(m.model, MeasurementModel::Position2D);
  EXPECT_NEAR(m.value(0), 0.0, 1e-6);
  EXPECT_NEAR(m.value(1), 0.0, 1e-6);
  EXPECT_EQ(m.covariance(0, 0), 100.0);  // 10^2
  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 211000000u);
  // poll should drain.
  EXPECT_TRUE(adapter.poll().empty());
}

TEST(AisAdapter, LowAccuracyHasLargerCovariance) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);
  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(0.0);
  r.lat_deg = 53.5;
  r.lon_deg = 8.0;
  r.high_accuracy = false;
  adapter.ingest(r);
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].covariance(0, 0), 900.0);  // 30^2
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `adapters/ais/AisAdapter.cpp` and the test source.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `adapters/ais/AisAdapter.hpp` not found.

- [ ] **Step 4: Create `adapters/ais/AisAdapter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

// Pre-decoded AIS dynamic position report. Wire-level AIVDM decoding is
// out of scope for this adapter.
struct AisDynamicReport {
  Timestamp time;
  std::uint32_t mmsi{0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  bool high_accuracy{false};  // GPS high-accuracy flag (DGNSS)
  std::string source_id{"ais"};
};

class AisAdapter : public ISensorAdapter {
 public:
  explicit AisAdapter(geo::Datum datum);

  void ingest(const AisDynamicReport& r);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/ais/AisAdapter.cpp`**

```cpp
#include "adapters/ais/AisAdapter.hpp"

#include <utility>

namespace navtracker {

AisAdapter::AisAdapter(geo::Datum datum) : datum_(std::move(datum)) {}

void AisAdapter::ingest(const AisDynamicReport& r) {
  const Eigen::Vector3d enu = datum_.toEnu({r.lat_deg, r.lon_deg, 0.0});
  Measurement m;
  m.time = r.time;
  m.sensor = SensorKind::Ais;
  m.source_id = r.source_id;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(enu.x(), enu.y());
  const double sigma = r.high_accuracy ? 10.0 : 30.0;
  m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
  if (r.mmsi != 0) m.hints.mmsi = r.mmsi;
  buffer_.push_back(std::move(m));
}

std::vector<Measurement> AisAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 7: Commit**

```bash
git add adapters/ais/AisAdapter.hpp adapters/ais/AisAdapter.cpp tests/adapters/ais/test_ais_adapter.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add record-based AisAdapter"
```

---

## Task 4: OwnShipNmeaAdapter (GGA + HDT)

**Math/Logic.** `$--GGA,utc,lat_ddmm.mmmm,N/S,lon_dddmm.mmmm,E/W,quality,...` — convert `ddmm.mmmm` to decimal degrees (`dd + mm.mmmm/60`), apply N/S, E/W signs. `$--HDT,heading,T` — heading in degrees true. The caller supplies a full `Timestamp` per ingested line (UTC date wiring is out of scope).
**Assumptions.** One NMEA sentence per `ingest`; non-position sentences ignored; quality-flag filtering deferred.
**Rationale.** Most maritime nav systems emit at least GGA and one of HDT/HDG; this covers the baseline.
**Ways to improve.** Quality-flag gating, RMC for SOG/COG, multi-frame reassembly, full UTC→Timestamp parsing using a supplied date.

**Files:**
- Create: `adapters/own_ship/OwnShipNmeaAdapter.hpp`, `adapters/own_ship/OwnShipNmeaAdapter.cpp`
- Test: `tests/adapters/own_ship/test_own_ship_nmea.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/adapters/own_ship/test_own_ship_nmea.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using navtracker::OwnShipNmeaAdapter;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;
using navtracker_test::makeNmea;

TEST(OwnShipNmeaAdapter, GgaUpdatesPositionInProvider) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // Position: 4807.038 N -> 48 + 7.038/60 deg = 48.1173 deg
  //           01131.000 E -> 11 + 31.000/60 deg = 11.5166666... deg
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->lat_deg, 48.1173, 1e-4);
  EXPECT_NEAR(provider.latest()->lon_deg, 11.5166666, 1e-4);
}

TEST(OwnShipNmeaAdapter, HdtUpdatesHeading) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  adapter.ingest(makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
                 Timestamp::fromSeconds(1000.0));
  EXPECT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"),
                             Timestamp::fromSeconds(1001.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_DOUBLE_EQ(provider.latest()->heading_true_deg, 123.5);
}

TEST(OwnShipNmeaAdapter, RejectsMalformedLines) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  EXPECT_FALSE(adapter.ingest("garbage", Timestamp::fromSeconds(0.0)));
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `adapters/own_ship/OwnShipNmeaAdapter.cpp` and the test source.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `adapters/own_ship/OwnShipNmeaAdapter.hpp` not found.

- [ ] **Step 4: Create `adapters/own_ship/OwnShipNmeaAdapter.hpp`**

```cpp
#pragma once

#include <string_view>

#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

// Parses NMEA 0183 GGA (position) and HDT (true heading) into OwnShipPose
// updates on the supplied OwnShipProvider. The caller supplies a full
// Timestamp per ingest (NMEA carries only UTC time-of-day).
class OwnShipNmeaAdapter {
 public:
  explicit OwnShipNmeaAdapter(OwnShipProvider& provider);

  // Returns true if the line was a recognized GGA or HDT and updated the
  // provider; false otherwise.
  bool ingest(std::string_view line, Timestamp t);

 private:
  OwnShipProvider& provider_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/own_ship/OwnShipNmeaAdapter.cpp`**

```cpp
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"

#include <cstdlib>

#include "adapters/util/Nmea.hpp"

namespace navtracker {
namespace {

double parseDdmm(const std::string& s) {
  // ddmm.mmmm or dddmm.mmmm -> decimal degrees.
  if (s.empty()) return 0.0;
  const auto dot = s.find('.');
  const std::size_t mm_digits = 2;
  const std::size_t deg_end = (dot == std::string::npos ? s.size() : dot) - mm_digits;
  if (deg_end > s.size()) return 0.0;
  const double deg = std::strtod(s.substr(0, deg_end).c_str(), nullptr);
  const double min = std::strtod(s.substr(deg_end).c_str(), nullptr);
  return deg + min / 60.0;
}

}  // namespace

OwnShipNmeaAdapter::OwnShipNmeaAdapter(OwnShipProvider& provider)
    : provider_(provider) {}

bool OwnShipNmeaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;
  OwnShipPose pose = provider_.latest().value_or(OwnShipPose{});
  pose.time = t;

  if (parsed->formatter == "GGA") {
    if (parsed->fields.size() < 5) return false;
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    pose.lat_deg = lat;
    pose.lon_deg = lon;
    provider_.update(pose);
    return true;
  }
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    pose.heading_true_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    provider_.update(pose);
    return true;
  }
  return false;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

(If the HDT checksum string above doesn't match your machine's parser, recompute: XOR all bytes between `$` and `*`, hex-uppercase. For `GPHDT,123.5,T` the XOR is 0x36.)

- [ ] **Step 7: Commit**

```bash
git add adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp tests/adapters/own_ship/test_own_ship_nmea.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add OwnShipNmeaAdapter for GGA/HDT"
```

---

## Task 5: ArpaAdapter (NMEA TTM + TLL)

**Math/Logic.** TTM fields: target_num, distance, bearing, bearing_units (T/R), speed, course, course_units, CPA, TCPA, distance_units (N/K/S), name, status, reference, utc, acquisition. Distance to meters: `N→·1852`, `K→·1000`, `S→·1609.344`. Bearing → true: if `T` already true; if `R` add `own_ship_heading_true`. Then call `projectRangeBearingToEnu` with `(σ_r, σ_β)` defaults of `(50 m, 1° in rad)` (sensor reference §2). Result is a `Position2D` measurement; `hints.sensor_track_id = target_num`.

TLL fields: target_num, lat, N/S, lon, E/W, name, utc, status, reference. Convert lat/lon → ENU via `Datum`. R is inflated isotropic `σ²·I` with `σ = 50 m` (own-ship error already folded by the radar — sensor reference §2). `hints.sensor_track_id = target_num`.

**Assumptions.** Own-ship pose is available when ingesting TTM (skip with `false` if not). Default σ's are configurable per radar.
**Rationale.** Covers the two ARPA target sentences with consistent ENU output. The two paths reuse the same downstream `Measurement` shape.
**Ways to improve.** Calibrate `(σ_r, σ_β)` per radar; fold own-ship pose error into TTM covariance; revisit measurement-vs-track-level fusion for ARPA tracks if biased (spec §13).

**Files:**
- Create: `adapters/arpa/ArpaAdapter.hpp`, `adapters/arpa/ArpaAdapter.cpp`
- Test: `tests/adapters/arpa/test_arpa_adapter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/adapters/arpa/test_arpa_adapter.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using navtracker::ArpaAdapter;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker_test::makeNmea;

namespace {
Datum kDatum({53.5, 8.0, 0.0});
}

TEST(ArpaAdapter, TllProducesPosition2D) {
  OwnShipProvider provider;
  ArpaAdapter adapter(kDatum, provider);
  // Target at (53.51, 8.00) ~ 1.1 km north of datum origin.
  // ddmm.mmmm: 53 + 30.6/60 = 53.51; 008 + 00.0/60 = 8.0
  EXPECT_TRUE(adapter.ingest(
      makeNmea("RATLL,01,5330.6,N,00800.0,E,TARG1,123456,T,R"),
      Timestamp::fromSeconds(10.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::ArpaTll);
  EXPECT_NEAR(out[0].value(0), 0.0, 5.0);
  EXPECT_GT(out[0].value(1), 1000.0);
  EXPECT_LT(out[0].value(1), 1200.0);
  ASSERT_TRUE(out[0].hints.sensor_track_id.has_value());
  EXPECT_EQ(*out[0].hints.sensor_track_id, 1);
}

TEST(ArpaAdapter, TtmProducesPositionUsingOwnShip) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow north
  provider.update(pose);

  ArpaAdapter adapter(kDatum, provider);
  // distance 1.0 NM = 1852 m, bearing 90 true (east), units N.
  EXPECT_TRUE(adapter.ingest(
      makeNmea("RATTM,01,1.0,90.0,T,12.0,90.0,T,0.0,0.0,N,TARG1,T,R,123456.78,A"),
      Timestamp::fromSeconds(5.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::ArpaTtm);
  EXPECT_NEAR(out[0].value(0), 1852.0, 1.0);  // east
  EXPECT_NEAR(out[0].value(1), 0.0, 1.0);     // north
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `adapters/arpa/ArpaAdapter.cpp` and the test source.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `adapters/arpa/ArpaAdapter.hpp` not found.

- [ ] **Step 4: Create `adapters/arpa/ArpaAdapter.hpp`**

```cpp
#pragma once

#include <string_view>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

// Parses NMEA 0183 TTM (range/bearing) and TLL (target lat/lon) into
// Position2D Measurements in the supplied Datum's ENU frame. TTM needs
// the latest own-ship pose to project relative measurements.
class ArpaAdapter : public ISensorAdapter {
 public:
  ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship);

  bool ingest(std::string_view line, Timestamp t);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/arpa/ArpaAdapter.cpp`**

```cpp
#include "adapters/arpa/ArpaAdapter.hpp"

#include <cmath>
#include <cstdlib>
#include <utility>

#include "adapters/util/Nmea.hpp"
#include "adapters/util/Projection.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

double parseDdmm(const std::string& s) {
  if (s.empty()) return 0.0;
  const auto dot = s.find('.');
  const std::size_t mm_digits = 2;
  const std::size_t deg_end = (dot == std::string::npos ? s.size() : dot) - mm_digits;
  if (deg_end > s.size()) return 0.0;
  const double deg = std::strtod(s.substr(0, deg_end).c_str(), nullptr);
  const double min = std::strtod(s.substr(deg_end).c_str(), nullptr);
  return deg + min / 60.0;
}

double distanceToMeters(double value, const std::string& units) {
  if (units == "N") return value * 1852.0;
  if (units == "K") return value * 1000.0;
  if (units == "S") return value * 1609.344;
  return value;  // assume meters if unknown
}

}  // namespace

ArpaAdapter::ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship)
    : datum_(std::move(datum)), own_ship_(own_ship) {}

bool ArpaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;

  if (parsed->formatter == "TLL") {
    if (parsed->fields.size() < 5) return false;
    const int target_num = std::atoi(parsed->fields[0].c_str());
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    const Eigen::Vector3d enu = datum_.toEnu({lat, lon, 0.0});

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::ArpaTll;
    m.source_id = "arpa";
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    const double sigma = 50.0;  // TLL: own-ship error already folded in.
    m.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
    m.hints.sensor_track_id = target_num;
    buffer_.push_back(std::move(m));
    return true;
  }

  if (parsed->formatter == "TTM") {
    if (parsed->fields.size() < 11) return false;
    const auto own_opt = own_ship_.latest();
    if (!own_opt) return false;
    const int target_num = std::atoi(parsed->fields[0].c_str());
    const double dist = std::strtod(parsed->fields[1].c_str(), nullptr);
    const double bearing = std::strtod(parsed->fields[2].c_str(), nullptr);
    const std::string bearing_units = parsed->fields[3];
    const std::string dist_units = parsed->fields[10];
    const double range_m = distanceToMeters(dist, dist_units);
    double bearing_true_deg = bearing;
    if (bearing_units == "R") bearing_true_deg += own_opt->heading_true_deg;
    const double bearing_true_rad = bearing_true_deg * kDeg2Rad;

    const Eigen::Vector3d own_enu = datum_.toEnu({own_opt->lat_deg, own_opt->lon_deg, 0.0});
    const Eigen::Vector2d own_xy(own_enu.x(), own_enu.y());

    const PointAndCov2D out =
        projectRangeBearingToEnu(range_m, bearing_true_rad, 50.0, 1.0 * kDeg2Rad, own_xy);

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::ArpaTtm;
    m.source_id = "arpa";
    m.model = MeasurementModel::Position2D;
    m.value = out.pos_enu;
    m.covariance = out.cov;
    m.hints.sensor_track_id = target_num;
    buffer_.push_back(std::move(m));
    return true;
  }

  return false;
}

std::vector<Measurement> ArpaAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. If a test fails only on checksum, recompute and update the literal as noted.

- [ ] **Step 7: Commit**

```bash
git add adapters/arpa/ArpaAdapter.hpp adapters/arpa/ArpaAdapter.cpp tests/adapters/arpa/test_arpa_adapter.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add ArpaAdapter for NMEA TTM and TLL"
```

---

## Task 6: EoIrAdapter (record-based, bearing + range)

**Math/Logic.** Convert relative bearing (camera-frame, equal to bow-relative for baseline) + range using own-ship pose via `projectRangeBearingToEnu`. Output Position2D measurement.
**Assumptions.** Bearing AND range provided (the "camera with range tracker" case from the user's brief); camera boresight aligned with own-ship bow (no PTZ for the baseline); own-ship pose available.
**Rationale.** Symmetric to ARPA TTM treatment; reuses the same projection helper.
**Ways to improve.** Pure bearing-only flow (needs UKF/particle on the EKF side, or cross-sensor constraint); PTZ + mount calibration; classification → attribute fusion (out of Plan 5).

**Files:**
- Create: `adapters/eoir/EoIrAdapter.hpp`, `adapters/eoir/EoIrAdapter.cpp`
- Test: `tests/adapters/eoir/test_eoir_adapter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/adapters/eoir/test_eoir_adapter.cpp`:
```cpp
#include <cmath>

#include <gtest/gtest.h>
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

using navtracker::CameraDetection;
using navtracker::EoIrAdapter;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;

TEST(EoIrAdapter, IngestProducesPositionAheadOfOwnShip) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow north
  provider.update(pose);

  Datum datum({53.5, 8.0, 0.0});
  EoIrAdapter adapter(datum, provider);

  CameraDetection d;
  d.time = Timestamp::fromSeconds(2.0);
  d.bearing_relative_deg = 0.0;  // straight ahead
  d.range_m = 500.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 20.0;
  d.sensor_track_id = 7;
  adapter.ingest(d);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::EoIr);
  EXPECT_NEAR(out[0].value(0), 0.0, 1.0);    // east
  EXPECT_NEAR(out[0].value(1), 500.0, 1.0);  // north
  ASSERT_TRUE(out[0].hints.sensor_track_id.has_value());
  EXPECT_EQ(*out[0].hints.sensor_track_id, 7);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `adapters/eoir/EoIrAdapter.cpp` and the test source.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `adapters/eoir/EoIrAdapter.hpp` not found.

- [ ] **Step 4: Create `adapters/eoir/EoIrAdapter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

// Pre-decoded camera detection. The baseline assumes the bearing is taken
// relative to own-ship bow (camera boresight aligned), and range is provided.
struct CameraDetection {
  Timestamp time;
  double bearing_relative_deg{0.0};
  double range_m{0.0};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  std::optional<std::int32_t> sensor_track_id;
  std::string source_id{"eo_ir"};
};

class EoIrAdapter : public ISensorAdapter {
 public:
  EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship);

  void ingest(const CameraDetection& d);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/eoir/EoIrAdapter.cpp`**

```cpp
#include "adapters/eoir/EoIrAdapter.hpp"

#include <utility>

#include "adapters/util/Projection.hpp"

namespace navtracker {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}

EoIrAdapter::EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship)
    : datum_(std::move(datum)), own_ship_(own_ship) {}

void EoIrAdapter::ingest(const CameraDetection& d) {
  const auto own_opt = own_ship_.latest();
  if (!own_opt) return;
  const double bearing_true_rad =
      (d.bearing_relative_deg + own_opt->heading_true_deg) * kDeg2Rad;

  const Eigen::Vector3d own_enu = datum_.toEnu({own_opt->lat_deg, own_opt->lon_deg, 0.0});
  const Eigen::Vector2d own_xy(own_enu.x(), own_enu.y());

  const PointAndCov2D out = projectRangeBearingToEnu(
      d.range_m, bearing_true_rad, d.range_std_m, d.bearing_std_deg * kDeg2Rad, own_xy);

  Measurement m;
  m.time = d.time;
  m.sensor = SensorKind::EoIr;
  m.source_id = d.source_id;
  m.model = MeasurementModel::Position2D;
  m.value = out.pos_enu;
  m.covariance = out.cov;
  if (d.sensor_track_id) m.hints.sensor_track_id = d.sensor_track_id;
  buffer_.push_back(std::move(m));
}

std::vector<Measurement> EoIrAdapter::poll() {
  std::vector<Measurement> out;
  out.swap(buffer_);
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 7: Commit**

```bash
git add adapters/eoir/EoIrAdapter.hpp adapters/eoir/EoIrAdapter.cpp tests/adapters/eoir/test_eoir_adapter.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(adapters): add EoIrAdapter for bearing+range detections"
```

---

## Task 7: End-to-end fusion test

A small scenario that runs measurements from AIS + ARPA + EO/IR adapters through the `Tracker` and asserts a fused single track forms with contributions from all three sources.

**Files:**
- Test: `tests/adapters/test_end_to_end.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the test**

Create `tests/adapters/test_end_to_end.cpp`:
```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(EndToEnd, MultiSensorMergesIntoSingleTrack) {
  // Datum at 53.5 N, 8.0 E. Single target ~1.1 km north of datum, bow-north
  // own-ship a few hundred meters south.
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.4955;       // ~500 m south of datum
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);

  AisAdapter ais(datum);
  ArpaAdapter arpa(datum, own);
  EoIrAdapter eo(datum, own);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator estimator(motion, 5.0);
  GnnAssociator associator(30.0);  // generous gate for cross-sensor noise
  TrackManager manager(/*confirm=*/2, /*delete=*/3);
  Tracker tracker(estimator, associator, manager, 10.0);

  // AIS at t=0: lat 53.51 -> ~1.11 km north of datum origin.
  AisDynamicReport a;
  a.time = Timestamp::fromSeconds(0.0);
  a.mmsi = 200000001u;
  a.lat_deg = 53.51;
  a.lon_deg = 8.0;
  a.high_accuracy = true;
  ais.ingest(a);
  for (auto& m : ais.poll()) tracker.process(m);

  // ARPA TLL at t=1, same target.
  arpa.ingest(navtracker_test::makeNmea("RATLL,1,5330.6,N,00800.0,E,T1,123456,T,R"),
              Timestamp::fromSeconds(1.0));
  for (auto& m : arpa.poll()) tracker.process(m);

  // EO/IR at t=2, bearing straight ahead, range ~1.6 km (target north of own-ship).
  CameraDetection d;
  d.time = Timestamp::fromSeconds(2.0);
  d.bearing_relative_deg = 0.0;
  d.range_m = 1600.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 50.0;
  eo.ingest(d);
  for (auto& m : eo.poll()) tracker.process(m);

  ASSERT_EQ(manager.size(), 1u);
  const Track& t = manager.tracks()[0];
  EXPECT_EQ(t.status, TrackStatus::Confirmed);
  EXPECT_GE(t.contributing_sources.size(), 2u);  // multiple sensor IDs
}
```


- [ ] **Step 2: Update CMakeLists.txt**

Append `tests/adapters/test_end_to_end.cpp` to `navtracker_tests`.

- [ ] **Step 3: Build and run**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected green. If the AIS/TLL positions don't fall inside the GNN gate, increase the gate or move the AIS/TLL positions closer to the EO/IR-derived target position.

- [ ] **Step 4: Commit**

```bash
git add tests/adapters/test_end_to_end.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "test(adapters): end-to-end fusion across AIS, ARPA, and EO/IR"
```

---

## Task 8: Adapter documentation

Documentation only.

**Files:**
- Create: `docs/algorithms/adapters.md`

- [ ] **Step 1: Create `docs/algorithms/adapters.md`**

```markdown
# Sensor Adapters

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec section 5, sensor reference
`docs/sensors/sensor-reference.md`.

## 1. Polar → ENU projection (`projectRangeBearingToEnu`)

**Math.** `east = own.east + r·sin β`, `north = own.north + r·cos β`.
`J = ∂(east, north)/∂(r, β) = [[sin β, r·cos β], [cos β, −r·sin β]]`.
`Σ_xy = J · diag(σ_r², σ_β²) · Jᵀ`.

**Assumptions.** Bearing is *true* (north-CW); own-ship pose known at the
measurement time; own-ship position uncertainty NOT folded in; flat-Earth ENU.

**Rationale.** Standard polar-to-Cartesian linearization; captures the
anisotropic ellipse (long along the LOS, wide cross-range as `r·σ_β`).

**Ways to improve / test next.** Time interpolation of own-ship pose; fold
own-ship position/heading uncertainty into Σ_xy; range-dependent bias.

## 2. AisAdapter

**Math/Logic.** `value = ENU(lat, lon, datum)`; `R = σ²·I₂` with
`σ = 10 m` (high accuracy / DGNSS) or `σ = 30 m` (low accuracy).

**Assumptions.** Caller pre-decodes AIVDM; alt ignored (surface vessel).

**Rationale.** Cooperative + identity → cleanest path; MMSI becomes a hint.

**Ways to improve / test next.** Plausibility gate; PositionVelocity2D using
SOG/COG; data-driven `σ` from reported accuracy.

## 3. OwnShipNmeaAdapter (GGA, HDT)

**Math/Logic.** GGA `ddmm.mmmm` → decimal degrees (`dd + mm.mmmm/60`),
sign by N/S, E/W. HDT `heading,T` → degrees true. Caller supplies a full
`Timestamp` per ingest.

**Assumptions.** One sentence per call; non-GGA/HDT ignored; quality flag
not gated.

**Rationale.** Minimal, common baseline; works with any nav feed that emits
position and heading.

**Ways to improve / test next.** Quality-flag gating, RMC parsing, full
UTC→Timestamp parsing, multi-frame combining.

## 4. ArpaAdapter (TTM, TLL)

**Math/Logic.** TTM: distance in `N/K/S` → meters; bearing made true if `R`
by adding `own_ship_heading_true`; then `projectRangeBearingToEnu` with
`(σ_r, σ_β) = (50 m, 1°)` defaults (sensor reference §2). TLL: lat/lon →
ENU via `Datum`; isotropic `R = σ²·I` with `σ = 50 m` (own-ship error
already folded in by the radar — sensor reference §2). Sensor track id →
`hints.sensor_track_id` in both.

**Assumptions.** Own-ship pose available for TTM; default `σ`s are
configurable per radar; bearing field correctly tagged `T`/`R`.

**Rationale.** Covers both ARPA target sentences with consistent ENU output;
shares the projection helper.

**Ways to improve / test next.** Per-radar calibrated `(σ_r, σ_β)`; fold
own-ship pose uncertainty into TTM covariance; revisit measurement-vs-
track-level fusion for ARPA tracks if biased.

## 5. EoIrAdapter (bearing + range)

**Math/Logic.** `bearing_true = bearing_relative + own_ship_heading_true`;
then `projectRangeBearingToEnu` using the supplied `(σ_r, σ_β)`.

**Assumptions.** Camera boresight aligned with own-ship bow (no PTZ for the
baseline); range and bearing both available; own-ship pose available.

**Rationale.** Symmetric to ARPA TTM; reuses projection helper. Useful when
EO/IR is equipped with a rangefinder (stereo / lidar / laser).

**Ways to improve / test next.** Bearing-only flow (needs UKF/particle on
the EKF side); PTZ + mount calibration; classification → attribute fusion.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/adapters.md
git -c commit.gpgsign=false commit -m "docs: add sensor-adapters algorithm reference"
```

---

## Done criteria

- Full suite green.
- `AisAdapter`, `ArpaAdapter`, `OwnShipNmeaAdapter`, `EoIrAdapter` exist in `adapters/` and implement (or feed) `ISensorAdapter`.
- Shared helpers: `OwnShipProvider`, `projectRangeBearingToEnu`, `parseNmea`.
- End-to-end test demonstrates AIS + ARPA + EO/IR converging on one fused track.
- `docs/algorithms/adapters.md` documents math, assumptions, rationale, and improvement paths.

## Roadmap (remaining plan)

6. **Scenario harness + metrics** — synthetic ground-truth scenarios, OSPA/track-accuracy for evaluating estimator/association choices and comparing alternatives flagged in plans 2–5.
