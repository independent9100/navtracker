#include <gtest/gtest.h>

#include <cmath>

#include "core/sensor_activity/DeclaredSensorActivity.hpp"

using navtracker::ChannelKind;
using navtracker::DeclaredSensorActivity;
using navtracker::Timestamp;

namespace {
// Own-ship fixed at the datum origin (the historical assumption). Passing this
// keeps the pre-W2.4a behaviour: coverage measured from (0,0).
const Eigen::Vector2d kAtOrigin{0.0, 0.0};

DeclaredSensorActivity::ChannelProfile radar() {
  DeclaredSensorActivity::ChannelProfile p;
  p.kind = ChannelKind::Surveillance;
  p.sensor = navtracker::SensorKind::ArpaTtm;
  p.duty_cycle_sec = 60.0;
  p.max_range_m = 10000.0;
  p.p_D = 0.9;
  return p;
}

// Same radar, but a 60-degree-wide sector centred on +x (azimuth 0).
DeclaredSensorActivity::ChannelProfile radarSector() {
  DeclaredSensorActivity::ChannelProfile p = radar();
  p.sector_center_rad = 0.0;
  p.sector_width_rad = M_PI / 3.0;  // 60 deg total (±30 deg about 0)
  return p;
}

DeclaredSensorActivity::ChannelProfile coop() {
  DeclaredSensorActivity::ChannelProfile p;
  p.kind = ChannelKind::Cooperative;
  p.sensor = navtracker::SensorKind::Cooperative;
  p.expected_report_interval_sec = 10.0;
  return p;
}
}  // namespace

TEST(DeclaredSensorActivity, SurveillanceChargesOneMissPerCompletedSweepInCoverage) {
  DeclaredSensorActivity act({radar()});
  // In coverage (5 km), a full 60 s duty cycle elapsed -> one miss at p_D.
  auto r = act.evaluate({5000.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(r.surveillance_miss);
  EXPECT_DOUBLE_EQ(r.p_D, 0.9);
  EXPECT_FALSE(r.cooperative_overdue);
}

TEST(DeclaredSensorActivity, NoMissOutsideCoverage) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({20000.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, NoMissMidSweep) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({5000.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(30.0));
  EXPECT_FALSE(r.surveillance_miss);
}

// A covered-range track at azimuth 0 sits INSIDE the 60-deg sector: a full
// duty cycle with no return -> surveillance miss. Exercises the
// std::remainder sector branch in DeclaredSensorActivity::inCoverage.
TEST(DeclaredSensorActivity, SurveillanceInSectorMisses) {
  DeclaredSensorActivity act({radarSector()});
  // (5000, 0) -> range 5 km (in), azimuth 0 (centre of sector).
  auto r = act.evaluate({5000.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(r.surveillance_miss);
  EXPECT_DOUBLE_EQ(r.p_D, 0.9);
}

// A covered-range track at azimuth 180 deg is OUTSIDE the 60-deg sector even
// though it is within max_range: the sector gate (off = pi > pi/6) suppresses
// the miss. Exercises the std::remainder sector branch (false path).
TEST(DeclaredSensorActivity, SurveillanceOutsideSectorNoMiss) {
  DeclaredSensorActivity act({radarSector()});
  // (-5000, 0) -> range 5 km (in range), azimuth pi (180 deg, behind sector).
  auto r = act.evaluate({-5000.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, CooperativeOverdueRaisesSignalNotMiss) {
  DeclaredSensorActivity act({coop()});
  auto r = act.evaluate({0.0, 0.0}, kAtOrigin, std::nullopt,
                        std::optional<std::uint64_t>{42},
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(25.0));
  EXPECT_FALSE(r.surveillance_miss);
  EXPECT_TRUE(r.cooperative_overdue);
}

// -----------------------------------------------------------------------------
// W2.4a — coverage is measured from own-ship, not the ENU datum origin.
// -----------------------------------------------------------------------------

TEST(DeclaredSensorActivity, RangeMeasuredFromOwnShipNotOrigin) {
  DeclaredSensorActivity act({radar()});  // max_range 10 km
  // Own-ship 8 km east of the datum; track 12 km east of the datum.
  //   from origin:   |12000| = 12 km  > 10 km  -> would be OUT (old bug)
  //   from own-ship: |12000-8000| = 4 km < 10 km -> IN -> miss.
  auto in = act.evaluate({12000.0, 0.0}, {8000.0, 0.0}, std::nullopt,
                         std::nullopt, Timestamp::fromSeconds(0.0),
                         Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(in.surveillance_miss);

  // Track 5 km east of the datum (would be IN from origin), but own-ship 8 km
  // WEST -> |5000-(-8000)| = 13 km > 10 km -> OUT -> no miss.
  auto out = act.evaluate({5000.0, 0.0}, {-8000.0, 0.0}, std::nullopt,
                          std::nullopt, Timestamp::fromSeconds(0.0),
                          Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(out.surveillance_miss);
}

TEST(DeclaredSensorActivity, SectorAzimuthMeasuredFromOwnShip) {
  DeclaredSensorActivity act({radarSector()});  // ±30° about east (az 0)
  // Track due north of the datum (az from origin = +90° -> OUTSIDE sector),
  // but own-ship at (-5000, 5000) makes rel = (5000, 0) -> az 0 -> INSIDE.
  auto r = act.evaluate({0.0, 5000.0}, {-5000.0, 5000.0}, std::nullopt,
                        std::nullopt, Timestamp::fromSeconds(0.0),
                        Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(r.surveillance_miss);
}

// -----------------------------------------------------------------------------
// W2.4b — cooperative-overdue is keyed on identity; a radar-only track (no
// MMSI / platform id) never goes "overdue" on a cooperative channel.
// -----------------------------------------------------------------------------

TEST(DeclaredSensorActivity, CooperativeOverdueRequiresIdentity) {
  DeclaredSensorActivity act({coop()});
  const auto t0 = Timestamp::fromSeconds(0.0);
  const auto t1 = Timestamp::fromSeconds(25.0);  // well past the 10 s interval

  // Radar-only track (no identity): NOT overdue — silence on AIS is not evidence
  // for a target that never announces there.
  auto none = act.evaluate({0.0, 0.0}, kAtOrigin, std::nullopt, std::nullopt,
                           t0, t1);
  EXPECT_FALSE(none.cooperative_overdue);

  // MMSI-carrying track: overdue.
  auto by_mmsi = act.evaluate({0.0, 0.0}, kAtOrigin,
                              std::optional<std::uint32_t>{123456789},
                              std::nullopt, t0, t1);
  EXPECT_TRUE(by_mmsi.cooperative_overdue);

  // Platform-id-carrying track: overdue.
  auto by_platform = act.evaluate({0.0, 0.0}, kAtOrigin, std::nullopt,
                                  std::optional<std::uint64_t>{7}, t0, t1);
  EXPECT_TRUE(by_platform.cooperative_overdue);
}
