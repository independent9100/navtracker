#include <gtest/gtest.h>

#include <cmath>

#include "core/sensor_activity/DeclaredSensorActivity.hpp"

using navtracker::ChannelKind;
using navtracker::DeclaredSensorActivity;
using navtracker::Timestamp;

namespace {
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
}  // namespace

TEST(DeclaredSensorActivity, SurveillanceChargesOneMissPerCompletedSweepInCoverage) {
  DeclaredSensorActivity act({radar()});
  // In coverage (5 km), a full 60 s duty cycle elapsed -> one miss at p_D.
  auto r = act.evaluate({5000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(r.surveillance_miss);
  EXPECT_DOUBLE_EQ(r.p_D, 0.9);
  EXPECT_FALSE(r.cooperative_overdue);
}

TEST(DeclaredSensorActivity, NoMissOutsideCoverage) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({20000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, NoMissMidSweep) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({5000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(30.0));
  EXPECT_FALSE(r.surveillance_miss);
}

// A covered-range track at azimuth 0 sits INSIDE the 60-deg sector: a full
// duty cycle with no return -> surveillance miss. Exercises the
// std::remainder sector branch in DeclaredSensorActivity::inCoverage.
TEST(DeclaredSensorActivity, SurveillanceInSectorMisses) {
  DeclaredSensorActivity act({radarSector()});
  // (5000, 0) -> range 5 km (in), azimuth 0 (centre of sector).
  auto r = act.evaluate({5000.0, 0.0}, std::nullopt, std::nullopt,
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
  auto r = act.evaluate({-5000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, CooperativeOverdueRaisesSignalNotMiss) {
  DeclaredSensorActivity::ChannelProfile coop;
  coop.kind = ChannelKind::Cooperative;
  coop.sensor = navtracker::SensorKind::Cooperative;
  coop.expected_report_interval_sec = 10.0;
  DeclaredSensorActivity act({coop});
  auto r = act.evaluate({0.0, 0.0}, std::nullopt, std::optional<std::uint64_t>{42},
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(25.0));
  EXPECT_FALSE(r.surveillance_miss);
  EXPECT_TRUE(r.cooperative_overdue);
}
