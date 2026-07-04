#include <gtest/gtest.h>
#include "core/types/Ids.hpp"

using navtracker::TrackId;
using navtracker::TrackStatus;
using navtracker::SensorKind;
using navtracker::isNonScanningSource;

TEST(Ids, TrackIdCompares) {
  EXPECT_EQ(TrackId{7}, TrackId{7});
  EXPECT_NE(TrackId{7}, TrackId{8});
  EXPECT_LT(TrackId{7}, TrackId{8});
}

TEST(Ids, DefaultsAreSafe) {
  EXPECT_EQ(TrackId{}.value, 0u);
  EXPECT_EQ(TrackStatus{}, TrackStatus::Tentative);
  EXPECT_EQ(SensorKind{}, SensorKind::Unknown);
}

// R10: a shore/VTS remote-track feed is a non-scanning position source (its
// returns are another tracker's filtered outputs, not a swept arc), so it must
// be excluded from occupancy coverage-sector self-estimation exactly like AIS
// and Cooperative, and it is strong vessel-evidence for the suppression veto.
TEST(Ids, RemoteTrackIsNonScanningSource) {
  EXPECT_TRUE(isNonScanningSource(SensorKind::RemoteTrack));
  EXPECT_TRUE(isNonScanningSource(SensorKind::Ais));
  EXPECT_TRUE(isNonScanningSource(SensorKind::Cooperative));
  // Scanning sources sweep a footprint and stay OUT of the set.
  EXPECT_FALSE(isNonScanningSource(SensorKind::ArpaTtm));
  EXPECT_FALSE(isNonScanningSource(SensorKind::ArpaTll));
  EXPECT_FALSE(isNonScanningSource(SensorKind::EoIr));
  EXPECT_FALSE(isNonScanningSource(SensorKind::Lidar));
}
