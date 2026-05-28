#include <gtest/gtest.h>
#include "core/types/Ids.hpp"

using navtracker::TrackId;
using navtracker::TrackStatus;
using navtracker::SensorKind;

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
