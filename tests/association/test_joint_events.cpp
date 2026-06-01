#include <algorithm>
#include <gtest/gtest.h>
#include "core/association/JointEvents.hpp"

using navtracker::enumerateJointEvents;

TEST(JointEvents, NoMeasurementsGivesOneEmptyEvent) {
  Eigen::MatrixXi V(0, 3);
  const auto events = enumerateJointEvents(V);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_TRUE(events[0].empty());
}

TEST(JointEvents, OneMeasurementGatedToOneTrackGivesTwoEvents) {
  Eigen::MatrixXi V(1, 2);
  V << 1, 0;
  const auto events = enumerateJointEvents(V);
  ASSERT_EQ(events.size(), 2u);
  std::vector<int> seen;
  for (const auto& e : events) {
    ASSERT_EQ(e.size(), 1u);
    seen.push_back(e[0]);
  }
  std::sort(seen.begin(), seen.end());
  EXPECT_EQ(seen, (std::vector<int>{0, 1}));
}

TEST(JointEvents, TwoMeasurementsTwoTracksFullGate) {
  Eigen::MatrixXi V(2, 2);
  V << 1, 1,
       1, 1;
  // Feasible events with constraint no two share same non-zero:
  // (0,0), (0,1), (0,2), (1,0), (1,2), (2,0), (2,1) -> 7 events.
  const auto events = enumerateJointEvents(V);
  EXPECT_EQ(events.size(), 7u);
  for (const auto& e : events) {
    if (e[0] != 0 && e[1] != 0) EXPECT_NE(e[0], e[1]);
  }
}

TEST(JointEvents, GatingRestrictsAssignments) {
  Eigen::MatrixXi V(2, 2);
  V << 1, 0,
       0, 1;
  // Measurement 0: clutter or track 1; measurement 1: clutter or track 2.
  // 4 events, no conflict possible.
  const auto events = enumerateJointEvents(V);
  EXPECT_EQ(events.size(), 4u);
}
