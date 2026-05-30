#include <gtest/gtest.h>
#include "core/pipeline/ReorderBuffer.hpp"

using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::ReorderBuffer;
using navtracker::Timestamp;

namespace {
Measurement at(double seconds) {
  Measurement m;
  m.time = Timestamp::fromSeconds(seconds);
  m.model = MeasurementModel::Position2D;
  return m;
}
}  // namespace

TEST(ReorderBuffer, ReleasesInTimeOrderAfterWindow) {
  ReorderBuffer buf(2.0);
  EXPECT_TRUE(buf.push(at(0.0)));
  EXPECT_TRUE(buf.push(at(3.0)));
  auto out = buf.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 0.0);

  EXPECT_TRUE(buf.push(at(1.0)));
  EXPECT_TRUE(buf.push(at(2.0)));
  out = buf.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 1.0);
}

TEST(ReorderBuffer, DropsLateMeasurements) {
  ReorderBuffer buf(2.0);
  buf.push(at(0.0));
  buf.push(at(5.0));
  EXPECT_FALSE(buf.push(at(-1.0)));
  EXPECT_EQ(buf.dropped(), 1u);
}
