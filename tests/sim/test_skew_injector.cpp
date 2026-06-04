#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "sim/SkewInjector.hpp"

using navtracker::applySkew;
using navtracker::defaultMaritimeSkewProfile;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::SkewProfile;
using navtracker::Timestamp;

namespace {
Measurement make(double t_s, SensorKind k, const std::string& src = "s") {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = k;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  return m;
}
}  // namespace

TEST(SkewInjector, ZeroProfileIsIdentity) {
  SkewProfile p;  // all zeros
  std::vector<Measurement> in = {
      make(0.0, SensorKind::Ais), make(1.0, SensorKind::Ais),
      make(2.0, SensorKind::OwnShip)};
  const auto out = applySkew(in, p, /*seed=*/1);
  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_EQ(out[i].time.nanos(), in[i].time.nanos());
  }
}

TEST(SkewInjector, TruthTimestampsPreserved) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in = {make(0.0, SensorKind::Ais),
                                 make(0.1, SensorKind::EoIr),
                                 make(0.2, SensorKind::ArpaTtm)};
  const auto out = applySkew(in, p, /*seed=*/42);
  for (const auto& m : out) {
    bool found = false;
    for (const auto& src : in) {
      if (src.time.nanos() == m.time.nanos()) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found);
  }
}

TEST(SkewInjector, DeterministicWithSameSeed) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in;
  for (int i = 0; i < 50; ++i) {
    in.push_back(make(0.1 * i, i % 2 ? SensorKind::Ais : SensorKind::EoIr));
  }
  const auto a = applySkew(in, p, /*seed=*/1234);
  const auto b = applySkew(in, p, /*seed=*/1234);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].time.nanos(), b[i].time.nanos());
    EXPECT_EQ(a[i].sensor, b[i].sensor);
  }
}

TEST(SkewInjector, DifferentSeedsCanProduceDifferentOrder) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in;
  for (int i = 0; i < 50; ++i) {
    in.push_back(make(0.1 * i, SensorKind::Ais));  // big jitter
  }
  const auto a = applySkew(in, p, /*seed=*/1);
  const auto b = applySkew(in, p, /*seed=*/999);
  bool any_diff = false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].time.nanos() != b[i].time.nanos()) {
      any_diff = true;
      break;
    }
  }
  EXPECT_TRUE(any_diff);
}

TEST(SkewInjector, SingleSensorWithZeroJitterIsStableInOrder) {
  SkewProfile p;
  p.at(SensorKind::Ais) = {0.5, 0.0};  // constant lag, no jitter
  std::vector<Measurement> in = {make(0.0, SensorKind::Ais),
                                 make(1.0, SensorKind::Ais),
                                 make(2.0, SensorKind::Ais)};
  const auto out = applySkew(in, p, /*seed=*/7);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 0.0);
  EXPECT_DOUBLE_EQ(out[1].time.seconds(), 1.0);
  EXPECT_DOUBLE_EQ(out[2].time.seconds(), 2.0);
}

TEST(SkewInjector, EmptyInputYieldsEmptyOutput) {
  const auto p = defaultMaritimeSkewProfile();
  const auto out = applySkew({}, p, /*seed=*/0);
  EXPECT_TRUE(out.empty());
}
