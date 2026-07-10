// Unit tests for the pedigree independence rule (ticket §6.3, second bullet).
// Full truth table including defaults, and absent == all-Unknown.

#include "core/t2t/Pedigree.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace navtracker::t2t {
namespace {

SourcePedigree ped(std::map<std::string, SensorUsage> s,
                   SensorUsage def = SensorUsage::Unknown) {
  return SourcePedigree{std::move(s), def};
}

// The overlap primitive: overlap possible iff neither side is NotUsed.
TEST(Pedigree, OverlapPrimitiveTruthTable) {
  using U = SensorUsage;
  EXPECT_TRUE(overlapPossible(U::Used, U::Used));
  EXPECT_TRUE(overlapPossible(U::Used, U::Unknown));
  EXPECT_TRUE(overlapPossible(U::Unknown, U::Used));
  EXPECT_TRUE(overlapPossible(U::Unknown, U::Unknown));
  EXPECT_FALSE(overlapPossible(U::Used, U::NotUsed));
  EXPECT_FALSE(overlapPossible(U::NotUsed, U::Used));
  EXPECT_FALSE(overlapPossible(U::Unknown, U::NotUsed));
  EXPECT_FALSE(overlapPossible(U::NotUsed, U::NotUsed));
}

TEST(Pedigree, SharedUsedStreamIsPossiblyCorrelated) {
  const auto a = ped({{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed);
  const auto b = ped({{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(a, b), IndependenceClass::PossiblyCorrelated);
}

TEST(Pedigree, DisjointUsedStreamsWithNotUsedDefaultsAreProvablyIndependent) {
  // A uses only radar, B uses only AIS, everything else declared NotUsed.
  const auto a = ped({{"radar:own", SensorUsage::Used}, {"ais:feed", SensorUsage::NotUsed}},
                     SensorUsage::NotUsed);
  const auto b = ped({{"ais:feed", SensorUsage::Used}, {"radar:own", SensorUsage::NotUsed}},
                     SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(a, b), IndependenceClass::ProvablyIndependent);
}

TEST(Pedigree, UnknownReachableOverlapIsPossiblyCorrelated) {
  // A says Used on ais, B is Unknown on ais (could have used it).
  const auto a = ped({{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed);
  const auto b = ped({{"ais:feed", SensorUsage::Unknown}}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(a, b), IndependenceClass::PossiblyCorrelated);

  // Two Unknowns on the same stream: possible overlap.
  const auto c = ped({{"ais:feed", SensorUsage::Unknown}}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(b, c), IndependenceClass::PossiblyCorrelated);
}

TEST(Pedigree, UnknownVsNotUsedOnAStreamIsSafe) {
  // A Unknown on ais, B NotUsed on ais; no other overlap -> independent.
  const auto a = ped({{"ais:feed", SensorUsage::Unknown}}, SensorUsage::NotUsed);
  const auto b = ped({{"ais:feed", SensorUsage::NotUsed}}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(a, b), IndependenceClass::ProvablyIndependent);
}

TEST(Pedigree, DefaultUsageDrivesTheUnlistedTail) {
  // Both all-Unknown (the honest default): unlisted streams could overlap.
  EXPECT_EQ(independenceOfPair(SourcePedigree{}, SourcePedigree{}),
            IndependenceClass::PossiblyCorrelated);

  // All-Unknown vs all-NotUsed: B refrained from everything, so nothing shared.
  const auto all_notused = ped({}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(SourcePedigree{}, all_notused),
            IndependenceClass::ProvablyIndependent);

  // A lists radar=Used (default NotUsed); B all-Unknown -> B could have used
  // radar (via its Unknown default) -> possibly correlated.
  const auto a = ped({{"radar:own", SensorUsage::Used}}, SensorUsage::NotUsed);
  EXPECT_EQ(independenceOfPair(a, SourcePedigree{}),
            IndependenceClass::PossiblyCorrelated);
}

TEST(Pedigree, DefaultConstructedIsAllUnknown) {
  const SourcePedigree def{};
  EXPECT_EQ(def.default_usage, SensorUsage::Unknown);
  EXPECT_TRUE(def.sensors.empty());
  // usageOf on any unlisted id resolves to the Unknown default.
  EXPECT_EQ(def.usageOf("anything"), SensorUsage::Unknown);
}

TEST(Pedigree, FusedIndependenceOverNSources) {
  const auto indep_a = ped({{"radar:own", SensorUsage::Used}}, SensorUsage::NotUsed);
  const auto indep_b = ped({{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed);
  const auto shares_ais = ped({{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed);

  // 0 or 1 source -> SingleSource.
  EXPECT_EQ(fusedIndependence({}), IndependenceClass::SingleSource);
  EXPECT_EQ(fusedIndependence({&indep_a}), IndependenceClass::SingleSource);

  // Two provably-independent sources.
  EXPECT_EQ(fusedIndependence({&indep_a, &indep_b}),
            IndependenceClass::ProvablyIndependent);

  // Any correlated pair taints the whole set.
  EXPECT_EQ(fusedIndependence({&indep_a, &indep_b, &shares_ais}),
            IndependenceClass::PossiblyCorrelated);
}

}  // namespace
}  // namespace navtracker::t2t
