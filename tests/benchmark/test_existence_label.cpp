// R8.1 — existence/region label fixture + loader. The labels are video-derived
// existence truth (region + window), NOT kinematic TruthSamples.
#include "core/benchmark/ExistenceLabel.hpp"

#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

using navtracker::benchmark::ExistenceLabel;
using navtracker::benchmark::ExistenceLabelClass;
using navtracker::benchmark::parseExistenceLabels;

namespace {
const ExistenceLabel* find(const std::vector<ExistenceLabel>& v,
                           const std::string& id) {
  for (const auto& l : v)
    if (l.region_id == id) return &l;
  return nullptr;
}
}  // namespace

// The parser handles windowed labels, whole-clip labels, embedded semicolons,
// and a notes column, and classifies the label strings.
TEST(ExistenceLabel, ParsesWindowedAndWholeClipRows) {
  const std::string csv =
      "# a comment line\n"
      "region_id,source_rank,lat,lon,radius_m,t_start_s,t_end_s,label,evidence,confidence,notes\n"
      "ferry,rank11,42.3783,-71.0464,40,0,98,KEEP_VESSEL,video+radar,high,outbound leg\n"
      "blob,astern,42.3746,-71.0482,120,,,SUPPRESS_STRUCTURE,extended; big; astern,med,notes here\n"
      "mystery,84/95,42.3747,-71.0446,60,,,UNKNOWN,,,defaults to KEEP; pending\n";
  std::istringstream is(csv);
  const auto labels = parseExistenceLabels(is);
  ASSERT_EQ(labels.size(), 3u);

  const ExistenceLabel* ferry = find(labels, "ferry");
  ASSERT_NE(ferry, nullptr);
  EXPECT_EQ(ferry->label, ExistenceLabelClass::KeepVessel);
  EXPECT_EQ(ferry->source_rank, "rank11");
  EXPECT_DOUBLE_EQ(ferry->lat_deg, 42.3783);
  EXPECT_DOUBLE_EQ(ferry->radius_m, 40.0);
  EXPECT_FALSE(ferry->covers_whole_clip);
  EXPECT_DOUBLE_EQ(ferry->t_start_s, 0.0);
  EXPECT_DOUBLE_EQ(ferry->t_end_s, 98.0);

  const ExistenceLabel* blob = find(labels, "blob");
  ASSERT_NE(blob, nullptr);
  EXPECT_EQ(blob->label, ExistenceLabelClass::SuppressStructure);
  EXPECT_TRUE(blob->covers_whole_clip);
  EXPECT_EQ(blob->evidence, "extended; big; astern");  // semicolons kept, not split
  EXPECT_EQ(blob->confidence, "med");
  EXPECT_EQ(blob->notes, "notes here");

  const ExistenceLabel* mystery = find(labels, "mystery");
  ASSERT_NE(mystery, nullptr);
  EXPECT_EQ(mystery->label, ExistenceLabelClass::Unknown);
  EXPECT_TRUE(mystery->evidence.empty());
  EXPECT_EQ(mystery->notes, "defaults to KEEP; pending");
}

// Window membership: whole-clip labels are always active; windowed labels test
// the relative offset against the clip start.
TEST(ExistenceLabel, ActiveAtUnixRespectsWindow) {
  const std::string csv =
      "region_id,source_rank,lat,lon,radius_m,t_start_s,t_end_s,label,evidence,confidence,notes\n"
      "w,r,42.0,-71.0,40,10,94,KEEP_VESSEL,e,c,n\n"
      "whole,r,42.0,-71.0,40,,,SUPPRESS_STRUCTURE,e,c,n\n";
  std::istringstream is(csv);
  const auto labels = parseExistenceLabels(is);
  const double clip0 = 1635458136.03;
  const ExistenceLabel* w = find(labels, "w");
  ASSERT_NE(w, nullptr);
  EXPECT_FALSE(w->activeAtUnix(clip0 + 5.0, clip0));   // before window
  EXPECT_TRUE(w->activeAtUnix(clip0 + 50.0, clip0));   // inside
  EXPECT_FALSE(w->activeAtUnix(clip0 + 120.0, clip0)); // after window
  const ExistenceLabel* whole = find(labels, "whole");
  ASSERT_NE(whole, nullptr);
  EXPECT_TRUE(whole->activeAtUnix(clip0 + 0.0, clip0));
  EXPECT_TRUE(whole->activeAtUnix(clip0 + 9999.0, clip0));
}

// The committed sunset_cruise fixture parses and contains the R8 label set.
TEST(ExistenceLabel, SunsetCruiseFixtureLoads) {
  const std::string path =
      std::string(NAVTRACKER_SOURCE_DIR) +
      "/tests/fixtures/philos/labels/sunset_cruise_labels.csv";
  std::ifstream f(path);
  ASSERT_TRUE(f.good()) << "missing fixture: " << path;
  const auto labels = parseExistenceLabels(f);
  // 3 KEEP_VESSEL + 2 SUPPRESS_STRUCTURE + 1 UNKNOWN.
  EXPECT_EQ(labels.size(), 6u);
  int keep = 0, suppress = 0, unknown = 0;
  for (const auto& l : labels) {
    if (l.label == ExistenceLabelClass::KeepVessel) ++keep;
    else if (l.label == ExistenceLabelClass::SuppressStructure) ++suppress;
    else if (l.label == ExistenceLabelClass::Unknown) ++unknown;
  }
  EXPECT_EQ(keep, 3);
  EXPECT_EQ(suppress, 2);
  EXPECT_EQ(unknown, 1);
  // The two ferry legs bracket the stop->go transition at t~90.
  const ExistenceLabel* a = find(labels, "ferry_v1_a");
  const ExistenceLabel* b = find(labels, "ferry_v1_b");
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  EXPECT_LT(a->t_start_s, 90.0);
  EXPECT_GT(a->t_end_s, 90.0);
  EXPECT_LT(b->t_start_s, 90.0);
  EXPECT_GT(b->t_end_s, 110.0);  // must still cover the "reports motion by ~110-116" window
}
