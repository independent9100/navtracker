#include "adapters/replay/OwnshipCsvReader.hpp"

#include <fstream>
#include <string>

#include <gtest/gtest.h>

using navtracker::replay::loadOwnshipCsv;

namespace {

std::string writeTempCsv(const std::string& name, const std::string& content) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream f(path);
  f << content;
  return path;
}

}  // namespace

// #26 M22: loadOwnshipCsv fed every row straight through strtod with no
// finite/range/null-island guard, so a blank lat/lon row became a (0,0) pose
// that poisons every body-frame projection in its window. The sibling AIS
// loader already rejects (0,0)/out-of-range; mirror it here.
TEST(OwnshipCsvReader, SkipsBlankImplausibleAndNullIslandRows) {
  const std::string csv =
      "unix_time,lat,lon,heading_deg\n"
      "100.0,53.5,9.9,90.0\n"    // valid
      "101.0,,,45.0\n"           // blank lat/lon -> (0,0) today
      "102.0,200.0,9.9,45.0\n"   // out-of-range latitude
      "103.0,0.0,0.0,10.0\n"     // explicit Null Island
      "104.0,53.6,10.0,80.0\n";  // valid
  const std::string path = writeTempCsv("nt_m22_ownship.csv", csv);

  const auto poses = loadOwnshipCsv(path);
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_DOUBLE_EQ(poses[0].lat_deg, 53.5);
  EXPECT_DOUBLE_EQ(poses[1].lat_deg, 53.6);
}

TEST(OwnshipCsvReader, ValidRowsAllLoad) {
  const std::string csv =
      "unix_time,lat,lon,heading_deg\n"
      "100.0,53.5,9.9,90.0\n"
      "101.0,53.6,10.0,45.0\n";
  const std::string path = writeTempCsv("nt_m22_ownship_valid.csv", csv);

  const auto poses = loadOwnshipCsv(path);
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_DOUBLE_EQ(poses[0].lat_deg, 53.5);
  EXPECT_DOUBLE_EQ(poses[1].lat_deg, 53.6);
}
