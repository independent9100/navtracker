#include <gtest/gtest.h>

#include "core/types/Measurement.hpp"

TEST(AssociationHints, CarriesNumericPlatformId) {
  navtracker::AssociationHints h;
  EXPECT_FALSE(h.platform_id.has_value());
  h.platform_id = std::uint64_t{1234567890123ULL};
  ASSERT_TRUE(h.platform_id.has_value());
  EXPECT_EQ(*h.platform_id, 1234567890123ULL);
}
