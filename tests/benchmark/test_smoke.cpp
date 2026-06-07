#include <gtest/gtest.h>

#include "core/benchmark/Placeholder.hpp"

TEST(BenchmarkSmoke, PlaceholderLinks) {
  EXPECT_EQ(navtracker::benchmark::placeholder(), 42);
}
