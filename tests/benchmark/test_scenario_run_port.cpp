#include <gtest/gtest.h>

#include "core/benchmark/ScenarioRun.hpp"
#include "core/scenario/Truth.hpp"

using navtracker::Scenario;
using navtracker::benchmark::ScenarioDescriptor;
using navtracker::benchmark::ScenarioRun;

namespace {
class FakeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"fake", /*is_multi_seed=*/true, /*seed_count=*/3};
  }
  Scenario generate(std::uint64_t seed) override {
    Scenario s;
    s.truth.push_back({{}, /*truth_id=*/seed + 1, {}, {}});
    return s;
  }
};
}  // namespace

TEST(ScenarioRunPort, DescriptorAndGenerateRoundtrip) {
  FakeScenarioRun run;
  const auto d = run.descriptor();
  EXPECT_EQ(d.label, "fake");
  EXPECT_TRUE(d.is_multi_seed);
  EXPECT_EQ(d.seed_count, 3u);

  const auto a = run.generate(0);
  const auto b = run.generate(1);
  ASSERT_EQ(a.truth.size(), 1u);
  ASSERT_EQ(b.truth.size(), 1u);
  EXPECT_EQ(a.truth[0].truth_id, 1u);
  EXPECT_EQ(b.truth[0].truth_id, 2u);
}
