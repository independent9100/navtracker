// W2.7 — point the fixture-resolving env vars at the source tree so every
// fixture-gated test reaches its data under `ctest` (which runs from the build
// dir), regardless of the working directory.
//
// The scenario-run adapters (SimMultisensor / R-BAD / philos-HAXR-AutoFerry
// replays) resolve their fixture base dirs from env vars with cwd-relative
// defaults. A gtest global environment sets those vars — IF THE USER HAS NOT
// ALREADY — to absolute paths derived from the compiled-in source dir, before
// any test runs. Setting only-if-unset preserves an explicit override (e.g. a
// developer pointing SIMMS_DIR at a scratch copy).

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace {

void setIfUnset(const char* name, const std::string& value) {
  const char* cur = std::getenv(name);
  if (cur == nullptr || *cur == '\0') {
    ::setenv(name, value.c_str(), /*overwrite=*/0);
  }
}

class FixturePathEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    const std::string root = NAVTRACKER_SOURCE_DIR;
    // Base dir for the philos / HAXR / AutoFerry replay adapters, which prefix
    // their (otherwise cwd-relative) fixture paths with this when set.
    setIfUnset("NAVTRACKER_FIXTURE_ROOT", root);
    // Base dirs already honoured by the sim-multisensor / R-BAD adapters.
    setIfUnset("SIMMS_DIR", root + "/tests/fixtures/sim_multisensor");
    setIfUnset("RBAD_DIR", root + "/tests/fixtures/rbad");
  }
};

// Register before main(); SetUp() runs once at RUN_ALL_TESTS start, ahead of
// every test, so the adapters read the absolute paths when they getenv().
const bool kFixtureEnvRegistered = [] {
  ::testing::AddGlobalTestEnvironment(new FixturePathEnvironment());
  return true;
}();

}  // namespace
