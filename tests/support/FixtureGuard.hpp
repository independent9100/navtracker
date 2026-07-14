#pragma once

// W2.7 — the fixture-skip guard.
//
// ~36 real-data tests resolve their fixtures relative to the current working
// directory, but ctest runs from the build dir, so under `ctest` they used to
// silently GTEST_SKIP while the suite still reported 100% green — the
// 2026-07-08 red-master-for-a-day failure mode. Two mechanisms close that hole:
//
//   1. srcAbs(rel) builds an ABSOLUTE fixture path from the compiled-in source
//      dir, so a test reaches its fixtures regardless of the working directory.
//      (The scenario-run adapters resolve their base dirs from the SIMMS_DIR /
//      RBAD_DIR / NAVTRACKER_FIXTURE_ROOT env vars, which the test binary's
//      global environment — FixtureGuard.cpp — points at the source tree.)
//
//   2. NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(missing, msg): the opt-in strict mode.
//      With env NAVTRACKER_REQUIRE_FIXTURES=1 a would-be fixture skip becomes a
//      FAILURE, so a verification ceremony proves every fixture-gated test
//      actually RAN. Default (env unset) behaviour is an ordinary GTEST_SKIP —
//      bit-identical to before.

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace navtracker_test {

// Absolute path to `rel` under the source tree (NAVTRACKER_SOURCE_DIR is a
// compile definition on the navtracker_tests target). Use this instead of a
// bare relative fixture path so the test runs from any working directory.
inline std::string srcAbs(const std::string& rel) {
  return std::string(NAVTRACKER_SOURCE_DIR) + "/" + rel;
}

// True when strict fixture mode is on (env NAVTRACKER_REQUIRE_FIXTURES set to a
// non-empty, non-"0" value). The codebase's "set and non-empty" convention.
inline bool fixturesRequired() {
  const char* v = std::getenv("NAVTRACKER_REQUIRE_FIXTURES");
  return v != nullptr && *v != '\0' && std::string(v) != "0";
}

}  // namespace navtracker_test

// Replaces `if (missing) GTEST_SKIP() << msg;` at every fixture-absence gate.
// missing_cond true → strict mode FAILs, otherwise SKIPs (both return from the
// test). Wrap BOTH skip idioms: a missing path AND an empty/!valid load result
// (the scenario-run family, where "fixtures absent" surfaces as an empty
// Scenario). msg is streamed, so callers pass a plain string / literal.
#define NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(missing_cond, msg)             \
  do {                                                                    \
    if (missing_cond) {                                                   \
      if (::navtracker_test::fixturesRequired()) {                        \
        FAIL() << "NAVTRACKER_REQUIRE_FIXTURES=1 but fixtures missing: "  \
               << msg;                                                    \
      } else {                                                            \
        GTEST_SKIP() << msg;                                              \
      }                                                                   \
    }                                                                     \
  } while (0)
