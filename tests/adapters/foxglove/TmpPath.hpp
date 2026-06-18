#pragma once
// Test-only helper: build a unique temp .mcap path under $TMPDIR.
//
// std::tmpnam() hard-codes /tmp (P_tmpdir), which is NOT writable under the
// command sandbox — only $TMPDIR is. Recorder tests write real .mcap files,
// so they must place them under $TMPDIR to run in-sandbox.
#include <atomic>
#include <cstdlib>
#include <string>
#include <unistd.h>

namespace navtracker::foxglove::test {

inline std::string tmpMcapPath(const char* stem) {
  const char* dir = std::getenv("TMPDIR");
  static std::atomic<unsigned> counter{0};
  return std::string(dir && *dir ? dir : "/tmp") + "/navtracker_" + stem + "_" +
         std::to_string(static_cast<long>(::getpid())) + "_" +
         std::to_string(counter.fetch_add(1)) + ".mcap";
}

}  // namespace navtracker::foxglove::test
