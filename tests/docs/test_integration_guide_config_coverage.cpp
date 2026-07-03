// Drift guard for docs/integration-guide.md.
//
// Every consumer-facing `struct <Name>Config` under core/, ports/, adapters/
// (excluding tests and the out-of-scope core/benchmark/ tree) must be mentioned
// in the integration guide, OR be listed in the explicit exclusion set below
// with a one-line reason. This turns "added a Config, forgot the guide" into a
// red test rather than a review hope. See CLAUDE.md,
// "Integration guide (REQUIRED to keep in sync)".
//
// The scan uses NAVTRACKER_SOURCE_DIR (a compile definition pointing at the
// repo root) so it reads the real source tree at test time, the same way the
// fixture-loading tests do.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace {

// Config structs deliberately kept OUT of the consumer integration guide.
// Each entry needs a reason — excluding a struct is a visible, reviewed act.
const std::map<std::string, std::string>& excludedConfigs() {
  static const std::map<std::string, std::string> kExcluded = {
      {"RecorderConfig",
       "adapters/foxglove debug/visualization recorder; diagnostic tooling, "
       "not part of the fusion contract (out of consumer scope)."},
  };
  return kExcluded;
}

std::string readFile(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// True if `path` is inside a directory segment we do not scan.
bool isExcludedPath(const fs::path& path) {
  for (const auto& part : path) {
    const std::string s = part.string();
    if (s == "benchmark" || s == "tests") return true;
  }
  return false;
}

// Collect `struct <Name>Config` names across a source subtree. The regex
// requires `struct` immediately before the name, so member declarations like
// `UereEstimatorConfig cfg{};` (no `struct` keyword) are not matched. A bare
// `struct Config` (the nested tracker configs) is intentionally not matched:
// `\w+Config` needs at least one word char before the literal "Config".
void collectConfigsIn(const fs::path& root,
                      std::map<std::string, std::string>* out) {
  static const std::regex kStructConfig(R"(\bstruct\s+(\w+Config)\b)");
  if (!fs::exists(root)) return;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    const fs::path& p = entry.path();
    const std::string ext = p.extension().string();
    if (ext != ".hpp" && ext != ".h") continue;
    if (isExcludedPath(p)) continue;

    const std::string content = readFile(p);
    for (std::sregex_iterator it(content.begin(), content.end(), kStructConfig),
         end;
         it != end; ++it) {
      const std::string name = (*it)[1].str();
      out->emplace(name, p.string());  // first location wins; enough to report
    }
  }
}

}  // namespace

TEST(IntegrationGuideConfigCoverage, EveryConsumerConfigIsDocumented) {
  const std::string root = NAVTRACKER_SOURCE_DIR;

  const std::string guide = readFile(fs::path(root) / "docs" / "integration-guide.md");
  ASSERT_FALSE(guide.empty())
      << "docs/integration-guide.md is missing or empty at " << root;

  std::map<std::string, std::string> found;  // name -> first header path
  collectConfigsIn(fs::path(root) / "core", &found);
  collectConfigsIn(fs::path(root) / "ports", &found);
  collectConfigsIn(fs::path(root) / "adapters", &found);

  // Sanity: the scan must find a non-trivial number of configs, otherwise the
  // source-tree path is wrong and the test would pass vacuously.
  ASSERT_GE(found.size(), 5u)
      << "Only " << found.size()
      << " *Config structs found under " << root
      << " — NAVTRACKER_SOURCE_DIR likely points at the wrong tree.";

  for (const auto& [name, header] : found) {
    if (excludedConfigs().count(name)) continue;
    EXPECT_NE(guide.find(name), std::string::npos)
        << "Config struct '" << name << "' (declared in " << header
        << ") is not mentioned in docs/integration-guide.md.\n"
        << "Either document it in the guide (with its defaults worth changing "
           "and a wiring note), or add it to excludedConfigs() in this test "
           "with a reason.";
  }
}

// Keep the exclusion set honest: an excluded name that no longer exists in the
// tree is dead weight and should be removed.
TEST(IntegrationGuideConfigCoverage, ExclusionsStillExist) {
  const std::string root = NAVTRACKER_SOURCE_DIR;

  std::map<std::string, std::string> found;
  collectConfigsIn(fs::path(root) / "core", &found);
  collectConfigsIn(fs::path(root) / "ports", &found);
  collectConfigsIn(fs::path(root) / "adapters", &found);

  for (const auto& [name, reason] : excludedConfigs()) {
    EXPECT_TRUE(found.count(name))
        << "Excluded config '" << name << "' (\"" << reason
        << "\") was not found in the source tree — remove the stale exclusion.";
  }
}
