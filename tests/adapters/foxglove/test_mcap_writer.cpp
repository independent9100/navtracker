#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <mcap/reader.hpp>
#include "adapters/foxglove/McapWriter.hpp"
#include "adapters/foxglove/Schemas.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

TEST(McapWriter, RoundTripsOneMessage) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    McapWriter w(path);
    w.ensureChannel("/diag/test", kDiagSchema, "");
    w.write("/diag/test", Timestamp{42}, R"({"value":7})");
    w.close();
  }
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(path).ok());
  int count = 0; std::string topic; std::uint64_t logtime = 0;
  auto view = reader.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) {
    ++count; topic = it->channel->topic; logtime = it->message.logTime;
  }
  reader.close();
  std::remove(path.c_str());
  EXPECT_EQ(count, 1);
  EXPECT_EQ(topic, "/diag/test");
  EXPECT_EQ(logtime, 42u);
}
