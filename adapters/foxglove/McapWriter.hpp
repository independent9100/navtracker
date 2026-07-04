#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include "core/types/Timestamp.hpp"

namespace navtracker::foxglove {

/**
 * RAII wrapper over mcap::McapWriter. One channel per topic; schema text is
 * loaded once per schema name from the vendored schemas dir.
 */
class McapWriter {
 public:
  explicit McapWriter(const std::string& path);   // opens, zstd compression
  ~McapWriter();                                   // closes if open

  McapWriter(const McapWriter&) = delete;
  McapWriter& operator=(const McapWriter&) = delete;

  /**
   * Register (idempotent) a topic bound to a Foxglove schema name. schema_text
   * is the jsonschema (may be empty -> name-only recognition).
   */
  void ensureChannel(const std::string& topic, const std::string& schema_name,
                     const std::string& schema_text);

  /** Write a JSON payload to a previously-registered topic at log time t. */
  void write(const std::string& topic, Timestamp t, const std::string& json_bytes);

  /** Flush and close the file (also done by the destructor). */
  void close();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace navtracker::foxglove
