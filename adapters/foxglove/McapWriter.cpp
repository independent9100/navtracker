#include "adapters/foxglove/McapWriter.hpp"
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
// Load-bearing: this is the single TU that defines MCAP_IMPLEMENTATION, so it
// must also emit the header-only McapReader symbols that tests link against.
// (Tests include <mcap/reader.hpp> WITHOUT the macro -> declarations only.)
#include <mcap/reader.hpp>
#include <stdexcept>

namespace navtracker::foxglove {

struct McapWriter::Impl {
  mcap::McapWriter writer;
  std::unordered_map<std::string, mcap::ChannelId> channels;     // topic -> id
  std::unordered_map<std::string, mcap::SchemaId> schemas;       // schema name -> id
  bool open = false;
  std::unordered_map<std::string, std::uint32_t> seq;            // per-topic sequence
};

McapWriter::McapWriter(const std::string& path) : impl_(new Impl) {
  mcap::McapWriterOptions opts("");                 // empty profile = generic
  opts.compression = mcap::Compression::Zstd;
  const auto status = impl_->writer.open(path, opts);
  if (!status.ok()) throw std::runtime_error("mcap open failed: " + status.message);
  impl_->open = true;
}

McapWriter::~McapWriter() { close(); delete impl_; }

void McapWriter::ensureChannel(const std::string& topic, const std::string& schema_name,
                               const std::string& schema_text) {
  if (impl_->channels.count(topic)) return;
  auto sit = impl_->schemas.find(schema_name);
  if (sit == impl_->schemas.end()) {
    mcap::Schema schema(schema_name, "jsonschema",
                        std::string_view(schema_text));   // empty ok: name-only
    impl_->writer.addSchema(schema);
    sit = impl_->schemas.emplace(schema_name, schema.id).first;
  }
  mcap::Channel channel(topic, "json", sit->second);
  impl_->writer.addChannel(channel);
  impl_->channels.emplace(topic, channel.id);
}

void McapWriter::write(const std::string& topic, Timestamp t, const std::string& bytes) {
  auto it = impl_->channels.find(topic);
  if (it == impl_->channels.end()) throw std::runtime_error("unknown topic: " + topic);
  mcap::Message msg;
  msg.channelId = it->second;
  msg.sequence = impl_->seq[topic]++;
  msg.logTime = static_cast<mcap::Timestamp>(t.nanos());
  msg.publishTime = msg.logTime;
  msg.data = reinterpret_cast<const std::byte*>(bytes.data());
  msg.dataSize = bytes.size();
  const auto status = impl_->writer.write(msg);
  if (!status.ok()) throw std::runtime_error("mcap write failed: " + status.message);
}

void McapWriter::close() {
  if (impl_ && impl_->open) { impl_->writer.close(); impl_->open = false; }
}

}  // namespace navtracker::foxglove
