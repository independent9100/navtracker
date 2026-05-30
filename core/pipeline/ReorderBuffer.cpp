#include "core/pipeline/ReorderBuffer.hpp"

namespace navtracker {

ReorderBuffer::ReorderBuffer(double window_seconds)
    : window_nanos_(static_cast<std::int64_t>(window_seconds * 1e9)) {}

bool ReorderBuffer::push(const Measurement& m) {
  if (seen_) {
    const Timestamp cutoff{latest_.nanos() - window_nanos_};
    if (m.time < cutoff) {
      ++dropped_;
      return false;
    }
    if (latest_ < m.time) latest_ = m.time;
  } else {
    latest_ = m.time;
    seen_ = true;
  }
  queue_.emplace(m.time, m);
  return true;
}

std::vector<Measurement> ReorderBuffer::drain() {
  std::vector<Measurement> out;
  if (!seen_) return out;
  const Timestamp cutoff{latest_.nanos() - window_nanos_};
  auto it = queue_.begin();
  while (it != queue_.end() && !(cutoff < it->first)) {
    out.push_back(it->second);
    it = queue_.erase(it);
  }
  return out;
}

}  // namespace navtracker
