#pragma once

#include <cstdint>

namespace navtracker {

/**
 * Monotonic, source-provided time as signed nanoseconds since an epoch.
 * The engine advances on these, never on wall-clock.
 */
class Timestamp {
 public:
  constexpr Timestamp() = default;
  constexpr explicit Timestamp(std::int64_t nanos) : nanos_(nanos) {}

  /** Construct from a floating-point seconds value (truncated to nanos). */
  static constexpr Timestamp fromSeconds(double seconds) {
    return Timestamp(static_cast<std::int64_t>(seconds * 1e9));
  }

  /** Nanoseconds since the epoch. */
  constexpr std::int64_t nanos() const { return nanos_; }
  /** Seconds since the epoch (as double). */
  constexpr double seconds() const { return static_cast<double>(nanos_) * 1e-9; }

  /** Signed difference in seconds: (*this - other). */
  constexpr double secondsSince(Timestamp other) const {
    return static_cast<double>(nanos_ - other.nanos_) * 1e-9;
  }

  friend constexpr bool operator<(Timestamp a, Timestamp b) { return a.nanos_ < b.nanos_; }
  friend constexpr bool operator>(Timestamp a, Timestamp b) { return a.nanos_ > b.nanos_; }
  friend constexpr bool operator<=(Timestamp a, Timestamp b) { return a.nanos_ <= b.nanos_; }
  friend constexpr bool operator>=(Timestamp a, Timestamp b) { return a.nanos_ >= b.nanos_; }
  friend constexpr bool operator==(Timestamp a, Timestamp b) { return a.nanos_ == b.nanos_; }
  friend constexpr bool operator!=(Timestamp a, Timestamp b) { return a.nanos_ != b.nanos_; }

 private:
  std::int64_t nanos_{0};
};

}  // namespace navtracker
