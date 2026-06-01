#include "sim/ArpaEmitter.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "sim/NmeaEncode.hpp"

namespace navtracker::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kMetresPerNm = 1852.0;

double wrap360(double deg) {
  double w = std::fmod(deg, 360.0);
  if (w < 0.0) w += 360.0;
  return w;
}

// Locale-independent "%d.%03d" of a non-negative double, rounded to milli.
std::string formatMilli3(double x) {
  long long milli = static_cast<long long>(std::llround(x * 1000.0));
  if (milli < 0) milli = 0;
  const long long int_part = milli / 1000LL;
  const long long frac_part = milli % 1000LL;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%lld.%03lld", int_part, frac_part);
  return std::string(buf);
}

}  // namespace

ArpaEmitter::ArpaEmitter(ArpaAdapter& adapter,
                         const geo::Datum& datum,
                         ArpaEmitterConfig cfg,
                         std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      cfg_(std::move(cfg)),
      rng_(seed),
      range_noise_(0.0, cfg_.range_std_m > 0.0 ? cfg_.range_std_m : 1.0),
      bearing_noise_(0.0, cfg_.bearing_std_deg > 0.0 ? cfg_.bearing_std_deg : 1.0) {}

void ArpaEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    for (const auto& te : cfg_.targets) next_emit_[te.truth_id] = ctx.now;
    initialised_ = true;
  }

  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  const Eigen::Vector2d own = ctx.ownship_truth.position;

  for (const auto& te : cfg_.targets) {
    auto it = truths.find(te.truth_id);
    if (it == truths.end()) continue;

    Timestamp& next = next_emit_[te.truth_id];
    while (next <= ctx.now) {
      const Eigen::Vector2d dxy = it->second.position - own;
      const double range = dxy.norm();
      if (range >= cfg_.min_range_m && range <= cfg_.max_range_m) {
        // Convention (v1): own-ship heading is 0 (see spec §14.9 deferral),
        // so true bearing = relative bearing. Compass bearing is measured
        // clockwise from north (+y in ENU): compass = atan2(east, north).
        // ArpaAdapter re-rotates by heading (= 0 here), so feeding the
        // compass-true bearing into the relative-bearing field works.
        const double bearing_true_deg =
            std::atan2(dxy.x(), dxy.y()) * kRad2Deg;  // compass from north
        const double bearing_rel_deg = bearing_true_deg;
        const double r_obs = range + (cfg_.range_std_m > 0.0 ? range_noise_(rng_) : 0.0);
        const double b_obs = wrap360(bearing_rel_deg +
                                     (cfg_.bearing_std_deg > 0.0 ? bearing_noise_(rng_) : 0.0));
        const double r_nm = r_obs / kMetresPerNm;
        std::string body = "RATTM,";
        char tn[8];
        std::snprintf(tn, sizeof(tn), "%02d", te.arpa_track_num);
        body += tn;
        body += ',';
        body += formatMilli3(r_nm);
        body += ',';
        body += formatMilli3(b_obs);
        body += ",R,0.0,0.0,T,0.0,0.0,N,T,,000000.00,A";
        const std::string sentence = wrapWithChecksum(body);
        adapter_.ingest(sentence, next);
      }
      next = Timestamp::fromSeconds(next.seconds() + cfg_.rotation_dt_s);
    }
  }
}

}  // namespace navtracker::sim
