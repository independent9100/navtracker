#include "sim/OwnShipEmitter.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "sim/NmeaEncode.hpp"

namespace navtracker::sim {

OwnShipEmitter::OwnShipEmitter(OwnShipNmeaAdapter& adapter,
                               const geo::Datum& datum,
                               const ITruthTrajectory& ownship_trajectory,
                               OwnShipEmitterConfig cfg,
                               std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      trajectory_(ownship_trajectory),
      cfg_(std::move(cfg)),
      rng_(seed),
      noise_(0.0, cfg_.gps_pos_std_m) {}

void OwnShipEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    next_emit_ = ctx.now;
    t0_ = ctx.now;
    initialised_ = true;
  }
  while (next_emit_ <= ctx.now) {
    const TruthState truth = trajectory_.eval(next_emit_);
    const double nx = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const double ny = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const Eigen::Vector3d enu(truth.position.x() + nx,
                              truth.position.y() + ny,
                              0.0);
    const geo::Geodetic g = datum_.toGeodetic(enu);

    // GGA
    {
      std::string body = "GPGGA,000000.00,";
      body += formatLatDdmm(g.lat_deg);
      body += ',';
      body += latHemisphere(g.lat_deg);
      body += ',';
      body += formatLonDdmm(g.lon_deg);
      body += ',';
      body += lonHemisphere(g.lon_deg);
      body += ",1,08,1.0,0.0,M,0.0,M,,";
      const std::string sentence = wrapWithChecksum(body);
      adapter_.ingest(sentence, next_emit_);
    }

    // HDT — heading written via integer arithmetic to avoid locale dependence
    // of snprintf %f.
    {
      const double dt = next_emit_.secondsSince(t0_);
      const double hdg = cfg_.heading_true_deg + cfg_.heading_bias_deg +
                         cfg_.heading_drift_deg_per_s * dt;
      double hdg_norm = std::fmod(hdg, 360.0);
      if (hdg_norm < 0.0) hdg_norm += 360.0;
      long long milli = static_cast<long long>(std::llround(hdg_norm * 1000.0));
      if (milli >= 360000LL) milli -= 360000LL;  // edge: fmod returned ≈360 due to round
      const int int_part = static_cast<int>(milli / 1000LL);
      const int frac_part = static_cast<int>(milli % 1000LL);
      char buf[48];
      std::snprintf(buf, sizeof(buf), "GPHDT,%d.%03d,T", int_part, frac_part);
      const std::string sentence = wrapWithChecksum(buf);
      adapter_.ingest(sentence, next_emit_);
    }

    next_emit_ = Timestamp::fromSeconds(next_emit_.seconds() + cfg_.dt_s);
  }
}

}  // namespace navtracker::sim
