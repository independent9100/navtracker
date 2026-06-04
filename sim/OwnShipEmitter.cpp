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
    // Report the configured GPS position std on every published pose so the
    // adapter stages (ARPA / EO/IR) can budget R-inflation for it — but only
    // when explicitly opted in. Default-off preserves pre-2026-06-03 test
    // behaviour where noise was injected on lat/lon without R modelling it.
    adapter_.setPositionStd(cfg_.report_gps_std ? cfg_.gps_pos_std_m : 0.0);

    const TruthState truth = trajectory_.eval(next_emit_);
    const double nx = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const double ny = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const Eigen::Vector3d enu(truth.position.x() + nx,
                              truth.position.y() + ny,
                              0.0);
    const geo::Geodetic g = datum_.toGeodetic(enu);

    // RMC — emitted only when opted in. Carries truth SOG/COG (m/s → knots,
    // ENU velocity → COG-from-true-north) optionally perturbed by the
    // sim's emit-side noise floors. Default OFF preserves pre-RMC sim
    // behaviour where pose.velocity_is_valid stayed false on the first
    // GGA tick.
    //
    // RMC is pushed BEFORE GGA so the adapter's rmc_buffer_ is fresh when
    // the GGA-handler composes the pose's velocity fields (precedence
    // rule lives in OwnShipNmeaAdapter::ingest's GGA branch).
    if (cfg_.emit_rmc) {
      Eigen::Vector2d v_truth = truth.velocity;
      if (cfg_.sigma_sog_emit_m_per_s > 0.0 ||
          cfg_.sigma_cog_emit_deg > 0.0) {
        const double sog_truth = v_truth.norm();
        const double cog_truth_rad = std::atan2(v_truth.x(), v_truth.y());
        double sog = sog_truth;
        double cog_rad = cog_truth_rad;
        if (cfg_.sigma_sog_emit_m_per_s > 0.0) {
          std::normal_distribution<double> sog_noise(
              0.0, cfg_.sigma_sog_emit_m_per_s);
          sog += sog_noise(rng_);
          if (sog < 0.0) sog = 0.0;  // SOG is non-negative by definition
        }
        if (cfg_.sigma_cog_emit_deg > 0.0) {
          std::normal_distribution<double> cog_noise(
              0.0, cfg_.sigma_cog_emit_deg * M_PI / 180.0);
          cog_rad += cog_noise(rng_);
        }
        v_truth = Eigen::Vector2d(sog * std::sin(cog_rad),
                                  sog * std::cos(cog_rad));
      }
      const double sog_m_per_s = v_truth.norm();
      const double sog_knots = sog_m_per_s / 0.514444;
      // COG: angle clockwise from true north. ENU east = sin(COG),
      // north = cos(COG), so COG = atan2(east, north). Wrap to [0, 360).
      double cog_deg = std::atan2(v_truth.x(), v_truth.y()) * 180.0 / M_PI;
      cog_deg = std::fmod(cog_deg, 360.0);
      if (cog_deg < 0.0) cog_deg += 360.0;
      // Format SOG/COG via integer arithmetic to avoid locale issues with
      // %f (mirrors the GPHDT path).
      const long long sog_milli =
          static_cast<long long>(std::llround(sog_knots * 1000.0));
      long long cog_milli =
          static_cast<long long>(std::llround(cog_deg * 1000.0));
      if (cog_milli >= 360000LL) cog_milli -= 360000LL;
      char sog_buf[24];
      std::snprintf(sog_buf, sizeof(sog_buf), "%lld.%03lld",
                    sog_milli / 1000LL, std::abs(sog_milli % 1000LL));
      char cog_buf[24];
      std::snprintf(cog_buf, sizeof(cog_buf), "%lld.%03lld",
                    cog_milli / 1000LL, cog_milli % 1000LL);

      std::string body = "GPRMC,000000.00,A,";
      body += formatLatDdmm(g.lat_deg);
      body += ',';
      body += latHemisphere(g.lat_deg);
      body += ',';
      body += formatLonDdmm(g.lon_deg);
      body += ',';
      body += lonHemisphere(g.lon_deg);
      body += ',';
      body += sog_buf;
      body += ',';
      body += cog_buf;
      body += ",010100,,,";  // date (DDMMYY), magvar, E/W, mode all empty/stub
      const std::string sentence = wrapWithChecksum(body);
      adapter_.ingest(sentence, next_emit_);
    }

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
      // HDOP field left empty — sim does not model dilution; sigma is
      // carried via the sticky setter above when report_gps_std is true.
      body += ",1,08,,0.0,M,0.0,M,,";
      const std::string sentence = wrapWithChecksum(body);
      adapter_.ingest(sentence, next_emit_);
    }

    // HDT — heading written via integer arithmetic to avoid locale dependence
    // of snprintf %f.
    {
      const double dt = next_emit_.secondsSince(t0_);
      double hdg = cfg_.heading_true_deg + cfg_.heading_bias_deg +
                   cfg_.heading_drift_deg_per_s * dt;
      if (cfg_.heading_noise_std_deg > 0.0) {
        std::normal_distribution<double> h_noise(
            0.0, cfg_.heading_noise_std_deg);
        hdg += h_noise(rng_);
      }
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
