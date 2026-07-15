#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/estimation/PolarVelocity.hpp"
#include "core/geo/Datum.hpp"
#include "core/own_ship/IDatumChangeSink.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

/**
 * One decoded AIS dynamic position report — the input a consumer feeds to
 * `AisAdapter`. Carries an absolute WGS84 fix plus the reporting vessel's
 * MMSI and an accuracy flag; the adapter turns it into an ENU `Position2D`
 * `Measurement`.
 */
struct AisDynamicReport {
  Timestamp time;
  std::uint32_t mmsi{0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  bool high_accuracy{false};
  std::string source_id{"ais"};

  // Target-reported kinematics (backlog #20). All optional: leave unset for a
  // report that does not carry the field, or set the AIS "not available"
  // sentinel value and the adapter will drop it at the edge (invariant #6):
  //   - sog_knots: speed over ground, knots. Only [0, 102.2] is valid velocity
  //     content; anything above (the impossible band up to and incl. the 1023
  //     "not available" sentinel) is dropped to Position2D (W5.6.1).
  //   - cog_deg:   course over ground, deg true ([0, 360); 3600 sentinel).
  //   - heading_deg: true heading, deg ([0, 360); 511 sentinel dropped).
  //   - nav_status: AIS navigational status code (0..15; 15 = undefined/default,
  //                 dropped). 1 = at anchor, 5 = moored — the "never suppress a
  //                 self-declared vessel" cue (ADR 0002 / R3).
  // SOG/COG become PositionVelocity2D measurement content; heading/nav_status
  // are attribute/corroboration hints. AIS is an independent witness, so all of
  // these are legitimate (contrast ARPA TTM speed/course — guide §3).
  std::optional<double> sog_knots;
  std::optional<double> cog_deg;
  std::optional<double> heading_deg;
  std::optional<std::uint8_t> nav_status;
};

/**
 * Tuning for `AisAdapter` (backlog #20). Defaults preserve the historical
 * behaviour for a report that carries no SOG (Position2D at 10/30 m) so existing
 * consumers see no change; each field is a per-deployment knob.
 */
struct AisAdapterConfig {
  // When the report carries SOG (and COG), emit a PositionVelocity2D with the
  // target-reported velocity as content. false → always Position2D (velocity
  // ignored). AIS SOG/COG come from the target's OWN GPS — a genuinely
  // independent witness — so they are legitimate measurement content, unlike
  // ARPA-derived speed/course (our own smoothed data; see guide §3).
  bool emit_velocity_from_sog_cog = true;
  // Below this ground speed COG is meaningless (a near-stationary / drifting
  // target reports a random course), so the measurement stays Position2D — the
  // "COG down-weighted at low SOG" rule in the limit. m/s. Defaults are the
  // shared constants in core/estimation/PolarVelocity.hpp so the NMEA and replay
  // paths share one source of truth (backlog #20); these remain per-deployment
  // knobs.
  double sog_velocity_min_mps = kAisSogVelocityMinMps;
  // 1-σ used to build the SOG/COG → ENU-velocity covariance (AIS carries no
  // velocity uncertainty of its own).
  double sog_std_mps = kAisSogStdMps;
  double cog_std_deg = kAisCogStdDeg;
  // Isotropic velocity-σ floor ADDED to the polar-Jacobian velocity covariance
  // so that even just above the SOG threshold a noisy COG cannot make the
  // velocity DIRECTION overconfident (the low-SOG down-weighting, continuous
  // form). m/s.
  double velocity_iso_floor_mps = kAisVelocityIsoFloorMps;
  // Position 1-σ (m) for high-accuracy vs standard AIS fixes.
  double position_std_high_accuracy_m = 10.0;
  double position_std_standard_m = 30.0;
};

/**
 * Sensor adapter that converts AIS dynamic reports into ENU `Position2D`
 * (or `PositionVelocity2D` when SOG/COG are present, backlog #20) measurements
 * in the supplied `Datum` frame. Validates at the edge (invariant #6):
 * implausible / sentinel / NaN fixes — e.g. AIS lat 91° / lon 181° "position
 * not available" — are rejected before they can become phantom tracks.
 * High-accuracy fixes get a tighter position σ than standard ones; the MMSI is
 * attached as a non-fusion identity hint; heading/nav-status ride on the hints.
 */
class AisAdapter : public ISensorAdapter, public IDatumChangeSink {
 public:
  explicit AisAdapter(geo::Datum datum, AisAdapterConfig cfg = {});

  /** Validate report `r` and, if plausible, buffer an ENU `Position2D` measurement. */
  void ingest(const AisDynamicReport& r);
  /** Drain and return all measurements buffered since the last poll. */
  std::vector<Measurement> poll() override;

  /**
   * IDatumChangeSink (W2.1): on an OwnShipProvider auto-recenter, adopt the new
   * datum for subsequent projections and re-express any already-buffered
   * measurements into the new frame. Register with
   * `provider.registerDatumSink(&adapter)` whenever auto-recenter is enabled,
   * or the adapter silently keeps projecting in the stale frame.
   */
  void onDatumRecentered(const geo::Datum& old_datum,
                         const geo::Datum& new_datum) override;

 private:
  geo::Datum datum_;
  AisAdapterConfig cfg_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
